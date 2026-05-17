#include "scan_tracking/vision/vision_pipeline_service.h"

#include <QtCore/QMetaType>
#include <QtCore/QPointer>

#include <thread>
#include <qdebug.h>
#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/vision/hik_camera_service.h"
#include "scan_tracking/vision/lb_pose_detection_adapter.h"

namespace scan_tracking {
namespace vision {

void VisionPipelineService::registerMetaTypes()
{
    static bool registered = false;
    if (registered) {
        return;
    }

    qRegisterMetaType<scan_tracking::vision::VisionPipelineState>(
        "scan_tracking::vision::VisionPipelineState");
    qRegisterMetaType<scan_tracking::vision::MultiCameraCaptureBundle>(
        "scan_tracking::vision::MultiCameraCaptureBundle");
    registered = true;
}

VisionPipelineService::VisionPipelineService(
    scan_tracking::mech_eye::MechEyeService* mechEyeService,
    HikCameraService* hikCameraAService,
    HikCameraService* hikCameraBService,
    QObject* parent)
    : QObject(parent)
    , m_mechEyeService(mechEyeService)
    , m_hikCameraAService(hikCameraAService)
    , m_hikCameraBService(hikCameraBService)
{
    registerMetaTypes();

    if (m_mechEyeService != nullptr) {
        connect(
            m_mechEyeService,
            &scan_tracking::mech_eye::MechEyeService::captureFinished,
            this,
            &VisionPipelineService::onMechEyeCaptureFinished,
            Qt::QueuedConnection);
    }
    if (m_hikCameraAService != nullptr) {
        connect(
            m_hikCameraAService,
            &HikCameraService::poseCaptureFinished,
            this,
            &VisionPipelineService::onHikPoseCaptureFinished,
            Qt::QueuedConnection);
    }
    if (m_hikCameraBService != nullptr) {
        connect(
            m_hikCameraBService,
            &HikCameraService::poseCaptureFinished,
            this,
            &VisionPipelineService::onHikPoseCaptureFinished,
            Qt::QueuedConnection);
    }
}

void VisionPipelineService::start(const scan_tracking::common::VisionConfig& config)
{
    m_config = config;
    m_pending = PendingCaptureContext{};
    m_processing = false;
    if (const auto* configManager = scan_tracking::common::ConfigManager::instance()) {
        m_lbPoseConfig = configManager->lbPoseConfig();
    } else {
        m_lbPoseConfig = {};
    }
    m_started = true;
    setState(
        VisionPipelineState::Ready,
        QStringLiteral("视觉流水线已启动，等待采集请求。"));
}

void VisionPipelineService::stop()
{
    if (!m_started) {
        return;
    }

    m_pending = PendingCaptureContext{};
    m_processing = false;
    m_started = false;
    setState(VisionPipelineState::Stopped, QStringLiteral("视觉流水线已停止。"));
}

quint64 VisionPipelineService::requestCaptureBundle(int segmentIndex, quint32 taskId)
{
    if (!m_started) {
        emit fatalError(VisionErrorCode::NotStarted, QStringLiteral("视觉流水线未启动。"));
        return 0;
    }
    if (m_pending.active || m_processing) {
        emit fatalError(VisionErrorCode::Busy, QStringLiteral("视觉采集请求正在进行中。"));
        return 0;
    }
    if (m_mechEyeService == nullptr || m_hikCameraAService == nullptr || m_hikCameraBService == nullptr) {
        emit fatalError(VisionErrorCode::InvalidConfig, QStringLiteral("视觉服务不完整。"));
        return 0;
    }

    MultiCameraCaptureRequest request;
    request.requestId = m_nextRequestId++;
    request.taskId = taskId;
    request.segmentIndex = segmentIndex;
    request.mechEyeCameraKey = m_config.mechEyeCameraKey;
    request.mechEyeTimeoutMs = m_config.mechCaptureTimeoutMs > 0 ? m_config.mechCaptureTimeoutMs : 5000;
    request.hikCameraAKey = m_config.hikCameraA.cameraKey;
    request.hikCameraBKey = m_config.hikCameraB.cameraKey;
    request.hikTimeoutMs = m_config.hikCaptureTimeoutMs > 0 ? m_config.hikCaptureTimeoutMs : 1000;

    PendingCaptureContext pending;
    pending.active = true;
    pending.bundle.request = request;

    // TODO: MechEye 暂时屏蔽（已验证通过），当前只测试海康 A/B
    // pending.mechRequestId = m_mechEyeService->requestCapture(
    //     request.mechEyeCameraKey,
    //     scan_tracking::mech_eye::CaptureMode::Capture3DOnly,
    //     request.mechEyeTimeoutMs);
    pending.mechRequestId = 1;  // 占位
    pending.mechDone = true;
    pending.bundle.mechEyeResult.requestId = 1;
    pending.bundle.mechEyeResult.errorCode = scan_tracking::mech_eye::CaptureErrorCode::Success;
    pending.bundle.mechEyeResult.pointCloud.pointCount = 0;  // MechEye 未连接，点云为空

    // 海康 A/B 正式采集
    pending.hikARequestId = m_hikCameraAService->requestPoseCapture(
        request.hikCameraAKey, request.hikTimeoutMs);
    pending.hikBRequestId = m_hikCameraBService->requestPoseCapture(
        request.hikCameraBKey, request.hikTimeoutMs);

    if (pending.hikARequestId == 0) {
        emit fatalError(VisionErrorCode::CaptureRejected, QStringLiteral("启动海康A采集失败。"));
        return 0;
    }
    if (pending.hikBRequestId == 0) {
        emit fatalError(VisionErrorCode::CaptureRejected, QStringLiteral("启动海康B采集失败。"));
        return 0;
    }

    m_pending = pending;
    setState(
        VisionPipelineState::Capturing,
        QStringLiteral("组合采集请求已发送：requestId=%1").arg(request.requestId));
    return request.requestId;
}

void VisionPipelineService::onMechEyeCaptureFinished(scan_tracking::mech_eye::CaptureResult result)
{
    if (!m_pending.active || result.requestId != m_pending.mechRequestId) {
        return;
    }

    m_pending.bundle.mechEyeResult = result;
    m_pending.mechDone = true;
    finishBundleIfReady();
}

void VisionPipelineService::onHikPoseCaptureFinished(scan_tracking::vision::HikPoseCaptureResult result)
{
    if (!m_pending.active) {
        return;
    }

    // 用 logicalName 区分来源（两个相机服务的 requestId 可能重复）
    if (result.logicalName == m_config.hikCameraA.logicalName) {
        m_pending.bundle.hikCameraAResult = result;
        m_pending.hikADone = true;
    } else if (result.logicalName == m_config.hikCameraB.logicalName) {
        m_pending.bundle.hikCameraBResult = result;
        m_pending.hikBDone = true;
    } else {
        return;
    }

    finishBundleIfReady();
}

void VisionPipelineService::setState(VisionPipelineState state, const QString& description)
{
    m_state = state;
    emit stateChanged(state, description);
}

void VisionPipelineService::finishBundleIfReady()
{
    if (!m_pending.active || !m_pending.mechDone || !m_pending.hikADone || !m_pending.hikBDone) {
        return;
    }

    const auto bundle = m_pending.bundle;
    m_pending = PendingCaptureContext{};

    if (!bundle.mechEyeResult.success() || !bundle.hikCameraAResult.success() || !bundle.hikCameraBResult.success()) {
        m_processing = false;
        setState(
            VisionPipelineState::Error,
            QStringLiteral("一个或多个采集步骤失败：%1").arg(bundle.summary()));
        emit bundleCaptureFinished(bundle);
        return;
    }

    m_processing = true;
    setState(
        VisionPipelineState::Capturing,
        QStringLiteral("所有图像已采集，正在处理视觉结果。"));

    // TODO: 海康 A/B 未接入时跳过 LB 位姿检测，直接完成 bundle
    // 当海康相机正式上线后，恢复 LB 位姿检测逻辑
    if (!bundle.hikCameraAResult.success() || !bundle.hikCameraBResult.success()) {
        m_processing = false;
        auto completedBundle = bundle;
        completedBundle.lbPoseResult.invoked = false;
        completedBundle.lbPoseResult.success = false;
        completedBundle.lbPoseResult.message = QStringLiteral("海康相机未就绪，跳过 LB 位姿检测");
        setState(VisionPipelineState::Ready, QStringLiteral("视觉组合采集完成（跳过LB位姿）。"));
        emit bundleCaptureFinished(completedBundle);
        return;
    }

    const auto lbConfig = m_lbPoseConfig;
    QPointer<VisionPipelineService> self(this);
    std::thread([self, bundle, lbConfig]() mutable {
        auto completedBundle = bundle;
        qInfo() << "[LB位姿] 开始检测, leftFrame=" << bundle.hikCameraAResult.frame.width << "x" << bundle.hikCameraAResult.frame.height
                << "rightFrame=" << bundle.hikCameraBResult.frame.width << "x" << bundle.hikCameraBResult.frame.height;

        completedBundle.lbPoseResult = runLbPoseDetection(
            completedBundle.hikCameraAResult.frame,
            completedBundle.hikCameraBResult.frame,
            lbConfig);

        const auto& lr = completedBundle.lbPoseResult;
        qInfo() << "[LB位姿] 完成: success=" << lr.success << "message=" << lr.message
                << "framePointCount=" << lr.framePointCount;
        if (lr.poseMatrix.valid) {
            qInfo() << "[LB位姿] Rt矩阵:";
            for (int row = 0; row < 4; ++row) {
                qInfo().noquote() << QString("  [%1, %2, %3, %4]")
                    .arg(static_cast<double>(lr.poseMatrix.values[row * 4 + 0]), 12, 'f', 6)
                    .arg(static_cast<double>(lr.poseMatrix.values[row * 4 + 1]), 12, 'f', 6)
                    .arg(static_cast<double>(lr.poseMatrix.values[row * 4 + 2]), 12, 'f', 6)
                    .arg(static_cast<double>(lr.poseMatrix.values[row * 4 + 3]), 12, 'f', 6);
            }
        }

        if (!self) {
            return;
        }

        QMetaObject::invokeMethod(
            self.data(),
            [self, completedBundle]() mutable {
                if (!self || !self->m_started) {
                    return;
                }

                self->m_processing = false;
                const bool ok = completedBundle.success();
                self->setState(
                    ok ? VisionPipelineState::Ready : VisionPipelineState::Error,
                    ok ? QStringLiteral("视觉组合采集成功完成。")
                       : QStringLiteral("视觉组合采集完成但有错误。"));
                emit self->bundleCaptureFinished(completedBundle);
            },
            Qt::QueuedConnection);
    }).detach();
}

}  // namespace vision
}  // namespace scan_tracking
