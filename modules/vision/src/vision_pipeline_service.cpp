#include "scan_tracking/vision/vision_pipeline_service.h"

#include <QtCore/QMetaType>
#include <QtCore/QPointer>
#include <QtCore/QTimer>

#include <thread>
#include <qdebug.h>
#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/vision/hik_cxp_camera_service.h"
#include "scan_tracking/vision/lb_pose_detection_adapter.h"
#include "scan_tracking/vision/lbn_pose_detection_adapter.h"

namespace scan_tracking {
namespace vision {

namespace {

// 标记点未安装时的 LBN 旁路结果：返回 4×4 单位阵，供调试/联调使用
LbnPoseResult makeIdentityLbnBypassResult()
{
    LbnPoseResult result;
    result.invoked = true;
    result.success = true;
    result.poseMatrix.valid = true;
    result.matchedPointCount = 0;
    result.message = QStringLiteral(
        "TODO(marker): 未安装标记点，useIdentityRtWithoutMarkers=true，Rt 使用 4×4 单位阵。");
    return result;
}

}  // namespace

// 注册跨线程信号槽所需的 Qt 元类型（仅执行一次）
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
    HikCxpCameraService* hikCameraAService,
    HikCxpCameraService* hikCameraBService,
    QObject* parent)
    : QObject(parent)
    , m_mechEyeService(mechEyeService)
    , m_hikCameraAService(hikCameraAService)
    , m_hikCameraBService(hikCameraBService)
{
    qInfo() << QStringLiteral("[VisionPipeline] 构造函数入口 mechEye=")
            << (m_mechEyeService != nullptr) << QStringLiteral(" hikA=")
            << (m_hikCameraAService != nullptr) << QStringLiteral(" hikB=")
            << (m_hikCameraBService != nullptr);
    registerMetaTypes();

    // 三路相机服务的完成信号均通过 QueuedConnection 回到主线程槽
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
            &HikCxpCameraService::poseCaptureFinished,
            this,
            &VisionPipelineService::onHikPoseCaptureFinished,
            Qt::QueuedConnection);
    }
    if (m_hikCameraBService != nullptr) {
        connect(
            m_hikCameraBService,
            &HikCxpCameraService::poseCaptureFinished,
            this,
            &VisionPipelineService::onHikPoseCaptureFinished,
            Qt::QueuedConnection);
    }
    qInfo() << QStringLiteral("[VisionPipeline] 构造函数完成。");
}

void VisionPipelineService::start(const scan_tracking::common::VisionConfig& config)
{
    m_config = config;
    m_pending = PendingCaptureContext{};
    m_processing = false;
    if (const auto* configManager = scan_tracking::common::ConfigManager::instance()) {
        m_lbPoseConfig = configManager->lbPoseConfig();
        m_lbnPoseConfig = configManager->lbnPoseConfig();
    } else {
        m_lbPoseConfig = {};
        m_lbnPoseConfig = {};
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

namespace {

// 梅卡与 CXP 之间的时间间隔（ms），避免两路相机同时曝光造成干扰
constexpr int kMechToHikCaptureDelayMs = 2000;

// 判断梅卡采集结果是否包含有效载荷（2D 模式看纹理，3D 模式看点云）
bool mechCapturePayloadReady(const scan_tracking::mech_eye::CaptureResult& result)
{
    if (!result.success()) {
        return false;
    }
    if (result.mode == scan_tracking::mech_eye::CaptureMode::Capture2DOnly) {
        return result.texture2D.isValid();
    }
    return result.pointCloud.isValid();
}

}  // namespace

quint64 VisionPipelineService::requestCaptureBundle(
    int segmentIndex,
    quint32 taskId,
    scan_tracking::mech_eye::CaptureMode mechCaptureMode,
    bool mechEdgeArtifactRemovalEnabled,
    bool mechComparisonCaptureEnabled)
{
    // ---- 前置校验 ----
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

    // ---- 组装请求并启动梅卡采集（CXP 在梅卡回调中延迟触发） ----
    MultiCameraCaptureRequest request;
    request.requestId = m_nextRequestId++;
    request.taskId = taskId;
    request.segmentIndex = segmentIndex;
    request.mechCaptureMode = mechCaptureMode;
    // Capture2DAnd3D 表示转盘段，后续会跑 LBN 而非 LB
    request.needMechEye2D =
        mechCaptureMode == scan_tracking::mech_eye::CaptureMode::Capture2DAnd3D;
    request.mechEdgeArtifactRemovalEnabled = mechEdgeArtifactRemovalEnabled;
    request.mechComparisonCaptureEnabled = mechComparisonCaptureEnabled;
    request.mechEyeCameraKey = m_config.mechEyeCameraKey;
    request.mechEyeTimeoutMs = m_config.mechCaptureTimeoutMs > 0 ? m_config.mechCaptureTimeoutMs : 5000;
    request.hikCameraAKey = m_config.hikCxpCameraA.cameraKey;
    request.hikCameraBKey = m_config.hikCxpCameraB.cameraKey;
    request.hikTimeoutMs =
        m_config.hikCxpCaptureTimeoutMs > 0 ? m_config.hikCxpCaptureTimeoutMs : 5000;

    PendingCaptureContext pending;
    pending.active = true;
    pending.bundle.request = request;

    qInfo() << QStringLiteral("[VisionPipeline] 段号=") << segmentIndex
            << QStringLiteral(" 需梅卡2D=") << request.needMechEye2D
            << QStringLiteral(" 梅卡模式=") << static_cast<int>(mechCaptureMode)
            << QStringLiteral(" CXP延迟ms=") << kMechToHikCaptureDelayMs;
    pending.mechRequestId = m_mechEyeService->requestCapture(
        request.mechEyeCameraKey,
        mechCaptureMode,
        request.mechEyeTimeoutMs,
        request.mechEdgeArtifactRemovalEnabled,
        request.mechComparisonCaptureEnabled);
    if (pending.mechRequestId == 0) {
        emit fatalError(VisionErrorCode::CaptureRejected, QStringLiteral("启动 Mech-Eye 采集失败。"));
        return 0;
    }
    pending.mechDone = false;
    pending.hikADone = false;
    pending.hikBDone = false;

    m_pending = pending;
    setState(
        VisionPipelineState::Capturing,
        QStringLiteral("梅卡采集已启动（CXP 将在梅卡完成后延迟 %1ms）").arg(kMechToHikCaptureDelayMs));
    return request.requestId;
}

quint64 VisionPipelineService::requestCaptureBundle(
    int segmentIndex,
    quint32 taskId,
    scan_tracking::mech_eye::CaptureMode mechCaptureMode)
{
    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    const auto visionConfig = configManager != nullptr
        ? configManager->visionConfig()
        : scan_tracking::common::VisionConfig{};
    return requestCaptureBundle(
        segmentIndex,
        taskId,
        mechCaptureMode,
        visionConfig.mechEdgeArtifactRemovalEnabled,
        visionConfig.mechEdgeArtifactRemovalComparisonEnabled);
}

// 梅卡延迟到期后调用：并行发起 CXP 左/右目 poseCapture
void VisionPipelineService::startPendingHikCapture()
{
    // hikARequestId != 0 表示 CXP 已启动，防止定时器重复触发
    if (!m_pending.active || m_pending.hikARequestId != 0) {
        return;
    }

    const auto& request = m_pending.bundle.request;
    qInfo() << QStringLiteral("[VisionPipeline] 梅卡已完成，启动 CXP 双目采集 段号=")
            << request.segmentIndex;

    m_pending.hikARequestId = m_hikCameraAService->requestPoseCapture(
        request.hikCameraAKey, request.hikTimeoutMs);
    m_pending.hikBRequestId = m_hikCameraBService->requestPoseCapture(
        request.hikCameraBKey, request.hikTimeoutMs);

    if (m_pending.hikARequestId == 0 || m_pending.hikBRequestId == 0) {
        m_pending.active = false;
        m_processing = false;
        emit fatalError(
            VisionErrorCode::CaptureRejected,
            QStringLiteral("梅卡完成后启动 CXP 双目采集失败。"));
        setState(VisionPipelineState::Error, QStringLiteral("CXP 双目采集启动失败。"));
        return;
    }

    setState(
        VisionPipelineState::Capturing,
        QStringLiteral("CXP 双目采集已启动：requestId=%1").arg(request.requestId));
}

void VisionPipelineService::onMechEyeCaptureFinished(scan_tracking::mech_eye::CaptureResult result)
{
    // 忽略非当前请求的过期回调
    if (!m_pending.active || result.requestId != m_pending.mechRequestId) {
        return;
    }

    m_pending.bundle.mechEyeResult = result;
    m_pending.mechDone = true;

    // 梅卡先完成，延迟后再启动 CXP，避免硬件/光照冲突
    qInfo() << QStringLiteral("[VisionPipeline] 梅卡采集结束，%1ms 后启动 CXP 双目")
                   .arg(kMechToHikCaptureDelayMs);
    QTimer::singleShot(kMechToHikCaptureDelayMs, this, [this]() {
        startPendingHikCapture();
    });
}

void VisionPipelineService::onHikPoseCaptureFinished(scan_tracking::vision::HikPoseCaptureResult result)
{
    if (!m_pending.active) {
        return;
    }

    // 用 logicalName 区分来源（两个相机服务的 requestId 可能重复）
    if (result.logicalName == m_config.hikCxpCameraA.logicalName) {
        m_pending.bundle.hikCameraAResult = result;
        m_pending.hikADone = true;
    } else if (result.logicalName == m_config.hikCxpCameraB.logicalName) {
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

// 三路采集全部完成后：校验梅卡载荷，再在后台线程分支执行 LBN / LB 位姿检测
void VisionPipelineService::finishBundleIfReady()
{
    if (!m_pending.active || !m_pending.mechDone || !m_pending.hikADone || !m_pending.hikBDone) {
        return;
    }

    const auto bundle = m_pending.bundle;
    m_pending = PendingCaptureContext{};

    // 梅卡载荷无效时直接结束，不再跑位姿算法
    if (!mechCapturePayloadReady(bundle.mechEyeResult)) {
        m_processing = false;
        setState(
            VisionPipelineState::Error,
            QStringLiteral("Mech-Eye 采集失败：%1").arg(bundle.mechEyeResult.errorMessage));
        emit bundleCaptureFinished(bundle);
        return;
    }

    const bool hikReady =
        bundle.hikCameraAResult.success() && bundle.hikCameraBResult.success();
    // 转盘段 + 配置启用 → LBN；封头段 → LB（见下方 runLb 分支）
    const bool runLbn = bundle.request.needMechEye2D && m_lbnPoseConfig.enabled;

    m_processing = true;
    setState(
        VisionPipelineState::Capturing,
        QStringLiteral("Mech-Eye 采集完成，正在处理视觉结果（海康=%1, LBN=%2）")
            .arg(hikReady ? QStringLiteral("就绪") : QStringLiteral("跳过"))
            .arg(runLbn ? QStringLiteral("开") : QStringLiteral("关")));

    const auto lbConfig = m_lbPoseConfig;
    const auto lbnConfig = m_lbnPoseConfig;
    QPointer<VisionPipelineService> self(this);
    // 位姿检测为 CPU 密集型，放到 detached 后台线程，完成后 invokeMethod 回主线程
    std::thread([self, bundle, lbConfig, lbnConfig, hikReady, runLbn]() mutable {
        auto completedBundle = bundle;

        if (runLbn) {
            if (lbnConfig.useIdentityRtWithoutMarkers) {
                completedBundle.lbnPoseResult = makeIdentityLbnBypassResult();
                qWarning() << "[LBN位姿]" << completedBundle.lbnPoseResult.message;
            } else {
                qInfo() << QStringLiteral("[LBN位姿] 开始检测")
                        << QStringLiteral(" 纹理=") << bundle.mechEyeResult.texture2D.width << QStringLiteral("x")
                        << bundle.mechEyeResult.texture2D.height
                        << QStringLiteral(" 点云网格=") << bundle.mechEyeResult.pointCloud.width << QStringLiteral("x")
                        << bundle.mechEyeResult.pointCloud.height;
                completedBundle.lbnPoseResult = runLbnPoseDetection(bundle.mechEyeResult, lbnConfig);
            }
            const auto& lbn = completedBundle.lbnPoseResult;
            qInfo() << QStringLiteral("[LBN位姿] 完成：已调用=") << lbn.invoked << QStringLiteral(" 成功=") << lbn.success
                    << QStringLiteral(" 说明=") << lbn.message << QStringLiteral(" 匹配点数=") << lbn.matchedPointCount;
        } else {
            completedBundle.lbnPoseResult.invoked = false;
            completedBundle.lbnPoseResult.message =
                bundle.request.needMechEye2D
                    ? QStringLiteral("LBN 位姿检测已在配置中禁用。")
                    : QStringLiteral("非转动点位，跳过 LBN 位姿检测。");
        }

        // 转盘段（needMechEye2D）仅 LBN 标定；封头段用 CXP 双目跑 LB
        const bool runLb = hikReady && !bundle.request.needMechEye2D;
        if (runLb) {
            qInfo() << QStringLiteral("[LB位姿] 开始检测，左目(CameraA)=")
                    << bundle.request.hikCameraAKey
                    << bundle.hikCameraAResult.frame.width << QStringLiteral("x")
                    << bundle.hikCameraAResult.frame.height
                    << QStringLiteral(" frameId=") << bundle.hikCameraAResult.frame.frameId
                    << QStringLiteral(" timestampMs=") << bundle.hikCameraAResult.frame.timestampMs
                    << QStringLiteral(" 右目(CameraB)=") << bundle.request.hikCameraBKey
                    << bundle.hikCameraBResult.frame.width << QStringLiteral("x")
                    << bundle.hikCameraBResult.frame.height
                    << QStringLiteral(" frameId=") << bundle.hikCameraBResult.frame.frameId
                    << QStringLiteral(" timestampMs=") << bundle.hikCameraBResult.frame.timestampMs;

            completedBundle.lbPoseResult = runLbPoseDetection(
                completedBundle.hikCameraAResult.frame,
                completedBundle.hikCameraBResult.frame,
                lbConfig);

            const auto& lr = completedBundle.lbPoseResult;
            qInfo() << QStringLiteral("[LB位姿] 完成：成功=") << lr.success << QStringLiteral(" 说明=") << lr.message
                    << QStringLiteral(" 帧点数=") << lr.framePointCount;
            if (lr.poseMatrix.valid) {
                qInfo() << QStringLiteral("[LB位姿] Rt 矩阵：");
                for (int row = 0; row < 4; ++row) {
                    qInfo().noquote() << QString("  [%1, %2, %3, %4]")
                        .arg(static_cast<double>(lr.poseMatrix.values[row * 4 + 0]), 12, 'f', 6)
                        .arg(static_cast<double>(lr.poseMatrix.values[row * 4 + 1]), 12, 'f', 6)
                        .arg(static_cast<double>(lr.poseMatrix.values[row * 4 + 2]), 12, 'f', 6)
                        .arg(static_cast<double>(lr.poseMatrix.values[row * 4 + 3]), 12, 'f', 6);
                }
            }
        } else if (hikReady && bundle.request.needMechEye2D) {
            completedBundle.lbPoseResult.invoked = false;
            completedBundle.lbPoseResult.success = false;
            completedBundle.lbPoseResult.message =
                QStringLiteral("转盘点位，跳过 LB 位姿检测（仅 LBN）。");
            qInfo() << QStringLiteral("[LB位姿] 转盘段跳过（needMechEye2D=true，仅 LBN）");
        } else {
            completedBundle.lbPoseResult.invoked = false;
            completedBundle.lbPoseResult.success = false;
            completedBundle.lbPoseResult.message = QStringLiteral("CXP 双目未就绪，跳过 LB 位姿检测");
        }

        if (!self) {
            return; // 对象已销毁，放弃回传
        }

        // 切回主线程 emit 结果，保证线程安全
        QMetaObject::invokeMethod(
            self.data(),
            [self, completedBundle, hikReady]() mutable {
                if (!self || !self->m_started) {
                    return;
                }

                self->m_processing = false;
                const bool mechOk = mechCapturePayloadReady(completedBundle.mechEyeResult);
                // 旁路模式（PLC 联调）：只验 MechEye 采集 + PLC 握手，CXP 未连接不应
                // 毒化流水线状态、阻断后续段。故旁路下以 MechEye 成功即判定回 Ready，
                // CXP A/B 失败仅记录、不影响状态。非旁路生产仍要求 CXP 双目成功。
                const bool bypassEnabled = []() {
                    if (const auto* cm = scan_tracking::common::ConfigManager::instance()) {
                        return cm->flowControlConfig().algorithmBypassEnabled;
                    }
                    return false;
                }();
                const bool ok = bypassEnabled
                    ? mechOk
                    : (mechOk
                       && completedBundle.hikCameraAResult.success()
                       && completedBundle.hikCameraBResult.success());
                QString description;
                if (ok) {
                    description = bypassEnabled
                        ? QStringLiteral("视觉组合采集完成（旁路模式：MechEye 成功，CXP 失败不阻断）。")
                        : QStringLiteral("视觉组合采集成功完成。");
                } else {
                    description = QStringLiteral("视觉组合采集完成但有错误。");
                }
                self->setState(
                    ok ? VisionPipelineState::Ready : VisionPipelineState::Error,
                    description);
                emit self->bundleCaptureFinished(completedBundle);
            },
            Qt::QueuedConnection);
    }).detach();
}

}  // namespace vision
}  // namespace scan_tracking
