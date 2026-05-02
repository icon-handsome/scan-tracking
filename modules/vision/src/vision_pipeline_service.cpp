#include "scan_tracking/vision/vision_pipeline_service.h"

#include <QtCore/QMetaType>
#include <QtCore/QPointer>

#include <thread>

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
        QStringLiteral("Vision pipeline started and waiting for capture requests."));
}

void VisionPipelineService::stop()
{
    if (!m_started) {
        return;
    }

    m_pending = PendingCaptureContext{};
    m_processing = false;
    m_started = false;
    setState(VisionPipelineState::Stopped, QStringLiteral("Vision pipeline stopped."));
}

quint64 VisionPipelineService::requestCaptureBundle(int segmentIndex, quint32 taskId)
{
    if (!m_started) {
        emit fatalError(VisionErrorCode::NotStarted, QStringLiteral("Vision pipeline is not started."));
        return 0;
    }
    if (m_pending.active || m_processing) {
        emit fatalError(VisionErrorCode::Busy, QStringLiteral("A vision capture request is already in progress."));
        return 0;
    }
    if (m_mechEyeService == nullptr || m_hikCameraAService == nullptr || m_hikCameraBService == nullptr) {
        emit fatalError(VisionErrorCode::InvalidConfig, QStringLiteral("Vision services are incomplete."));
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

    pending.mechRequestId = m_mechEyeService->requestCapture(
        request.mechEyeCameraKey,
        scan_tracking::mech_eye::CaptureMode::Capture3DOnly,
        request.mechEyeTimeoutMs);
    if (pending.mechRequestId == 0) {
        emit fatalError(VisionErrorCode::CaptureRejected, QStringLiteral("Failed to start MechEye capture."));
        return 0;
    }

    pending.hikARequestId = m_hikCameraAService->requestPoseCapture(
        request.hikCameraAKey,
        request.hikTimeoutMs);
    pending.hikBRequestId = m_hikCameraBService->requestPoseCapture(
        request.hikCameraBKey,
        request.hikTimeoutMs);
    if (pending.hikARequestId == 0 || pending.hikBRequestId == 0) {
        emit fatalError(
            VisionErrorCode::CaptureRejected,
            QStringLiteral("Failed to start Hik capture for one or both cameras."));
        return 0;
    }

    m_pending = pending;
    setState(
        VisionPipelineState::Capturing,
        QStringLiteral("Capture bundle requested: requestId=%1").arg(request.requestId));
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

    if (result.requestId == m_pending.hikARequestId) {
        m_pending.bundle.hikCameraAResult = result;
        m_pending.hikADone = true;
    } else if (result.requestId == m_pending.hikBRequestId) {
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
            QStringLiteral("One or more capture steps failed: %1").arg(bundle.summary()));
        emit bundleCaptureFinished(bundle);
        return;
    }

    m_processing = true;
    setState(
        VisionPipelineState::Capturing,
        QStringLiteral("All images captured, processing vision results."));

    const auto lbConfig = m_lbPoseConfig;
    QPointer<VisionPipelineService> self(this);
    std::thread([self, bundle, lbConfig]() mutable {
        auto completedBundle = bundle;
        completedBundle.lbPoseResult = runLbPoseDetection(
            completedBundle.hikCameraAResult.frame,
            completedBundle.hikCameraBResult.frame,
            lbConfig);

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
                    ok ? QStringLiteral("Vision bundle completed successfully.")
                       : QStringLiteral("Vision bundle completed with errors."));
                emit self->bundleCaptureFinished(completedBundle);
            },
            Qt::QueuedConnection);
    }).detach();
}

}  // namespace vision
}  // namespace scan_tracking
