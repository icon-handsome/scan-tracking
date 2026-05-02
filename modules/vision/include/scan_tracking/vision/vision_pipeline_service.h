#pragma once

#include <QtCore/QObject>

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/mech_eye/mech_eye_service.h"
#include "scan_tracking/vision/vision_types.h"

namespace scan_tracking {
namespace vision {

class HikCameraService;

class VisionPipelineService : public QObject {
    Q_OBJECT

public:
    VisionPipelineService(
        scan_tracking::mech_eye::MechEyeService* mechEyeService,
        HikCameraService* hikCameraAService,
        HikCameraService* hikCameraBService,
        QObject* parent = nullptr);
    ~VisionPipelineService() override = default;

    void start(const scan_tracking::common::VisionConfig& config);
    void stop();

    bool isStarted() const { return m_started; }
    VisionPipelineState state() const { return m_state; }

    quint64 requestCaptureBundle(int segmentIndex, quint32 taskId);

signals:
    void bundleCaptureFinished(scan_tracking::vision::MultiCameraCaptureBundle bundle);
    void stateChanged(scan_tracking::vision::VisionPipelineState state, QString description);
    void fatalError(scan_tracking::vision::VisionErrorCode code, QString message);

private slots:
    void onMechEyeCaptureFinished(scan_tracking::mech_eye::CaptureResult result);
    void onHikPoseCaptureFinished(scan_tracking::vision::HikPoseCaptureResult result);

private:
    struct PendingCaptureContext {
        bool active = false;
        bool mechDone = false;
        bool hikADone = false;
        bool hikBDone = false;
        quint64 mechRequestId = 0;
        quint64 hikARequestId = 0;
        quint64 hikBRequestId = 0;
        scan_tracking::vision::MultiCameraCaptureBundle bundle;
    };

    static void registerMetaTypes();
    void setState(VisionPipelineState state, const QString& description);
    void finishBundleIfReady();

    scan_tracking::mech_eye::MechEyeService* m_mechEyeService = nullptr;
    HikCameraService* m_hikCameraAService = nullptr;
    HikCameraService* m_hikCameraBService = nullptr;
    scan_tracking::common::VisionConfig m_config;
    scan_tracking::common::LbPoseConfig m_lbPoseConfig;
    PendingCaptureContext m_pending;
    quint64 m_nextRequestId = 1;
    bool m_started = false;
    bool m_processing = false;
    VisionPipelineState m_state = VisionPipelineState::Idle;
};

}  // namespace vision
}  // namespace scan_tracking
