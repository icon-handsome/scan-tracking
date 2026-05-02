#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include <QtCore/QMetaType>
#include <QtCore/QString>
#include <QtCore/QtGlobal>

#include "scan_tracking/mech_eye/mech_eye_types.h"

namespace scan_tracking {
namespace vision {

enum class VisionErrorCode {
    Success = 0,
    NotStarted = 1,
    Busy = 2,
    InvalidConfig = 3,
    CaptureRejected = 4,
    NotImplemented = 5,
    DeviceNotFound = 6,
    DeviceOpenFailed = 7,
    SdkInitFailed = 8,
    UnknownError = 9,
};

enum class VisionPipelineState {
    Idle = 0,
    Ready = 1,
    Capturing = 2,
    Error = 3,
    Stopped = 4,
};

struct PoseMatrix4x4 {
    std::array<float, 16> values = {
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f,
    };
    quint64 frameId = 0;
    qint64 timestampMs = 0;
    QString sourceCameraKey;
    bool valid = false;

    bool isValid() const { return valid; }
};

struct HikMonoFrame {
    std::shared_ptr<std::vector<std::uint8_t>> pixels;
    int width = 0;
    int height = 0;
    int stride = 0;
    quint64 frameId = 0;
    qint64 timestampMs = 0;
    QString sourceCameraKey;
    QString pixelFormat = QStringLiteral("Mono8");

    bool isValid() const
    {
        return pixels != nullptr && !pixels->empty() && width > 0 && height > 0;
    }
};

struct HikPoseCaptureRequest {
    quint64 requestId = 0;
    QString cameraKey;
    QString logicalName;
    int timeoutMs = 1000;
};

struct HikPoseCaptureResult {
    quint64 requestId = 0;
    QString cameraKey;
    QString logicalName;
    VisionErrorCode errorCode = VisionErrorCode::Success;
    QString errorMessage;
    HikMonoFrame frame;
    PoseMatrix4x4 poseMatrix;
    qint64 elapsedMs = 0;

    bool success() const
    {
        return errorCode == VisionErrorCode::Success && frame.isValid();
    }

    bool hasPose() const
    {
        return poseMatrix.isValid();
    }
};

struct LbPoseResult {
    bool invoked = false;
    bool success = false;
    QString message;
    int leftImageWidth = 0;
    int leftImageHeight = 0;
    int rightImageWidth = 0;
    int rightImageHeight = 0;
    int framePointCount = 0;
    PoseMatrix4x4 poseMatrix;
};

struct MultiCameraCaptureRequest {
    quint64 requestId = 0;
    quint32 taskId = 0;
    int segmentIndex = 0;
    QString mechEyeCameraKey;
    int mechEyeTimeoutMs = 5000;
    QString hikCameraAKey;
    QString hikCameraBKey;
    int hikTimeoutMs = 1000;
};

struct MultiCameraCaptureBundle {
    MultiCameraCaptureRequest request;
    scan_tracking::mech_eye::CaptureResult mechEyeResult;
    HikPoseCaptureResult hikCameraAResult;
    HikPoseCaptureResult hikCameraBResult;
    LbPoseResult lbPoseResult;

    bool success() const
    {
        return mechEyeResult.success() &&
               hikCameraAResult.success() &&
               hikCameraBResult.success() &&
               lbPoseResult.success;
    }

    QString summary() const
    {
        return QStringLiteral(
                   "bundle requestId=%1 taskId=%2 segment=%3 mech=%4 hikA=%5 hikB=%6 lb=%7")
            .arg(request.requestId)
            .arg(request.taskId)
            .arg(request.segmentIndex)
            .arg(mechEyeResult.success() ? QStringLiteral("ok") : QStringLiteral("fail"))
            .arg(hikCameraAResult.success() ? QStringLiteral("ok") : QStringLiteral("fail"))
            .arg(hikCameraBResult.success() ? QStringLiteral("ok") : QStringLiteral("fail"))
            .arg(lbPoseResult.success ? QStringLiteral("ok") : QStringLiteral("fail"));
    }
};

}  // namespace vision
}  // namespace scan_tracking

Q_DECLARE_METATYPE(scan_tracking::vision::VisionErrorCode)
Q_DECLARE_METATYPE(scan_tracking::vision::VisionPipelineState)
Q_DECLARE_METATYPE(scan_tracking::vision::PoseMatrix4x4)
Q_DECLARE_METATYPE(scan_tracking::vision::HikMonoFrame)
Q_DECLARE_METATYPE(scan_tracking::vision::HikPoseCaptureRequest)
Q_DECLARE_METATYPE(scan_tracking::vision::HikPoseCaptureResult)
Q_DECLARE_METATYPE(scan_tracking::vision::LbPoseResult)
Q_DECLARE_METATYPE(scan_tracking::vision::MultiCameraCaptureRequest)
Q_DECLARE_METATYPE(scan_tracking::vision::MultiCameraCaptureBundle)
