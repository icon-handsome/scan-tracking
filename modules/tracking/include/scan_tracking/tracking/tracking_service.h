#pragma once

#include <array>
#include <string>

#include <QtCore/QMap>
#include <QtCore/QString>
#include <QtCore/QtGlobal>

#include "scan_tracking/mech_eye/mech_eye_types.h"

namespace scan_tracking {
namespace tracking {

struct InspectionResult {
    quint16 resultCode = 0;
    quint16 ngReasonWord0 = 0;
    quint16 ngReasonWord1 = 0;
    quint16 measureItemCount = 0;
    int segmentCount = 0;
    int totalPointCount = 0;
    float offsetXmm = 0.0f;
    float offsetYmm = 0.0f;
    float offsetZmm = 0.0f;
    QString message;
};

struct PoseCheckResult {
    bool invoked = false;
    bool success = false;
    quint16 resultCode = 7;
    int inputPointCount = 0;
    double poseDeviationMm = 0.0;
    std::array<double, 16> rt = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    };
    QString message;
};

class TrackingService {
public:
    std::string statusText() const;
    InspectionResult inspectSegments(
        const QMap<int, scan_tracking::mech_eye::CaptureResult>& segmentCaptureResults) const;
    PoseCheckResult checkPose() const;
};

}  // namespace tracking
}  // namespace scan_tracking
