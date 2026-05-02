#include "scan_tracking/vision/lanyou_first_station_adapter.h"

#include <exception>

#include "detection/first/FirstInlinerSurfaceDetection.h"
#include "detection/first/FirstOutSurfaceDetection.h"
#include "scan_tracking/vision/lanyou_detection_adapter.h"

namespace scan_tracking::vision::lanyou {

FirstStationDetectionResult runFirstStationDetection(
    const FirstStationFrameSet& frames)
{
    FirstStationDetectionResult result;

    auto outerCloud = toPclPointCloud(frames.outerSurfaceFrame);
    auto innerCloud = toPclPointCloud(frames.innerSurfaceFrame);
    auto holeCloud = toPclPointCloud(frames.innerHoleFrame);

    result.outerPointCount = static_cast<int>(outerCloud->size());
    result.innerPointCount = static_cast<int>(innerCloud->size());
    result.holePointCount = static_cast<int>(holeCloud->size());

    if (outerCloud->empty()) {
        result.message = QStringLiteral("Missing outer surface cloud for FirstOut detection.");
        return result;
    }

    if (innerCloud->empty()) {
        result.message = QStringLiteral("Missing inner surface cloud for FirstInliner detection.");
        return result;
    }

    if (holeCloud->empty()) {
        result.message = QStringLiteral("Missing inner hole cloud for FirstInliner detection.");
        return result;
    }

    result.invoked = true;

    try {
        ResetGlobalFirstPoseParams();

        FirstOutSurfaceDetection firstOutDetector;
        result.firstOutInvoked = true;
        result.firstOutSuccess = firstOutDetector.Detect(outerCloud);
        result.params = firstOutDetector.GetParams();

        if (!result.firstOutSuccess) {
            result.message = QStringLiteral(
                "FirstOutSurfaceDetection returned false. TODO(tracking-first-station): map outliner_error_log to business NG reason words.");
            return result;
        }

        FirstInlinerSurfaceDetection firstInlinerDetector;
        result.firstInlinerInvoked = true;
        result.firstInlinerSuccess = firstInlinerDetector.Detect(innerCloud, holeCloud);
        result.params = firstInlinerDetector.GetParams();

        if (!result.firstInlinerSuccess) {
            result.message = QStringLiteral(
                "FirstInlinerSurfaceDetection returned false. TODO(tracking-first-station): map inliner_error_log to business NG reason words.");
            return result;
        }

        result.message = QStringLiteral(
            "First station detection pipeline completed. TODO(tracking-first-station): map params to InspectionResult and stable offsets.");
        return result;
    } catch (const std::exception& ex) {
        result.message = QStringLiteral(
            "First station detection threw exception: %1. TODO(tracking-first-station): surface structured diagnostics to tracking.")
                             .arg(QString::fromLocal8Bit(ex.what()));
        return result;
    } catch (...) {
        result.message = QStringLiteral(
            "First station detection threw unknown exception. TODO(tracking-first-station): add structured error classification.");
        return result;
    }
}

}  // namespace scan_tracking::vision::lanyou
