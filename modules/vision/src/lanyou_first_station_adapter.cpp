#include "scan_tracking/vision/lanyou_first_station_adapter.h"

#include <cmath>
#include <exception>

#include "detection/first/FirstInlinerSurfaceDetection.h"
#include "detection/first/FirstOutSurfaceDetection.h"
#include "scan_tracking/vision/lanyou_detection_adapter.h"

namespace scan_tracking::vision::lanyou {

namespace {

QString toQtString(const std::string& text)
{
    return text.empty() ? QString() : QString::fromLocal8Bit(text.c_str());
}

enum class FirstStationNgReason : quint16 {
    None = 0,
    MissingInput = 1u << 4,
    OutlinerFailed = 1u << 5,
    InlinerFailed = 1u << 6,
    UnknownFailure = 1u << 7,
};

quint16 mapOutlinerErrorToNgWord(const QString& errorLog)
{
    const QString lowered = errorLog.trimmed().toLower();
    if (lowered.isEmpty()) {
        return static_cast<quint16>(FirstStationNgReason::UnknownFailure);
    }
    if (lowered.contains(QStringLiteral("empty")) ||
        lowered.contains(QStringLiteral("missing")) ||
        lowered.contains(QStringLiteral("invalid axis")) ||
        lowered.contains(QStringLiteral("too few points"))) {
        return static_cast<quint16>(FirstStationNgReason::MissingInput);
    }
    return static_cast<quint16>(FirstStationNgReason::OutlinerFailed);
}

quint16 mapInlinerErrorToNgWord(const QString& errorLog)
{
    const QString lowered = errorLog.trimmed().toLower();
    if (lowered.isEmpty()) {
        return static_cast<quint16>(FirstStationNgReason::UnknownFailure);
    }
    if (lowered.contains(QStringLiteral("empty")) ||
        lowered.contains(QStringLiteral("missing")) ||
        lowered.contains(QStringLiteral("invalid axis")) ||
        lowered.contains(QStringLiteral("too few points")) ||
        lowered.contains(QStringLiteral("invalid axis projection range")) ||
        lowered.contains(QStringLiteral("hole cylinder fit failed"))) {
        return static_cast<quint16>(FirstStationNgReason::MissingInput);
    }
    return static_cast<quint16>(FirstStationNgReason::InlinerFailed);
}

bool finiteVec3(const Eigen::Vector3f& vec)
{
    return std::isfinite(vec.x()) && std::isfinite(vec.y()) && std::isfinite(vec.z());
}

}  // namespace

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
        result.message = QStringLiteral("FirstOut 检测缺少外表面点云。");
        return result;
    }

    if (innerCloud->empty()) {
        result.message = QStringLiteral("FirstInliner 检测缺少内表面点云。");
        return result;
    }

    if (holeCloud->empty()) {
        result.message = QStringLiteral("FirstInliner 检测缺少内孔点云。");
        return result;
    }

    result.invoked = true;

    try {
        ResetGlobalFirstPoseParams();

        FirstOutSurfaceDetection firstOutDetector;
        result.firstOutInvoked = true;
        result.firstOutSuccess = firstOutDetector.Detect(outerCloud);
        result.params = firstOutDetector.GetParams();
        result.outlinerErrorLog = toQtString(result.params.outliner_error_log);

        if (!result.firstOutSuccess) {
            const quint16 ngWord0 = mapOutlinerErrorToNgWord(result.outlinerErrorLog);
            result.message = QStringLiteral(
                "FirstOutSurfaceDetection returned false. ngWord0=%1, log=%2")
                                 .arg(ngWord0)
                                 .arg(result.outlinerErrorLog);
            return result;
        }

        FirstInlinerSurfaceDetection firstInlinerDetector;
        result.firstInlinerInvoked = true;
        result.firstInlinerSuccess = firstInlinerDetector.Detect(innerCloud, holeCloud);
        result.params = firstInlinerDetector.GetParams();
        result.inlinerErrorLog = toQtString(result.params.inliner_error_log);

        if (!result.firstInlinerSuccess) {
            const quint16 ngWord0 = mapInlinerErrorToNgWord(result.inlinerErrorLog);
            result.message = QStringLiteral(
                "FirstInlinerSurfaceDetection returned false. ngWord0=%1, log=%2")
                                 .arg(ngWord0)
                                 .arg(result.inlinerErrorLog);
            return result;
        }

        result.stableOffsetXmm = result.params.cylinder_center.x();
        result.stableOffsetYmm = result.params.cylinder_center.y();
        result.stableOffsetZmm = result.params.cylinder_center.z();
        result.message = QStringLiteral(
            "First station detection pipeline completed. stableOffset=(%1,%2,%3)")
                             .arg(result.stableOffsetXmm, 0, 'f', 3)
                             .arg(result.stableOffsetYmm, 0, 'f', 3)
                             .arg(result.stableOffsetZmm, 0, 'f', 3);
        return result;
    } catch (const std::exception& ex) {
        result.message = QStringLiteral(
            "First station detection threw exception: %1")
                             .arg(QString::fromLocal8Bit(ex.what()));
        return result;
    } catch (...) {
        result.message = QStringLiteral(
            "First station detection threw unknown exception.");
        return result;
    }
}

}  // namespace scan_tracking::vision::lanyou
