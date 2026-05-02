#include "scan_tracking/tracking/tracking_service.h"

#include <cmath>
#include <vector>

#include "scan_tracking/common/application_info.h"
#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/tracking/lb_pose_check.h"
#include "scan_tracking/vision/lanyou_first_station_adapter.h"

namespace scan_tracking::tracking {

namespace {

struct ValidSegmentEntry {
    int segmentIndex = -1;
    const scan_tracking::mech_eye::CaptureResult* captureResult = nullptr;
};

std::vector<ValidSegmentEntry> collectValidSegments(
    const QMap<int, scan_tracking::mech_eye::CaptureResult>& segmentCaptureResults,
    int* totalPointCount)
{
    std::vector<ValidSegmentEntry> validSegments;
    int points = 0;

    for (auto it = segmentCaptureResults.cbegin(); it != segmentCaptureResults.cend(); ++it) {
        const auto& captureResult = it.value();
        if (!captureResult.success() || !captureResult.pointCloud.isValid()) {
            continue;
        }

        points += captureResult.pointCloud.pointCount;
        validSegments.push_back(ValidSegmentEntry{it.key(), &captureResult});
    }

    if (totalPointCount != nullptr) {
        *totalPointCount = points;
    }

    return validSegments;
}

struct FirstStationSegmentMapping {
    int outerSurfaceSegmentIndex = 1;
    int innerSurfaceSegmentIndex = 2;
    int innerHoleSegmentIndex = 3;
};

FirstStationSegmentMapping loadFirstStationSegmentMapping()
{
    FirstStationSegmentMapping mapping;
    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    if (configManager == nullptr) {
        return mapping;
    }

    const auto& config = configManager->trackingConfig();
    mapping.outerSurfaceSegmentIndex = config.firstStationOuterSegmentIndex;
    mapping.innerSurfaceSegmentIndex = config.firstStationInnerSegmentIndex;
    mapping.innerHoleSegmentIndex = config.firstStationHoleSegmentIndex;
    return mapping;
}

const scan_tracking::mech_eye::CaptureResult* findValidSegment(
    const std::vector<ValidSegmentEntry>& validSegments,
    int segmentIndex)
{
    for (const auto& segment : validSegments) {
        if (segment.segmentIndex == segmentIndex) {
            return segment.captureResult;
        }
    }
    return nullptr;
}

QString selectedSegmentText(const FirstStationSegmentMapping& mapping)
{
    return QStringLiteral("[%1,%2,%3]")
        .arg(mapping.outerSurfaceSegmentIndex)
        .arg(mapping.innerSurfaceSegmentIndex)
        .arg(mapping.innerHoleSegmentIndex);
}

quint16 countMeasuredItems(
    
    const scan_tracking::vision::lanyou::FirstStationDetectionResult& detectionResult)
{
    quint16 count = 0;
    if (std::isfinite(detectionResult.params.head_angle_tol)) {
        ++count;
    }
    if (std::isfinite(detectionResult.params.straight_height_tol)) {
        ++count;
    }
    if (std::isfinite(detectionResult.params.straight_slope_tol)) {
        ++count;
    }
    if (std::isfinite(detectionResult.params.inner_diameter)) {
        ++count;
    }
    return count;
}

}  // namespace

std::string TrackingService::statusText() const
{
    return scan_tracking::common::ApplicationInfo::name() + " core is ready.";
}

InspectionResult TrackingService::inspectSegments(
    const QMap<int, scan_tracking::mech_eye::CaptureResult>& segmentCaptureResults) const
{
    InspectionResult result;
    result.segmentCount = segmentCaptureResults.size();
    result.measureItemCount = 0;

    const auto validSegments = collectValidSegments(segmentCaptureResults, &result.totalPointCount);
    if (result.totalPointCount <= 0) {
        result.resultCode = 2;
        result.ngReasonWord0 = (1u << 4);
        result.message = QStringLiteral("综合检测没有可用点云。");
        return result;
    }

    const auto segmentMapping = loadFirstStationSegmentMapping();
    const auto* outerSurfaceResult = findValidSegment(validSegments, segmentMapping.outerSurfaceSegmentIndex);
    const auto* innerSurfaceResult = findValidSegment(validSegments, segmentMapping.innerSurfaceSegmentIndex);
    const auto* innerHoleResult = findValidSegment(validSegments, segmentMapping.innerHoleSegmentIndex);

    if (outerSurfaceResult == nullptr || innerSurfaceResult == nullptr || innerHoleResult == nullptr) {
        result.resultCode = 2;
        result.ngReasonWord0 = (1u << 4);
        result.message = QStringLiteral(
            "第一工位检测缺少必需分段，当前配置段位为 %1。")
                             .arg(selectedSegmentText(segmentMapping));
        return result;
    }

    scan_tracking::vision::lanyou::FirstStationFrameSet frameSet;
    frameSet.outerSurfaceFrame = outerSurfaceResult->pointCloud;
    frameSet.innerSurfaceFrame = innerSurfaceResult->pointCloud;
    frameSet.innerHoleFrame = innerHoleResult->pointCloud;

    const auto detection = scan_tracking::vision::lanyou::runFirstStationDetection(frameSet);

    if (!detection.invoked) {
        result.resultCode = 2;
        result.ngReasonWord0 = (1u << 4);
        result.message = QStringLiteral(
            "第一工位算法适配层未真正启动，段位为 %1,详情：%2")
                             .arg(selectedSegmentText(segmentMapping), detection.message);
        return result;
    }

    if (!detection.firstOutSuccess) {
        result.resultCode = 2;
        result.ngReasonWord0 = (1u << 5);
        result.message = QStringLiteral(
            "第一工位外表面算法失败，段位为 %1,详情：%2")
                             .arg(selectedSegmentText(segmentMapping), detection.message);
        return result;
    }

    if (!detection.firstInlinerSuccess) {
        result.resultCode = 2;
        result.ngReasonWord0 = (1u << 6);
        result.message = QStringLiteral(
            "第一工位内表面算法失败，段位为 %1,详情：%2")
                             .arg(selectedSegmentText(segmentMapping), detection.message);
        return result;
    }

    result.resultCode = 1;
    result.measureItemCount = countMeasuredItems(detection);
    result.offsetXmm = detection.params.cylinder_center.x();
    result.offsetYmm = detection.params.cylinder_center.y();
    result.offsetZmm = detection.params.cylinder_center.z();
    result.message = QStringLiteral(
        "第一工位检测通过，段位为 %1,坡口角=%2,直边高度=%3,直边斜度=%4,内径=%5。")
                         .arg(selectedSegmentText(segmentMapping))
                         .arg(detection.params.head_angle_tol, 0, 'f', 3)
                         .arg(detection.params.straight_height_tol, 0, 'f', 3)
                         .arg(detection.params.straight_slope_tol, 0, 'f', 3)
                         .arg(detection.params.inner_diameter, 0, 'f', 3);
    return result;
}

PoseCheckResult TrackingService::checkPose() const
{
    PoseCheckResult result;
    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    if (configManager == nullptr) {
        result.resultCode = 7;
        result.message = QStringLiteral("ConfigManager unavailable for LB pose check.");
        return result;
    }

    const auto lbResult = runLegacyLbPoseCheck(configManager->lbPoseConfig());
    result.invoked = lbResult.invoked;
    result.success = lbResult.success;
    result.resultCode = lbResult.resultCode;
    result.inputPointCount = lbResult.inputPointCount;
    result.poseDeviationMm = lbResult.poseDeviationMm;
    result.rt = lbResult.rt;
    result.message = lbResult.message;
    return result;
}

}  // namespace scan_tracking::tracking
