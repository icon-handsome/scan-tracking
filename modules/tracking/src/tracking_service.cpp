/**
 * @file tracking_service.cpp
 * @brief 跟踪服务实现文件
 *
 * 实现跟踪服务类，提供分段点云的综合检测和位姿校验功能。
 * 整合蓝优第一工位检测算法，对外表面、内表面和内孔点云进行综合分析，
 * 并输出检测结果、偏移量和测量参数。
 */

#include "scan_tracking/tracking/tracking_service.h"

#include <cmath>
#include <vector>

#include "scan_tracking/common/application_info.h"
#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/tracking/lb_pose_check.h"
#include "scan_tracking/vision/lanyou_first_station_adapter.h"

namespace scan_tracking::tracking {

namespace {

/// 有效分段条目结构体
struct ValidSegmentEntry {
    int segmentIndex = -1;                                    ///< 分段索引
    const scan_tracking::mech_eye::CaptureResult* captureResult = nullptr; ///< 采集结果指针
};

/// 收集有效的分段采集结果
// @param segmentCaptureResults 分段采集结果映射表
// @param totalPointCount 总点数输出参数
// @return 有效分段列表
std::vector<ValidSegmentEntry> collectValidSegments(
    const QMap<int, scan_tracking::mech_eye::CaptureResult>& segmentCaptureResults,
    int* totalPointCount)
{
    std::vector<ValidSegmentEntry> validSegments;
    int points = 0;

    // 遍历所有分段，筛选出有效的采集结果
    for (auto it = segmentCaptureResults.cbegin(); it != segmentCaptureResults.cend(); ++it) {
        const auto& captureResult = it.value();
        // 检查采集是否成功且点云有效
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

/// 第一工位分段映射配置
struct FirstStationSegmentMapping {
    int outerSurfaceSegmentIndex = 1;   ///< 外表面分段索引
    int innerSurfaceSegmentIndex = 2;   ///< 内表面分段索引
    int innerHoleSegmentIndex = 3;      ///< 内孔分段索引
};

/// 从配置管理器加载第一工位分段映射
// @return 分段映射配置
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

/// 在有效分段列表中查找指定索引的分段
// @param validSegments 有效分段列表
// @param segmentIndex 要查找的分段索引
// @return 找到则返回采集结果指针，否则返回nullptr
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

/// 生成选中的分段索引文本
// @param mapping 分段映射配置
// @return 格式化后的分段索引字符串
QString selectedSegmentText(const FirstStationSegmentMapping& mapping)
{
    return QStringLiteral("[%1,%2,%3]")
        .arg(mapping.outerSurfaceSegmentIndex)
        .arg(mapping.innerSurfaceSegmentIndex)
        .arg(mapping.innerHoleSegmentIndex);
}

/// 统计检测结果中的有效测量项数量
// @param detectionResult 蓝优检测结果
// @return 测量项数量
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

/// 组合NG原因文本
// @param label 原因标签
// @param errorLog 错误日志
// @return 组合后的原因文本
QString composeNgReasonText(const QString& label, const QString& errorLog)
{
    if (errorLog.trimmed().isEmpty()) {
        return label;
    }
    return QStringLiteral("%1: %2").arg(label, errorLog.trimmed());
}

}  // namespace

/// 获取跟踪服务状态文本
// @return 应用名称和状态描述
std::string TrackingService::statusText() const
{
    return scan_tracking::common::ApplicationInfo::name() + " core is ready.";
}

/// 执行分段点云的综合检测（第一工位）
//
// 该函数接收多个分段的采集结果，根据配置的映射关系提取外表面、内表面和内孔点云，
// 调用蓝优第一工位检测算法进行分析，并返回检测结果。
//
// @param segmentCaptureResults 分段采集结果映射表
// @return 检测结果结构体，包含结果码、NG原因、偏移量和测量参数
InspectionResult TrackingService::inspectSegments(
    const QMap<int, scan_tracking::mech_eye::CaptureResult>& segmentCaptureResults) const
{
    InspectionResult result;
    result.segmentCount = segmentCaptureResults.size();
    result.measureItemCount = 0;

    // 收集有效的分段采集结果
    const auto validSegments = collectValidSegments(segmentCaptureResults, &result.totalPointCount);

    // 检查是否有可用点云
    if (result.totalPointCount <= 0) {
        result.resultCode = 2;
        result.ngReasonWord0 = (1u << 4);
        result.message = QStringLiteral("综合检测没有可用点云。");
        return result;
    }

    // 加载第一工位分段映射配置
    const auto segmentMapping = loadFirstStationSegmentMapping();

    // 查找所需的三个分段：外表面、内表面、内孔
    const auto* outerSurfaceResult = findValidSegment(validSegments, segmentMapping.outerSurfaceSegmentIndex);
    const auto* innerSurfaceResult = findValidSegment(validSegments, segmentMapping.innerSurfaceSegmentIndex);
    const auto* innerHoleResult = findValidSegment(validSegments, segmentMapping.innerHoleSegmentIndex);

    // 检查必需的三个分段是否都存在
    if (outerSurfaceResult == nullptr || innerSurfaceResult == nullptr || innerHoleResult == nullptr) {
        result.resultCode = 2;
        result.ngReasonWord0 = (1u << 4);
        result.message = QStringLiteral(
            "第一工位检测缺少必需分段，当前配置段位为 %1。")
                             .arg(selectedSegmentText(segmentMapping));
        return result;
    }

    // 构建帧集合，调用蓝优第一工位检测算法
    scan_tracking::vision::lanyou::FirstStationFrameSet frameSet;
    frameSet.outerSurfaceFrame = outerSurfaceResult->pointCloud;
    frameSet.innerSurfaceFrame = innerSurfaceResult->pointCloud;
    frameSet.innerHoleFrame = innerHoleResult->pointCloud;

    const auto detection = scan_tracking::vision::lanyou::runFirstStationDetection(frameSet);

    // 检查算法是否真正启动
    if (!detection.invoked) {
        result.resultCode = 2;
        result.ngReasonWord0 = (1u << 4);
        result.outlinerErrorLog = detection.outlinerErrorLog;
        result.inlinerErrorLog = detection.inlinerErrorLog;
        result.message = QStringLiteral(
            "第一工位算法适配层未真正启动，段位为 %1,详情：%2")
                             .arg(selectedSegmentText(segmentMapping), detection.message);
        return result;
    }

    // 检查外表面检测结果
    if (!detection.firstOutSuccess) {
        result.resultCode = 2;
        result.ngReasonWord0 = (1u << 5);
        result.outlinerErrorLog = detection.outlinerErrorLog;
        result.message = QStringLiteral(
            "%1，段位为 %2。")
                             .arg(
                                 composeNgReasonText(
                                     QStringLiteral("第一工位外表面算法失败"),
                                     detection.outlinerErrorLog.isEmpty() ? detection.message : detection.outlinerErrorLog),
                                 selectedSegmentText(segmentMapping));
        return result;
    }

    // 检查内表面检测结果
    if (!detection.firstInlinerSuccess) {
        result.resultCode = 2;
        result.ngReasonWord0 = (1u << 6);
        result.inlinerErrorLog = detection.inlinerErrorLog;
        result.message = QStringLiteral(
            "%1，段位为 %2。")
                             .arg(
                                 composeNgReasonText(
                                     QStringLiteral("第一工位内表面算法失败"),
                                     detection.inlinerErrorLog.isEmpty() ? detection.message : detection.inlinerErrorLog),
                                 selectedSegmentText(segmentMapping));
        return result;
    }

    // 检测成功，填充结果
    result.resultCode = 1;
    result.measureItemCount = countMeasuredItems(detection);
    result.offsetXmm = detection.params.cylinder_center.x();
    result.offsetYmm = detection.params.cylinder_center.y();
    result.offsetZmm = detection.params.cylinder_center.z();
    result.stableOffsetXmm = detection.stableOffsetXmm;
    result.stableOffsetYmm = detection.stableOffsetYmm;
    result.stableOffsetZmm = detection.stableOffsetZmm;
    result.outlinerErrorLog = detection.outlinerErrorLog;
    result.inlinerErrorLog = detection.inlinerErrorLog;
    result.message = QStringLiteral(
        "第一工位检测通过，段位为 %1,稳定偏移=(%2,%3,%4),坡口角=%5,直边高度=%6,直边斜度=%7,内径=%8。")
                         .arg(selectedSegmentText(segmentMapping))
                         .arg(result.stableOffsetXmm, 0, 'f', 3)
                         .arg(result.stableOffsetYmm, 0, 'f', 3)
                         .arg(result.stableOffsetZmm, 0, 'f', 3)
                         .arg(detection.params.head_angle_tol, 0, 'f', 3)
                         .arg(detection.params.straight_height_tol, 0, 'f', 3)
                         .arg(detection.params.straight_slope_tol, 0, 'f', 3)
                         .arg(detection.params.inner_diameter, 0, 'f', 3);
    return result;
}

/// 执行位姿校验（LB位姿检测）
//
// 使用LB位姿检测算法对左右相机图像进行三维重建和模板匹配，
// 计算目标物体的位姿并输出4x4变换矩阵。
//
// @return 位姿校验结果结构体
PoseCheckResult TrackingService::checkPose() const
{
    PoseCheckResult result;
    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    if (configManager == nullptr) {
        result.resultCode = 7;
        result.message = QStringLiteral("ConfigManager unavailable for LB pose check.");
        return result;
    }

    // 调用LB位姿检测算法
    const auto lbResult = runLegacyLbPoseCheck(configManager->lbPoseConfig());
    result.invoked = lbResult.invoked;
    result.success = lbResult.success;
    result.resultCode = lbResult.resultCode;
    result.inputPointCount = lbResult.inputPointCount;
    result.poseDeviationMm = lbResult.poseDeviationMm;
    result.rt = lbResult.rt;
    result.message = lbResult.message;
    if (result.success && result.hasPoseMatrix()) {
        result.message += QStringLiteral(" Pose matrix is ready for downstream use.");
    }
    return result;
}

}  // namespace scan_tracking::tracking