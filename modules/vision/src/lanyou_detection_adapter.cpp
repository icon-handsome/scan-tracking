#include "scan_tracking/vision/lanyou_detection_adapter.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <string>
#include "log_manager/LogMacros.h"
#include "detection/first/FirstOutSurfaceDetection.h"

namespace scan_tracking {
namespace vision {
namespace lanyou {

namespace {

bool isFinitePoint(const pcl::PointXYZ& point)
{
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

bool isValidDetectionResult(const FirstPoseDetectionParams& params)
{
    const auto finiteVec3 = [](const auto& vec) {
        return std::isfinite(vec.x()) && std::isfinite(vec.y()) && std::isfinite(vec.z());
    };
    const auto finiteVec4 = [](const auto& vec) {
        return std::isfinite(vec.x()) && std::isfinite(vec.y()) && std::isfinite(vec.z()) &&
               std::isfinite(vec.w());
    };

    return finiteVec4(params.bbox_min_pt) &&
           finiteVec4(params.bbox_max_pt) &&
           finiteVec3(params.cylinder_axis) &&
           params.cylinder_axis.norm() > 1e-6f &&
           finiteVec3(params.cylinder_center) &&
           std::isfinite(params.head_angle_tol) &&
           std::isfinite(params.straight_height_tol) &&
           std::isfinite(params.straight_slope_tol);
}

LanyouSmokeResult makeFailure(LanyouIssueCode code, int inputPointCount, const QString& message)
{
    LanyouSmokeResult result;
    result.success = false;
    result.inputPointCount = inputPointCount;
    result.issueCode = code;
    result.issueTag = issueTag(code);
    result.message = message;
    return result;
}

void logFailure(const LanyouSmokeResult& result)
{
    const auto tag = result.issueTag.isEmpty() ? issueTag(result.issueCode) : result.issueTag;
    const auto summary = issueSummary(result.issueCode);
    const auto text = QStringLiteral("%1 %2 %3")
                          .arg(tag)
                          .arg(summary)
                          .arg(result.message);
    const std::string utf8 = text.toLocal8Bit().toStdString();
    LOG_ERROR(utf8);
}

}  // namespace

pcl::PointCloud<pcl::PointXYZ>::Ptr toPclPointCloud(
    const scan_tracking::mech_eye::PointCloudFrame& frame)
{
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    if (!frame.pointsXYZ || frame.pointCount <= 0) {
        return cloud;
    }

    const auto& points = *frame.pointsXYZ;
    const int availablePointCount = static_cast<int>(points.size() / 3);
    const int pointCount = std::min(frame.pointCount, availablePointCount);
    if (pointCount <= 0) {
        return cloud;
    }

    cloud->points.reserve(static_cast<std::size_t>(pointCount));
    bool allFinite = true;
    for (int index = 0; index < pointCount; ++index) {
        const auto base = static_cast<std::size_t>(index * 3);
        const float x = points[base];
        const float y = points[base + 1];
        const float z = points[base + 2];
        allFinite = allFinite && std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
        cloud->points.emplace_back(x, y, z);
    }

    const int expectedGridCount = frame.width * frame.height;
    if (frame.width > 0 && frame.height > 0 && expectedGridCount == pointCount) {
        cloud->width = static_cast<std::uint32_t>(frame.width);
        cloud->height = static_cast<std::uint32_t>(frame.height);
    } else {
        cloud->width = static_cast<std::uint32_t>(pointCount);
        cloud->height = 1;
    }
    cloud->is_dense = allFinite;

    return cloud;
}

LanyouSmokeResult runFirstOutDetectionSmoke(
    const scan_tracking::mech_eye::PointCloudFrame& frame)
{
    auto cloud = toPclPointCloud(frame);
    const int inputPointCount = static_cast<int>(cloud->size());

    if (cloud->empty()) {
        const auto result = makeFailure(
            LanyouIssueCode::InputEmpty,
            inputPointCount,
            QStringLiteral("蓝友 FirstOut 检测器没有有效的输入点。"));
        logFailure(result);
        return result;
    }

    int invalidPointCount = 0;
    for (const auto& point : cloud->points) {
        if (!isFinitePoint(point)) {
            ++invalidPointCount;
        }
    }
    if (invalidPointCount > 0) {
        const auto result = makeFailure(
            LanyouIssueCode::PointCloudInvalid,
            inputPointCount,
            QStringLiteral("在输入点云中检测到 %1 个无效点。").arg(invalidPointCount));
        logFailure(result);
        return result;
    }

    LanyouSmokeResult result;
    result.inputPointCount = inputPointCount;
    result.invoked = true;
    try {
        FirstOutSurfaceDetection detector;
        result.success = detector.Detect(cloud);
        if (!result.success) {
            result.issueCode = LanyouIssueCode::AlgorithmFailure;
            result.issueTag = issueTag(result.issueCode);
            result.message = QStringLiteral("蓝友 FirstOut 检测器对烟雾输入返回失败。");
            logFailure(result);
            return result;
        }

        const auto& params = detector.GetParams();
        if (!isValidDetectionResult(params)) {
            result.success = false;
            result.issueCode = LanyouIssueCode::ResultInvalid;
            result.issueTag = issueTag(result.issueCode);
            result.message = QStringLiteral("蓝友 FirstOut 检测器返回成功，但输出参数无效。");
            logFailure(result);
            return result;
        }

        result.issueCode = LanyouIssueCode::Success;
        result.issueTag = issueTag(result.issueCode);
        result.message = QStringLiteral("蓝友 FirstOut 检测器返回成功。");
    } catch (const std::exception& ex) {
        result.success = false;
        result.issueCode = LanyouIssueCode::AlgorithmFailure;
        result.issueTag = issueTag(result.issueCode);
        result.message = QStringLiteral("蓝友 FirstOut 检测器抛出异常：%1")
                             .arg(QString::fromLocal8Bit(ex.what()));
        logFailure(result);
    } catch (...) {
        result.success = false;
        result.issueCode = LanyouIssueCode::AlgorithmFailure;
        result.issueTag = issueTag(result.issueCode);
        result.message = QStringLiteral("蓝友 FirstOut 检测器抛出未知异常。");
        logFailure(result);
    }

    return result;
}

}  // namespace lanyou
}  // namespace vision
}  // namespace scan_tracking
