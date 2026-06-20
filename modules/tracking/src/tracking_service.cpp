#include "scan_tracking/tracking/tracking_service.h"

#include <cmath>

#include <QtCore/QJsonObject>
#include <QtCore/QLoggingCategory>
#include <QtCore/QMetaType>

#include "scan_tracking/common/application_info.h"
#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/tracking/lb_pose_check.h"
#include "scan_tracking/vision/bevel_measurement_adapter.h"
#ifdef SCAN_TRACKING_HAS_INTERNAL_SURFACE_MEASUREMENT
#include "scan_tracking/vision/internal_surface_measurement_adapter.h"
#endif
#ifdef SCAN_TRACKING_HAS_HOLE_MEASUREMENT
#include "scan_tracking/vision/hole_measurement_adapter.h"
#endif
#ifdef SCAN_TRACKING_HAS_THICKNESS_MEASUREMENT
#include "scan_tracking/vision/thickness_measurement_adapter.h"
#endif

namespace scan_tracking::tracking {

Q_LOGGING_CATEGORY(LOG_TRACKING, "tracking")

namespace {

void ensureInspectionMeasurementMetaTypeRegistered()
{
    static const bool registered = []() {
        qRegisterMetaType<InspectionMeasurement>(
            "scan_tracking::tracking::InspectionMeasurement");
        return true;
    }();
    Q_UNUSED(registered);
}

double measurementJsonValue(float value)
{
    return std::isfinite(value) ? static_cast<double>(value) : 0.0;
}

quint16 countMeasuredItems(const InspectionMeasurement& measurement)
{
    quint16 count = 0;
    if (measurement.algorithm == InspectionAlgorithm::Thickness) {
        if (std::isfinite(measurement.thicknessMm) && measurement.thicknessMm > 0.0f) {
            ++count;
        }
        return count;
    }

    if (measurement.algorithm == InspectionAlgorithm::InternalSurface) {
        if (std::isfinite(measurement.headDepthMm) && measurement.headDepthMm > 0.0f) {
            ++count;
        }
        if (std::isfinite(measurement.headVolumeM3) && measurement.headVolumeM3 > 0.0f) {
            ++count;
        }
        return count;
    }

    if (measurement.algorithm == InspectionAlgorithm::CodeRead
        || measurement.algorithm == InspectionAlgorithm::Defect) {
        return 1;
    }

    if (measurement.algorithm == InspectionAlgorithm::Hole) {
        if (std::isfinite(measurement.innerDiameterMm) && measurement.innerDiameterMm > 0.0f) {
            ++count;
        }
        if (std::isfinite(measurement.roundnessToleranceMm)) {
            ++count;
        }
        if (std::isfinite(measurement.straightSideHeightMm)) {
            ++count;
        }
        if (std::isfinite(measurement.holeOpeningMm)) {
            ++count;
        }
        if (std::isfinite(measurement.headDepthMm) && measurement.headDepthMm > 0.0f) {
            ++count;
        }
        if (std::isfinite(measurement.headVolumeM3) && measurement.headVolumeM3 > 0.0f) {
            ++count;
        }
        return count;
    }

    if (std::isfinite(measurement.headAngleTol)) {
        ++count;
    }
    if (std::isfinite(measurement.bluntHeightTol)) {
        ++count;
    }
    if (std::isfinite(measurement.headDepthMm) && measurement.headDepthMm > 0.0f) {
        ++count;
    }
    if (std::isfinite(measurement.headVolumeM3) && measurement.headVolumeM3 > 0.0f) {
        ++count;
    }
    return count;
}

InspectionMeasurement measurementFromBevelResult(
    const scan_tracking::vision::bevel::BevelInspectionResult& detection)
{
    InspectionMeasurement measurement;
    measurement.algorithm = InspectionAlgorithm::Bevel;
    measurement.headAngleTol = detection.angleDeg;
    measurement.bluntHeightTol = detection.lengthMm;
    measurement.bevelType = detection.bevelType;
    measurement.icpFitness = detection.icpFitness;
    measurement.qualityCode = detection.qualityCode;
    return measurement;
}

#ifdef SCAN_TRACKING_HAS_HOLE_MEASUREMENT
InspectionMeasurement measurementFromHoleResult(
    const scan_tracking::vision::hole::HoleInspectionResult& detection)
{
    InspectionMeasurement measurement;
    measurement.algorithm = InspectionAlgorithm::Hole;
    measurement.innerDiameterMm = static_cast<float>(detection.measureResult.innerDiameterMm);
    measurement.innerCircumferenceMm =
        static_cast<float>(detection.measureResult.innerCircumferenceMm);
    measurement.roundnessToleranceMm =
        static_cast<float>(detection.measureResult.roundnessToleranceMm);
    measurement.straightSideSlopeDeg =
        static_cast<float>(detection.measureResult.straightSideSlopeDeg);
    measurement.straightSideHeightMm =
        static_cast<float>(detection.measureResult.straightSideHeightMm);
    measurement.holeOpeningMm =
        static_cast<float>(detection.measureResult.opening.centerToInnerWallDistanceMm);
    measurement.jointFitUpAngleDeg =
        static_cast<float>(detection.measureResult.opening.axisToHeadAxisAngleDeg);
    measurement.icpFitness = static_cast<float>(detection.icpRmsMm);
    measurement.qualityCode = detection.ok ? 0 : 1;
    return measurement;
}
#endif

#ifdef SCAN_TRACKING_HAS_THICKNESS_MEASUREMENT
InspectionMeasurement measurementFromThicknessResult(
    const scan_tracking::vision::thickness::ThicknessInspectionResult& detection)
{
    InspectionMeasurement measurement;
    measurement.algorithm = InspectionAlgorithm::Thickness;
    measurement.thicknessMm = static_cast<float>(detection.thicknessMm);
    measurement.icpFitness = static_cast<float>(detection.icpFitnessScore);
    measurement.qualityCode = detection.ok ? 0 : 1;
    return measurement;
}
#endif

#ifdef SCAN_TRACKING_HAS_INTERNAL_SURFACE_MEASUREMENT
InspectionMeasurement measurementFromInternalSurfaceResult(
    const scan_tracking::vision::internal_surface::InternalSurfaceInspectionResult& detection)
{
    InspectionMeasurement measurement;
    measurement.algorithm = InspectionAlgorithm::InternalSurface;
    measurement.headDepthMm = static_cast<float>(detection.headDepthMm);
    measurement.headVolumeM3 = static_cast<float>(detection.headVolumeM3);
    measurement.qualityCode = detection.ok ? 0 : 1;
    return measurement;
}
#endif

scan_tracking::common::InspectionType resolveInspectionType(
    const scan_tracking::common::ConfigManager* configManager,
    int inspectionPathId)
{
    if (configManager == nullptr) {
        return scan_tracking::common::InspectionType::Bevel;
    }
    if (inspectionPathId > 0) {
        return configManager->inspectionTypeForPath(inspectionPathId);
    }
    return scan_tracking::common::InspectionType::Bevel;
}

}  // namespace

void appendInspectionMeasurementFields(QJsonObject& payload, const InspectionMeasurement& measurement)
{
    QString algorithmName = QStringLiteral("bevel");
    if (measurement.algorithm == InspectionAlgorithm::Hole) {
        algorithmName = QStringLiteral("hole");
    } else if (measurement.algorithm == InspectionAlgorithm::Thickness) {
        algorithmName = QStringLiteral("thickness");
    } else if (measurement.algorithm == InspectionAlgorithm::InternalSurface) {
        algorithmName = QStringLiteral("internal_surface");
    } else if (measurement.algorithm == InspectionAlgorithm::CodeRead) {
        algorithmName = QStringLiteral("code_read");
    } else if (measurement.algorithm == InspectionAlgorithm::Defect) {
        algorithmName = QStringLiteral("defect");
    }
    payload[QStringLiteral("inspection_algorithm")] = algorithmName;
    payload[QStringLiteral("head_angle_tol")] = measurementJsonValue(measurement.headAngleTol);
    payload[QStringLiteral("blunt_height_tol")] = measurementJsonValue(measurement.bluntHeightTol);
    payload[QStringLiteral("bevel_type")] = measurement.bevelType;
    payload[QStringLiteral("icp_fitness")] = measurementJsonValue(measurement.icpFitness);
    payload[QStringLiteral("quality_code")] = measurement.qualityCode;
    payload[QStringLiteral("inner_diameter_mm")] = measurementJsonValue(measurement.innerDiameterMm);
    payload[QStringLiteral("inner_circumference_mm")] =
        measurementJsonValue(measurement.innerCircumferenceMm);
    payload[QStringLiteral("roundness_tolerance_mm")] =
        measurementJsonValue(measurement.roundnessToleranceMm);
    payload[QStringLiteral("straight_side_slope_deg")] =
        measurementJsonValue(measurement.straightSideSlopeDeg);
    payload[QStringLiteral("straight_side_height_mm")] =
        measurementJsonValue(measurement.straightSideHeightMm);
    payload[QStringLiteral("hole_opening_mm")] = measurementJsonValue(measurement.holeOpeningMm);
    payload[QStringLiteral("joint_fit_up_angle_deg")] =
        measurementJsonValue(measurement.jointFitUpAngleDeg);
    payload[QStringLiteral("thickness_mm")] = measurementJsonValue(measurement.thicknessMm);
    payload[QStringLiteral("head_depth_mm")] = measurementJsonValue(measurement.headDepthMm);
    payload[QStringLiteral("head_volume_m3")] = measurementJsonValue(measurement.headVolumeM3);
}

void appendHeadDisplayMetricsFields(QJsonObject& payload, const InspectionMeasurement& measurement)
{
    QJsonObject headMetrics;
    headMetrics[QStringLiteral("inner_diameter_mm")] =
        measurementJsonValue(measurement.innerDiameterMm);
    headMetrics[QStringLiteral("roundness_tol")] =
        measurementJsonValue(measurement.roundnessToleranceMm);
    headMetrics[QStringLiteral("straight_slope_tol")] =
        measurementJsonValue(measurement.straightSideSlopeDeg);
    headMetrics[QStringLiteral("head_depth_mm")] = measurementJsonValue(measurement.headDepthMm);
    headMetrics[QStringLiteral("straight_height_tol")] =
        measurementJsonValue(measurement.straightSideHeightMm);
    headMetrics[QStringLiteral("bevel_angle_deg")] = measurementJsonValue(measurement.headAngleTol);
    headMetrics[QStringLiteral("blunt_height_mm")] = measurementJsonValue(measurement.bluntHeightTol);
    headMetrics[QStringLiteral("inner_circumference_mm")] =
        measurementJsonValue(measurement.innerCircumferenceMm);
    headMetrics[QStringLiteral("hole_opening_mm")] = measurementJsonValue(measurement.holeOpeningMm);
    headMetrics[QStringLiteral("joint_fit_up_angle_deg")] =
        measurementJsonValue(measurement.jointFitUpAngleDeg);
    headMetrics[QStringLiteral("thickness_mm")] = measurementJsonValue(measurement.thicknessMm);
    headMetrics[QStringLiteral("head_volume_m3")] = measurementJsonValue(measurement.headVolumeM3);
    payload[QStringLiteral("headMetrics")] = headMetrics;
}

std::string TrackingService::statusText() const
{
    return scan_tracking::common::ApplicationInfo::name() + " core is ready.";
}

void TrackingService::setInspectionResultNotifier(InspectionResultNotifier notifier)
{
    m_inspectionResultNotifier = std::move(notifier);
}

InspectionResult TrackingService::deliverInspectionResult(
    InspectionResult result, bool notifyListener) const
{
    if (notifyListener && m_inspectionResultNotifier) {
        m_inspectionResultNotifier(result);
    }
    return result;
}

InspectionResult TrackingService::inspectPointCloud(
    const scan_tracking::mech_eye::PointCloudFrame& pointCloud,
    int sourcePointCount,
    int inspectionPathId,
    bool notifyListener) const
{
    ensureInspectionMeasurementMetaTypeRegistered();

    InspectionResult result;
    result.sourcePointCount = sourcePointCount > 0 ? sourcePointCount : pointCloud.pointCount;

    if (!pointCloud.isValid() || result.sourcePointCount <= 0) {
        result.resultCode = 2;
        result.ngReasonWord0 = (1u << 4);
        result.message = QStringLiteral("综合检测没有可用点云。");
        return deliverInspectionResult(result, notifyListener);
    }

    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    const scan_tracking::common::InspectionType inspectionType =
        resolveInspectionType(configManager, inspectionPathId);

#ifdef SCAN_TRACKING_HAS_INTERNAL_SURFACE_MEASUREMENT
    if (inspectionType == scan_tracking::common::InspectionType::InternalSurface) {
        const auto detection =
            scan_tracking::vision::internal_surface::runInternalSurfaceMeasurement(pointCloud);
        if (!detection.invoked) {
            result.resultCode = 2;
            result.ngReasonWord0 = (1u << 4);
            result.message = detection.message.isEmpty()
                ? QStringLiteral("内表面测量适配层未启动。")
                : detection.message;
            return deliverInspectionResult(result, notifyListener);
        }

        result.measurement = measurementFromInternalSurfaceResult(detection);
        if (!detection.ok) {
            result.resultCode = 2;
            result.ngReasonWord0 = (1u << 5);
            result.message = detection.message.isEmpty()
                ? QStringLiteral("内表面测量算法失败。")
                : detection.message;
            return deliverInspectionResult(result, notifyListener);
        }

        result.resultCode = 1;
        result.message = detection.message;
        result.measureItemCount = countMeasuredItems(result.measurement);
        return deliverInspectionResult(result, notifyListener);
    }
#else
    if (inspectionType == scan_tracking::common::InspectionType::InternalSurface) {
        result.resultCode = 2;
        result.ngReasonWord0 = (1u << 4);
        result.message = QStringLiteral(
            "内表面测量未编译（SCAN_TRACKING_ENABLE_INTERNAL_SURFACE_MEASUREMENT=OFF）。");
        return deliverInspectionResult(result, notifyListener);
    }
#endif

    if (inspectionType == scan_tracking::common::InspectionType::CodeRead) {
        result.resultCode = 2;
        result.ngReasonWord0 = (1u << 4);
        result.message = QStringLiteral(
            "编号识别应通过 inspectCodeRead 调用，不应走 inspectPointCloud。");
        return deliverInspectionResult(result, notifyListener);
    }

    if (inspectionType == scan_tracking::common::InspectionType::Defect) {
        result.resultCode = 2;
        result.ngReasonWord0 = (1u << 4);
        result.message = QStringLiteral(
            "缺陷识别应通过 inspectSurfaceDefect 调用，不应走 inspectPointCloud。");
        return deliverInspectionResult(result, notifyListener);
    }

#ifdef SCAN_TRACKING_HAS_THICKNESS_MEASUREMENT
    if (inspectionType == scan_tracking::common::InspectionType::Thickness) {
        result.resultCode = 2;
        result.ngReasonWord0 = (1u << 4);
        result.message = QStringLiteral(
            "厚度测量需要 inner/outer 双点云，请使用 inspectThicknessPointClouds。");
        return deliverInspectionResult(result, notifyListener);
    }
#else
    if (inspectionType == scan_tracking::common::InspectionType::Thickness) {
        result.resultCode = 2;
        result.ngReasonWord0 = (1u << 4);
        result.message = QStringLiteral(
            "厚度测量未编译（SCAN_TRACKING_ENABLE_THICKNESS_MEASUREMENT=OFF）。");
        return deliverInspectionResult(result, notifyListener);
    }
#endif

#ifdef SCAN_TRACKING_HAS_HOLE_MEASUREMENT
    if (inspectionType == scan_tracking::common::InspectionType::Hole) {
        const auto detection =
            scan_tracking::vision::hole::runHoleMeasurement(pointCloud, inspectionPathId);

        if (!detection.invoked) {
            result.resultCode = 2;
            result.ngReasonWord0 = (1u << 4);
            result.message = detection.message.isEmpty()
                ? QStringLiteral("Hole 测量适配层未启动。")
                : detection.message;
            return deliverInspectionResult(result, notifyListener);
        }

        result.measurement = measurementFromHoleResult(detection);

        if (!detection.ok) {
            result.resultCode = 2;
            result.ngReasonWord0 = (1u << 5);
            result.message = detection.message.isEmpty()
                ? QStringLiteral("Hole 测量算法失败。")
                : detection.message;
            return deliverInspectionResult(result, notifyListener);
        }

        result.resultCode = 1;
        result.message = detection.message;
        result.measureItemCount = countMeasuredItems(result.measurement);
        return deliverInspectionResult(result, notifyListener);
    }
#else
    if (inspectionType == scan_tracking::common::InspectionType::Hole) {
        result.resultCode = 2;
        result.ngReasonWord0 = (1u << 4);
        result.message = QStringLiteral("Hole 测量未编译（SCAN_TRACKING_ENABLE_HOLE_MEASUREMENT=OFF）。");
        return deliverInspectionResult(result, notifyListener);
    }
#endif

    if (configManager == nullptr || !configManager->hasActiveBevelRecipe()) {
        result.resultCode = 2;
        result.ngReasonWord0 = (1u << 4);
        result.message = QStringLiteral("请先通过 HMI 设置坡口配方（cmd.set_bevel_recipe）。");
        return deliverInspectionResult(result, notifyListener);
    }

    const scan_tracking::common::BevelRecipe recipe = configManager->bevelRecipe();
    const scan_tracking::common::BevelConfig& bevelConfig = configManager->bevelConfig();
    const auto detection = scan_tracking::vision::bevel::runBevelMeasurement(
        pointCloud, recipe, bevelConfig.angleTolDeg, bevelConfig.lengthTolMm);

    if (!detection.invoked) {
        result.resultCode = 2;
        result.ngReasonWord0 = (1u << 4);
        result.message = detection.message.isEmpty()
            ? QStringLiteral("坡口测量适配层未启动。")
            : detection.message;
        return deliverInspectionResult(result, notifyListener);
    }

    result.measurement = measurementFromBevelResult(detection);

    if (!detection.ok) {
        result.resultCode = 2;
        result.ngReasonWord0 = (1u << 5);
        result.message = detection.message.isEmpty()
            ? QStringLiteral("坡口测量算法失败。")
            : detection.message;
        return deliverInspectionResult(result, notifyListener);
    }

    if (detection.qualityCode != 0) {
        result.resultCode = 2;
        result.ngReasonWord0 = (1u << 6);
        result.message = detection.message.isEmpty()
            ? QStringLiteral("坡口测量超出标准范围，qualityCode=%1。")
                  .arg(detection.qualityCode)
            : detection.message;
        return deliverInspectionResult(result, notifyListener);
    }

    result.resultCode = 1;
    result.message = QStringLiteral(
        "坡口测量通过：angle=%1 deg, length=%2 mm, bevelType=%3, icpFitness=%4。")
                         .arg(detection.angleDeg, 0, 'f', 3)
                         .arg(detection.lengthMm, 0, 'f', 3)
                         .arg(detection.bevelType)
                         .arg(detection.icpFitness, 0, 'f', 6);
    result.measureItemCount = countMeasuredItems(result.measurement);
    return deliverInspectionResult(result, notifyListener);
}

InspectionResult TrackingService::inspectCodeRead(int inspectionPathId, bool notifyListener) const
{
    ensureInspectionMeasurementMetaTypeRegistered();

    InspectionResult result;
    result.sourcePointCount = 0;
    result.measurement.algorithm = InspectionAlgorithm::CodeRead;
    result.measureItemCount = 1;

    Q_UNUSED(inspectionPathId);
    // TODO: 接入 HikCameraCController CaptureType::NumberRecognition → kCodeValueAscii
    result.resultCode = 1;
    result.message = QStringLiteral(
        "编号识别已执行（占位）：路径 %1，待接入海康 C OCR。").arg(inspectionPathId);
    qInfo(LOG_TRACKING).noquote() << result.message;
    return deliverInspectionResult(result, notifyListener);
}

InspectionResult TrackingService::inspectSurfaceDefect(int inspectionPathId, bool notifyListener) const
{
    ensureInspectionMeasurementMetaTypeRegistered();

    InspectionResult result;
    result.sourcePointCount = 0;
    result.measurement.algorithm = InspectionAlgorithm::Defect;
    result.measureItemCount = 1;

    Q_UNUSED(inspectionPathId);
    // TODO: 接入 HikCameraCController CaptureType::SurfaceDefect
    result.resultCode = 1;
    result.message = QStringLiteral(
        "缺陷识别已执行（占位）：路径 %1，待接入海康 C 表面缺陷算法。").arg(inspectionPathId);
    qInfo(LOG_TRACKING).noquote() << result.message;
    return deliverInspectionResult(result, notifyListener);
}

InspectionResult TrackingService::inspectThicknessPointClouds(
    const scan_tracking::mech_eye::PointCloudFrame& innerCloud,
    const scan_tracking::mech_eye::PointCloudFrame& outerCloud,
    int innerPointCount,
    int outerPointCount,
    int inspectionPathId,
    bool notifyListener) const
{
    ensureInspectionMeasurementMetaTypeRegistered();

    InspectionResult result;
    result.sourcePointCount = innerPointCount + outerPointCount;

    if (!innerCloud.isValid() || !outerCloud.isValid() || innerPointCount <= 0 || outerPointCount <= 0) {
        result.resultCode = 2;
        result.ngReasonWord0 = (1u << 4);
        result.message = QStringLiteral("厚度测量没有可用的 inner/outer 点云。");
        return deliverInspectionResult(result, notifyListener);
    }

#ifdef SCAN_TRACKING_HAS_THICKNESS_MEASUREMENT
    const auto detection = scan_tracking::vision::thickness::runThicknessMeasurement(
        innerCloud, outerCloud, inspectionPathId);

    if (!detection.invoked) {
        result.resultCode = 2;
        result.ngReasonWord0 = (1u << 4);
        result.message = detection.message.isEmpty()
            ? QStringLiteral("厚度测量适配层未启动。")
            : detection.message;
        return deliverInspectionResult(result, notifyListener);
    }

    result.measurement = measurementFromThicknessResult(detection);

    if (!detection.ok) {
        result.resultCode = 2;
        result.ngReasonWord0 = (1u << 5);
        result.message = detection.message.isEmpty()
            ? QStringLiteral("厚度测量算法失败。")
            : detection.message;
        return deliverInspectionResult(result, notifyListener);
    }

    result.resultCode = 1;
    result.message = detection.message;
    result.measureItemCount = countMeasuredItems(result.measurement);
    return deliverInspectionResult(result, notifyListener);
#else
    result.resultCode = 2;
    result.ngReasonWord0 = (1u << 4);
    result.message = QStringLiteral("厚度测量未编译（SCAN_TRACKING_ENABLE_THICKNESS_MEASUREMENT=OFF）。");
    return deliverInspectionResult(result, notifyListener);
#endif
}

PoseCheckResult TrackingService::checkPose() const
{
    PoseCheckResult result;
    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    if (configManager == nullptr) {
        result.resultCode = 7;
        result.message = QStringLiteral("LB 位姿检查时 ConfigManager 不可用。");
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
    if (result.success && result.hasPoseMatrix()) {
        result.message += QStringLiteral(" Pose matrix is ready for downstream use.");
    }
    return result;
}

}  // namespace scan_tracking::tracking
