#include "scan_tracking/vision/hole_measurement_adapter.h"

#include <cmath>
#include <cstdlib>
#include <exception>
#include <mutex>
#include <vector>

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtCore/QLoggingCategory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "HeadMeasure/Config.h"
#include "HeadMeasure/MeasurePipeline.h"
#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/mech_eye/point_cloud_processor.h"

namespace scan_tracking::vision::hole {

namespace {

Q_LOGGING_CATEGORY(LOG_HOLE, "vision.hole")

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
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            allFinite = false;
            continue;
        }
        cloud->points.emplace_back(x, y, z);
    }

    cloud->width = static_cast<std::uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = allFinite;
    return cloud;
}

QString localPathFromEnv(const char* name)
{
    const char* value = std::getenv(name);
    if (value == nullptr || value[0] == '\0') {
        return {};
    }
    return QString::fromLocal8Bit(value);
}

QString resolveConfiguredPath(const QString& configured)
{
    if (configured.trimmed().isEmpty()) {
        return {};
    }

    const QFileInfo configuredInfo(configured);
    if (configuredInfo.isAbsolute() && configuredInfo.exists()) {
        return configuredInfo.absoluteFilePath();
    }

    const QFileInfo relativeToExe(
        QDir(QCoreApplication::applicationDirPath()).filePath(configured));
    if (relativeToExe.exists()) {
        return relativeToExe.absoluteFilePath();
    }

    return configuredInfo.absoluteFilePath();
}

QString resolvePathRelativeToConfigFile(const QString& configFilePath, const QString& pathValue)
{
    if (pathValue.trimmed().isEmpty()) {
        return {};
    }

    const QFileInfo pathInfo(pathValue);
    if (pathInfo.isAbsolute()) {
        return pathInfo.absoluteFilePath();
    }

    const QFileInfo configInfo(configFilePath);
    const QFileInfo relativeToConfig(QDir(configInfo.absolutePath()).filePath(pathValue));
    if (relativeToConfig.exists()) {
        return relativeToConfig.absoluteFilePath();
    }

    const QFileInfo relativeToHoleRoot(
        QDir(configInfo.absolutePath()).filePath(QStringLiteral("../") + pathValue));
    if (relativeToHoleRoot.exists()) {
        return relativeToHoleRoot.absoluteFilePath();
    }

    return resolveConfiguredPath(pathValue);
}

hm::MeasureConfig prepareMeasureConfig(const QString& configFilePath, hm::MeasureConfig config)
{
    config.inputFrames.clear();
    config.templateCloud = resolvePathRelativeToConfigFile(configFilePath, QString::fromStdString(config.templateCloud))
                               .toLocal8Bit()
                               .toStdString();
    return config;
}

bool isFiniteDouble(double value)
{
    return std::isfinite(value);
}

HoleInspectionResult runHoleMeasurementWithScanClouds(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& scanClouds,
    int inspectionPathId,
    int rawPointCount)
{
    HoleInspectionResult result;

    const QString configPath = resolveHoleConfigPath(inspectionPathId);
    if (!QFileInfo::exists(configPath)) {
        result.message = QStringLiteral("Hole 测量配置文件不存在：%1").arg(configPath);
        return result;
    }

    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    const double icpRmsMaxMm = configManager != nullptr
        ? configManager->holeConfig().icpRmsMaxMm
        : 5.0;
    const double cylinderRmsMaxMm = configManager != nullptr
        ? configManager->holeConfig().cylinderRmsMaxMm
        : 3.0;

    result.invoked = true;

    try {
        std::lock_guard<std::mutex> pclGuard(
            scan_tracking::mech_eye::pointCloudAlgorithmMutex());

        hm::MeasureConfig measureConfig = hm::loadConfig(configPath.toLocal8Bit().toStdString());
        measureConfig = prepareMeasureConfig(configPath, measureConfig);

        if (measureConfig.templateCloud.empty()) {
            result.message = QStringLiteral("Hole 测量配置缺少 template_cloud。");
            return result;
        }
        if (!QFileInfo::exists(QString::fromStdString(measureConfig.templateCloud))) {
            result.message = QStringLiteral("Hole 模板点云不存在：%1")
                                 .arg(QString::fromStdString(measureConfig.templateCloud));
            return result;
        }

        std::vector<hm::CloudConstPtr> rawScans;
        rawScans.reserve(scanClouds.size());
        int convertedPointCount = 0;
        for (const auto& scanCloud : scanClouds) {
            if (!scanCloud || scanCloud->empty()) {
                continue;
            }
            rawScans.push_back(scanCloud);
            convertedPointCount += static_cast<int>(scanCloud->size());
        }

        if (rawScans.empty()) {
            result.message = QStringLiteral("Hole 测量缺少有效输入点云。");
            return result;
        }

        qInfo(LOG_HOLE).noquote()
            << QStringLiteral("[Hole] 开始测量 pathId=") << inspectionPathId
            << QStringLiteral(" 原始点数=") << (rawPointCount > 0 ? rawPointCount : convertedPointCount)
            << QStringLiteral(" 分段数=") << rawScans.size()
            << QStringLiteral(" 模板=") << QString::fromStdString(measureConfig.templateCloud)
            << QStringLiteral(" voxel_leaf_mm=") << measureConfig.voxelLeafMm
            << QStringLiteral(" statistical_mean_k=") << measureConfig.statisticalMeanK;

        hm::MeasurePipeline pipeline(measureConfig);
        if (rawScans.size() == 1) {
            qInfo(LOG_HOLE) << QStringLiteral("[Hole] 调用 runWithScanCloud（与 demo 单帧一致）");
            result.measureResult = pipeline.runWithScanCloud(rawScans.front());
        } else {
            qInfo(LOG_HOLE).noquote()
                << QStringLiteral("[Hole] 调用 runWithScanClouds（与 demo mergeFrames 一致） frameCount=")
                << rawScans.size();
            result.measureResult = pipeline.runWithScanClouds(rawScans);
        }

        result.icpRmsMm = result.measureResult.icpFit.rmsMm;
        result.cylinderRmsMm = result.measureResult.cylinderFit.rmsMm;

        const bool icpOk = isFiniteDouble(result.icpRmsMm) && result.icpRmsMm <= icpRmsMaxMm;
        const bool cylinderOk =
            isFiniteDouble(result.cylinderRmsMm) && result.cylinderRmsMm <= cylinderRmsMaxMm;
        const bool metricsOk = isFiniteDouble(result.measureResult.innerDiameterMm)
            && result.measureResult.innerDiameterMm > 0.0;

        result.ok = icpOk && cylinderOk && metricsOk;
        if (result.ok) {
            result.message = QStringLiteral(
                "Hole 测量通过：内径=%1 mm, 圆度=%2 mm, 开孔距=%3 mm, 接头角=%4 deg。")
                                   .arg(result.measureResult.innerDiameterMm, 0, 'f', 3)
                                   .arg(result.measureResult.roundnessToleranceMm, 0, 'f', 3)
                                   .arg(result.measureResult.opening.centerToInnerWallDistanceMm,
                                        0,
                                        'f',
                                        3)
                                   .arg(result.measureResult.opening.axisToHeadAxisAngleDeg,
                                        0,
                                        'f',
                                        3);
        } else if (!isFiniteDouble(result.icpRmsMm)
                   || result.icpRmsMm > icpRmsMaxMm * 20.0) {
            result.message = QStringLiteral(
                "Hole 模板 ICP 未收敛（rms=%1 mm，阈值 %2 mm）。"
                "请检查 LB 位姿/T0 标定，或确认扫描点云与模板在同一坐标系。")
                                   .arg(result.icpRmsMm, 0, 'f', 1)
                                   .arg(icpRmsMaxMm, 0, 'f', 1);
        } else {
            result.message = QStringLiteral(
                "Hole 测量未通过：icpRms=%1 mm (max %2), cylinderRms=%3 mm (max %4), innerDiameter=%5 mm。")
                                   .arg(result.icpRmsMm, 0, 'f', 3)
                                   .arg(icpRmsMaxMm, 0, 'f', 3)
                                   .arg(result.cylinderRmsMm, 0, 'f', 3)
                                   .arg(cylinderRmsMaxMm, 0, 'f', 3)
                                   .arg(result.measureResult.innerDiameterMm, 0, 'f', 3);
        }
    } catch (const std::exception& ex) {
        result.ok = false;
        result.message = QStringLiteral("Hole 测量抛出异常：%1")
                             .arg(QString::fromUtf8(ex.what()));
    } catch (...) {
        result.ok = false;
        result.message = QStringLiteral("Hole 测量抛出未知异常。");
    }

    return result;
}

}  // namespace

QString resolveHoleConfigPath(int inspectionPathId)
{
    const QString envRoot = localPathFromEnv("SCAN_TRACKING_HOLE_CONFIG_DIR");
    if (!envRoot.isEmpty()) {
        const QFileInfo envConfig(QDir(envRoot).filePath(QStringLiteral("default.json")));
        if (envConfig.exists()) {
            return envConfig.absoluteFilePath();
        }
    }

    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    QString configured;
    if (configManager != nullptr) {
        configured = inspectionPathId > 0
            ? configManager->holeConfigPathForPath(inspectionPathId)
            : configManager->holeConfig().configPath;
    }

    const QString resolved = resolveConfiguredPath(configured.trimmed());
    if (QFileInfo::exists(resolved)) {
        return resolved;
    }

    const QFileInfo fallback(
        QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("hole/config/default.json")));
    return fallback.absoluteFilePath();
}

HoleInspectionResult runHoleMeasurement(
    const scan_tracking::mech_eye::PointCloudFrame& cloud,
    int inspectionPathId)
{
    const auto pclCloud = toPclPointCloud(cloud);
    return runHoleMeasurementWithScanClouds({pclCloud}, inspectionPathId, cloud.pointCount);
}

HoleInspectionResult runHoleMeasurementFromSegmentFrames(
    const QList<scan_tracking::mech_eye::PointCloudFrame>& segmentClouds,
    int inspectionPathId,
    int sourcePointCount)
{
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> scanClouds;
    scanClouds.reserve(static_cast<std::size_t>(segmentClouds.size()));
    for (const auto& segmentCloud : segmentClouds) {
        auto pclCloud = toPclPointCloud(segmentCloud);
        if (pclCloud && !pclCloud->empty()) {
            scanClouds.push_back(std::move(pclCloud));
        }
    }
    return runHoleMeasurementWithScanClouds(
        scanClouds,
        inspectionPathId,
        sourcePointCount > 0 ? sourcePointCount : 0);
}

}  // namespace scan_tracking::vision::hole
