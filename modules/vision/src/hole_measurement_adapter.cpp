#include "scan_tracking/vision/hole_measurement_adapter.h"

#include <cmath>
#include <cstdlib>
#include <exception>
#include <functional>
#include <mutex>
#include <vector>

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtCore/QLoggingCategory>
#include <QtCore/QStringList>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

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

pcl::PointCloud<pcl::PointXYZ>::Ptr loadPclFromPointCloudFile(const QString& absolutePath)
{
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    const QFileInfo fileInfo(absolutePath);
    if (!fileInfo.exists()) {
        return cloud;
    }

    const QByteArray localPath = fileInfo.absoluteFilePath().toLocal8Bit();
    const int loadResult = absolutePath.endsWith(QStringLiteral(".ply"), Qt::CaseInsensitive)
        ? pcl::io::loadPLYFile(localPath.constData(), *cloud)
        : pcl::io::loadPCDFile(localPath.constData(), *cloud);
    if (loadResult != 0) {
        cloud->clear();
    }
    return cloud;
}

void appendPointCloud(hm::CloudPtr& merged, const hm::CloudConstPtr& part)
{
    if (!merged || !part || part->empty()) {
        return;
    }
    const std::size_t oldSize = merged->points.size();
    merged->points.reserve(oldSize + part->points.size());
    merged->points.insert(merged->points.end(), part->points.begin(), part->points.end());
    merged->width = static_cast<std::uint32_t>(merged->points.size());
    merged->height = 1;
}

bool isFiniteDouble(double value)
{
    return std::isfinite(value);
}

hm::MeasureConfig prepareMeasureConfig(const QString& configFilePath, hm::MeasureConfig config);

void applyHoleMeasurementVerdict(
    HoleInspectionResult& result,
    double icpRmsMaxMm,
    double cylinderRmsMaxMm,
    std::size_t configuredOpeningCount)
{
    result.icpRmsMm = result.measureResult.icpFit.rmsMm;
    result.cylinderRmsMm = result.measureResult.cylinderFit.rmsMm;

    const bool icpOk = isFiniteDouble(result.icpRmsMm) && result.icpRmsMm <= icpRmsMaxMm;
    const bool cylinderOk =
        isFiniteDouble(result.cylinderRmsMm) && result.cylinderRmsMm <= cylinderRmsMaxMm;
    const bool metricsOk = isFiniteDouble(result.measureResult.innerDiameterMm)
        && result.measureResult.innerDiameterMm > 0.0;
    const bool openingOk = configuredOpeningCount == 0
        || !result.measureResult.opening.name.empty();

    result.ok = icpOk && cylinderOk && metricsOk && openingOk;
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
    } else if (!openingOk) {
        result.message = QStringLiteral(
            "Hole 测量未通过：开孔未检出（配置 %1 个开孔，检出 name=%2）。"
            "请查日志 opening_projection_crop / opening_feature_search。")
                               .arg(configuredOpeningCount)
                               .arg(result.measureResult.opening.name.empty()
                                        ? QStringLiteral("(none)")
                                        : QString::fromStdString(result.measureResult.opening.name));
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
}

HoleInspectionResult runHoleMeasurementIncremental(
    int inspectionPathId,
    int sourcePointCount,
    int frameCount,
    const std::function<hm::CloudPtr(int index, QString* segmentLabel)>& loadFrame)
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

        qInfo(LOG_HOLE).noquote()
            << QStringLiteral("[Hole] 开始测量 pathId=") << inspectionPathId
            << QStringLiteral(" 原始点数=") << sourcePointCount
            << QStringLiteral(" 分段数=") << frameCount
            << QStringLiteral(" 模板=") << QString::fromStdString(measureConfig.templateCloud)
            << QStringLiteral(" voxel_leaf_mm=") << measureConfig.voxelLeafMm
            << QStringLiteral(" statistical_mean_k=") << measureConfig.statisticalMeanK;

        hm::MeasurePipeline pipeline(measureConfig);
        if (frameCount <= 1) {
            QString segmentLabel;
            const hm::CloudPtr singleFrame = loadFrame(0, &segmentLabel);
            if (!singleFrame || singleFrame->empty()) {
                result.message = QStringLiteral("Hole 测量缺少有效输入点云。");
                return result;
            }
            qInfo(LOG_HOLE) << QStringLiteral("[Hole] 调用 runWithScanCloud（与 demo 单帧一致）");
            result.measureResult = pipeline.runWithScanCloud(singleFrame);
        } else {
            qInfo(LOG_HOLE).noquote()
                << QStringLiteral("[Hole] 分段增量 preprocess 后合并 frameCount=") << frameCount;
            hm::CloudPtr merged(new hm::Cloud);
            for (int index = 0; index < frameCount; ++index) {
                QString segmentLabel;
                hm::CloudPtr frame = loadFrame(index, &segmentLabel);
                if (!frame || frame->empty()) {
                    continue;
                }

                hm::PointT minPt;
                hm::PointT maxPt;
                pcl::getMinMax3D(*frame, minPt, maxPt);
                qInfo(LOG_HOLE).noquote()
                    << QStringLiteral("[Hole] frame_bounds index=") << index
                    << (segmentLabel.isEmpty() ? QString() : QStringLiteral(" file=") + segmentLabel)
                    << QStringLiteral(" X:[") << minPt.x << QStringLiteral(",") << maxPt.x
                    << QStringLiteral("] Y:[") << minPt.y << QStringLiteral(",") << maxPt.y
                    << QStringLiteral("] Z:[") << minPt.z << QStringLiteral(",") << maxPt.z
                    << QStringLiteral("] points=") << frame->size();

                hm::CloudPtr current = pipeline.preprocessScan(frame);
                frame.reset();
                appendPointCloud(merged, current);
                qInfo(LOG_HOLE).noquote()
                    << QStringLiteral("[Hole] frame_preprocessed index=") << index
                    << QStringLiteral(" points=") << current->size()
                    << QStringLiteral(" merged=") << merged->size();
                current.reset();
            }
            if (!merged || merged->empty()) {
                result.message = QStringLiteral("Hole 测量分段预处理后点云为空。");
                return result;
            }
            result.measureResult = pipeline.runWithPreprocessedScanCloud(merged);
        }

        applyHoleMeasurementVerdict(
            result,
            icpRmsMaxMm,
            cylinderRmsMaxMm,
            measureConfig.templateOpenings.size());

        if (!measureConfig.templateOpenings.empty()) {
            const auto& opening = result.measureResult.opening;
            qInfo(LOG_HOLE).noquote()
                << QStringLiteral("[Hole] 开孔结果 name=")
                << (opening.name.empty() ? QStringLiteral("(none)")
                                         : QString::fromStdString(opening.name))
                << QStringLiteral(" 开孔距=") << opening.centerToInnerWallDistanceMm
                << QStringLiteral(" 接头角=") << opening.axisToHeadAxisAngleDeg
                << QStringLiteral(" 配置开孔数=") << measureConfig.templateOpenings.size()
                << QStringLiteral(" verdict=") << (result.ok ? QStringLiteral("OK") : QStringLiteral("NG"));
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

HoleInspectionResult runHoleMeasurementWithScanClouds(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& scanClouds,
    int inspectionPathId,
    int rawPointCount)
{
    std::vector<hm::CloudPtr> frames;
    frames.reserve(scanClouds.size());
    int convertedPointCount = 0;
    for (const auto& scanCloud : scanClouds) {
        if (!scanCloud || scanCloud->empty()) {
            continue;
        }
        frames.push_back(scanCloud);
        convertedPointCount += static_cast<int>(scanCloud->size());
    }
    if (frames.empty()) {
        HoleInspectionResult result;
        result.message = QStringLiteral("Hole 测量缺少有效输入点云。");
        return result;
    }

    return runHoleMeasurementIncremental(
        inspectionPathId,
        rawPointCount > 0 ? rawPointCount : convertedPointCount,
        static_cast<int>(frames.size()),
        [&frames](int index, QString* segmentLabel) -> hm::CloudPtr {
            if (segmentLabel != nullptr) {
                segmentLabel->clear();
            }
            return frames[static_cast<std::size_t>(index)];
        });
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
    QList<scan_tracking::mech_eye::PointCloudFrame> validFrames;
    validFrames.reserve(segmentClouds.size());
    int convertedPointCount = 0;
    for (const auto& segmentCloud : segmentClouds) {
        if (!segmentCloud.isValid()) {
            continue;
        }
        validFrames.push_back(segmentCloud);
        convertedPointCount += segmentCloud.pointCount;
    }
    if (validFrames.isEmpty()) {
        HoleInspectionResult result;
        result.message = QStringLiteral("Hole 测量缺少有效输入点云。");
        return result;
    }

    return runHoleMeasurementIncremental(
        inspectionPathId,
        sourcePointCount > 0 ? sourcePointCount : convertedPointCount,
        validFrames.size(),
        [&validFrames](int index, QString* segmentLabel) -> hm::CloudPtr {
            if (segmentLabel != nullptr) {
                segmentLabel->clear();
            }
            return toPclPointCloud(validFrames.at(index));
        });
}

HoleInspectionResult runHoleMeasurementFromSegmentPcdFiles(
    const QStringList& segmentPcdPaths,
    int inspectionPathId,
    int sourcePointCount)
{
    if (segmentPcdPaths.isEmpty()) {
        HoleInspectionResult result;
        result.message = QStringLiteral("Hole 测量缺少分段点云文件。");
        return result;
    }

    qInfo(LOG_HOLE).noquote()
        << QStringLiteral("[Hole] 从落盘 PCD 逐段加载 frameCount=") << segmentPcdPaths.size();

    return runHoleMeasurementIncremental(
        inspectionPathId,
        sourcePointCount,
        segmentPcdPaths.size(),
        [&segmentPcdPaths](int index, QString* segmentLabel) -> hm::CloudPtr {
            const QString& path = segmentPcdPaths.at(index);
            if (segmentLabel != nullptr) {
                *segmentLabel = path;
            }
            return loadPclFromPointCloudFile(path);
        });
}

}  // namespace scan_tracking::vision::hole
