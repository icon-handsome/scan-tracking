#include "scan_tracking/vision/thickness_measurement_adapter.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <mutex>
#include <vector>

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtCore/QLoggingCategory>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "Config.h"
#include "ThicknessMeasurement.h"
#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/mech_eye/point_cloud_processor.h"

Q_LOGGING_CATEGORY(LOG_THICKNESS_ADAPTER, "vision.thickness")

namespace scan_tracking::vision::thickness {

namespace {

pcl::PointCloud<pcl::PointXYZ>::Ptr toPclPointCloud(
    const scan_tracking::mech_eye::PointCloudFrame& frame,
    std::size_t maxPointCount = 300000)
{
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    const auto pointsPtr = frame.pointsXYZ;
    if (!pointsPtr || frame.pointCount <= 0) {
        return cloud;
    }

    const std::vector<float> points(*pointsPtr);
    const int availablePointCount = static_cast<int>(points.size() / 3);
    const int pointCount = std::min(frame.pointCount, availablePointCount);
    if (pointCount <= 0) {
        return cloud;
    }

    const std::size_t targetPointCount = std::min<std::size_t>(
        static_cast<std::size_t>(pointCount), maxPointCount);
    const std::size_t stride = std::max<std::size_t>(
        1, (static_cast<std::size_t>(pointCount) + targetPointCount - 1) / targetPointCount);

    cloud->points.reserve(targetPointCount);
    bool allFinite = true;
    for (std::size_t index = 0; index < static_cast<std::size_t>(pointCount); index += stride) {
        const auto base = static_cast<std::size_t>(index * 3);
        const float x = points[base];
        const float y = points[base + 1];
        const float z = points[base + 2];
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            allFinite = false;
            continue;
        }
        cloud->points.emplace_back(x, y, z);
        if (cloud->points.size() >= targetPointCount) {
            break;
        }
    }

    cloud->width = static_cast<std::uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = allFinite;
    return cloud;
}

bool computeCentroid(
    const scan_tracking::mech_eye::PointCloudFrame& frame,
    Eigen::Vector3d* centroid)
{
    if (centroid == nullptr) {
        return false;
    }
    centroid->setZero();

    const auto pointsPtr = frame.pointsXYZ;
    if (!pointsPtr || frame.pointCount <= 0) {
        return false;
    }

    const std::vector<float>& points = *pointsPtr;
    const int availablePointCount = static_cast<int>(points.size() / 3);
    const int pointCount = std::min(frame.pointCount, availablePointCount);
    if (pointCount <= 0) {
        return false;
    }

    std::size_t validCount = 0;
    for (int index = 0; index < pointCount; ++index) {
        const auto base = static_cast<std::size_t>(index * 3);
        const float x = points[base];
        const float y = points[base + 1];
        const float z = points[base + 2];
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            continue;
        }
        centroid->x() += static_cast<double>(x);
        centroid->y() += static_cast<double>(y);
        centroid->z() += static_cast<double>(z);
        ++validCount;
    }

    if (validCount == 0) {
        return false;
    }

    *centroid /= static_cast<double>(validCount);
    return true;
}

Point3d toPoint3d(const Eigen::Vector3d& value);

bool findNearestPointInRawCloud(
    const scan_tracking::mech_eye::PointCloudFrame& frame,
    const Eigen::Vector3d& query,
    Eigen::Vector3d* nearest,
    std::string* error,
    std::size_t maxSamplePoints = 300000)
{
    if (nearest == nullptr) {
        if (error != nullptr) {
            *error = "nearest output pointer is null";
        }
        return false;
    }

    const auto pointsPtr = frame.pointsXYZ;
    if (!pointsPtr || frame.pointCount <= 0) {
        if (error != nullptr) {
            *error = "raw point cloud is empty";
        }
        return false;
    }

    const std::vector<float>& points = *pointsPtr;
    const int availablePointCount = static_cast<int>(points.size() / 3);
    const int pointCount = std::min(frame.pointCount, availablePointCount);
    if (pointCount <= 0) {
        if (error != nullptr) {
            *error = "raw point cloud is empty";
        }
        return false;
    }

    const std::size_t targetCount = std::min<std::size_t>(static_cast<std::size_t>(pointCount), maxSamplePoints);
    const std::size_t stride = std::max<std::size_t>(
        1, (static_cast<std::size_t>(pointCount) + targetCount - 1) / targetCount);

    double bestDistanceSq = std::numeric_limits<double>::infinity();
    Eigen::Vector3d bestPoint = Eigen::Vector3d::Zero();
    bool found = false;
    for (std::size_t index = 0; index < static_cast<std::size_t>(pointCount); index += stride) {
        const auto base = static_cast<std::size_t>(index * 3);
        const float x = points[base];
        const float y = points[base + 1];
        const float z = points[base + 2];
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            continue;
        }

        const double dx = static_cast<double>(x) - query.x();
        const double dy = static_cast<double>(y) - query.y();
        const double dz = static_cast<double>(z) - query.z();
        const double distanceSq = dx * dx + dy * dy + dz * dz;
        if (!found || distanceSq < bestDistanceSq) {
            bestDistanceSq = distanceSq;
            bestPoint = Eigen::Vector3d(x, y, z);
            found = true;
        }
    }

    if (!found) {
        if (error != nullptr) {
            *error = "nearest point search failed";
        }
        return false;
    }

    *nearest = bestPoint;
    return true;
}

bool measureThicknessDirect(
    const ThicknessConfig& config,
    const scan_tracking::mech_eye::PointCloudFrame& innerScan,
    const scan_tracking::mech_eye::PointCloudFrame& outerScan,
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& innerTemplateCloud,
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& outerTemplateCloud,
    ThicknessResult* result,
    std::string* error)
{
    Eigen::Vector3d innerScanCentroid;
    Eigen::Vector3d outerScanCentroid;
    Eigen::Vector3d innerTemplateCentroid;
    Eigen::Vector3d outerTemplateCentroid;

    if (!computeCentroid(innerScan, &innerScanCentroid)
        || !computeCentroid(outerScan, &outerScanCentroid)
        || !innerTemplateCloud
        || !outerTemplateCloud
        || innerTemplateCloud->empty()
        || outerTemplateCloud->empty())
    {
        if (error != nullptr) {
            *error = "direct thickness measurement input is empty";
        }
        return false;
    }

    auto computePclCentroid = [](const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, Eigen::Vector3d* centroid) -> bool {
        if (centroid == nullptr || !cloud || cloud->empty()) {
            return false;
        }
        centroid->setZero();
        std::size_t validCount = 0;
        for (const auto& point : cloud->points) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
                continue;
            }
            centroid->x() += static_cast<double>(point.x);
            centroid->y() += static_cast<double>(point.y);
            centroid->z() += static_cast<double>(point.z);
            ++validCount;
        }
        if (validCount == 0) {
            return false;
        }
        *centroid /= static_cast<double>(validCount);
        return true;
    };

    if (!computePclCentroid(innerTemplateCloud, &innerTemplateCentroid)
        || !computePclCentroid(outerTemplateCloud, &outerTemplateCentroid))
    {
        if (error != nullptr) {
            *error = "template point cloud centroid computation failed";
        }
        return false;
    }

    const Eigen::Vector3d scanCentroid = 0.5 * (innerScanCentroid + outerScanCentroid);
    const Eigen::Vector3d templateCentroid = 0.5 * (innerTemplateCentroid + outerTemplateCentroid);
    const Eigen::Vector3d offset = templateCentroid - scanCentroid;

    const Eigen::Vector3d outerFeature = ToEigen(config.templateFeaturePoints[0]) - offset;
    const Eigen::Vector3d innerFeature = ToEigen(config.templateFeaturePoints[1]) - offset;
    result->templateFeaturePoints[0] = config.templateFeaturePoints[0];
    result->templateFeaturePoints[1] = config.templateFeaturePoints[1];

    Eigen::Vector3d nearest[2];
    if (!findNearestPointInRawCloud(outerScan, outerFeature, &nearest[0], error))
    {
        return false;
    }
    if (!findNearestPointInRawCloud(innerScan, innerFeature, &nearest[1], error))
    {
        return false;
    }

    result->nearestScanPoints[0] = toPoint3d(nearest[0]);
    result->nearestScanPoints[1] = toPoint3d(nearest[1]);

    result->projectedPoints[0] = toPoint3d(nearest[0]);
    result->projectedPoints[1] = toPoint3d(nearest[1]);
    result->thickness = (nearest[0] - nearest[1]).norm();

    result->innerIcpFitnessScore = 0.0;
    result->outerIcpFitnessScore = 0.0;
    result->thicknessMethod = "direct_raw_nearest_between_surfaces";
    return true;
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

    return resolveConfiguredPath(pathValue);
}

ThicknessConfig prepareThicknessConfig(const QString& configFilePath, ThicknessConfig config)
{
    config.pointCloud.innerTemplateCloudPath =
        resolvePathRelativeToConfigFile(
            configFilePath,
            QString::fromStdString(config.pointCloud.innerTemplateCloudPath))
            .toLocal8Bit()
            .toStdString();
    config.pointCloud.outerTemplateCloudPath =
        resolvePathRelativeToConfigFile(
            configFilePath,
            QString::fromStdString(config.pointCloud.outerTemplateCloudPath))
            .toLocal8Bit()
            .toStdString();
    return config;
}

bool loadTemplateCloud(const std::string& path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    if (path.empty()) {
        return false;
    }

    const QString qPath = QString::fromStdString(path);
    if (qPath.endsWith(QStringLiteral(".ply"), Qt::CaseInsensitive)) {
        return pcl::io::loadPLYFile<pcl::PointXYZ>(path, *cloud) >= 0 && !cloud->empty();
    }
    return pcl::io::loadPCDFile<pcl::PointXYZ>(path, *cloud) >= 0 && !cloud->empty();
}

scan_tracking::mech_eye::PointCloudFrame snapshotPointCloudFrame(
    const scan_tracking::mech_eye::PointCloudFrame& frame)
{
    scan_tracking::mech_eye::PointCloudFrame copy = frame;
    if (frame.pointsXYZ) {
        copy.pointsXYZ = std::make_shared<std::vector<float>>(*frame.pointsXYZ);
    }
    if (frame.normalsXYZ) {
        copy.normalsXYZ = std::make_shared<std::vector<float>>(*frame.normalsXYZ);
    }
    return copy;
}

Point3d toPoint3d(const Eigen::Vector3d& value)
{
    Point3d point;
    point.x = value.x();
    point.y = value.y();
    point.z = value.z();
    return point;
}

}  // namespace

QString resolveThicknessConfigPath(int inspectionPathId)
{
    const QString envRoot = localPathFromEnv("SCAN_TRACKING_THICKNESS_CONFIG_DIR");
    if (!envRoot.isEmpty()) {
        const QFileInfo envConfig(QDir(envRoot).filePath(QStringLiteral("thickness_config.json")));
        if (envConfig.exists()) {
            return envConfig.absoluteFilePath();
        }
    }

    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    QString configured;
    if (configManager != nullptr) {
        configured = inspectionPathId > 0
            ? configManager->thicknessConfigPathForPath(inspectionPathId)
            : configManager->thicknessConfig().configPath;
    }

    const QString resolved = resolveConfiguredPath(configured.trimmed());
    if (QFileInfo::exists(resolved)) {
        return resolved;
    }

    const QFileInfo fallback(QDir(QCoreApplication::applicationDirPath())
                                 .filePath(QStringLiteral("thickness/config/thickness_config.json")));
    return fallback.absoluteFilePath();
}

QString resolveRuntimeThicknessConfigPath(int inspectionPathId)
{
    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    if (configManager != nullptr && configManager->thicknessConfig().offlineReplayEnabled) {
        const QString offlinePath =
            configManager->thicknessConfig().offlineReplayAlgorithmConfigPath.trimmed();
        if (!offlinePath.isEmpty()) {
            const QString resolved = resolveConfiguredPath(offlinePath);
            if (QFileInfo::exists(resolved)) {
                return resolved;
            }
        }
    }
    return resolveThicknessConfigPath(inspectionPathId);
}

ThicknessInspectionResult runThicknessMeasurement(
    const scan_tracking::mech_eye::PointCloudFrame& innerCloud,
    const scan_tracking::mech_eye::PointCloudFrame& outerCloud,
    int inspectionPathId)
{
    ThicknessInspectionResult result;

    const QString configPath = resolveRuntimeThicknessConfigPath(inspectionPathId);
    qInfo(LOG_THICKNESS_ADAPTER).noquote()
        << QStringLiteral("[Thickness] 开始测量 innerPoints=") << innerCloud.pointCount
        << QStringLiteral(" outerPoints=") << outerCloud.pointCount
        << QStringLiteral(" config=") << configPath;
    if (!QFileInfo::exists(configPath)) {
        result.message = QStringLiteral("厚度测量配置文件不存在：%1").arg(configPath);
        return result;
    }

    result.invoked = true;

    try {
        const scan_tracking::mech_eye::PointCloudFrame innerSnapshot = snapshotPointCloudFrame(innerCloud);
        const scan_tracking::mech_eye::PointCloudFrame outerSnapshot = snapshotPointCloudFrame(outerCloud);

        std::lock_guard<std::mutex> pclGuard(
            scan_tracking::mech_eye::pointCloudAlgorithmMutex());

        qInfo(LOG_THICKNESS_ADAPTER) << QStringLiteral("[Thickness] 转换 inner 点云至 PCL...");
        auto innerPcl = toPclPointCloud(innerSnapshot);
        qInfo(LOG_THICKNESS_ADAPTER).noquote()
            << QStringLiteral("[Thickness] inner PCL 点数=") << innerPcl->size();
        qInfo(LOG_THICKNESS_ADAPTER) << QStringLiteral("[Thickness] 转换 outer 点云至 PCL...");
        auto outerPcl = toPclPointCloud(outerSnapshot);
        qInfo(LOG_THICKNESS_ADAPTER).noquote()
            << QStringLiteral("[Thickness] outer PCL 点数=") << outerPcl->size();
        if (innerPcl->empty() || outerPcl->empty()) {
            result.message = QStringLiteral("厚度测量缺少有效 inner/outer 点云。");
            return result;
        }

        ThicknessConfig measureConfig;
        std::string loadError;
        if (!LoadConfig(configPath.toLocal8Bit().toStdString(), &measureConfig, &loadError)) {
            result.message = QStringLiteral("厚度测量配置加载失败：%1")
                                 .arg(QString::fromStdString(loadError));
            return result;
        }
        measureConfig = prepareThicknessConfig(configPath, measureConfig);

        if (measureConfig.pointCloud.innerTemplateCloudPath.empty()
            || measureConfig.pointCloud.outerTemplateCloudPath.empty()) {
            result.message = QStringLiteral(
                "厚度测量配置缺少 inner_template_cloud_path / outer_template_cloud_path。");
            return result;
        }

        const QString innerTemplatePath =
            QString::fromStdString(measureConfig.pointCloud.innerTemplateCloudPath);
        const QString outerTemplatePath =
            QString::fromStdString(measureConfig.pointCloud.outerTemplateCloudPath);
        if (!QFileInfo::exists(innerTemplatePath)) {
            result.message = QStringLiteral("内表面模板点云不存在：%1").arg(innerTemplatePath);
            return result;
        }
        if (!QFileInfo::exists(outerTemplatePath)) {
            result.message = QStringLiteral("外表面模板点云不存在：%1").arg(outerTemplatePath);
            return result;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr innerTemplateCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr outerTemplateCloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (!loadTemplateCloud(measureConfig.pointCloud.innerTemplateCloudPath, innerTemplateCloud)) {
            result.message = QStringLiteral("内表面模板点云加载失败：%1").arg(innerTemplatePath);
            return result;
        }
        if (!loadTemplateCloud(measureConfig.pointCloud.outerTemplateCloudPath, outerTemplateCloud)) {
            result.message = QStringLiteral("外表面模板点云加载失败：%1").arg(outerTemplatePath);
            return result;
        }

        qInfo(LOG_THICKNESS_ADAPTER).noquote()
            << QStringLiteral("[Thickness] 模板已加载，开始 direct_raw 厚度测量...");

        ThicknessResult directResult;
        std::string directError;
        if (!measureThicknessDirect(
                measureConfig,
                innerSnapshot,
                outerSnapshot,
                innerTemplateCloud,
                outerTemplateCloud,
                &directResult,
                &directError)) {
            result.ok = false;
            result.message = directError.empty()
                ? QStringLiteral("厚度测量直接路径失败。")
                : QString::fromStdString(directError);
            qWarning(LOG_THICKNESS_ADAPTER).noquote()
                << QStringLiteral("[Thickness] 直接路径失败：") << result.message;
            return result;
        }

        qInfo(LOG_THICKNESS_ADAPTER).noquote()
            << QStringLiteral("[Thickness] 直接路径返回 thickness=") << directResult.thickness
            << QStringLiteral(" method=") << QString::fromStdString(directResult.thicknessMethod);

        result.thicknessMm = directResult.thickness;
        result.innerIcpFitnessScore = directResult.innerIcpFitnessScore;
        result.outerIcpFitnessScore = directResult.outerIcpFitnessScore;
        result.icpFitnessScore = std::max(result.innerIcpFitnessScore, result.outerIcpFitnessScore);
        result.thicknessMethod = QString::fromStdString(directResult.thicknessMethod);

        const bool thicknessOk =
            std::isfinite(result.thicknessMm) && result.thicknessMm > 0.0;
        result.ok = thicknessOk;
        if (result.ok) {
            result.message = QStringLiteral("厚度测量通过：thickness=%1 mm, method=%2。")
                                 .arg(result.thicknessMm, 0, 'f', 3)
                                 .arg(result.thicknessMethod);
        } else {
            result.message = QStringLiteral("厚度测量未通过：thickness=%1 mm, method=%2。")
                                 .arg(result.thicknessMm, 0, 'f', 3)
                                 .arg(result.thicknessMethod);
        }
    } catch (const std::exception& ex) {
        result.ok = false;
        result.message = QStringLiteral("厚度测量抛出异常：%1")
                             .arg(QString::fromUtf8(ex.what()));
    } catch (...) {
        result.ok = false;
        result.message = QStringLiteral("厚度测量抛出未知异常。");
    }

    return result;
}

}  // namespace scan_tracking::vision::thickness
