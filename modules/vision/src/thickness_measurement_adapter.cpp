#include "scan_tracking/vision/thickness_measurement_adapter.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <mutex>

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QFileInfo>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "Config.h"
#include "ThicknessMeasurement.h"
#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/mech_eye/point_cloud_processor.h"

namespace scan_tracking::vision::thickness {

namespace {

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

    if (frame.width > 0 && frame.height > 0 && frame.width * frame.height == pointCount) {
        cloud->width = static_cast<std::uint32_t>(frame.width);
        cloud->height = static_cast<std::uint32_t>(frame.height);
    } else {
        cloud->width = static_cast<std::uint32_t>(pointCount);
        cloud->height = 1;
    }
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

ThicknessInspectionResult runThicknessMeasurement(
    const scan_tracking::mech_eye::PointCloudFrame& innerCloud,
    const scan_tracking::mech_eye::PointCloudFrame& outerCloud,
    int inspectionPathId)
{
    ThicknessInspectionResult result;

    const QString configPath = resolveThicknessConfigPath(inspectionPathId);
    if (!QFileInfo::exists(configPath)) {
        result.message = QStringLiteral("厚度测量配置文件不存在：%1").arg(configPath);
        return result;
    }

    auto innerPcl = toPclPointCloud(innerCloud);
    auto outerPcl = toPclPointCloud(outerCloud);
    if (innerPcl->empty() || outerPcl->empty()) {
        result.message = QStringLiteral("厚度测量缺少有效 inner/outer 点云。");
        return result;
    }

    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    const double icpFitnessMax = configManager != nullptr
        ? configManager->thicknessConfig().icpFitnessMax
        : 50.0;

    result.invoked = true;

    try {
        std::lock_guard<std::mutex> pclGuard(
            scan_tracking::mech_eye::pointCloudAlgorithmMutex());

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

        ThicknessResult algoResult;
        std::string algoError;
        if (!MeasureThicknessFromClouds(
                measureConfig,
                innerTemplateCloud,
                outerTemplateCloud,
                innerPcl,
                outerPcl,
                &algoResult,
                &algoError)) {
            result.ok = false;
            result.message = algoError.empty()
                ? QStringLiteral("厚度测量算法失败。")
                : QString::fromStdString(algoError);
            return result;
        }

        result.thicknessMm = algoResult.thickness;
        result.innerIcpFitnessScore = algoResult.innerIcpFitnessScore;
        result.outerIcpFitnessScore = algoResult.outerIcpFitnessScore;
        result.icpFitnessScore = std::max(result.innerIcpFitnessScore, result.outerIcpFitnessScore);
        result.thicknessMethod = QString::fromStdString(algoResult.thicknessMethod);

        const bool thicknessOk =
            std::isfinite(result.thicknessMm) && result.thicknessMm > 0.0;
        const bool innerIcpOk =
            std::isfinite(result.innerIcpFitnessScore) && result.innerIcpFitnessScore <= icpFitnessMax;
        const bool outerIcpOk =
            std::isfinite(result.outerIcpFitnessScore) && result.outerIcpFitnessScore <= icpFitnessMax;

        result.ok = thicknessOk && innerIcpOk && outerIcpOk;
        if (result.ok) {
            result.message = QStringLiteral(
                "厚度测量通过：thickness=%1 mm, method=%2, innerIcp=%3, outerIcp=%4。")
                                     .arg(result.thicknessMm, 0, 'f', 3)
                                     .arg(result.thicknessMethod)
                                     .arg(result.innerIcpFitnessScore, 0, 'f', 6)
                                     .arg(result.outerIcpFitnessScore, 0, 'f', 6);
        } else {
            result.message = QStringLiteral(
                "厚度测量未通过：thickness=%1 mm, method=%2, innerIcp=%3, outerIcp=%4 (max %5)。")
                                     .arg(result.thicknessMm, 0, 'f', 3)
                                     .arg(result.thicknessMethod)
                                     .arg(result.innerIcpFitnessScore, 0, 'f', 6)
                                     .arg(result.outerIcpFitnessScore, 0, 'f', 6)
                                     .arg(icpFitnessMax, 0, 'f', 3);
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
