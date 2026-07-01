#include "scan_tracking/vision/thickness_measurement_adapter.h"

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

#include "Config.h"
#include "ThicknessMeasurement.h"
#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/mech_eye/point_cloud_processor.h"

Q_LOGGING_CATEGORY(LOG_THICKNESS_ADAPTER, "vision.thickness")

namespace scan_tracking::vision::thickness {

namespace {

pcl::PointCloud<pcl::PointXYZ>::Ptr toPclPointCloud(
    const scan_tracking::mech_eye::PointCloudFrame& frame)
{
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    const auto pointsPtr = frame.pointsXYZ;
    if (!pointsPtr || frame.pointCount <= 0) {
        return cloud;
    }

    const std::vector<float>& points = *pointsPtr;
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

        qInfo(LOG_THICKNESS_ADAPTER) << QStringLiteral("[Thickness] 转换 inner 点云至 PCL（全量）...");
        auto innerPcl = toPclPointCloud(innerSnapshot);
        qInfo(LOG_THICKNESS_ADAPTER).noquote()
            << QStringLiteral("[Thickness] inner PCL 点数=") << innerPcl->size();
        qInfo(LOG_THICKNESS_ADAPTER) << QStringLiteral("[Thickness] 转换 outer 点云至 PCL（全量）...");
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

        qInfo(LOG_THICKNESS_ADAPTER).noquote()
            << QStringLiteral("[Thickness] 调用 MeasureThicknessFromScanClouds（与 demo 一致）...");

        ThicknessResult algorithmResult;
        std::string algorithmError;
        if (!MeasureThicknessFromScanClouds(
                measureConfig, innerPcl, outerPcl, &algorithmResult, &algorithmError)) {
            result.ok = false;
            result.message = algorithmError.empty()
                ? QStringLiteral("厚度测量算法失败。")
                : QString::fromStdString(algorithmError);
            qWarning(LOG_THICKNESS_ADAPTER).noquote()
                << QStringLiteral("[Thickness] 算法失败：") << result.message;
            return result;
        }

        qInfo(LOG_THICKNESS_ADAPTER).noquote()
            << QStringLiteral("[Thickness] 算法返回 thickness=") << algorithmResult.thickness
            << QStringLiteral(" method=") << QString::fromStdString(algorithmResult.thicknessMethod)
            << QStringLiteral(" innerIcpFitness=") << algorithmResult.innerIcpFitnessScore
            << QStringLiteral(" outerIcpFitness=") << algorithmResult.outerIcpFitnessScore;

        result.thicknessMm = algorithmResult.thickness;
        result.innerIcpFitnessScore = algorithmResult.innerIcpFitnessScore;
        result.outerIcpFitnessScore = algorithmResult.outerIcpFitnessScore;
        result.icpFitnessScore = std::max(result.innerIcpFitnessScore, result.outerIcpFitnessScore);
        result.thicknessMethod = QString::fromStdString(algorithmResult.thicknessMethod);

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
