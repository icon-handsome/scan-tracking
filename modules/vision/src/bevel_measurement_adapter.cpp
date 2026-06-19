#include "scan_tracking/vision/bevel_measurement_adapter.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <mutex>

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QFileInfo>

#include "BevelMeasurement.h"
#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/mech_eye/point_cloud_processor.h"

namespace scan_tracking::vision::bevel {

namespace {

QString localPathFromEnv(const char* name)
{
    const char* value = std::getenv(name);
    if (value == nullptr || value[0] == '\0') {
        return {};
    }
    return QString::fromLocal8Bit(value);
}

QString defaultBevelRootDirectory()
{
    return QCoreApplication::applicationDirPath() + QStringLiteral("/bevel");
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

QString resolveBevelConfigPath()
{
    // 1. 基础 config.txt 解析（env / config.ini / exe 旁 bevel/）
    QString baseConfig;

    const QString envRoot = localPathFromEnv("SCAN_TRACKING_BEVEL_CONFIG_DIR");
    if (!envRoot.isEmpty()) {
        const QFileInfo envConfig(QDir(envRoot).filePath(QStringLiteral("config.txt")));
        if (envConfig.exists()) {
            baseConfig = envConfig.absoluteFilePath();
        }
    }

    if (baseConfig.isEmpty()) {
        const auto* configManager = scan_tracking::common::ConfigManager::instance();
        if (configManager != nullptr) {
            const QString configured = configManager->bevelConfig().configPath.trimmed();
            if (!configured.isEmpty()) {
                QFileInfo configuredInfo(configured);
                if (configuredInfo.isAbsolute() && configuredInfo.exists()) {
                    baseConfig = configuredInfo.absoluteFilePath();
                } else {
                    const QFileInfo relativeToExe(
                        QDir(QCoreApplication::applicationDirPath()).filePath(configured));
                    if (relativeToExe.exists()) {
                        baseConfig = relativeToExe.absoluteFilePath();
                    }
                }
            }
        }
    }

    if (baseConfig.isEmpty()) {
        const QFileInfo defaultConfig(defaultBevelRootDirectory() + QStringLiteral("/config.txt"));
        baseConfig = defaultConfig.absoluteFilePath();
    }

    // 2. V1.2 单模板单 config：按当前配方坡口类型选 config_type{N}.txt。
    //    type0 → config_type0.txt（随程序部署）；type1 → config_type1.txt，
    //    未部署则由 runBevelMeasurement 的存在性检查给出明确错误（type1 待标定）。
    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    if (configManager != nullptr && configManager->hasActiveBevelRecipe()) {
        const int bevelType = configManager->bevelRecipe().bevelType;
        if (bevelType >= 0) {
            const QFileInfo baseInfo(baseConfig);
            return baseInfo.dir().filePath(
                QStringLiteral("config_type%1.txt").arg(bevelType));
        }
    }

    return baseConfig;
}

QString resolveBevelTemplateDir()
{
    const QString envRoot = localPathFromEnv("SCAN_TRACKING_BEVEL_CONFIG_DIR");
    if (!envRoot.isEmpty()) {
        const QDir envTemplateDir(QDir(envRoot).filePath(QStringLiteral("data/templates")));
        if (envTemplateDir.exists()) {
            return envTemplateDir.absolutePath();
        }
    }

    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    if (configManager != nullptr) {
        const QString configured = configManager->bevelConfig().templateDir.trimmed();
        if (!configured.isEmpty()) {
            QFileInfo configuredInfo(configured);
            if (configuredInfo.isAbsolute() && configuredInfo.exists()) {
                return configuredInfo.absoluteFilePath();
            }
            const QFileInfo relativeToExe(
                QDir(QCoreApplication::applicationDirPath()).filePath(configured));
            if (relativeToExe.exists()) {
                return relativeToExe.absoluteFilePath();
            }
        }
    }

    const QDir defaultTemplateDir(defaultBevelRootDirectory() + QStringLiteral("/data/templates"));
    return defaultTemplateDir.exists() ? defaultTemplateDir.absolutePath() : QString();
}

BevelSolveOptions buildBevelSolveOptions(
    const scan_tracking::common::BevelRecipe& recipe,
    float angleTolDeg,
    float lengthTolMm)
{
    BevelSolveOptions options;
    options.forcedBevelType = recipe.bevelType;
    options.overrideStandard = true;
    options.standardAngleMinDeg =
        static_cast<double>(recipe.angleDeg) - static_cast<double>(angleTolDeg);
    options.standardAngleMaxDeg =
        static_cast<double>(recipe.angleDeg) + static_cast<double>(angleTolDeg);
    options.standardLengthMin =
        static_cast<double>(recipe.lengthMm) - static_cast<double>(lengthTolMm);
    options.standardLengthMax =
        static_cast<double>(recipe.lengthMm) + static_cast<double>(lengthTolMm);
    return options;
}

BevelInspectionResult runBevelMeasurement(
    const scan_tracking::mech_eye::PointCloudFrame& cloud,
    const scan_tracking::common::BevelRecipe& recipe,
    float angleTolDeg,
    float lengthTolMm)
{
    BevelInspectionResult result;

    auto pclCloud = toPclPointCloud(cloud);
    if (pclCloud->empty()) {
        result.message = QStringLiteral("坡口测量缺少有效输入点云。");
        return result;
    }

    const QString configPath = resolveBevelConfigPath();
    if (!QFileInfo::exists(configPath)) {
        result.message = QStringLiteral("坡口测量配置文件不存在：%1").arg(configPath);
        return result;
    }

    const QString templateDir = resolveBevelTemplateDir();
    result.invoked = true;

    try {
        std::lock_guard<std::mutex> pclGuard(
            scan_tracking::mech_eye::pointCloudAlgorithmMutex());

        const std::string configPathUtf8 = configPath.toLocal8Bit().toStdString();
        const std::string templateDirUtf8 = templateDir.toLocal8Bit().toStdString();
        // V1.2：算法不再接受 BevelSolveOptions，公差/合格判定上移至 IPC 侧。
        const BevelSolveOptions solveOptions =
            buildBevelSolveOptions(recipe, angleTolDeg, lengthTolMm);

        const ::bevel::BevelMeasurementResult algorithmResult =
            templateDir.isEmpty()
                ? ::bevel::solveBevelFromRawCloud(
                      pclCloud, configPathUtf8, std::string())
                : ::bevel::solveBevelFromRawCloud(
                      pclCloud, configPathUtf8, templateDirUtf8);

        result.ok = algorithmResult.ok;
        result.bevelType = recipe.bevelType;  // V1.2 不再返回类型，取自配方
        result.angleDeg = static_cast<float>(algorithmResult.angleDeg);
        result.lengthMm = static_cast<float>(algorithmResult.length);
        result.icpFitness = static_cast<float>(algorithmResult.icpFitness);

        // qualityCode 由 IPC 按公差判定：0=合格，1=角度超差，2=长度超差，3=均超差。
        // 算法失败（ok=false）时保留 10000 哨兵，由上层 ok 判定短路。
        if (!result.ok) {
            result.qualityCode = 10000;
        } else {
            const bool angleOk =
                result.angleDeg >= static_cast<float>(solveOptions.standardAngleMinDeg) &&
                result.angleDeg <= static_cast<float>(solveOptions.standardAngleMaxDeg);
            const bool lengthOk =
                result.lengthMm >= static_cast<float>(solveOptions.standardLengthMin) &&
                result.lengthMm <= static_cast<float>(solveOptions.standardLengthMax);
            result.qualityCode = (angleOk ? 0 : 1) | (lengthOk ? 0 : 2);
        }

        result.message = algorithmResult.message.empty()
            ? QString()
            : QString::fromLocal8Bit(algorithmResult.message.c_str());

        if (result.ok && result.message.isEmpty()) {
            result.message = QStringLiteral(
                "坡口测量完成：angle=%1 deg, length=%2 mm, bevelType=%3, qualityCode=%4")
                                 .arg(result.angleDeg, 0, 'f', 3)
                                 .arg(result.lengthMm, 0, 'f', 3)
                                 .arg(result.bevelType)
                                 .arg(result.qualityCode);
        }
    } catch (const std::exception& ex) {
        result.ok = false;
        result.message = QStringLiteral("坡口测量抛出异常：%1")
                             .arg(QString::fromLocal8Bit(ex.what()));
    } catch (...) {
        result.ok = false;
        result.message = QStringLiteral("坡口测量抛出未知异常。");
    }

    return result;
}

BevelInspectionResult runBevelMeasurement(
    const scan_tracking::mech_eye::PointCloudFrame& cloud)
{
    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    if (configManager == nullptr || !configManager->hasActiveBevelRecipe()) {
        BevelInspectionResult result;
        result.message = QStringLiteral("坡口测量缺少有效工艺配方。");
        return result;
    }

    const scan_tracking::common::BevelRecipe recipe = configManager->bevelRecipe();
    const scan_tracking::common::BevelConfig& bevelConfig = configManager->bevelConfig();
    return runBevelMeasurement(
        cloud, recipe, bevelConfig.angleTolDeg, bevelConfig.lengthTolMm);
}

}  // namespace scan_tracking::vision::bevel
