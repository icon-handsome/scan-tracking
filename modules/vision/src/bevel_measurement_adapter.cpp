#include "scan_tracking/vision/bevel_measurement_adapter.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <mutex>
#include <vector>

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtCore/QLoggingCategory>

#include "BevelMeasurement.h"
#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/mech_eye/point_cloud_io.h"
#include "scan_tracking/mech_eye/point_cloud_processor.h"

namespace scan_tracking::vision::bevel {

Q_LOGGING_CATEGORY(LOG_BEVEL_ADAPTER, "vision.bevel")

QString resolveBevelConfigPath();
QString resolveBevelTemplateDir();

BevelSolveOptions buildBevelSolveOptions(
    const scan_tracking::common::BevelRecipe& recipe,
    float angleTolDeg,
    float lengthTolMm);

namespace {

constexpr int kBevelOfflineReplayMaxInputPoints = 200000;

QString localPathFromEnv(const char* name)
{
    const char* value = std::getenv(name);
    if (value == nullptr || value[0] == '\0') {
        return {};
    }
    return QString::fromLocal8Bit(value);
}

std::vector<float> limitXyzPointCountByStride(
    const std::vector<float>& xyz,
    int maxPointCount)
{
    std::vector<float> limited;
    if (xyz.size() < 3 || (xyz.size() % 3) != 0 || maxPointCount <= 0) {
        return limited;
    }

    const int pointCount = static_cast<int>(xyz.size() / 3);
    if (pointCount <= maxPointCount) {
        return xyz;
    }

    const int stride = std::max(1, (pointCount + maxPointCount - 1) / maxPointCount);
    limited.reserve(static_cast<std::size_t>(maxPointCount) * 3);
    for (int index = 0; index < pointCount; index += stride) {
        const std::size_t base = static_cast<std::size_t>(index * 3);
        limited.push_back(xyz[base]);
        limited.push_back(xyz[base + 1]);
        limited.push_back(xyz[base + 2]);
    }

    return limited;
}

QString defaultBevelRootDirectory()
{
    return QCoreApplication::applicationDirPath() + QStringLiteral("/bevel");
}

/// Po_Kou 配置内路径（如 data/templates/...）均相对 bevel 资产根目录解析。
QString normalizeBevelAssetRoot(const QString& path)
{
    const QFileInfo info(path);
    if (!info.exists()) {
        return path;
    }

    QString root = info.isDir() ? info.absoluteFilePath() : info.dir().absolutePath();
    const QString normalized = QDir::fromNativeSeparators(root);
    if (normalized.endsWith(QStringLiteral("/data/templates"))) {
        QDir dir(root);
        if (dir.cdUp() && dir.cdUp()) {
            return dir.absolutePath();
        }
    }
    if (info.fileName() == QStringLiteral("data")
        && QDir(root).exists(QStringLiteral("templates"))) {
        QDir dir(root);
        if (dir.cdUp()) {
            return dir.absolutePath();
        }
    }
    return root;
}

QString describeBevelQualityCode(int qualityCode)
{
    switch (qualityCode) {
    case 0:
        return QStringLiteral("合格");
    case 1:
        return QStringLiteral("角度超差");
    case 2:
        return QStringLiteral("长度超差");
    case 3:
        return QStringLiteral("角度与长度均超差");
    case 10000:
        return QStringLiteral("算法失败");
    default:
        return QStringLiteral("未知(%1)").arg(qualityCode);
    }
}

QString formatBevelInspectionSummary(const BevelInspectionResult& result)
{
    return QStringLiteral(
        "坡口测量完成：angle=%1 deg, length=%2 mm, bevelType=%3, icpFitness=%4, "
        "qualityCode=%5（%6）")
        .arg(result.angleDeg, 0, 'f', 3)
        .arg(result.lengthMm, 0, 'f', 3)
        .arg(result.bevelType)
        .arg(result.icpFitness, 0, 'f', 6)
        .arg(result.qualityCode)
        .arg(describeBevelQualityCode(result.qualityCode));
}

void logBevelInspectionResult(const BevelInspectionResult& result)
{
    qInfo(LOG_BEVEL_ADAPTER).noquote()
        << QStringLiteral("[Bevel] 算法结果")
        << QStringLiteral(" ok=") << result.ok
        << QStringLiteral(" bevelType=") << result.bevelType
        << QStringLiteral(" angleDeg=") << result.angleDeg
        << QStringLiteral(" lengthMm=") << result.lengthMm
        << QStringLiteral(" icpFitness=") << result.icpFitness
        << QStringLiteral(" qualityCode=") << result.qualityCode
        << QStringLiteral(" quality=") << describeBevelQualityCode(result.qualityCode)
        << QStringLiteral(" message=") << result.message;
}

QString readBevelTemplateRelativePath(const QString& configPath)
{
    QFile file(configPath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return {};
    }

    while (!file.atEnd()) {
        QString line = QString::fromUtf8(file.readLine()).trimmed();
        const int commentIndex = line.indexOf(QLatin1Char('#'));
        if (commentIndex >= 0) {
            line = line.left(commentIndex).trimmed();
        }
        if (!line.isEmpty()) {
            return line;
        }
    }

    return {};
}

QString resolveBevelTemplateFilePath(const QString& configPath, const QString& templateDir)
{
    const QString relativePath = readBevelTemplateRelativePath(configPath);
    if (relativePath.isEmpty()) {
        return {};
    }

    QFileInfo relativeInfo(relativePath);
    if (relativeInfo.isAbsolute()) {
        return relativeInfo.absoluteFilePath();
    }

    if (!templateDir.isEmpty()) {
        return QDir(templateDir).filePath(relativePath);
    }

    return QFileInfo(configPath).dir().filePath(relativePath);
}

void logBevelAlgorithmAssetsBeforeRun(const QString& configPath, const QString& templateDir)
{
    const QString templatePath = resolveBevelTemplateFilePath(configPath, templateDir);
    qInfo(LOG_BEVEL_ADAPTER).noquote()
        << QStringLiteral("[Bevel] 算法资源")
        << QStringLiteral(" config=") << configPath
        << QStringLiteral(" configExists=") << QFileInfo::exists(configPath)
        << QStringLiteral(" templateDir=") << (templateDir.isEmpty() ? QStringLiteral("<empty>") : templateDir)
        << QStringLiteral(" template=") << templatePath
        << QStringLiteral(" templateExists=") << (!templatePath.isEmpty() && QFileInfo::exists(templatePath));
}

BevelInspectionResult mapAlgorithmResult(
    const ::bevel::BevelMeasurementResult& algorithmResult,
    const scan_tracking::common::BevelRecipe& recipe,
    float angleTolDeg,
    float lengthTolMm)
{
    BevelInspectionResult result;
    result.invoked = true;

    const BevelSolveOptions solveOptions =
        buildBevelSolveOptions(recipe, angleTolDeg, lengthTolMm);

    result.ok = algorithmResult.ok;
    result.bevelType = recipe.bevelType;
    result.angleDeg = static_cast<float>(algorithmResult.angleDeg);
    result.lengthMm = static_cast<float>(algorithmResult.length);
    result.icpFitness = static_cast<float>(algorithmResult.icpFitness);

    if (!result.ok) {
        result.qualityCode = 10000;
        result.message = algorithmResult.message.empty()
            ? QStringLiteral("坡口测量算法失败。")
            : QString::fromLocal8Bit(algorithmResult.message.c_str());
    } else {
        const bool angleOk =
            result.angleDeg >= static_cast<float>(solveOptions.standardAngleMinDeg) &&
            result.angleDeg <= static_cast<float>(solveOptions.standardAngleMaxDeg);
        const bool lengthOk =
            result.lengthMm >= static_cast<float>(solveOptions.standardLengthMin) &&
            result.lengthMm <= static_cast<float>(solveOptions.standardLengthMax);
        result.qualityCode = (angleOk ? 0 : 1) | (lengthOk ? 0 : 2);
        result.message = formatBevelInspectionSummary(result);
    }

    return result;
}

BevelInspectionResult invokeBevelAlgorithmFromXyzBuffer(
    const std::vector<float>& xyz,
    const scan_tracking::common::BevelRecipe& recipe,
    float angleTolDeg,
    float lengthTolMm)
{
    BevelInspectionResult result;
    if (xyz.size() < 3 || (xyz.size() % 3) != 0) {
        result.message = QStringLiteral("坡口测量点云缓冲无效。");
        return result;
    }

    const QString configPath = resolveBevelConfigPath();
    if (!QFileInfo::exists(configPath)) {
        result.message = QStringLiteral("坡口测量配置文件不存在：%1").arg(configPath);
        return result;
    }

    const QString templateDir = resolveBevelTemplateDir();

    try {
        std::lock_guard<std::mutex> pclGuard(
            scan_tracking::mech_eye::pointCloudAlgorithmMutex());

        const std::string configPathUtf8 = configPath.toLocal8Bit().toStdString();
        const std::string templateDirUtf8 = templateDir.toLocal8Bit().toStdString();

        // 仅传递 float xyz 缓冲；PCL 点云在 po_kou_ce_liang 模块内分配/释放。
        logBevelAlgorithmAssetsBeforeRun(configPath, templateDir);
        qInfo(LOG_BEVEL_ADAPTER).noquote()
            << QStringLiteral("[Bevel] 开始算法测量 floatCount=") << xyz.size()
            << QStringLiteral(" pointCount=") << (xyz.size() / 3);

        const ::bevel::BevelMeasurementResult algorithmResult =
            ::bevel::solveBevelFromXyzBuffer(
                xyz.data(), xyz.size(), configPathUtf8, templateDirUtf8);

        const BevelInspectionResult mapped =
            mapAlgorithmResult(algorithmResult, recipe, angleTolDeg, lengthTolMm);
        logBevelInspectionResult(mapped);
        return mapped;
    } catch (const std::exception& ex) {
        result.ok = false;
        result.invoked = true;
        result.message = QStringLiteral("坡口测量抛出异常：%1")
                             .arg(QString::fromUtf8(ex.what()));
    } catch (...) {
        result.ok = false;
        result.invoked = true;
        result.message = QStringLiteral("坡口测量抛出未知异常。");
    }

    return result;
}

BevelInspectionResult invokeBevelAlgorithmFromCloudFilePath(
    const QString& algorithmCloudPath,
    const scan_tracking::common::BevelRecipe& recipe,
    float angleTolDeg,
    float lengthTolMm)
{
    BevelInspectionResult result;

    const QString configPath = resolveBevelConfigPath();
    if (!QFileInfo::exists(configPath)) {
        result.message = QStringLiteral("坡口测量配置文件不存在：%1").arg(configPath);
        return result;
    }

    const QString templateDir = resolveBevelTemplateDir();

    try {
        std::lock_guard<std::mutex> pclGuard(
            scan_tracking::mech_eye::pointCloudAlgorithmMutex());

        const std::string cloudPathUtf8 = algorithmCloudPath.toLocal8Bit().toStdString();
        const std::string configPathUtf8 = configPath.toLocal8Bit().toStdString();
        const std::string templateDirUtf8 = templateDir.toLocal8Bit().toStdString();

        // TXT 等文本点云仍由 Po_Kou 按文件路径加载（PCL 仅在 po_kou 模块内使用）。
        logBevelAlgorithmAssetsBeforeRun(configPath, templateDir);
        qInfo(LOG_BEVEL_ADAPTER).noquote()
            << QStringLiteral("[Bevel] 开始算法测量 cloud=") << algorithmCloudPath;

        const ::bevel::BevelMeasurementResult algorithmResult =
            ::bevel::solveBevelFromPointCloudFile(
                cloudPathUtf8, configPathUtf8, templateDirUtf8);

        const BevelInspectionResult mapped =
            mapAlgorithmResult(algorithmResult, recipe, angleTolDeg, lengthTolMm);
        logBevelInspectionResult(mapped);
        return mapped;
    } catch (const std::exception& ex) {
        result.ok = false;
        result.invoked = true;
        result.message = QStringLiteral("坡口测量抛出异常：%1")
                             .arg(QString::fromUtf8(ex.what()));
    } catch (...) {
        result.ok = false;
        result.invoked = true;
        result.message = QStringLiteral("坡口测量抛出未知异常。");
    }

    return result;
}

std::vector<float> collectFiniteXyz(const scan_tracking::mech_eye::PointCloudFrame& frame)
{
    std::vector<float> xyz;
    if (!frame.isValid() || frame.pointsXYZ == nullptr || frame.pointCount <= 0) {
        return xyz;
    }

    const auto& points = *frame.pointsXYZ;
    const int availablePointCount = static_cast<int>(points.size() / 3);
    const int pointCount = std::min(frame.pointCount, availablePointCount);
    if (pointCount <= 0) {
        return xyz;
    }

    xyz.reserve(static_cast<std::size_t>(pointCount) * 3);
    for (int index = 0; index < pointCount; ++index) {
        const auto base = static_cast<std::size_t>(index * 3);
        const float x = points[base];
        const float y = points[base + 1];
        const float z = points[base + 2];
        if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
            xyz.push_back(x);
            xyz.push_back(y);
            xyz.push_back(z);
        }
    }

    return xyz;
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
        const QDir envTemplateRoot(envRoot);
        if (envTemplateRoot.exists()) {
            return normalizeBevelAssetRoot(envTemplateRoot.absolutePath());
        }
    }

    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    if (configManager != nullptr) {
        const QString configured = configManager->bevelConfig().templateDir.trimmed();
        if (!configured.isEmpty()) {
            QFileInfo configuredInfo(configured);
            if (configuredInfo.isAbsolute() && configuredInfo.exists()) {
                return normalizeBevelAssetRoot(configuredInfo.absoluteFilePath());
            }
            const QFileInfo relativeToExe(
                QDir(QCoreApplication::applicationDirPath()).filePath(configured));
            if (relativeToExe.exists()) {
                return normalizeBevelAssetRoot(relativeToExe.absoluteFilePath());
            }
        }
    }

    const QDir defaultTemplateRoot(defaultBevelRootDirectory());
    return defaultTemplateRoot.exists()
        ? normalizeBevelAssetRoot(defaultTemplateRoot.absolutePath())
        : QString();
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
    float lengthTolMm,
    int maxInputPointCount)
{
    BevelInspectionResult result;
    if (!cloud.isValid() || cloud.pointCount <= 0) {
        result.message = QStringLiteral("坡口测量缺少有效输入点云。");
        return result;
    }

    std::vector<float> xyz = collectFiniteXyz(cloud);
    if (xyz.empty()) {
        result.message = QStringLiteral("坡口测量点云无有效 XYZ 点。");
        return result;
    }

    if (maxInputPointCount > 0) {
        xyz = limitXyzPointCountByStride(xyz, maxInputPointCount);
    }
    return invokeBevelAlgorithmFromXyzBuffer(xyz, recipe, angleTolDeg, lengthTolMm);
}

BevelInspectionResult runBevelMeasurementFromPointCloudFile(
    const QString& cloudPath,
    const scan_tracking::common::BevelRecipe& recipe,
    float angleTolDeg,
    float lengthTolMm)
{
    BevelInspectionResult result;

    const QFileInfo sourceInfo(cloudPath.trimmed());
    if (!sourceInfo.exists() || !sourceInfo.isFile()) {
        result.message = QStringLiteral("坡口测量点云文件不存在：%1").arg(cloudPath);
        return result;
    }

    const QString suffix = sourceInfo.suffix().toLower();
    if (suffix == QStringLiteral("ply") || suffix == QStringLiteral("pcd")) {
        std::vector<float> xyz;
        const bool loaded = suffix == QStringLiteral("ply")
            ? scan_tracking::mech_eye::loadPointCloudXyzFromPly(
                  sourceInfo.absoluteFilePath(),
                  &xyz,
                  kBevelOfflineReplayMaxInputPoints)
            : scan_tracking::mech_eye::loadPointCloudXyzFromPcd(
                  sourceInfo.absoluteFilePath(),
                  &xyz,
                  kBevelOfflineReplayMaxInputPoints);
        if (!loaded || xyz.empty()) {
            result.message = QStringLiteral("坡口测量无法解析点云：%1").arg(cloudPath);
            return result;
        }

        return invokeBevelAlgorithmFromXyzBuffer(xyz, recipe, angleTolDeg, lengthTolMm);
    }

    return invokeBevelAlgorithmFromCloudFilePath(
        sourceInfo.absoluteFilePath(), recipe, angleTolDeg, lengthTolMm);
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
