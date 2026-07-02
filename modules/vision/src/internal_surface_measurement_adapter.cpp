#include "scan_tracking/vision/internal_surface_measurement_adapter.h"

#include <cmath>
#include <cstdlib>
#include <exception>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

#include <QtCore/QCoreApplication>
#include <QtCore/QDateTime>
#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtCore/QLoggingCategory>

#include "MeasurementAlgorithm.h"
#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/mech_eye/point_cloud_io.h"
#include "scan_tracking/mech_eye/point_cloud_processor.h"

Q_LOGGING_CATEGORY(LOG_INTERNAL_SURFACE_ADAPTER, "vision.internal_surface")

namespace scan_tracking::vision::internal_surface {

QString resolveInternalSurfaceConfigPath();

namespace {

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

QString buildTempScanCloudPath()
{
    const QDir root(QCoreApplication::applicationDirPath());
    const QString tempDirPath = root.filePath(QStringLiteral("internal_surface/tmp"));
    QDir().mkpath(tempDirPath);

    const QString stamp = QDateTime::currentDateTime()
        .toString(QStringLiteral("yyyyMMdd_HHmmss_zzz"));
    return QDir(tempDirPath).filePath(
        QStringLiteral("internal_surface_input_%1.pcd").arg(stamp));
}

bool isPositiveFinite(double value)
{
    return std::isfinite(value) && value > 0.0;
}

class ScopedCurrentDir
{
public:
    explicit ScopedCurrentDir(const QString& dir)
    {
        previous_ = QDir::currentPath();
        QDir::setCurrent(dir);
    }

    ~ScopedCurrentDir()
    {
        QDir::setCurrent(previous_);
    }

private:
    QString previous_;
};

QString resolveInternalSurfaceConfigPathForRun(bool useOfflineReplayAlgorithmConfig)
{
    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    if (useOfflineReplayAlgorithmConfig && configManager != nullptr) {
        const QString offlineConfigPath =
            configManager->internalSurfaceConfig().offlineReplayAlgorithmConfigPath.trimmed();
        if (!offlineConfigPath.isEmpty()) {
            const QString resolved = resolveConfiguredPath(offlineConfigPath);
            if (QFileInfo::exists(resolved)) {
                return resolved;
            }
            qWarning(LOG_INTERNAL_SURFACE_ADAPTER).noquote()
                << QStringLiteral("[InternalSurface] 离线算法配置不存在，回退默认 configPath：")
                << resolved;
        }
    }
    return resolveInternalSurfaceConfigPath();
}

InternalSurfaceInspectionResult runMeasurementWithPreparedScanCloud(
    const QString& scanCloudPath,
    bool useOfflineReplayAlgorithmConfig)
{
    InternalSurfaceInspectionResult result;

    const QString configPath =
        resolveInternalSurfaceConfigPathForRun(useOfflineReplayAlgorithmConfig);
    if (!QFileInfo::exists(configPath)) {
        result.message = QStringLiteral("内表面测量配置文件不存在：%1").arg(configPath);
        return result;
    }

    if (!QFileInfo::exists(scanCloudPath)) {
        result.message = QStringLiteral("内表面测量扫描点云不存在：%1").arg(scanCloudPath);
        return result;
    }

    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    const auto surfaceConfig = configManager != nullptr
        ? configManager->internalSurfaceConfig()
        : scan_tracking::common::InternalSurfaceConfig{};

    result.invoked = true;

    qInfo(LOG_INTERNAL_SURFACE_ADAPTER).noquote()
        << QStringLiteral("[InternalSurface] 开始测量 config=") << configPath
        << QStringLiteral(" scan=") << scanCloudPath
        << QStringLiteral(" templateType=") << surfaceConfig.templateType
        << QStringLiteral("（大点云 ICP/网格化可能需数分钟，非卡死）");

    try {
        std::atomic<bool> algorithmDone{false};
        std::thread heartbeatThread([&algorithmDone]() {
            int elapsedSec = 0;
            while (!algorithmDone.load(std::memory_order_acquire)) {
                std::this_thread::sleep_for(std::chrono::seconds(15));
                if (algorithmDone.load(std::memory_order_acquire)) {
                    break;
                }
                elapsedSec += 15;
                qInfo(LOG_INTERNAL_SURFACE_ADAPTER).noquote()
                    << QStringLiteral("[InternalSurface] 算法仍在运行，已耗时约")
                    << elapsedSec << QStringLiteral("s（ICP/法向/三角网格阶段）");
            }
        });

        std::lock_guard<std::mutex> pclGuard(
            scan_tracking::mech_eye::pointCloudAlgorithmMutex());

        MeasurementInput input;
        input.configPath = configPath.toLocal8Bit().toStdString();
        input.scanCloudPath =
            QFileInfo(scanCloudPath).absoluteFilePath().toLocal8Bit().toStdString();
        input.templateType = surfaceConfig.templateType;

        const ScopedCurrentDir algorithmCwd(QFileInfo(configPath).absolutePath());

        MeasurementResult algoResult;
        const bool algoOk = RunMeasurement(input, &algoResult);
        algorithmDone.store(true, std::memory_order_release);
        if (heartbeatThread.joinable()) {
            heartbeatThread.join();
        }

        qInfo(LOG_INTERNAL_SURFACE_ADAPTER).noquote()
            << QStringLiteral("[InternalSurface] RunMeasurement 已返回 ok=") << algoOk;

        if (!algoOk) {
            result.message = QStringLiteral("内表面测量算法失败：%1")
                                 .arg(QString::fromLocal8Bit(algoResult.message));
            qWarning(LOG_INTERNAL_SURFACE_ADAPTER).noquote() << result.message;
            return result;
        }

        result.headDepthMm = algoResult.lowestDistanceToPlaneMm;
        result.volumeLiter = algoResult.volumeLiter;
        result.headVolumeM3 = algoResult.volumeLiter / 1000.0;
        result.filteredPointCount = algoResult.filteredPointCount;
        result.downsampledPointCount = algoResult.downsampledPointCount;
        result.meshVertexCount = algoResult.meshVertexCount;
        result.meshFaceCount = algoResult.meshFaceCount;

        const bool depthOk =
            isPositiveFinite(result.headDepthMm)
            && result.headDepthMm >= surfaceConfig.minDepthMm;
        const bool volumeOk =
            isPositiveFinite(result.headVolumeM3)
            && result.headVolumeM3 >= surfaceConfig.minVolumeM3;
        const bool meshOk = result.meshVertexCount > 0 && result.meshFaceCount > 0;
        result.ok = depthOk && volumeOk && meshOk;

        if (result.ok) {
            result.message = QStringLiteral(
                "内表面测量通过：depth=%1 mm, volume=%2 m3。")
                                 .arg(result.headDepthMm, 0, 'f', 3)
                                 .arg(result.headVolumeM3, 0, 'f', 6);
        } else {
            result.message = QStringLiteral(
                "内表面测量未通过：depth=%1 mm, volume=%2 m3, mesh=%3/%4。")
                                 .arg(result.headDepthMm, 0, 'f', 3)
                                 .arg(result.headVolumeM3, 0, 'f', 6)
                                 .arg(result.meshVertexCount)
                                 .arg(result.meshFaceCount);
        }
    } catch (const std::exception& ex) {
        result.ok = false;
        result.message = QStringLiteral("内表面测量抛出异常：%1")
                             .arg(QString::fromUtf8(ex.what()));
    } catch (...) {
        result.ok = false;
        result.message = QStringLiteral("内表面测量抛出未知异常。");
    }

    return result;
}

}  // namespace

QString resolveInternalSurfaceConfigPath()
{
    const QString envRoot = localPathFromEnv("SCAN_TRACKING_INTERNAL_SURFACE_CONFIG_DIR");
    if (!envRoot.isEmpty()) {
        const QFileInfo envConfig(QDir(envRoot).filePath(QStringLiteral("algorithm_config.json")));
        if (envConfig.exists()) {
            return envConfig.absoluteFilePath();
        }
    }

    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    const QString configured = configManager != nullptr
        ? configManager->internalSurfaceConfig().configPath
        : QStringLiteral("internal_surface/config/algorithm_config.json");

    const QString resolved = resolveConfiguredPath(configured.trimmed());
    if (QFileInfo::exists(resolved)) {
        return resolved;
    }

    const QFileInfo fallback(QDir(QCoreApplication::applicationDirPath())
                                 .filePath(QStringLiteral("internal_surface/config/algorithm_config.json")));
    return fallback.absoluteFilePath();
}

InternalSurfaceInspectionResult runInternalSurfaceMeasurement(
    const scan_tracking::mech_eye::PointCloudFrame& cloud)
{
    InternalSurfaceInspectionResult result;

    if (!cloud.isValid()) {
        result.message = QStringLiteral("内表面测量缺少有效输入点云。");
        return result;
    }

    const QString scanCloudPath = buildTempScanCloudPath();
    {
        std::lock_guard<std::mutex> pclGuard(
            scan_tracking::mech_eye::pointCloudAlgorithmMutex());
        if (!scan_tracking::mech_eye::savePointCloudFrameToPcd(cloud, scanCloudPath)) {
            result.message = QStringLiteral("内表面测量临时点云保存失败：%1").arg(scanCloudPath);
            return result;
        }
    }

    return runMeasurementWithPreparedScanCloud(scanCloudPath, false);
}

InternalSurfaceInspectionResult runInternalSurfaceMeasurementFromScanFile(
    const QString& scanCloudPath)
{
    InternalSurfaceInspectionResult result;

    const QString resolved = resolveConfiguredPath(scanCloudPath.trimmed());
    if (resolved.trimmed().isEmpty() || !QFileInfo::exists(resolved)) {
        result.message = QStringLiteral("内表面测量扫描点云不存在：%1").arg(scanCloudPath);
        return result;
    }

    const QString suffix = QFileInfo(resolved).suffix().toLower();
    if (suffix == QStringLiteral("pcd") || suffix == QStringLiteral("ply")) {
        return runMeasurementWithPreparedScanCloud(resolved, true);
    }

    if (suffix != QStringLiteral("txt")) {
        result.message = QStringLiteral("内表面测量不支持的点云格式：%1").arg(resolved);
        return result;
    }

    const QString convertedPcdPath = buildTempScanCloudPath();
    {
        std::lock_guard<std::mutex> pclGuard(
            scan_tracking::mech_eye::pointCloudAlgorithmMutex());
        if (!scan_tracking::mech_eye::convertTxtPointCloudToPcd(resolved, convertedPcdPath)) {
            result.message = QStringLiteral("内表面测量 TXT 转 PCD 失败：%1").arg(resolved);
            return result;
        }
    }

    return runMeasurementWithPreparedScanCloud(convertedPcdPath, true);
}

InternalSurfaceInspectionResult runInternalSurfaceMeasurementFromSegmentFrames(
    const QList<scan_tracking::mech_eye::PointCloudFrame>& segmentClouds,
    int sourcePointCount)
{
    InternalSurfaceInspectionResult result;

    if (segmentClouds.isEmpty()) {
        result.message = QStringLiteral("内表面测量缺少有效分段点云。");
        return result;
    }

    const QString scanCloudPath = buildTempScanCloudPath();
    int mergedPointCount = 0;
    {
        std::lock_guard<std::mutex> pclGuard(
            scan_tracking::mech_eye::pointCloudAlgorithmMutex());
        if (!scan_tracking::mech_eye::mergePointCloudFramesToPcd(
                segmentClouds, scanCloudPath, &mergedPointCount)) {
            result.message = QStringLiteral("内表面测量分段合并 PCD 失败。");
            return result;
        }
    }

    qInfo(LOG_INTERNAL_SURFACE_ADAPTER).noquote()
        << QStringLiteral("[InternalSurface] 分段已合并为 PCD（与 demo 单文件输入一致）")
        << QStringLiteral(" segmentCount=") << segmentClouds.size()
        << QStringLiteral(" sourcePointCount=")
        << (sourcePointCount > 0 ? sourcePointCount : mergedPointCount)
        << QStringLiteral(" validPoints=") << mergedPointCount
        << QStringLiteral(" path=") << scanCloudPath;

    return runMeasurementWithPreparedScanCloud(scanCloudPath, false);
}

}  // namespace scan_tracking::vision::internal_surface
