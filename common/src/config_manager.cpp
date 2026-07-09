#include "scan_tracking/common/config_manager.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLoggingCategory>
#include <QStringList>

#include "scan_tracking/common/logger.h"

namespace scan_tracking {
namespace common {

Q_LOGGING_CATEGORY(LOG_CONFIG, "config")

ConfigManager* ConfigManager::s_instance = nullptr;

namespace {

void configureUtf8IniSettings(QSettings& settings)
{
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    settings.setIniCodec("UTF-8");
#else
    Q_UNUSED(settings);
#endif
}

QString projectRootConfigPath()
{
    const QString exeDirConfig =
        QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("config.ini"));

    // Dev：从 build/<preset>/ 运行时优先读仓库根 config.ini，避免 build 目录陈旧副本覆盖联调配置。
    QDir sourceDir(QCoreApplication::applicationDirPath());
    if (sourceDir.cdUp() && sourceDir.cdUp()) {
        const QString sourceTreeConfig = sourceDir.filePath(QStringLiteral("config.ini"));
        if (QFileInfo::exists(sourceTreeConfig)) {
            return QDir::cleanPath(sourceTreeConfig);
        }
    }

    if (QFileInfo::exists(exeDirConfig)) {
        return QDir::cleanPath(exeDirConfig);
    }
    return QDir::cleanPath(exeDirConfig);
}

QString resolveConfigRelativePath(const QString& rawPath, const QString& configFilePath)
{
    const QString trimmed = rawPath.trimmed();
    if (trimmed.isEmpty()) {
        return QString();
    }

    const QFileInfo pathInfo(trimmed);
    if (pathInfo.isAbsolute()) {
        return QDir::cleanPath(trimmed);
    }

    const QString configDirPath = QFileInfo(configFilePath).absoluteDir().filePath(trimmed);
    if (QFileInfo::exists(configDirPath)) {
        return QDir::cleanPath(configDirPath);
    }

    const QString exeDirPath = QDir(QCoreApplication::applicationDirPath()).filePath(trimmed);
    if (QFileInfo::exists(exeDirPath)) {
        return QDir::cleanPath(exeDirPath);
    }

    return QDir::cleanPath(configDirPath);
}

/**
 * @brief 获取扫描路径配置文件的路径
 * 
 * Stage 1: 工位 profile 显式配置优先；未配置时保持旧 scan_paths_config.json fallback。
 * 
 * @return 扫描路径配置文件的完整路径
 */
QString scanPathsConfigPath(const StationProfile& stationProfile, const QString& configFilePath)
{
    if (!stationProfile.scanPathsConfigPath.trimmed().isEmpty()) {
        return resolveConfigRelativePath(stationProfile.scanPathsConfigPath, configFilePath);
    }

    const QString exeDirPath =
        QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("scan_paths_config.json"));
    if (QFileInfo::exists(exeDirPath)) {
        return exeDirPath;
    }

    QDir rootDir(QCoreApplication::applicationDirPath());
    if (rootDir.cdUp() && rootDir.cdUp() && rootDir.cdUp()) {
        QString rootPath = rootDir.filePath(QStringLiteral("scan_paths_config.json"));
        if (QFileInfo::exists(rootPath)) {
            return rootPath;
        }
    }

    return exeDirPath;
}

void applyStationSettings(QSettings& settings, StationProfile& profile, QString* profileIni)
{
    if (settings.contains(QStringLiteral("stationId"))) {
        profile.stationId = stationIdFromInt(settings.value(QStringLiteral("stationId"), 1).toInt());
    }
    if (settings.contains(QStringLiteral("stationName"))) {
        profile.stationName = settings.value(QStringLiteral("stationName"), profile.stationName).toString();
    }
    if (settings.contains(QStringLiteral("scanPathsConfigPath"))) {
        profile.scanPathsConfigPath =
            settings.value(QStringLiteral("scanPathsConfigPath"), profile.scanPathsConfigPath).toString().trimmed();
    }
    if (settings.contains(QStringLiteral("defaultWorkMode"))) {
        bool ok = false;
        const QString raw = settings.value(QStringLiteral("defaultWorkMode")).toString();
        profile.defaultWorkMode = workModeIdFromString(raw, &ok);
        if (!ok) {
            qWarning(LOG_CONFIG).noquote()
                << QStringLiteral("[Station] 未知 defaultWorkMode=") << raw
                << QStringLiteral("，已回退 Unknown");
        }
    }
    if (settings.contains(QStringLiteral("enableLoadGrasp"))) {
        profile.enableLoadGrasp = settings.value(QStringLiteral("enableLoadGrasp"), profile.enableLoadGrasp).toBool();
    }
    if (settings.contains(QStringLiteral("enableUnloadCalc"))) {
        profile.enableUnloadCalc = settings.value(QStringLiteral("enableUnloadCalc"), profile.enableUnloadCalc).toBool();
    }
    if (settings.contains(QStringLiteral("enablePoseCheck"))) {
        profile.enablePoseCheck = settings.value(QStringLiteral("enablePoseCheck"), profile.enablePoseCheck).toBool();
    }
    if (settings.contains(QStringLiteral("enableTelescopicScan"))) {
        profile.enableTelescopicScan =
            settings.value(QStringLiteral("enableTelescopicScan"), profile.enableTelescopicScan).toBool();
    }
    if (settings.contains(QStringLiteral("enableHoistAssist"))) {
        profile.enableHoistAssist = settings.value(QStringLiteral("enableHoistAssist"), profile.enableHoistAssist).toBool();
    }
    if (settings.contains(QStringLiteral("enableCollisionMonitor"))) {
        profile.enableCollisionMonitor =
            settings.value(QStringLiteral("enableCollisionMonitor"), profile.enableCollisionMonitor).toBool();
    }
    if (profileIni != nullptr && settings.contains(QStringLiteral("profileIni"))) {
        *profileIni = settings.value(QStringLiteral("profileIni")).toString().trimmed();
    }
}

}  // namespace

void ConfigManager::initialize()
{
    if (!s_instance) {
        s_instance = new ConfigManager();
        qInfo(LOG_CONFIG) << "ConfigManager 已初始化。";
    }
}

void ConfigManager::cleanup()
{
    if (s_instance) {
        delete s_instance;
        s_instance = nullptr;
        qInfo(LOG_CONFIG) << "ConfigManager 已清理。";
    }
}

ConfigManager* ConfigManager::instance()
{
    if (!s_instance) {
        qWarning(LOG_CONFIG) << "ConfigManager::instance() 在 initialize() 之前被调用！";
    }
    return s_instance;
}

ConfigManager::ConfigManager()
{
    const QString configPath = projectRootConfigPath();
    load(configPath);
    
    // 加载扫描路径配置
    const QString scanPathsPath = scanPathsConfigPath(m_stationProfile, configPath);
    loadScanPathsConfig(scanPathsPath);
}

ConfigManager::~ConfigManager() = default;

const AppConfig& ConfigManager::appConfig() const { return m_appConfig; }
const LoggerConfig& ConfigManager::loggerConfig() const { return m_loggerConfig; }
const ModbusConfig& ConfigManager::modbusConfig() const { return m_modbusConfig; }
const CameraConfig& ConfigManager::cameraConfig() const { return m_cameraConfig; }
const VisionConfig& ConfigManager::visionConfig() const { return m_visionConfig; }
const FlowControlConfig& ConfigManager::flowControlConfig() const { return m_flowControlConfig; }
const SegmentCaptureExportConfig& ConfigManager::segmentCaptureExportConfig() const
{
    return m_segmentCaptureExportConfig;
}
const TrackingConfig& ConfigManager::trackingConfig() const { return m_trackingConfig; }
const BevelConfig& ConfigManager::bevelConfig() const { return m_bevelConfig; }

const HoleConfig& ConfigManager::holeConfig() const { return m_holeConfig; }

const ThicknessConfig& ConfigManager::thicknessConfig() const { return m_thicknessConfig; }

const InternalSurfaceConfig& ConfigManager::internalSurfaceConfig() const
{
    return m_internalSurfaceConfig;
}

const SelfCheckConfig& ConfigManager::selfCheckConfig() const
{
    return m_selfCheckConfig;
}

PointCloudSaveFormat pointCloudSaveFormatFromString(const QString& value)
{
    const QString normalized = value.trimmed().toLower();
    if (normalized == QStringLiteral("ply")) {
        return PointCloudSaveFormat::Ply;
    }
    return PointCloudSaveFormat::Pcd;
}

QString pointCloudSaveFormatToString(PointCloudSaveFormat format)
{
    return format == PointCloudSaveFormat::Ply ? QStringLiteral("ply") : QStringLiteral("pcd");
}

QString pointCloudSaveFormatExtension(PointCloudSaveFormat format)
{
    return format == PointCloudSaveFormat::Ply ? QStringLiteral("ply") : QStringLiteral("pcd");
}

InspectionType inspectionTypeFromString(const QString& value)
{
    const QString normalized = value.trimmed().toLower();
    if (normalized == QStringLiteral("hole")
        || normalized == QStringLiteral("opening")
        || normalized == QStringLiteral("cylinder_hole")
        || normalized == QStringLiteral("柱面")
        || normalized == QStringLiteral("开孔")) {
        return InspectionType::Hole;
    }
    if (normalized == QStringLiteral("internal_surface")
        || normalized == QStringLiteral("internal-surface")
        || normalized == QStringLiteral("内表面")) {
        return InspectionType::InternalSurface;
    }
    if (normalized == QStringLiteral("code_read")
        || normalized == QStringLiteral("code")
        || normalized == QStringLiteral("ocr")
        || normalized == QStringLiteral("number")
        || normalized == QStringLiteral("编号")) {
        return InspectionType::CodeRead;
    }
    if (normalized == QStringLiteral("defect")
        || normalized == QStringLiteral("surface_defect")
        || normalized == QStringLiteral("缺陷")) {
        return InspectionType::Defect;
    }
    if (normalized == QStringLiteral("thickness")
        || normalized == QStringLiteral("weld")
        || normalized == QStringLiteral("焊缝")
        || normalized == QStringLiteral("厚度")) {
        return InspectionType::Thickness;
    }
    if (normalized == QStringLiteral("bevel") || normalized == QStringLiteral("坡口")) {
        return InspectionType::Bevel;
    }
    return InspectionType::Bevel;
}

QString inspectionTypeToString(InspectionType type)
{
    switch (type) {
    case InspectionType::Hole:
        return QStringLiteral("hole");
    case InspectionType::InternalSurface:
        return QStringLiteral("internal_surface");
    case InspectionType::CodeRead:
        return QStringLiteral("code_read");
    case InspectionType::Defect:
        return QStringLiteral("defect");
    case InspectionType::Thickness:
        return QStringLiteral("thickness");
    case InspectionType::Bevel:
    default:
        return QStringLiteral("bevel");
    }
}

InspectionType ConfigManager::inspectionTypeForPath(int pathId) const
{
    for (const ScanPathConfig& path : m_scanPathsConfig.scanPaths) {
        if (path.pathId == pathId) {
            return path.inspectionType;
        }
    }
    return InspectionType::Bevel;
}

QString ConfigManager::pathNameForPath(int pathId) const
{
    for (const ScanPathConfig& path : m_scanPathsConfig.scanPaths) {
        if (path.pathId == pathId) {
            return path.pathName.isEmpty() ? QStringLiteral("路径%1").arg(pathId) : path.pathName;
        }
    }
    return QStringLiteral("路径%1").arg(pathId);
}

QString ConfigManager::holeConfigPathForPath(int pathId) const
{
    for (const ScanPathConfig& path : m_scanPathsConfig.scanPaths) {
        if (path.pathId == pathId && !path.holeConfigPath.trimmed().isEmpty()) {
            return path.holeConfigPath.trimmed();
        }
    }
    return m_holeConfig.configPath;
}

QString ConfigManager::thicknessConfigPathForPath(int pathId) const
{
    for (const ScanPathConfig& path : m_scanPathsConfig.scanPaths) {
        if (path.pathId == pathId && !path.thicknessConfigPath.trimmed().isEmpty()) {
            return path.thicknessConfigPath.trimmed();
        }
    }
    return m_thicknessConfig.configPath;
}

int ConfigManager::innerScanSegmentIndexForPath(int pathId) const
{
    for (const ScanPathConfig& path : m_scanPathsConfig.scanPaths) {
        if (path.pathId == pathId) {
            return path.innerScanSegmentIndex;
        }
    }
    return 0;
}

int ConfigManager::outerScanSegmentIndexForPath(int pathId) const
{
    for (const ScanPathConfig& path : m_scanPathsConfig.scanPaths) {
        if (path.pathId == pathId) {
            return path.outerScanSegmentIndex;
        }
    }
    return 0;
}

QVector<BevelRecipePreset> standardBevelRecipePresets()
{
    return {
        BevelRecipePreset{
            0,
            QStringLiteral("45deg_1mm"),
            45.0f,
            1.0f,
        },
        BevelRecipePreset{
            1,
            QStringLiteral("30deg_6mm"),
            30.0f,
            6.0f,
        },
    };
}

BevelRecipe bevelRecipePresetForType(int bevelType)
{
    for (const BevelRecipePreset& preset : standardBevelRecipePresets()) {
        if (preset.bevelType == bevelType) {
            BevelRecipe recipe;
            recipe.active = true;
            recipe.bevelType = preset.bevelType;
            recipe.angleDeg = preset.angleDeg;
            recipe.lengthMm = preset.lengthMm;
            return recipe;
        }
    }
    return BevelRecipe{};
}

void ConfigManager::setBevelRecipe(const BevelRecipe& recipe)
{
    std::lock_guard<std::mutex> lock(m_bevelRecipeMutex);
    m_runtimeBevelRecipe = recipe;
    m_runtimeBevelRecipe.active = true;
    m_runtimeRecipeSet = true;
}

BevelRecipe ConfigManager::bevelRecipe() const
{
    std::lock_guard<std::mutex> lock(m_bevelRecipeMutex);
    if (m_runtimeRecipeSet) {
        return m_runtimeBevelRecipe;
    }
    return m_bevelConfig.defaultRecipe;
}

bool ConfigManager::hasActiveBevelRecipe() const
{
    return bevelRecipe().active;
}

QString ConfigManager::configFilePath() const { return m_configFilePath; }
const HmiConfig& ConfigManager::hmiConfig() const { return m_hmiConfig; }
const LbPoseConfig& ConfigManager::lbPoseConfig() const { return m_lbPoseConfig; }
const LbnPoseConfig& ConfigManager::lbnPoseConfig() const { return m_lbnPoseConfig; }
const PointCloudProcessingConfig& ConfigManager::pointCloudProcessingConfig() const
{
    return m_pointCloudProcessingConfig;
}
const ScanPathsConfig& ConfigManager::scanPathsConfig() const { return m_scanPathsConfig; }
const StationProfile& ConfigManager::stationProfile() const { return m_stationProfile; }

void ConfigManager::writeDefaults(QSettings& settings)
{
    settings.beginGroup("App");
    settings.setValue("version", "0.1.0");
    settings.setValue("environment", "production");
    settings.endGroup();

    settings.beginGroup("Logger");
    settings.setValue("level", 0);
    settings.setValue("rotateDays", 0);  // 保留项，不触发日志删除/覆盖
    settings.endGroup();

    settings.beginGroup("Modbus");
    settings.setValue("host", "127.0.0.1");
    settings.setValue("port", 502);
    settings.setValue("unitId", 3);
    settings.setValue("timeoutMs", 1000);
    settings.setValue("reconnectIntervalMs", 2000);
    settings.endGroup();

    settings.beginGroup("Camera");
    settings.setValue("defaultCamera", "Mech-Eye Nano");
    settings.setValue("scanTimeoutMs", 5000);
    settings.endGroup();

    settings.beginGroup("Station");
    settings.setValue("stationId", 1);
    settings.setValue("stationName", QStringLiteral("第一工位-封头"));
    settings.setValue("scanPathsConfigPath", QStringLiteral("config/scan_paths/station1_default.json"));
    settings.setValue("defaultWorkMode", QStringLiteral("MODE_END_CAP"));
    settings.setValue("profileIni", QStringLiteral("config/station_profiles/station1_endcap.ini"));
    settings.endGroup();

    settings.beginGroup("Vision");
    settings.setValue("mechEyeCameraKey", "Mech-Eye Nano");
    settings.setValue("mechCaptureTimeoutMs", 5000);
    settings.setValue("mechScan3DSingleExposureEnabled", true);
    settings.setValue("mechScan3DSingleExposureMs", 40.0);
    settings.setValue("mechScan3DMultiExposureEnabled", false);
    settings.setValue("mechScan3DMultiExposureSequenceMs", "5,10,20");
    settings.setValue("mechScan2DExposureMs", 0.0);
    settings.setValue("mechScan3DGain", 0.0);
    settings.setValue("mechSaveExposureToDevice", false);
    settings.setValue("hikConnectTimeoutMs", 3000);
    settings.setValue("hikCaptureTimeoutMs", 1000);
    settings.setValue("hikExposureTimeUs", 50000);
    settings.setValue("hikGain", 0.0);
    settings.setValue("hikSdkRoot", "D:/work/scan-tracking/third_party/hik_mvs");
    settings.setValue("hikCameraAName", "hik_camera_a");
    settings.setValue("hikCameraAKey", "192.168.10.12");
    settings.setValue("hikCameraAIp", "192.168.10.12");
    settings.setValue("hikCameraASerial", "");
    settings.setValue("hikCameraBName", "hik_camera_b");
    settings.setValue("hikCameraBKey", "192.168.10.13");
    settings.setValue("hikCameraBIp", "192.168.10.13");
    settings.setValue("hikCameraBSerial", "");
    settings.setValue("hikCameraCName", "hik_camera_c");
    settings.setValue("hikCameraCKey", "192.168.8.100");
    settings.setValue("hikCameraCIp", "192.168.8.100");
    settings.setValue("hikCameraCSerial", "");
    settings.setValue("hikCameraCAccessMode", "monitor");
    settings.setValue("hikCameraCTcpListenIp", "192.168.8.13");
    settings.setValue("hikCameraCTcpListenPort", 8999);
    settings.setValue("hikCameraCFtpDirectory", "D:/HikCameraFTP");
    settings.setValue("hikCxpEnabled", true);
    settings.setValue("hikCxpCaptureTimeoutMs", 5000);
    settings.setValue("hikCxpExposureTimeUs", 50000);
    settings.setValue("hikCxpGain", 0.0);
    settings.setValue("hikCxpTriggerMode", 1);
    settings.setValue("hikCxpSmokeOutputDir", "D:/CxpSmokeTest");
    settings.setValue("hikCxpCameraAName", "ch250_left");
    settings.setValue("hikCxpCameraAKey", "DA9122998");
    settings.setValue("hikCxpCameraASerial", "DA9122998");
    settings.setValue("hikCxpCameraBName", "ch250_right");
    settings.setValue("hikCxpCameraBKey", "DA9122997");
    settings.setValue("hikCxpCameraBSerial", "DA9122997");
    settings.endGroup();

    settings.beginGroup("LbPose");
    settings.setValue(
        "trackConfigFile",
        QStringLiteral("third_party/LB/track_config.ini"));
    settings.setValue("dataRoot", QStringLiteral("data/LB"));
    settings.setValue("leftPattern", "");
    settings.setValue("rightPattern", "");
    settings.setValue("templateFile", QStringLiteral("third_party/LB/template_for_scanner_ori.txt"));
    settings.setValue("angleToleranceDeg", 2.0);
    settings.setValue("lengthTolerance", 0.5);
    settings.setValue("minPercent", 0.5);
    settings.setValue("cosTolerance", 0.015);
    settings.endGroup();

    // 首次生成 config.ini 时的 [LbnPose] 默认（与 150200 离线验收一致，生产宜再标定）
    settings.beginGroup("LbnPose");
    settings.setValue("enabled", true);
    settings.setValue("useIdentityRtWithoutMarkers", false);
    settings.setValue("dataRoot", "D:/work/LY/IPC-192.168.110.173_track-main/third_party/LBN/data");
    settings.setValue("templateFile", "D:/work/LY/IPC-192.168.110.173_track-main/third_party/LBN/data/template-3D-ALL-Shift-Cut-Cut.txt");
    settings.setValue("minDistance", 20.0);
    settings.setValue("maxDistance", 650.0);
    settings.setValue("cosTolerance", 0.05);
    settings.setValue("minPercent", 0.2);
    settings.setValue("cloudSearchRadiusPx", 20);
    settings.setValue("markerMinArea", 200);
    settings.setValue("markerMaxArea", 30000);
    settings.setValue("markerIntensityThreshold", 40);
    settings.setValue("markerDebscanDistPx", 120.0);
    settings.endGroup();

    settings.beginGroup("FlowControl");
    settings.setValue("pollIntervalMs", 100);
    settings.setValue("heartbeatIntervalMs", 1000);
    settings.setValue("simulatedProcessingMs", 300);
    settings.setValue("algorithmBypassEnabled", false);
    settings.setValue("firstPathPauseAfterPoint", 0);
    settings.setValue("internalSurfaceOnlyEnabled", false);
    settings.setValue("personZoneAlarmToPlcEnabled", true);
    settings.endGroup();

    settings.beginGroup("SegmentCaptureExport");
    settings.setValue("enabled", true);
    settings.setValue("outputRoot", QStringLiteral("output"));
    settings.setValue("saveRawPointCloud", true);
    settings.setValue("pointCloudSaveFormat", QStringLiteral("pcd"));
    settings.endGroup();

    settings.beginGroup("Tracking");
    settings.setValue("scanSegmentTotal", 3);
    settings.endGroup();

    settings.beginGroup("Bevel");
    settings.setValue("configPath", "bevel/config.txt");
    settings.setValue("templateDir", "bevel");
    settings.setValue("angleTolDeg", 2.0);
    settings.setValue("lengthTolMm", 1.0);
    settings.setValue("defaultBevelType", 0);
    settings.setValue("defaultAngleDeg", 45.0);
    settings.setValue("defaultLengthMm", 1.0);
    settings.setValue("offlineReplayEnabled", false);
    settings.setValue("offlineReplayDataDir", QString());
    settings.setValue("offlineReplayPathId", 4);
    settings.setValue("offlineReplayDelayMs", 5000);
    settings.endGroup();

    settings.beginGroup("Hole");
    settings.setValue("configPath", "hole/config/default.json");
    settings.setValue("icpRmsMaxMm", 5.0);
    settings.setValue("cylinderRmsMaxMm", 3.0);
    settings.setValue("offlineReplayEnabled", false);
    settings.setValue("offlineReplaySessionDir", QString());
    settings.setValue("offlineReplayPathId", 1);
    settings.setValue("offlineReplayDelayMs", 5000);
    settings.setValue("offlineReplayPlyFileName", "pointcloud_stitched.ply");
    settings.endGroup();

    settings.beginGroup("InternalSurface");
    settings.setValue("configPath", "internal_surface/config/algorithm_config.json");
    settings.setValue("templateType", 1);
    settings.setValue("minDepthMm", 0.0);
    settings.setValue("minVolumeM3", 0.0);
    settings.setValue("offlineReplayEnabled", false);
    settings.setValue("offlineReplayPointCloudPath", QString());
    settings.setValue("offlineReplayPathId", 2);
    settings.setValue("offlineReplayDelayMs", 5000);
    settings.endGroup();

    settings.beginGroup("SelfCheck");
    settings.setValue("totalPoints", 2);
    settings.setValue("configPath", "self_check/self_check.ini");
    settings.endGroup();

    settings.beginGroup("Hmi");
    settings.setValue("enabled", true);
    settings.setValue("tcpPort", 9900);
    settings.endGroup();

    settings.sync();
    qInfo(LOG_CONFIG) << "已在" << settings.fileName() << "生成默认 config.ini";
}

void ConfigManager::loadStationProfile(QSettings& settings, const QString& configFilePath)
{
    StationProfile profile;
    QString profileIni;

    settings.beginGroup(QStringLiteral("Station"));
    applyStationSettings(settings, profile, &profileIni);
    settings.endGroup();

    // Stage 1 merge priority:
    // 1. config.ini [Station] is the base.
    // 2. profileIni, when present and existing, overrides same-name fields.
    // 3. The merged StationProfile is read-only for this stage.
    const QString resolvedProfileIni = resolveConfigRelativePath(profileIni, configFilePath);
    if (!resolvedProfileIni.isEmpty() && QFileInfo::exists(resolvedProfileIni)) {
        QSettings profileSettings(resolvedProfileIni, QSettings::IniFormat);
        configureUtf8IniSettings(profileSettings);
        profileSettings.beginGroup(QStringLiteral("Station"));
        applyStationSettings(profileSettings, profile, nullptr);
        profileSettings.endGroup();
        qInfo(LOG_CONFIG).noquote()
            << QStringLiteral("[Station] 已合并 profileIni=") << resolvedProfileIni;
    } else if (!profileIni.isEmpty()) {
        qWarning(LOG_CONFIG).noquote()
            << QStringLiteral("[Station] profileIni 不存在，忽略：")
            << resolvedProfileIni;
    }

    m_stationProfile = profile;
    qInfo(LOG_CONFIG).noquote()
        << QStringLiteral("[Station] stationId=") << stationIdToInt(m_stationProfile.stationId)
        << QStringLiteral(" name=") << m_stationProfile.stationName
        << QStringLiteral(" scanPaths=") << (m_stationProfile.scanPathsConfigPath.isEmpty()
                                                ? QStringLiteral("<fallback scan_paths_config.json>")
                                                : m_stationProfile.scanPathsConfigPath)
        << QStringLiteral(" workMode=") << workModeIdToString(m_stationProfile.defaultWorkMode);
}

void ConfigManager::load(const QString& filePath)
{
    const QFileInfo fileInfo(filePath);
    const bool fileExists = fileInfo.exists() && fileInfo.size() > 0;
    m_configFilePath = QDir::cleanPath(filePath);

    QSettings settings(filePath, QSettings::IniFormat);
    configureUtf8IniSettings(settings);
    if (!fileExists) {
        qWarning(LOG_CONFIG) << "config.ini 未找到或为空。正在生成默认配置...";
        writeDefaults(settings);
    }

    settings.beginGroup("App");
    m_appConfig.version = settings.value("version", "0.1.0").toString();
    m_appConfig.environment = settings.value("environment", "production").toString();
    settings.endGroup();

    settings.beginGroup("Logger");
    m_loggerConfig.level = settings.value("level", 0).toInt();
    m_loggerConfig.rotateDays = settings.value("rotateDays", 7).toInt();
    settings.endGroup();

    settings.beginGroup("Modbus");
    m_modbusConfig.host = settings.value("host", "127.0.0.1").toString();
    m_modbusConfig.port = settings.value("port", 502).toInt();
    m_modbusConfig.unitId = settings.value("unitId", 1).toInt();
    m_modbusConfig.timeoutMs = settings.value("timeoutMs", 1000).toInt();
    m_modbusConfig.reconnectIntervalMs = settings.value("reconnectIntervalMs", 2000).toInt();
    settings.endGroup();

    settings.beginGroup("Camera");
    m_cameraConfig.defaultCamera = settings.value("defaultCamera", "Mech-Eye Nano").toString();
    m_cameraConfig.scanTimeoutMs = settings.value("scanTimeoutMs", 5000).toInt();
    settings.endGroup();

    loadStationProfile(settings, filePath);

    settings.beginGroup("Vision");
    m_visionConfig.mechEyeCameraKey = settings.value("mechEyeCameraKey", m_cameraConfig.defaultCamera).toString();
    m_visionConfig.mechCaptureTimeoutMs = settings.value("mechCaptureTimeoutMs", m_cameraConfig.scanTimeoutMs).toInt();
    m_visionConfig.mechDepthRangeMin = settings.value("mechDepthRangeMin", 100).toInt();
    m_visionConfig.mechDepthRangeMax = settings.value("mechDepthRangeMax", 2000).toInt();
    m_visionConfig.mechPointCloudProcessingComparisonEnabled =
        settings.value("mechPointCloudProcessingComparisonEnabled", false).toBool();
    m_visionConfig.mechScan3DSingleExposureEnabled =
        settings.value("mechScan3DSingleExposureEnabled", true).toBool();
    m_visionConfig.mechScan3DSingleExposureMs =
        settings.value("mechScan3DSingleExposureMs", 40.0).toDouble();
    m_visionConfig.mechScan3DMultiExposureEnabled =
        settings.value("mechScan3DMultiExposureEnabled", false).toBool();
    m_visionConfig.mechScan3DMultiExposureSequenceMs =
        settings.value("mechScan3DMultiExposureSequenceMs").toString();
    m_visionConfig.mechScan2DExposureMs =
        settings.value("mechScan2DExposureMs", 0.0).toDouble();
    m_visionConfig.mechScan3DGain =
        settings.value("mechScan3DGain", 0.0).toDouble();
    m_visionConfig.mechSaveExposureToDevice =
        settings.value("mechSaveExposureToDevice", false).toBool();
    m_visionConfig.hikConnectTimeoutMs = settings.value("hikConnectTimeoutMs", 3000).toInt();
    m_visionConfig.hikCaptureTimeoutMs = settings.value("hikCaptureTimeoutMs", 1000).toInt();
    m_visionConfig.hikExposureTimeUs =
        static_cast<float>(settings.value("hikExposureTimeUs", 50000).toDouble());
    m_visionConfig.hikGain = static_cast<float>(settings.value("hikGain", 0.0).toDouble());
    m_visionConfig.hikSdkRoot = settings.value("hikSdkRoot", "D:/work/scan-tracking/third_party/hik_mvs").toString();
    m_visionConfig.hikCameraA.logicalName = settings.value("hikCameraAName", "hik_camera_a").toString();
    m_visionConfig.hikCameraA.cameraKey = settings.value("hikCameraAKey", "192.168.10.12").toString();
    m_visionConfig.hikCameraA.ipAddress = settings.value("hikCameraAIp", "192.168.10.12").toString();
    m_visionConfig.hikCameraA.serialNumber = settings.value("hikCameraASerial", "").toString();
    m_visionConfig.hikCameraB.logicalName = settings.value("hikCameraBName", "hik_camera_b").toString();
    m_visionConfig.hikCameraB.cameraKey = settings.value("hikCameraBKey", "192.168.10.13").toString();
    m_visionConfig.hikCameraB.ipAddress = settings.value("hikCameraBIp", "192.168.10.13").toString();
    m_visionConfig.hikCameraB.serialNumber = settings.value("hikCameraBSerial", "").toString();
    m_visionConfig.hikCameraC.logicalName = settings.value("hikCameraCName", "hik_camera_c").toString();
    m_visionConfig.hikCameraC.cameraKey = settings.value("hikCameraCKey", "192.168.8.100").toString();
    m_visionConfig.hikCameraC.ipAddress = settings.value("hikCameraCIp", "192.168.8.100").toString();
    m_visionConfig.hikCameraC.serialNumber = settings.value("hikCameraCSerial", "").toString();
    m_visionConfig.hikCameraC.accessMode = settings.value("hikCameraCAccessMode", "monitor").toString();
    m_visionConfig.hikCameraCTcpListenIp = settings.value("hikCameraCTcpListenIp", "192.168.8.13").toString();
    m_visionConfig.hikCameraCTcpListenPort = static_cast<quint16>(settings.value("hikCameraCTcpListenPort", 8999).toUInt());
    m_visionConfig.hikCameraCFtpDirectory = settings.value("hikCameraCFtpDirectory", "D:/HikCameraFTP").toString();
    m_visionConfig.hikCxpEnabled = settings.value("hikCxpEnabled", false).toBool();
    m_visionConfig.hikCxpCaptureTimeoutMs = settings.value("hikCxpCaptureTimeoutMs", 5000).toInt();
    m_visionConfig.hikCxpExposureTimeUs =
        static_cast<float>(settings.value("hikCxpExposureTimeUs", 50000).toDouble());
    m_visionConfig.hikCxpGain = static_cast<float>(settings.value("hikCxpGain", 0.0).toDouble());
    m_visionConfig.hikCxpTriggerMode =
        settings.value("hikCxpTriggerMode", 1).toUInt();
    m_visionConfig.hikCxpSmokeOutputDir =
        settings.value("hikCxpSmokeOutputDir", "D:/CxpSmokeTest").toString();
    m_visionConfig.hikCxpCameraA.logicalName =
        settings.value("hikCxpCameraAName", "ch250_left").toString();
    m_visionConfig.hikCxpCameraA.cameraKey =
        settings.value("hikCxpCameraAKey", "DA9122998").toString();
    m_visionConfig.hikCxpCameraA.serialNumber =
        settings.value("hikCxpCameraASerial", "DA9122998").toString();
    m_visionConfig.hikCxpCameraB.logicalName =
        settings.value("hikCxpCameraBName", "ch250_right").toString();
    m_visionConfig.hikCxpCameraB.cameraKey =
        settings.value("hikCxpCameraBKey", "DA9122997").toString();
    m_visionConfig.hikCxpCameraB.serialNumber =
        settings.value("hikCxpCameraBSerial", "DA9122997").toString();
    settings.endGroup();

    settings.beginGroup("LbPose");
    m_lbPoseConfig.trackConfigFile = resolveConfigRelativePath(
        settings.value(
            "trackConfigFile",
            QStringLiteral("third_party/LB/track_config.ini"))
            .toString(),
        m_configFilePath);
    m_lbPoseConfig.dataRoot = resolveConfigRelativePath(
        settings.value(
            "dataRoot",
            QStringLiteral("data/LB"))
            .toString(),
        m_configFilePath);
    m_lbPoseConfig.leftPattern = settings.value("leftPattern", "").toString();
    m_lbPoseConfig.rightPattern = settings.value("rightPattern", "").toString();
    m_lbPoseConfig.templateFile = resolveConfigRelativePath(
        settings.value(
            "templateFile",
            QStringLiteral("third_party/LB/template_for_scanner_ori.txt"))
            .toString(),
        m_configFilePath);
    const float legacyCosTolerance = settings.value("cosTolerance", 0.015).toFloat();
    m_lbPoseConfig.angleToleranceDeg = settings.value("angleToleranceDeg", legacyCosTolerance).toFloat();
    m_lbPoseConfig.lengthTolerance = settings.value("lengthTolerance", 0.5).toFloat();
    m_lbPoseConfig.minPercent = settings.value("minPercent", 0.5).toFloat();
    settings.endGroup();

    // [LbnPose] 默认值与 testdata/test 150200 离线调通一致；上线前请多扫描验证，见 docs/station1/算法使用API.md
    settings.beginGroup("LbnPose");
    m_lbnPoseConfig.enabled = settings.value("enabled", true).toBool();
    m_lbnPoseConfig.useIdentityRtWithoutMarkers =
        settings.value("useIdentityRtWithoutMarkers", false).toBool();
    m_lbnPoseConfig.dataRoot = settings.value(
        "dataRoot",
        QStringLiteral("D:/work/LY/IPC-192.168.110.173_track-main/third_party/LBN/data"))
        .toString();
    m_lbnPoseConfig.templateFile = settings.value(
        "templateFile",
        QStringLiteral("D:/work/LY/IPC-192.168.110.173_track-main/third_party/LBN/data/template-3D-ALL-Shift-Cut-Cut.txt"))
        .toString();
    m_lbnPoseConfig.minDistance = settings.value("minDistance", 20.0).toFloat();
    m_lbnPoseConfig.maxDistance = settings.value("maxDistance", 650.0).toFloat();
    m_lbnPoseConfig.cosTolerance = settings.value("cosTolerance", 0.05).toFloat();
    m_lbnPoseConfig.minPercent = settings.value("minPercent", 0.2).toFloat();
    m_lbnPoseConfig.cloudSearchRadiusPx = settings.value("cloudSearchRadiusPx", 20).toInt();
    m_lbnPoseConfig.markerMinArea = settings.value("markerMinArea", 200).toInt();
    m_lbnPoseConfig.markerMaxArea = settings.value("markerMaxArea", 30000).toInt();
    m_lbnPoseConfig.markerIntensityThreshold = settings.value("markerIntensityThreshold", 40).toInt();
    m_lbnPoseConfig.markerDebscanDistPx =
        settings.value("markerDebscanDistPx", 120.0).toFloat();
    settings.endGroup();

    settings.beginGroup("FlowControl");
    m_flowControlConfig.pollIntervalMs = settings.value("pollIntervalMs", 100).toInt();
    m_flowControlConfig.heartbeatIntervalMs = settings.value("heartbeatIntervalMs", 1000).toInt();
    m_flowControlConfig.simulatedProcessingMs = settings.value("simulatedProcessingMs", 300).toInt();
    m_flowControlConfig.algorithmBypassEnabled =
        settings.value("algorithmBypassEnabled", false).toBool();
    m_flowControlConfig.firstPathPauseAfterPoint =
        qMax(0, settings.value("firstPathPauseAfterPoint", 0).toInt());
    m_flowControlConfig.internalSurfaceOnlyEnabled =
        settings.value("internalSurfaceOnlyEnabled", false).toBool();
    m_flowControlConfig.personZoneAlarmToPlcEnabled =
        settings.value("personZoneAlarmToPlcEnabled", true).toBool();
    m_flowControlConfig.scanCacheDirectory = settings.value("scanCacheDirectory").toString().trimmed();
    m_flowControlConfig.retainSegmentPly = settings.value("retainSegmentPly", true).toBool();
    settings.endGroup();

    if (m_flowControlConfig.firstPathPauseAfterPoint > 0) {
        qInfo(LOG_CONFIG).noquote()
            << QStringLiteral("路径1联调步进已启用：firstPathPauseAfterPoint=")
            << m_flowControlConfig.firstPathPauseAfterPoint
            << QStringLiteral("（路径1第 N 点 CXP+梅卡落盘后暂停；改 0 或调大 N 后重启 IPC 再继续）");
    }

    if (m_flowControlConfig.internalSurfaceOnlyEnabled) {
        qInfo(LOG_CONFIG).noquote()
            << QStringLiteral("内表面单路径模式已启用（[FlowControl] internalSurfaceOnlyEnabled=true）："
                              "将仅执行 inspectionType=internal_surface 的扫描路径。");
    }

    if (!m_flowControlConfig.personZoneAlarmToPlcEnabled) {
        qWarning(LOG_CONFIG).noquote()
            << QStringLiteral("人员区域报警 PLC 联锁已关闭（[FlowControl] personZoneAlarmToPlcEnabled=false）："
                              "显控上报不会写 IPC_SafetyAction_Word，流程不会因人员报警停止。");
    }

    settings.beginGroup("SegmentCaptureExport");
    m_segmentCaptureExportConfig.enabled = settings.value("enabled", false).toBool();
    m_segmentCaptureExportConfig.outputRoot = resolveConfigRelativePath(
        settings.value("outputRoot", QStringLiteral("output")).toString(),
        m_configFilePath);
    m_segmentCaptureExportConfig.saveRawPointCloud =
        settings.value("saveRawPointCloud", true).toBool();
    m_segmentCaptureExportConfig.pointCloudSaveFormat = pointCloudSaveFormatFromString(
        settings.value("pointCloudSaveFormat", QStringLiteral("pcd")).toString());
    settings.endGroup();

    settings.beginGroup("PointCloudProcessing");
    m_pointCloudProcessingConfig.enabled = settings.value("enabled", true).toBool();
    m_pointCloudProcessingConfig.depthMinMm =
        settings.value("depthMinMm", m_visionConfig.mechDepthRangeMin).toFloat();
    m_pointCloudProcessingConfig.depthMaxMm =
        settings.value("depthMaxMm", m_visionConfig.mechDepthRangeMax).toFloat();
    m_pointCloudProcessingConfig.outlierRemovalEnabled =
        settings.value("outlierRemovalEnabled", true).toBool();
    m_pointCloudProcessingConfig.outlierMeanK = settings.value("outlierMeanK", 50).toInt();
    m_pointCloudProcessingConfig.outlierStddevMul =
        settings.value("outlierStddevMul", 1.0).toFloat();
    m_pointCloudProcessingConfig.smoothingEnabled = settings.value("smoothingEnabled", true).toBool();
    m_pointCloudProcessingConfig.mlsSearchRadiusMm =
        settings.value("mlsSearchRadiusMm", 5.0).toFloat();
    m_pointCloudProcessingConfig.mlsPolynomialOrder =
        settings.value("mlsPolynomialOrder", 2).toInt();
    m_pointCloudProcessingConfig.downsampleEnabled =
        settings.value("downsampleEnabled", true).toBool();
    m_pointCloudProcessingConfig.voxelLeafSizeMm =
        settings.value("voxelLeafSizeMm", 2.0).toFloat();
    m_pointCloudProcessingConfig.minPointsAfterProcessing =
        settings.value("minPointsAfterProcessing", 1000).toInt();
    settings.endGroup();

    settings.beginGroup("Tracking");
    m_trackingConfig.scanSegmentTotal = settings.value("scanSegmentTotal", 3).toInt();
    settings.endGroup();

    settings.beginGroup("Bevel");
    m_bevelConfig.configPath =
        settings.value("configPath", QStringLiteral("bevel/config.txt")).toString();
    m_bevelConfig.templateDir =
        settings.value("templateDir", QStringLiteral("bevel")).toString();
    m_bevelConfig.angleTolDeg =
        settings.value("angleTolDeg", 2.0).toFloat();
    m_bevelConfig.lengthTolMm =
        settings.value("lengthTolMm", 1.0).toFloat();
    m_bevelConfig.defaultRecipe.bevelType =
        settings.value("defaultBevelType", 0).toInt();
    m_bevelConfig.defaultRecipe.angleDeg =
        settings.value("defaultAngleDeg", 45.0).toFloat();
    m_bevelConfig.defaultRecipe.lengthMm =
        settings.value("defaultLengthMm", 1.0).toFloat();
    m_bevelConfig.defaultRecipe.active =
        m_bevelConfig.defaultRecipe.angleDeg > 0.0f
        && m_bevelConfig.defaultRecipe.lengthMm > 0.0f;
    m_bevelConfig.offlineReplayEnabled =
        settings.value("offlineReplayEnabled", false).toBool();
    m_bevelConfig.offlineReplayDataDir = resolveConfigRelativePath(
        settings.value("offlineReplayDataDir").toString().trimmed(),
        m_configFilePath);
    m_bevelConfig.offlineReplayPathId =
        qMax(1, settings.value("offlineReplayPathId", 4).toInt());
    m_bevelConfig.offlineReplayDelayMs =
        qMax(0, settings.value("offlineReplayDelayMs", 5000).toInt());
    settings.endGroup();

    if (m_bevelConfig.offlineReplayEnabled) {
        qInfo(LOG_CONFIG).noquote()
            << QStringLiteral("[Bevel] 离线回放已启用 pathId=")
            << m_bevelConfig.offlineReplayPathId
            << QStringLiteral(" dataDir=") << m_bevelConfig.offlineReplayDataDir
            << QStringLiteral(" delayMs=") << m_bevelConfig.offlineReplayDelayMs;
    }

    settings.beginGroup("Hole");
    m_holeConfig.configPath =
        settings.value("configPath", QStringLiteral("hole/config/default.json")).toString();
    m_holeConfig.icpRmsMaxMm =
        settings.value("icpRmsMaxMm", 5.0).toDouble();
    m_holeConfig.cylinderRmsMaxMm =
        settings.value("cylinderRmsMaxMm", 3.0).toDouble();
    m_holeConfig.offlineReplayEnabled =
        settings.value("offlineReplayEnabled", false).toBool();
    m_holeConfig.offlineReplaySessionDir = resolveConfigRelativePath(
        settings.value("offlineReplaySessionDir").toString(),
        m_configFilePath);
    m_holeConfig.offlineReplayPathId =
        qMax(1, settings.value("offlineReplayPathId", 1).toInt());
    m_holeConfig.offlineReplayDelayMs =
        qMax(0, settings.value("offlineReplayDelayMs", 5000).toInt());
    m_holeConfig.offlineReplayPlyFileName =
        settings.value("offlineReplayPlyFileName", QStringLiteral("pointcloud_stitched.ply"))
            .toString()
            .trimmed();
    if (m_holeConfig.offlineReplayPlyFileName.isEmpty()) {
        m_holeConfig.offlineReplayPlyFileName = QStringLiteral("pointcloud_stitched.ply");
    }
    settings.endGroup();

    if (m_holeConfig.offlineReplayEnabled) {
        qInfo(LOG_CONFIG).noquote()
            << QStringLiteral("[Hole] 离线回放已启用 pathId=")
            << m_holeConfig.offlineReplayPathId
            << QStringLiteral(" session=") << m_holeConfig.offlineReplaySessionDir
            << QStringLiteral(" ply=") << m_holeConfig.offlineReplayPlyFileName
            << QStringLiteral(" delayMs=") << m_holeConfig.offlineReplayDelayMs;
    }

    settings.beginGroup("Thickness");
    m_thicknessConfig.configPath = settings.value(
        "configPath", QStringLiteral("thickness/config/thickness_config.json")).toString();
    m_thicknessConfig.icpFitnessMax = settings.value("icpFitnessMax", 50.0).toDouble();
    m_thicknessConfig.offlineReplayEnabled =
        settings.value("offlineReplayEnabled", false).toBool();
    m_thicknessConfig.offlineReplayDataDir = resolveConfigRelativePath(
        settings.value("offlineReplayDataDir").toString().trimmed(),
        m_configFilePath);
    m_thicknessConfig.offlineReplayInnerCloudFile = resolveConfigRelativePath(
        settings.value("offlineReplayInnerCloudFile").toString().trimmed(),
        m_configFilePath);
    m_thicknessConfig.offlineReplayOuterCloudFile = resolveConfigRelativePath(
        settings.value("offlineReplayOuterCloudFile").toString().trimmed(),
        m_configFilePath);
    m_thicknessConfig.offlineReplayPathId =
        qMax(1, settings.value("offlineReplayPathId", 5).toInt());
    m_thicknessConfig.offlineReplayDelayMs =
        qMax(0, settings.value("offlineReplayDelayMs", 5000).toInt());
    m_thicknessConfig.offlineReplayAlgorithmConfigPath = resolveConfigRelativePath(
        settings.value("offlineReplayAlgorithmConfigPath").toString().trimmed(),
        m_configFilePath);
    settings.endGroup();

    if (m_thicknessConfig.offlineReplayEnabled) {
        qInfo(LOG_CONFIG).noquote()
            << QStringLiteral("[Thickness] 离线回放已启用 pathId=")
            << m_thicknessConfig.offlineReplayPathId
            << QStringLiteral(" dataDir=") << m_thicknessConfig.offlineReplayDataDir
            << QStringLiteral(" inner=")
            << (m_thicknessConfig.offlineReplayInnerCloudFile.isEmpty()
                    ? QStringLiteral("(auto)")
                    : m_thicknessConfig.offlineReplayInnerCloudFile)
            << QStringLiteral(" outer=")
            << (m_thicknessConfig.offlineReplayOuterCloudFile.isEmpty()
                    ? QStringLiteral("(auto)")
                    : m_thicknessConfig.offlineReplayOuterCloudFile)
            << QStringLiteral(" algoConfig=")
            << (m_thicknessConfig.offlineReplayAlgorithmConfigPath.isEmpty()
                    ? m_thicknessConfig.configPath
                    : m_thicknessConfig.offlineReplayAlgorithmConfigPath)
            << QStringLiteral(" delayMs=") << m_thicknessConfig.offlineReplayDelayMs;
    }

    settings.beginGroup("InternalSurface");
    m_internalSurfaceConfig.configPath = settings.value(
        "configPath", QStringLiteral("internal_surface/config/algorithm_config.json")).toString();
    m_internalSurfaceConfig.templateType = settings.value("templateType", 1).toInt();
    m_internalSurfaceConfig.minDepthMm = settings.value("minDepthMm", 0.0).toDouble();
    m_internalSurfaceConfig.minVolumeM3 = settings.value("minVolumeM3", 0.0).toDouble();
    m_internalSurfaceConfig.offlineReplayEnabled =
        settings.value("offlineReplayEnabled", false).toBool();
    m_internalSurfaceConfig.offlineReplayPointCloudPath = resolveConfigRelativePath(
        settings.value("offlineReplayPointCloudPath").toString().trimmed(),
        m_configFilePath);
    m_internalSurfaceConfig.offlineReplayPathId =
        qMax(1, settings.value("offlineReplayPathId", 2).toInt());
    m_internalSurfaceConfig.offlineReplayDelayMs =
        qMax(0, settings.value("offlineReplayDelayMs", 5000).toInt());
    m_internalSurfaceConfig.offlineReplayAlgorithmConfigPath = resolveConfigRelativePath(
        settings.value("offlineReplayAlgorithmConfigPath").toString().trimmed(),
        m_configFilePath);
    settings.endGroup();

    if (m_internalSurfaceConfig.offlineReplayEnabled) {
        qInfo(LOG_CONFIG).noquote()
            << QStringLiteral("[InternalSurface] 离线回放已启用 pathId=")
            << m_internalSurfaceConfig.offlineReplayPathId
            << QStringLiteral(" cloud=") << m_internalSurfaceConfig.offlineReplayPointCloudPath
            << QStringLiteral(" algoConfig=")
            << (m_internalSurfaceConfig.offlineReplayAlgorithmConfigPath.isEmpty()
                    ? m_internalSurfaceConfig.configPath
                    : m_internalSurfaceConfig.offlineReplayAlgorithmConfigPath)
            << QStringLiteral(" delayMs=") << m_internalSurfaceConfig.offlineReplayDelayMs;
    }

    settings.beginGroup("SelfCheck");
    m_selfCheckConfig.totalPoints = settings.value("totalPoints", 2).toInt();
    m_selfCheckConfig.configPath = settings.value(
        "configPath", QStringLiteral("self_check/self_check.ini")).toString();
    settings.endGroup();

    settings.beginGroup("Hmi");
    {
        const int port = settings.value("tcpPort", 9900).toInt();
        m_hmiConfig.tcpPort = static_cast<quint16>(
            qBound(1, port, 65535));
    }
    m_hmiConfig.allowDebugTriggerInspection =
        settings.value("allowDebugTriggerInspection", false).toBool();
    m_hmiConfig.emitPresetInspectionOnPathsComplete =
        settings.value("emitPresetInspectionOnPathsComplete", false).toBool();
    m_hmiConfig.emitDemoScanPathStatusOnConnect =
        settings.value("emitDemoScanPathStatusOnConnect", false).toBool();
    settings.endGroup();

    QtMsgType minType = QtDebugMsg;
    switch (m_loggerConfig.level) {
        case 0: minType = QtDebugMsg; break;
        case 1: minType = QtInfoMsg; break;
        case 2: minType = QtWarningMsg; break;
        case 3: minType = QtCriticalMsg; break;
        default: minType = QtDebugMsg; break;
    }

    if (Logger* logger = Logger::instance()) {
        logger->setMinLevel(minType);
    }

    qInfo(LOG_CONFIG) << "已从以下位置加载 config.ini：" << filePath;
    qInfo(LOG_CONFIG).noquote()
        << QStringLiteral("Modbus 配置：")
        << QStringLiteral(" host=") << m_modbusConfig.host
        << QStringLiteral(" port=") << m_modbusConfig.port
        << QStringLiteral(" unitId=") << m_modbusConfig.unitId
        << QStringLiteral(" timeoutMs=") << m_modbusConfig.timeoutMs
        << QStringLiteral(" reconnectIntervalMs=") << m_modbusConfig.reconnectIntervalMs;
}

/**
 * @brief 加载扫描路径配置（JSON 格式）
 * 
 * 从 scan_paths_config.json 文件加载多路径扫描配置，包括：
 * - 标定矩阵 T0
 * - 所有扫描路径定义
 * - 执行策略
 * - 转盘配置
 * 
 * @param jsonFilePath JSON 配置文件路径
 */
void ConfigManager::loadScanPathsConfig(const QString& jsonFilePath)
{
    // 检查文件是否存在
    if (!QFileInfo::exists(jsonFilePath)) {
        qWarning(LOG_CONFIG) << "扫描路径配置文件不存在：" << jsonFilePath;
        qWarning(LOG_CONFIG) << "将使用空配置，多路径扫描功能不可用。";
        return;
    }
    
    // 读取 JSON 文件
    QFile file(jsonFilePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qCritical(LOG_CONFIG) << "无法打开扫描路径配置文件：" << jsonFilePath;
        return;
    }
    
    const QByteArray jsonData = file.readAll();
    file.close();
    
    // 解析 JSON
    QJsonParseError parseError;
    const QJsonDocument doc = QJsonDocument::fromJson(jsonData, &parseError);
    
    if (parseError.error != QJsonParseError::NoError) {
        qCritical(LOG_CONFIG) << "扫描路径配置文件 JSON 解析失败："
                              << parseError.errorString()
                              << "位置：" << parseError.offset;
        return;
    }
    
    if (!doc.isObject()) {
        qCritical(LOG_CONFIG) << "扫描路径配置文件格式错误：根节点不是对象";
        return;
    }
    
    const QJsonObject root = doc.object();
    
    // 1. 读取配置文件元数据
    m_scanPathsConfig.version = root.value("version").toString("1.0");
    m_scanPathsConfig.lastModified = root.value("lastModified").toString();
    
    // 2. 读取标定矩阵 T0
    const QJsonObject calibMatrixObj = root.value("calibrationMatrix").toObject();
    const QJsonArray t0Array = calibMatrixObj.value("T0").toArray();
    
    if (t0Array.size() == 4) {
        // 4x4 矩阵，行优先存储
        int index = 0;
        for (int row = 0; row < 4; ++row) {
            const QJsonArray rowArray = t0Array.at(row).toArray();
            for (int col = 0; col < 4; ++col) {
                m_scanPathsConfig.calibrationMatrixT0[index++] = 
                    static_cast<float>(rowArray.at(col).toDouble(row == col ? 1.0 : 0.0));
            }
        }
    } else {
        qWarning(LOG_CONFIG) << "标定矩阵 T0 格式错误，使用单位矩阵";
        // 初始化为单位矩阵
        m_scanPathsConfig.calibrationMatrixT0.fill(0.0f);
        m_scanPathsConfig.calibrationMatrixT0[0] = 1.0f;
        m_scanPathsConfig.calibrationMatrixT0[5] = 1.0f;
        m_scanPathsConfig.calibrationMatrixT0[10] = 1.0f;
        m_scanPathsConfig.calibrationMatrixT0[15] = 1.0f;
    }
    
    // 3. 读取扫描路径列表
    const QJsonArray pathsArray = root.value("scanPaths").toArray();
    m_scanPathsConfig.scanPaths.clear();
    m_scanPathsConfig.scanPaths.reserve(pathsArray.size());
    
    for (const QJsonValue& pathValue : pathsArray) {
        const QJsonObject pathObj = pathValue.toObject();
        
        ScanPathConfig pathConfig;
        pathConfig.pathId = pathObj.value("pathId").toInt();
        pathConfig.pathName = pathObj.value("pathName").toString().trimmed();
        if (pathConfig.pathName.isEmpty()) {
            pathConfig.pathName = QStringLiteral("路径%1").arg(pathConfig.pathId);
        }
        pathConfig.enabled = pathObj.value("enabled").toBool(true);
        pathConfig.totalPoints = pathObj.value("totalPoints").toInt();
        pathConfig.inspectionType = inspectionTypeFromString(
            pathObj.value("inspectionType").toString(QStringLiteral("bevel")));
        pathConfig.holeConfigPath = pathObj.value("holeConfigPath").toString().trimmed();
        pathConfig.thicknessConfigPath = pathObj.value("thicknessConfigPath").toString().trimmed();
        pathConfig.innerScanSegmentIndex = pathObj.value("innerScanSegmentIndex").toInt(0);
        pathConfig.outerScanSegmentIndex = pathObj.value("outerScanSegmentIndex").toInt(0);

        // 读取点位列表
        const QJsonArray pointsArray = pathObj.value("points").toArray();
        pathConfig.points.clear();
        pathConfig.points.reserve(pointsArray.size());
        
        for (const QJsonValue& pointValue : pointsArray) {
            const QJsonObject pointObj = pointValue.toObject();
            
            ScanPointConfig pointConfig;
            pointConfig.pointIndex = pointObj.value("pointIndex").toInt();
            pointConfig.needRotation = pointObj.value("needRotation").toBool(false);
            
            pathConfig.points.push_back(pointConfig);
        }
        
        m_scanPathsConfig.scanPaths.push_back(pathConfig);
    }
    
    // 4. 读取执行策略
    const QJsonObject execConfigObj = root.value("executionConfig").toObject();
    m_scanPathsConfig.executeAllPaths = execConfigObj.value("executeAllPaths").toBool(false);
    m_scanPathsConfig.allowPathSkipOnError = execConfigObj.value("allowPathSkipOnError").toBool(false);
    
    const QJsonArray selectedIdsArray = execConfigObj.value("selectedPathIds").toArray();
    m_scanPathsConfig.selectedPathIds.clear();
    m_scanPathsConfig.selectedPathIds.reserve(selectedIdsArray.size());
    for (const QJsonValue& idValue : selectedIdsArray) {
        m_scanPathsConfig.selectedPathIds.push_back(idValue.toInt());
    }
    
    // 5. 读取转盘配置
    const QJsonObject turntableObj = root.value("turntableConfig").toObject();
    m_scanPathsConfig.turntableEnabled = turntableObj.value("enabled").toBool(true);
    
    if (m_flowControlConfig.internalSurfaceOnlyEnabled) {
        std::vector<int> internalSurfacePathIds;
        internalSurfacePathIds.reserve(m_scanPathsConfig.scanPaths.size());
        for (const auto& path : m_scanPathsConfig.scanPaths) {
            if (path.enabled && path.inspectionType == InspectionType::InternalSurface) {
                internalSurfacePathIds.push_back(path.pathId);
            }
        }

        if (internalSurfacePathIds.empty()) {
            qWarning(LOG_CONFIG).noquote()
                << QStringLiteral("internalSurfaceOnlyEnabled=true 但未找到启用的 internal_surface 路径，"
                                  "多路径扫描将不可用。");
        } else {
            m_scanPathsConfig.executeAllPaths = false;
            m_scanPathsConfig.selectedPathIds = std::move(internalSurfacePathIds);
            QStringList pathLabels;
            pathLabels.reserve(static_cast<int>(m_scanPathsConfig.selectedPathIds.size()));
            for (int pathId : m_scanPathsConfig.selectedPathIds) {
                pathLabels << QStringLiteral("path%1").arg(pathId);
            }
            qInfo(LOG_CONFIG).noquote()
                << QStringLiteral("内表面单路径模式：已覆盖 executionConfig，仅执行 ")
                << pathLabels.join(QStringLiteral(", "));
        }
    }

    // 6. 验证配置
    QString validationError;
    if (!validateScanPathsConfig(&validationError)) {
        qWarning(LOG_CONFIG) << "扫描路径配置验证失败：" << validationError;
    }
    
    // 7. 输出加载信息
    qInfo(LOG_CONFIG) << "已从以下位置加载扫描路径配置：" << jsonFilePath;
    qInfo(LOG_CONFIG).noquote()
        << "扫描路径配置："
        << "版本=" << m_scanPathsConfig.version
        << "路径数=" << m_scanPathsConfig.scanPaths.size()
        << "执行所有路径=" << m_scanPathsConfig.executeAllPaths
        << "选中路径数=" << m_scanPathsConfig.selectedPathIds.size()
        << "转盘启用=" << m_scanPathsConfig.turntableEnabled;
    
    // 输出每条路径的详细信息
    for (const auto& path : m_scanPathsConfig.scanPaths) {
        qInfo(LOG_CONFIG).noquote()
            << "  路径" << path.pathId
            << "启用=" << path.enabled
            << "点位数=" << path.points.size()
            << "inspectionType=" << inspectionTypeToString(path.inspectionType)
            << (path.holeConfigPath.isEmpty()
                    ? QString()
                    : QStringLiteral(" holeConfig=") + path.holeConfigPath)
            << (path.thicknessConfigPath.isEmpty()
                    ? QString()
                    : QStringLiteral(" thicknessConfig=") + path.thicknessConfigPath)
            << (path.innerScanSegmentIndex > 0
                    ? QStringLiteral(" innerSeg=") + QString::number(path.innerScanSegmentIndex)
                    : QString())
            << (path.outerScanSegmentIndex > 0
                    ? QStringLiteral(" outerSeg=") + QString::number(path.outerScanSegmentIndex)
                    : QString());
    }
}

/**
 * @brief 验证扫描路径配置的合法性
 * 
 * 检查配置是否符合以下规则：
 * - 路径 ID 唯一
 * - 点位索引连续（从 1 开始）
 * - 点位数量与 totalPoints 一致
 * - 选中的路径 ID 存在
 * 
 * @param errorMessage 输出参数，验证失败时包含错误信息
 * @return 验证是否通过
 */
bool ConfigManager::validateScanPathsConfig(QString* errorMessage) const
{
    // 1. 检查路径 ID 唯一性
    std::vector<int> pathIds;
    pathIds.reserve(m_scanPathsConfig.scanPaths.size());
    
    for (const auto& path : m_scanPathsConfig.scanPaths) {
        if (std::find(pathIds.begin(), pathIds.end(), path.pathId) != pathIds.end()) {
            if (errorMessage) {
                *errorMessage = QStringLiteral("路径 ID 重复：%1").arg(path.pathId);
            }
            return false;
        }
        pathIds.push_back(path.pathId);
        
        // 2. 检查点位数量
        if (static_cast<int>(path.points.size()) != path.totalPoints) {
            if (errorMessage) {
                *errorMessage = QStringLiteral("路径 %1 的点位数量不匹配：配置 %2，实际 %3")
                    .arg(path.pathId)
                    .arg(path.totalPoints)
                    .arg(path.points.size());
            }
            return false;
        }
        
        // 3. 检查点位索引连续性（从 1 开始）
        for (size_t i = 0; i < path.points.size(); ++i) {
            const int expectedIndex = static_cast<int>(i) + 1;
            if (path.points[i].pointIndex != expectedIndex) {
                if (errorMessage) {
                    *errorMessage = QStringLiteral("路径 %1 的点位索引不连续：期望 %2，实际 %3")
                        .arg(path.pathId)
                        .arg(expectedIndex)
                        .arg(path.points[i].pointIndex);
                }
                return false;
            }
        }
    }
    
    // 4. 检查选中的路径 ID 是否存在
    for (int selectedId : m_scanPathsConfig.selectedPathIds) {
        if (std::find(pathIds.begin(), pathIds.end(), selectedId) == pathIds.end()) {
            if (errorMessage) {
                *errorMessage = QStringLiteral("选中的路径 ID 不存在：%1").arg(selectedId);
            }
            return false;
        }
    }
    
    return true;
}

}  // namespace common
}  // namespace scan_tracking
