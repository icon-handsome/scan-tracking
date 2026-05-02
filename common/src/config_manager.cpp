#include "scan_tracking/common/config_manager.h"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QLoggingCategory>

#include "scan_tracking/common/logger.h"

namespace scan_tracking {
namespace common {

Q_LOGGING_CATEGORY(LOG_CONFIG, "config")

ConfigManager* ConfigManager::s_instance = nullptr;

namespace {

QString projectRootConfigPath()
{
    QDir rootDir(QCoreApplication::applicationDirPath());
    if (rootDir.cdUp() && rootDir.cdUp() && rootDir.cdUp()) {
        return rootDir.filePath(QStringLiteral("config.ini"));
    }
    return QCoreApplication::applicationDirPath() + QStringLiteral("/config.ini");
}

}  // namespace

void ConfigManager::initialize()
{
    if (!s_instance) {
        s_instance = new ConfigManager();
        qInfo(LOG_CONFIG) << "ConfigManager initialized.";
    }
}

void ConfigManager::cleanup()
{
    if (s_instance) {
        delete s_instance;
        s_instance = nullptr;
        qInfo(LOG_CONFIG) << "ConfigManager cleaned up.";
    }
}

ConfigManager* ConfigManager::instance()
{
    if (!s_instance) {
        qWarning(LOG_CONFIG) << "ConfigManager::instance() called before initialize()!";
    }
    return s_instance;
}

ConfigManager::ConfigManager()
{
    const QString configPath = projectRootConfigPath();
    load(configPath);
}

ConfigManager::~ConfigManager() = default;

const AppConfig& ConfigManager::appConfig() const { return m_appConfig; }
const LoggerConfig& ConfigManager::loggerConfig() const { return m_loggerConfig; }
const ModbusConfig& ConfigManager::modbusConfig() const { return m_modbusConfig; }
const CameraConfig& ConfigManager::cameraConfig() const { return m_cameraConfig; }
const VisionConfig& ConfigManager::visionConfig() const { return m_visionConfig; }
const FlowControlConfig& ConfigManager::flowControlConfig() const { return m_flowControlConfig; }
const TrackingConfig& ConfigManager::trackingConfig() const { return m_trackingConfig; }
const LbPoseConfig& ConfigManager::lbPoseConfig() const { return m_lbPoseConfig; }

void ConfigManager::writeDefaults(QSettings& settings)
{
    settings.beginGroup("App");
    settings.setValue("version", "0.1.0");
    settings.setValue("environment", "production");
    settings.endGroup();

    settings.beginGroup("Logger");
    settings.setValue("level", 0);
    settings.setValue("rotateDays", 7);
    settings.endGroup();

    settings.beginGroup("Modbus");
    settings.setValue("host", "127.0.0.1");
    settings.setValue("port", 502);
    settings.setValue("unitId", 1);
    settings.setValue("timeoutMs", 1000);
    settings.setValue("reconnectIntervalMs", 2000);
    settings.endGroup();

    settings.beginGroup("Camera");
    settings.setValue("defaultCamera", "Mech-Eye Nano");
    settings.setValue("scanTimeoutMs", 5000);
    settings.endGroup();

    settings.beginGroup("Vision");
    settings.setValue("mechEyeCameraKey", "Mech-Eye Nano");
    settings.setValue("mechCaptureTimeoutMs", 5000);
    settings.setValue("hikConnectTimeoutMs", 3000);
    settings.setValue("hikCaptureTimeoutMs", 1000);
    settings.setValue("hikSdkRoot", "D:/work/scan-tracking/third_party/hik_mvs");
    settings.setValue("hikCameraAName", "hik_camera_a");
    settings.setValue("hikCameraAKey", "192.168.10.12");
    settings.setValue("hikCameraAIp", "192.168.10.12");
    settings.setValue("hikCameraASerial", "");
    settings.setValue("hikCameraBName", "hik_camera_b");
    settings.setValue("hikCameraBKey", "192.168.10.13");
    settings.setValue("hikCameraBIp", "192.168.10.13");
    settings.setValue("hikCameraBSerial", "");
    settings.endGroup();

    settings.beginGroup("LbPose");
    settings.setValue("dataRoot", "D:/work/scan-tracking/third_party/lb_pose_detection/data");
    settings.setValue("leftPattern", "");
    settings.setValue("rightPattern", "");
    settings.setValue("templateFile", "");
    settings.setValue("minDistance", 30.0);
    settings.setValue("maxDistance", 650.0);
    settings.setValue("cosTolerance", 0.015);
    settings.setValue("minPercent", 0.5);
    settings.endGroup();

    settings.beginGroup("FlowControl");
    settings.setValue("pollIntervalMs", 100);
    settings.setValue("heartbeatIntervalMs", 1000);
    settings.setValue("simulatedProcessingMs", 300);
    settings.endGroup();

    settings.beginGroup("Tracking");
    settings.setValue("firstStationOuterSegmentIndex", 1);
    settings.setValue("firstStationInnerSegmentIndex", 2);
    settings.setValue("firstStationHoleSegmentIndex", 3);
    settings.endGroup();

    settings.sync();
    qInfo(LOG_CONFIG) << "Generated default config.ini at" << settings.fileName();
}

void ConfigManager::load(const QString& filePath)
{
    const QFileInfo fileInfo(filePath);
    const bool fileExists = fileInfo.exists() && fileInfo.size() > 0;

    QSettings settings(filePath, QSettings::IniFormat);
    if (!fileExists) {
        qWarning(LOG_CONFIG) << "config.ini not found or empty. Generating defaults...";
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

    settings.beginGroup("Vision");
    m_visionConfig.mechEyeCameraKey = settings.value("mechEyeCameraKey", m_cameraConfig.defaultCamera).toString();
    m_visionConfig.mechCaptureTimeoutMs = settings.value("mechCaptureTimeoutMs", m_cameraConfig.scanTimeoutMs).toInt();
    m_visionConfig.hikConnectTimeoutMs = settings.value("hikConnectTimeoutMs", 3000).toInt();
    m_visionConfig.hikCaptureTimeoutMs = settings.value("hikCaptureTimeoutMs", 1000).toInt();
    m_visionConfig.hikSdkRoot = settings.value("hikSdkRoot", "D:/work/scan-tracking/third_party/hik_mvs").toString();
    m_visionConfig.hikCameraA.logicalName = settings.value("hikCameraAName", "hik_camera_a").toString();
    m_visionConfig.hikCameraA.cameraKey = settings.value("hikCameraAKey", "192.168.10.12").toString();
    m_visionConfig.hikCameraA.ipAddress = settings.value("hikCameraAIp", "192.168.10.12").toString();
    m_visionConfig.hikCameraA.serialNumber = settings.value("hikCameraASerial", "").toString();
    m_visionConfig.hikCameraB.logicalName = settings.value("hikCameraBName", "hik_camera_b").toString();
    m_visionConfig.hikCameraB.cameraKey = settings.value("hikCameraBKey", "192.168.10.13").toString();
    m_visionConfig.hikCameraB.ipAddress = settings.value("hikCameraBIp", "192.168.10.13").toString();
    m_visionConfig.hikCameraB.serialNumber = settings.value("hikCameraBSerial", "").toString();
    settings.endGroup();

    settings.beginGroup("LbPose");
    m_lbPoseConfig.dataRoot = settings.value("dataRoot", "D:/work/scan-tracking/third_party/lb_pose_detection/data").toString();
    m_lbPoseConfig.leftPattern = settings.value("leftPattern", "").toString();
    m_lbPoseConfig.rightPattern = settings.value("rightPattern", "").toString();
    m_lbPoseConfig.templateFile = settings.value("templateFile", "").toString();
    m_lbPoseConfig.minDistance = settings.value("minDistance", 30.0).toFloat();
    m_lbPoseConfig.maxDistance = settings.value("maxDistance", 650.0).toFloat();
    m_lbPoseConfig.cosTolerance = settings.value("cosTolerance", 0.015).toFloat();
    m_lbPoseConfig.minPercent = settings.value("minPercent", 0.5).toFloat();
    settings.endGroup();

    settings.beginGroup("FlowControl");
    m_flowControlConfig.pollIntervalMs = settings.value("pollIntervalMs", 100).toInt();
    m_flowControlConfig.heartbeatIntervalMs = settings.value("heartbeatIntervalMs", 1000).toInt();
    m_flowControlConfig.simulatedProcessingMs = settings.value("simulatedProcessingMs", 300).toInt();
    settings.endGroup();

    settings.beginGroup("Tracking");
    m_trackingConfig.firstStationOuterSegmentIndex = settings.value("firstStationOuterSegmentIndex", 1).toInt();
    m_trackingConfig.firstStationInnerSegmentIndex = settings.value("firstStationInnerSegmentIndex", 2).toInt();
    m_trackingConfig.firstStationHoleSegmentIndex = settings.value("firstStationHoleSegmentIndex", 3).toInt();
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

    qInfo(LOG_CONFIG) << "Loaded config.ini from:" << filePath;
}

}  // namespace common
}  // namespace scan_tracking
