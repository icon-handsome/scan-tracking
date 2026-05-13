#pragma once

#include <QSettings>
#include <QString>

namespace scan_tracking {
namespace common {

struct AppConfig {
    QString version;
    QString environment;
};

struct LoggerConfig {
    int level;
    int rotateDays;
};

struct ModbusConfig {
    QString host;
    int port;
    int unitId;
    int timeoutMs;
    int reconnectIntervalMs;
};

struct CameraConfig {
    QString defaultCamera;
    int scanTimeoutMs;
};

struct VisionCameraEndpointConfig {
    QString logicalName;
    QString cameraKey;
    QString ipAddress;
    QString serialNumber;
};

struct VisionConfig {
    QString mechEyeCameraKey;
    int mechCaptureTimeoutMs;
    int hikConnectTimeoutMs;
    int hikCaptureTimeoutMs;
    QString hikSdkRoot;
    VisionCameraEndpointConfig hikCameraA;
    VisionCameraEndpointConfig hikCameraB;
    VisionCameraEndpointConfig hikCameraC;
};

struct FlowControlConfig {
    int pollIntervalMs;
    int heartbeatIntervalMs;
    int simulatedProcessingMs;
};

struct TrackingConfig {
    int firstStationOuterSegmentIndex;
    int firstStationInnerSegmentIndex;
    int firstStationHoleSegmentIndex;
};

struct LbPoseConfig {
    QString dataRoot;
    QString leftPattern;
    QString rightPattern;
    QString templateFile;
    float minDistance;
    float maxDistance;
    float cosTolerance;
    float minPercent;
};

class ConfigManager {
public:
    static void initialize();
    static void cleanup();
    static ConfigManager* instance();

    const AppConfig& appConfig() const;
    const LoggerConfig& loggerConfig() const;
    const ModbusConfig& modbusConfig() const;
    const CameraConfig& cameraConfig() const;
    const VisionConfig& visionConfig() const;
    const FlowControlConfig& flowControlConfig() const;
    const TrackingConfig& trackingConfig() const;
    const LbPoseConfig& lbPoseConfig() const;

private:
    ConfigManager();
    ~ConfigManager();

    ConfigManager(const ConfigManager&) = delete;
    ConfigManager& operator=(const ConfigManager&) = delete;

    void load(const QString& filePath);
    void writeDefaults(QSettings& settings);

    static ConfigManager* s_instance;

    AppConfig m_appConfig;
    LoggerConfig m_loggerConfig;
    ModbusConfig m_modbusConfig;
    CameraConfig m_cameraConfig;
    VisionConfig m_visionConfig;
    FlowControlConfig m_flowControlConfig;
    TrackingConfig m_trackingConfig;
    LbPoseConfig m_lbPoseConfig;
};

}  // namespace common
}  // namespace scan_tracking
