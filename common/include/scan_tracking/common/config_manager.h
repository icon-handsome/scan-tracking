#pragma once

#include <QSettings>
#include <QString>
#include <QtCore/QtGlobal>

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
    // "exclusive"（默认，独占控制）或 "monitor"（只读监控，允许与 SCMVS 共存）
    QString accessMode = QStringLiteral("exclusive");
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
    // 智能相机 C 的 TCP 服务端参数（IPC 作为服务端，相机主动连入）
    QString hikCameraCTcpListenIp;
    quint16 hikCameraCTcpListenPort;
    // 智能相机 C 的 FTP 落图目录（FileZilla Server 存图根目录）
    QString hikCameraCFtpDirectory;
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
