#pragma once

// 海康相机 C 控制器
// 独立管理第三台海康相机（智能相机/视觉传感器）
// 使用 TCP 通信协议进行控制和图像获取

#include <QtCore/QObject>
#include <QtCore/QTimer>

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/vision/vision_types.h"
#include "scan_tracking/vision/hik_smart_camera_ftp_monitor.h"

namespace scan_tracking {
namespace vision {

class HikCameraService;
class HikSmartCameraTcpServer;
class HikSmartCameraFtpMonitor;

enum class HikCameraCState {
    Idle = 0,
    Initializing = 1,
    Ready = 2,
    Error = 3,
    Stopped = 4,
};

// 拍照类型枚举已在 vision_types.h 中定义（CaptureType）

class HikCameraCController : public QObject {
    Q_OBJECT

public:
    explicit HikCameraCController(
        HikCameraService* hikCameraCService,
        QObject* parent = nullptr);
    ~HikCameraCController() override;

    void start(const scan_tracking::common::VisionConfig& config);
    void stop();
    bool isStarted() const { return m_started; }
    HikCameraCState state() const { return m_state; }

    // TCP 通信接口
    bool isTcpServerRunning() const;
    bool isCameraConnectedToTcp() const;
    
    // FTP 监控接口
    bool isFtpMonitorRunning() const;
    QString ftpDirectory() const;
    
    // 拍照接口
    bool requestCapture(CaptureType type = CaptureType::SurfaceDefect);
    
    // 测试接口
    void enableTestMode(bool enable, int intervalMs = 10000);

signals:
    void stateChanged(scan_tracking::vision::HikCameraCState state, QString description);
    void fatalError(scan_tracking::vision::VisionErrorCode code, QString message);
    void captureCompleted(CaptureType type, QByteArray imageData);
    void imageReceived(CaptureType type, QString filePath, qint64 fileSize);  // FTP图像接收信号

private slots:
    void onCameraCStateChanged(QString roleName, QString stateText, QString description);
    void onCameraError(scan_tracking::vision::VisionErrorCode code, QString message);
    
    // TCP 服务器信号槽
    void onTcpServerStarted(QString listenIp, quint16 port);
    void onTcpServerStopped();
    void onTcpCameraConnected(QString cameraIp, quint16 cameraPort);
    void onTcpCameraDisconnected(QString cameraIp);
    void onTcpHeartbeatReceived(QString cameraIp);
    void onTcpCommandReceived(QString cameraIp, QString command);
    void onTcpImageDataReceived(QString cameraIp, QByteArray imageData);
    void onTcpError(QString errorMessage);
    
    // FTP 监控器信号槽
    void onFtpMonitorStarted(QString directory);
    void onFtpMonitorStopped();
    void onFtpNewImageDetected(ImageFileInfo imageInfo);
    void onFtpImageReady(ImageFileInfo imageInfo);
    void onFtpError(QString errorMessage);
    
    // 测试定时器
    void onTestCaptureTimer();

private:
    static void registerMetaTypes();
    void setState(HikCameraCState state, const QString& description);
    void initializeTcpServer();
    void cleanupTcpServer();
    void initializeFtpMonitor();
    void cleanupFtpMonitor();
    bool saveImageToFile(const QByteArray& imageData, CaptureType type);
    QString getCaptureTypeString(CaptureType type) const;

    HikCameraService* m_hikCameraCService = nullptr;
    HikSmartCameraTcpServer* m_tcpServer = nullptr;
    HikSmartCameraFtpMonitor* m_ftpMonitor = nullptr;
    QTimer* m_testCaptureTimer = nullptr;
    scan_tracking::common::VisionConfig m_config;
    bool m_started = false;
    HikCameraCState m_state = HikCameraCState::Idle;
    QString m_smartCameraIp;      // 智能相机 IP，来自配置 hikCameraCIp
    QString m_tcpListenIp;        // TCP 服务端监听 IP，来自配置 hikCameraCTcpListenIp
    quint16 m_tcpListenPort = 0;  // TCP 服务端监听端口，来自配置 hikCameraCTcpListenPort
    QString m_ftpDirectory;       // FTP 落图目录，来自配置 hikCameraCFtpDirectory
    CaptureType m_currentCaptureType = CaptureType::SurfaceDefect;
    int m_captureCounter = 0;
};

}  // namespace vision
}  // namespace scan_tracking
