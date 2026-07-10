#pragma once

// 海康智能相机 C 控制器（视觉传感器）。
//
// 纯 TCP + FTP 模式：IPC 作 TCP 服务端接收相机心跳与回包，通过 FTP 目录监控落图；
// 不经过 MVS SDK 打开设备，可与 SCMVS 同时运行。
// 由 StateMachine / HmiTcpServer 触发表面缺陷、焊缝、编号识别等拍照任务。

#include <QtCore/QObject>
#include <QtCore/QTimer>

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/vision/vision_types.h"
#include "scan_tracking/vision/hik_smart_camera_ftp_monitor.h"

namespace scan_tracking {
namespace vision {

class HikSmartCameraTcpServer;
class HikSmartCameraFtpMonitor;

/// 智能相机 C 控制器生命周期状态
enum class HikCameraCState {
    Idle = 0,          ///< 未 start
    Initializing = 1,  ///< 正在启动 TCP 服务与 FTP 监控
    Ready = 2,         ///< 可接受拍照请求
    Error = 3,         ///< 不可恢复错误
    Stopped = 4,       ///< 已 stop
};

class HikCameraCController : public QObject {
    Q_OBJECT

public:
    explicit HikCameraCController(QObject* parent = nullptr);
    ~HikCameraCController() override;

    /* 根据 VisionConfig 启动 TCP 监听与 FTP 目录监控 */
    void start(const scan_tracking::common::VisionConfig& config);

    /* 停止 TCP/FTP 并进入 Stopped */
    void stop();

    bool isStarted() const { return m_started; }
    HikCameraCState state() const { return m_state; }

    bool isTcpServerRunning() const;
    bool isCameraConnectedToTcp() const;

    bool isFtpMonitorRunning() const;
    QString ftpDirectory() const;

    /* 向已连接的智能相机发送拍照指令（TCP start\r\n），类型决定后续 FTP 文件名解析 */
    bool requestCapture(CaptureType type = CaptureType::SurfaceDefect);

    /* 触发一次编号识别并等待 OCR 文本返回（发 start + 同步等待） */
    bool captureAndWaitForOcr(QString* codeValue, QString* errorMessage = nullptr, int timeoutMs = -1);

    /* path3 方案 B：Trig_ScanSegment 仅发 start，不等待 OCR */
    bool triggerCodeReadCapture(QString* errorMessage = nullptr);

    /* path3 方案 B：Trig_Inspection 读取 ScanSegment 阶段触发的 OCR（不再发 start） */
    bool collectCodeReadResult(
        QString* codeValue,
        QString* errorMessage = nullptr,
        int timeoutMs = -1);

    /* 启用周期性自动拍照（联调/冒烟，默认 10s 间隔） */
    void enableTestMode(bool enable, int intervalMs = 10000);

signals:
    void stateChanged(scan_tracking::vision::HikCameraCState state, QString description);
    void fatalError(scan_tracking::vision::VisionErrorCode code, QString message);
    void captureCompleted(CaptureType type, QByteArray imageData);
    void imageReceived(CaptureType type, QString filePath, qint64 fileSize);  ///< FTP 落图就绪
    void ocrResultReceived(CaptureType type, QString codeValue);

private slots:
    void onTcpServerStarted(QString listenIp, quint16 port);
    void onTcpServerStopped();
    void onTcpCameraConnected(QString cameraIp, quint16 cameraPort);
    void onTcpCameraDisconnected(QString cameraIp);
    void onTcpHeartbeatReceived(QString cameraIp);
    void onTcpCommandReceived(QString cameraIp, QString command);
    void onTcpImageDataReceived(QString cameraIp, QByteArray imageData);
    void onTcpError(QString errorMessage);

    void onFtpMonitorStarted(QString directory);
    void onFtpMonitorStopped();
    void onFtpNewImageDetected(ImageFileInfo imageInfo);
    void onFtpImageReady(ImageFileInfo imageInfo);
    void onFtpError(QString errorMessage);

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

    HikSmartCameraTcpServer* m_tcpServer = nullptr;
    HikSmartCameraFtpMonitor* m_ftpMonitor = nullptr;
    QTimer* m_testCaptureTimer = nullptr;
    scan_tracking::common::VisionConfig m_config;
    bool m_started = false;
    HikCameraCState m_state = HikCameraCState::Idle;
    QString m_smartCameraIp;      ///< 智能相机 IP，来自配置 hikCameraCIp
    QString m_tcpListenIp;        ///< TCP 服务端监听 IP
    quint16 m_tcpListenPort = 0;  ///< TCP 服务端监听端口
    QString m_ftpDirectory;       ///< FTP 落图根目录
    CaptureType m_currentCaptureType = CaptureType::SurfaceDefect;
    int m_captureCounter = 0;

    QString m_lastOcrText;        ///< OCR 结果限频：上次已打印文本
    qint64 m_lastOcrLogMs = 0;
    int m_suppressedOcrLogCount = 0;

    QString m_codeReadSessionValue;  ///< 当前编号识别会话 OCR（trigger 后由 TCP 回包写入）
};

}  // namespace vision
}  // namespace scan_tracking
