#include "scan_tracking/vision/hik_camera_c_controller.h"

#include <QtCore/QDateTime>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QLoggingCategory>
#include <QtCore/QThread>

#include "scan_tracking/vision/hik_camera_service.h"
#include "scan_tracking/vision/hik_smart_camera_tcp_server.h"
#include "scan_tracking/vision/hik_smart_camera_ftp_monitor.h"

Q_LOGGING_CATEGORY(hikCControllerLog, "vision.hik_camera_c_controller")

namespace scan_tracking {
namespace vision {

void HikCameraCController::registerMetaTypes()
{
    static bool registered = false;
    if (registered) {
        return;
    }
    qRegisterMetaType<scan_tracking::vision::HikCameraCState>("scan_tracking::vision::HikCameraCState");
    qRegisterMetaType<scan_tracking::vision::CaptureType>("scan_tracking::vision::CaptureType");
    qRegisterMetaType<scan_tracking::vision::ImageFileInfo>("scan_tracking::vision::ImageFileInfo");
    registered = true;
}

HikCameraCController::HikCameraCController(
    HikCameraService* hikCameraCService,
    QObject* parent)
    : QObject(parent)
    , m_hikCameraCService(hikCameraCService)
    , m_tcpServer(nullptr)
    , m_testCaptureTimer(nullptr)
{
    registerMetaTypes();
}

HikCameraCController::~HikCameraCController()
{
    cleanupFtpMonitor();
    cleanupTcpServer();
    
    if (m_testCaptureTimer) {
        m_testCaptureTimer->stop();
        delete m_testCaptureTimer;
        m_testCaptureTimer = nullptr;
    }
}

void HikCameraCController::start(const scan_tracking::common::VisionConfig& config)
{
    if (m_started) {
        qWarning(hikCControllerLog) << "HikCameraCController already started.";
        return;
    }

    m_config = config;
    m_smartCameraIp = config.hikCameraC.ipAddress;  // 192.168.8.100
    m_ftpDirectory = QStringLiteral("D:\\HikCameraFTP");  // FTP 上传目录

    if (m_hikCameraCService == nullptr) {
        setState(HikCameraCState::Error, QStringLiteral("海康相机 C 服务未初始化"));
        emit fatalError(VisionErrorCode::InvalidConfig, QStringLiteral("HikCameraC service is null."));
        return;
    }

    // 连接相机状态变化信号
    connect(
        m_hikCameraCService,
        &HikCameraService::stateChanged,
        this,
        &HikCameraCController::onCameraCStateChanged,
        Qt::QueuedConnection);

    // 连接错误信号
    connect(
        m_hikCameraCService,
        &HikCameraService::fatalError,
        this,
        &HikCameraCController::onCameraError,
        Qt::QueuedConnection);

    m_started = true;
    setState(HikCameraCState::Initializing, QStringLiteral("海康相机 C 控制器正在初始化"));

    qInfo(hikCControllerLog) << "HikCameraCController started with camera:"
                             << m_config.hikCameraC.logicalName
                             << "IP:" << m_config.hikCameraC.ipAddress
                             << "Key:" << m_config.hikCameraC.cameraKey;

    // 初始化 TCP 服务器
    initializeTcpServer();

    // 初始化 FTP 监控器
    initializeFtpMonitor();

    // 检查相机是否已连接
    if (m_hikCameraCService->isConnected()) {
        qInfo(hikCControllerLog) << "Camera C SDK connection established";
    }
}

void HikCameraCController::stop()
{
    if (!m_started) {
        return;
    }

    // 停止测试定时器
    if (m_testCaptureTimer && m_testCaptureTimer->isActive()) {
        m_testCaptureTimer->stop();
        qInfo(hikCControllerLog) << "Test capture timer stopped";
    }

    m_started = false;
    cleanupFtpMonitor();
    cleanupTcpServer();
    setState(HikCameraCState::Stopped, QStringLiteral("海康相机 C 控制器已停止"));
    qInfo(hikCControllerLog) << "HikCameraCController stopped.";
}

void HikCameraCController::initializeTcpServer()
{
    if (m_tcpServer != nullptr) {
        qWarning(hikCControllerLog) << "TCP server already initialized";
        return;
    }

    m_tcpServer = new HikSmartCameraTcpServer(this);

    // 连接 TCP 服务器信号
    connect(m_tcpServer, &HikSmartCameraTcpServer::serverStarted,
            this, &HikCameraCController::onTcpServerStarted);
    connect(m_tcpServer, &HikSmartCameraTcpServer::serverStopped,
            this, &HikCameraCController::onTcpServerStopped);
    connect(m_tcpServer, &HikSmartCameraTcpServer::cameraConnected,
            this, &HikCameraCController::onTcpCameraConnected);
    connect(m_tcpServer, &HikSmartCameraTcpServer::cameraDisconnected,
            this, &HikCameraCController::onTcpCameraDisconnected);
    connect(m_tcpServer, &HikSmartCameraTcpServer::heartbeatReceived,
            this, &HikCameraCController::onTcpHeartbeatReceived);
    connect(m_tcpServer, &HikSmartCameraTcpServer::commandReceived,
            this, &HikCameraCController::onTcpCommandReceived);
    connect(m_tcpServer, &HikSmartCameraTcpServer::imageDataReceived,
            this, &HikCameraCController::onTcpImageDataReceived);
    connect(m_tcpServer, &HikSmartCameraTcpServer::error,
            this, &HikCameraCController::onTcpError);

    // 启动 TCP 服务器：监听 192.168.8.13:8999
    QString listenIp = QStringLiteral("192.168.8.13");
    quint16 listenPort = 8999;

    // 尝试启动服务器，如果失败则等待一段时间后重试
    if (!m_tcpServer->start(listenIp, listenPort)) {
        qWarning(hikCControllerLog) << "First attempt to start TCP server failed, waiting 2 seconds and retrying...";
        
        // 等待2秒让系统释放端口
        QThread::msleep(2000);
        
        // 重试一次
        if (!m_tcpServer->start(listenIp, listenPort)) {
            qCritical(hikCControllerLog) << "Failed to start TCP server on" << listenIp << ":" << listenPort << "after retry";
            setState(HikCameraCState::Error, QStringLiteral("TCP 服务器启动失败（端口可能被占用）"));
            emit fatalError(VisionErrorCode::DeviceOpenFailed, 
                          QStringLiteral("TCP server start failed. Port %1 may be in use by another process.").arg(listenPort));
        } else {
            qInfo(hikCControllerLog) << "TCP server started successfully on retry";
        }
    } else {
        qInfo(hikCControllerLog) << "TCP server started successfully on" << listenIp << ":" << listenPort;
    }
}
// 清理 TCP 服务器资源
void HikCameraCController::cleanupTcpServer()
{
    if (m_tcpServer != nullptr) {
        m_tcpServer->stop();
        m_tcpServer->deleteLater();
        m_tcpServer = nullptr;
        qInfo(hikCControllerLog) << "TCP server cleaned up";
    }
}

void HikCameraCController::initializeFtpMonitor()
{
    if (m_ftpMonitor != nullptr) {
        qWarning(hikCControllerLog) << "FTP monitor already initialized";
        return;
    }

    m_ftpMonitor = new HikSmartCameraFtpMonitor(this);

    // 连接 FTP 监控器信号
    connect(m_ftpMonitor, &HikSmartCameraFtpMonitor::monitorStarted,
            this, &HikCameraCController::onFtpMonitorStarted);
    connect(m_ftpMonitor, &HikSmartCameraFtpMonitor::monitorStopped,
            this, &HikCameraCController::onFtpMonitorStopped);
    connect(m_ftpMonitor, &HikSmartCameraFtpMonitor::newImageDetected,
            this, &HikCameraCController::onFtpNewImageDetected);
    connect(m_ftpMonitor, &HikSmartCameraFtpMonitor::imageReady,
            this, &HikCameraCController::onFtpImageReady);
    connect(m_ftpMonitor, &HikSmartCameraFtpMonitor::error,
            this, &HikCameraCController::onFtpError);

    // 启动 FTP 监控器
    if (!m_ftpMonitor->start(m_ftpDirectory)) {
        qCritical(hikCControllerLog) << "Failed to start FTP monitor for directory:" << m_ftpDirectory;
        emit fatalError(VisionErrorCode::InvalidConfig, 
                      QStringLiteral("FTP monitor start failed for directory: %1").arg(m_ftpDirectory));
    } else {
        qInfo(hikCControllerLog) << "FTP monitor started successfully for directory:" << m_ftpDirectory;
    }
}

void HikCameraCController::cleanupFtpMonitor()
{
    if (m_ftpMonitor != nullptr) {
        m_ftpMonitor->stop();
        m_ftpMonitor->deleteLater();
        m_ftpMonitor = nullptr;
        qInfo(hikCControllerLog) << "FTP monitor cleaned up";
    }
}

// 检查 TCP 服务器是否正在运行
bool HikCameraCController::isTcpServerRunning() const
{
    return m_tcpServer != nullptr && m_tcpServer->isListening();
}

bool HikCameraCController::isCameraConnectedToTcp() const
{
    if (m_tcpServer == nullptr) {
        return false;
    }
    return m_tcpServer->connectedCameras().contains(m_smartCameraIp);
}

bool HikCameraCController::isFtpMonitorRunning() const
{
    return m_ftpMonitor != nullptr && m_ftpMonitor->isMonitoring();
}

QString HikCameraCController::ftpDirectory() const
{
    return m_ftpDirectory;
}

bool HikCameraCController::requestCapture(CaptureType type)
{
    if (!isTcpServerRunning()) {
        qWarning(hikCControllerLog) << "Cannot request capture: TCP server not running";
        return false;
    }

    if (!isCameraConnectedToTcp()) {
        qWarning(hikCControllerLog) << "Cannot request capture: camera not connected to TCP";
        return false;
    }

    m_currentCaptureType = type;
    m_captureCounter++;
    
    qInfo(hikCControllerLog) << "Requesting capture #" << m_captureCounter 
                             << "from camera" << m_smartCameraIp
                             << "type:" << getCaptureTypeString(type);
    
    return m_tcpServer->sendStartCaptureToCamera(m_smartCameraIp);
}

void HikCameraCController::enableTestMode(bool enable, int intervalMs)
{
    if (enable) {
        if (m_testCaptureTimer == nullptr) {
            m_testCaptureTimer = new QTimer(this);
            connect(m_testCaptureTimer, &QTimer::timeout, 
                    this, &HikCameraCController::onTestCaptureTimer);
        }
        
        m_testCaptureTimer->setInterval(intervalMs);
        m_testCaptureTimer->start();
        qInfo(hikCControllerLog) << "Test mode enabled, capture interval:" << intervalMs << "ms";
    } else {
        if (m_testCaptureTimer && m_testCaptureTimer->isActive()) {
            m_testCaptureTimer->stop();
            qInfo(hikCControllerLog) << "Test mode disabled";
        }
    }
}

QString HikCameraCController::getCaptureTypeString(CaptureType type) const
{
    switch (type) {
        case CaptureType::SurfaceDefect:
            return QStringLiteral("SurfaceDefect");
        case CaptureType::WeldDefect:
            return QStringLiteral("WeldDefect");
        case CaptureType::NumberRecognition:
            return QStringLiteral("NumberRecognition");
        default:
            return QStringLiteral("Unknown");
    }
}

bool HikCameraCController::saveImageToFile(const QByteArray& imageData, CaptureType type)
{
    // 创建保存目录
    QString saveDir = QStringLiteral("./smart_camera_images");
    QDir dir;
    if (!dir.exists(saveDir)) {
        if (!dir.mkpath(saveDir)) {
            qWarning(hikCControllerLog) << "Failed to create directory:" << saveDir;
            return false;
        }
    }

    // 生成文件名：类型_时间戳_计数器.jpg
    QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    QString typeStr = getCaptureTypeString(type);
    QString filename = QString("%1/%2_%3_%4.jpg")
                           .arg(saveDir)
                           .arg(typeStr)
                           .arg(timestamp)
                           .arg(m_captureCounter, 4, 10, QChar('0'));

    // 保存文件
    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly)) {
        qWarning(hikCControllerLog) << "Failed to open file for writing:" << filename;
        return false;
    }

    qint64 written = file.write(imageData);
    file.close();

    if (written != imageData.size()) {
        qWarning(hikCControllerLog) << "Failed to write complete image data to file:" << filename;
        return false;
    }

    qInfo(hikCControllerLog) << "Image saved successfully:" << filename 
                             << "size:" << imageData.size() << "bytes";
    return true;
}

void HikCameraCController::setState(HikCameraCState state, const QString& description)
{
    if (m_state != state) {
        m_state = state;
        emit stateChanged(state, description);
        qInfo(hikCControllerLog) << "State changed to" << static_cast<int>(state) << ":" << description;
    }
}

// ============================================================================
// 相机服务信号槽
// ============================================================================

void HikCameraCController::onCameraCStateChanged(
    QString roleName,
    QString stateText,
    QString description)
{
    qInfo(hikCControllerLog) << "Camera SDK state:" << roleName << stateText << description;

    // 当相机 SDK 连接成功后，记录状态（但主要依赖 TCP 连接）
    if (stateText == QStringLiteral("ready") && description.contains(QStringLiteral("已连接"))) {
        qInfo(hikCControllerLog) << "Camera SDK connection ready (TCP communication will be used)";
    }
}

void HikCameraCController::onCameraError(
    scan_tracking::vision::VisionErrorCode code,
    QString message)
{
    qWarning(hikCControllerLog) << "Camera SDK error:" << static_cast<int>(code) << message;
    // 智能相机主要使用 TCP 通信，SDK 错误不一定影响功能
}

// ============================================================================
// TCP 服务器信号槽
// ============================================================================

void HikCameraCController::onTcpServerStarted(QString listenIp, quint16 port)
{
    qInfo(hikCControllerLog) << "TCP server started on" << listenIp << ":" << port;
    qInfo(hikCControllerLog) << "Waiting for smart camera" << m_smartCameraIp << "to connect...";
}

void HikCameraCController::onTcpServerStopped()
{
    qInfo(hikCControllerLog) << "TCP server stopped";
}

void HikCameraCController::onTcpCameraConnected(QString cameraIp, quint16 cameraPort)
{
    qInfo(hikCControllerLog) << "Smart camera connected via TCP:" << cameraIp << ":" << cameraPort;
    
    if (cameraIp == m_smartCameraIp) {
        setState(HikCameraCState::Ready, QStringLiteral("智能相机已通过 TCP 连接并就绪"));
        
        // 连接成功后，启动自动拍照测试（每10秒一次）
        enableTestMode(true, 10000);
        qInfo(hikCControllerLog) << "Auto capture enabled: every 10 seconds";
    } else {
        qWarning(hikCControllerLog) << "Unexpected camera IP connected:" << cameraIp 
                                    << "(expected:" << m_smartCameraIp << ")";
    }
}

void HikCameraCController::onTcpCameraDisconnected(QString cameraIp)
{
    qWarning(hikCControllerLog) << "Smart camera disconnected from TCP:" << cameraIp;
    
    if (cameraIp == m_smartCameraIp) {
        setState(HikCameraCState::Error, QStringLiteral("智能相机 TCP 连接断开"));
        
        // 停止测试定时器
        if (m_testCaptureTimer && m_testCaptureTimer->isActive()) {
            m_testCaptureTimer->stop();
            qInfo(hikCControllerLog) << "Test capture timer stopped due to disconnection";
        }
    }
}

void HikCameraCController::onTcpHeartbeatReceived(QString cameraIp)
{
    // 心跳日志可以设置为 Debug 级别，避免刷屏
    qDebug(hikCControllerLog) << "Heartbeat received from" << cameraIp;
}

void HikCameraCController::onTcpCommandReceived(QString cameraIp, QString command)
{
    qInfo(hikCControllerLog) << "Command received from" << cameraIp << ":" << command;
    // TODO: 处理相机发送的其他指令
}

void HikCameraCController::onTcpImageDataReceived(QString cameraIp, QByteArray imageData)
{
    qInfo(hikCControllerLog) << "Image data received from" << cameraIp 
                             << "size:" << imageData.size() << "bytes"
                             << "type:" << getCaptureTypeString(m_currentCaptureType);
    
    // 保存图像到文件
    if (saveImageToFile(imageData, m_currentCaptureType)) {
        qInfo(hikCControllerLog) << "Image saved successfully";
    } else {
        qWarning(hikCControllerLog) << "Failed to save image";
    }
    
    // 发射图像数据信号
    emit captureCompleted(m_currentCaptureType, imageData);
}

void HikCameraCController::onTcpError(QString errorMessage)
{
    qWarning(hikCControllerLog) << "TCP error:" << errorMessage;
    emit fatalError(VisionErrorCode::DeviceOpenFailed, errorMessage);
}

void HikCameraCController::onTestCaptureTimer()
{
    if (!m_started || !isCameraConnectedToTcp()) {
        qWarning(hikCControllerLog) << "Test capture skipped: not ready";
        return;
    }
    
    // 循环测试三种拍照类型
    CaptureType types[] = {
        CaptureType::SurfaceDefect,
        CaptureType::WeldDefect,
        CaptureType::NumberRecognition
    };
    
    static int typeIndex = 0;
    CaptureType currentType = types[typeIndex];
    typeIndex = (typeIndex + 1) % 3;
    
    qInfo(hikCControllerLog) << "Test capture triggered, type:" << getCaptureTypeString(currentType);
    requestCapture(currentType);
}

// ============================================================================
// FTP 监控器信号槽
// ============================================================================

void HikCameraCController::onFtpMonitorStarted(QString directory)
{
    qInfo(hikCControllerLog) << "FTP monitor started, watching directory:" << directory;
}

void HikCameraCController::onFtpMonitorStopped()
{
    qInfo(hikCControllerLog) << "FTP monitor stopped";
}

void HikCameraCController::onFtpNewImageDetected(ImageFileInfo imageInfo)
{
    // 静默处理，不打印日志
}

void HikCameraCController::onFtpImageReady(ImageFileInfo imageInfo)
{
    // 静默处理，只发射信号
    emit imageReceived(imageInfo.captureType, imageInfo.filePath, imageInfo.fileSize);
    
    // TODO: 这里可以触发后续的图像处理流程
    // 例如：调用缺陷识别算法、OCR识别等
}

void HikCameraCController::onFtpError(QString errorMessage)
{
    qWarning(hikCControllerLog) << "FTP monitor error:" << errorMessage;
}

}  // namespace vision
}  // namespace scan_tracking
