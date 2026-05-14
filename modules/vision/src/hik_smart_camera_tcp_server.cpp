#include "scan_tracking/vision/hik_smart_camera_tcp_server.h"

#include <QtCore/QDateTime>
#include <QtCore/QLoggingCategory>

Q_LOGGING_CATEGORY(hikTcpLog, "vision.hik_tcp_server")

namespace scan_tracking {
namespace vision {

// ============================================================================
// HikSmartCameraSession 实现
// ============================================================================

HikSmartCameraSession::HikSmartCameraSession(QTcpSocket* socket, QObject* parent)
    : QObject(parent)
    , m_socket(socket)
{
    if (m_socket) {
        m_socket->setParent(this);
        m_cameraIp = m_socket->peerAddress().toString();
        m_cameraPort = m_socket->peerPort();

        connect(m_socket, &QTcpSocket::readyRead, this, &HikSmartCameraSession::onReadyRead);
        connect(m_socket, &QTcpSocket::disconnected, this, &HikSmartCameraSession::onDisconnected);
        connect(m_socket, QOverload<QAbstractSocket::SocketError>::of(&QTcpSocket::error),
                this, &HikSmartCameraSession::onSocketError);

        updateHeartbeat();
        qInfo(hikTcpLog) << "Camera session created:" << m_cameraIp << ":" << m_cameraPort;
    }
}

HikSmartCameraSession::~HikSmartCameraSession()
{
    if (m_socket) {
        m_socket->disconnectFromHost();
        if (m_socket->state() != QAbstractSocket::UnconnectedState) {
            m_socket->waitForDisconnected(1000);
        }
    }
    qInfo(hikTcpLog) << "相机会话已销毁：" << m_cameraIp;
}

bool HikSmartCameraSession::isConnected() const
{
    return m_socket && m_socket->state() == QAbstractSocket::ConnectedState;
}

bool HikSmartCameraSession::sendCommand(const QString& command)
{
    if (!isConnected()) {
        qWarning(hikTcpLog) << "Cannot send command: not connected" << m_cameraIp;
        return false;
    }

    QByteArray data = command.toUtf8();
    if (!data.endsWith("\r\n")) {
        data.append("\r\n");
    }

    qint64 written = m_socket->write(data);
    if (written == -1) {
        qWarning(hikTcpLog) << "Failed to send command to" << m_cameraIp << ":" << command;
        return false;
    }

    m_socket->flush();
    qInfo(hikTcpLog) << "发送命令到" << m_cameraIp << "：" << command.trimmed();
    return true;
}

bool HikSmartCameraSession::sendStartCapture()
{
    return sendCommand("start");
}

void HikSmartCameraSession::onReadyRead()
{
    if (!m_socket) {
        return;
    }

    QByteArray data = m_socket->readAll();
    m_receiveBuffer.append(data);

    // 处理接收到的数据（按行分割）
    while (m_receiveBuffer.contains("\r\n")) {
        int index = m_receiveBuffer.indexOf("\r\n");
        QByteArray line = m_receiveBuffer.left(index);
        m_receiveBuffer.remove(0, index + 2);

        processReceivedData(line);
    }

    // 防止缓冲区无限增长
    if (m_receiveBuffer.size() > 1024 * 1024) {  // 1MB
        qWarning(hikTcpLog) << "Receive buffer too large, clearing:" << m_cameraIp;
        m_receiveBuffer.clear();
    }
}

void HikSmartCameraSession::processReceivedData(const QByteArray& data)
{
    QString message = QString::fromUtf8(data).trimmed();

    if (message.isEmpty()) {
        return;
    }

    qInfo(hikTcpLog) << "从" << m_cameraIp << "接收到：" << message;

    // 处理心跳包
    if (message == "hello") {
        updateHeartbeat();
        emit heartbeatReceived(m_cameraIp);
        return;
    }

    // 其他指令
    emit commandReceived(m_cameraIp, message);

    // TODO: 处理图像数据（如果相机发送图像）
    // 可能需要特殊的协议头来识别图像数据
}

void HikSmartCameraSession::updateHeartbeat()
{
    m_lastHeartbeatMs = QDateTime::currentMSecsSinceEpoch();
}

void HikSmartCameraSession::onDisconnected()
{
    qInfo(hikTcpLog) << "相机已断开连接：" << m_cameraIp;
    emit disconnected(m_cameraIp);
}

void HikSmartCameraSession::onSocketError(QAbstractSocket::SocketError socketError)
{
    QString errorMsg = m_socket ? m_socket->errorString() : "Unknown error";
    qWarning(hikTcpLog) << "Socket error for" << m_cameraIp << ":" << socketError << errorMsg;
    emit error(m_cameraIp, errorMsg);
}

// ============================================================================
// HikSmartCameraTcpServer 实现
// ============================================================================

HikSmartCameraTcpServer::HikSmartCameraTcpServer(QObject* parent)
    : QObject(parent)
    , m_tcpServer(new QTcpServer(this))
    , m_heartbeatTimer(new QTimer(this))
{
    connect(m_tcpServer, &QTcpServer::newConnection, this, &HikSmartCameraTcpServer::onNewConnection);

    m_heartbeatTimer->setInterval(5000);  // 每5秒检查一次心跳
    connect(m_heartbeatTimer, &QTimer::timeout, this, &HikSmartCameraTcpServer::onHeartbeatTimeout);
}

HikSmartCameraTcpServer::~HikSmartCameraTcpServer()
{
    stop();
}

bool HikSmartCameraTcpServer::start(const QString& listenIp, quint16 port)
{
    if (m_tcpServer->isListening()) {
        qWarning(hikTcpLog) << "Server already listening";
        return false;
    }

    QHostAddress address(listenIp);
    
    // 设置地址重用选项，允许快速重启
    m_tcpServer->setMaxPendingConnections(30);
    
    if (!m_tcpServer->listen(address, port)) {
        QString errorMsg = m_tcpServer->errorString();
        qCritical(hikTcpLog) << "Failed to start TCP server on" << listenIp << ":" << port << "-" << errorMsg;
        emit error(QStringLiteral("TCP 服务器启动失败: %1").arg(errorMsg));
        return false;
    }

    startHeartbeatMonitor();

    qInfo(hikTcpLog) << "TCP 服务器已启动，地址" << listenIp << ":" << port;
    emit serverStarted(listenIp, port);
    return true;
}

void HikSmartCameraTcpServer::stop()
{
    if (!m_tcpServer->isListening()) {
        return;
    }

    stopHeartbeatMonitor();

    // 断开所有会话
    for (auto* session : m_sessions) {
        session->deleteLater();
    }
    m_sessions.clear();

    m_tcpServer->close();

    qInfo(hikTcpLog) << "TCP 服务器已停止";
    emit serverStopped();
}

bool HikSmartCameraTcpServer::isListening() const
{
    return m_tcpServer->isListening();
}

QString HikSmartCameraTcpServer::serverAddress() const
{
    return m_tcpServer->serverAddress().toString();
}

quint16 HikSmartCameraTcpServer::serverPort() const
{
    return m_tcpServer->serverPort();
}

QStringList HikSmartCameraTcpServer::connectedCameras() const
{
    return m_sessions.keys();
}

bool HikSmartCameraTcpServer::sendCommandToCamera(const QString& cameraIp, const QString& command)
{
    if (!m_sessions.contains(cameraIp)) {
        qWarning(hikTcpLog) << "Camera not connected:" << cameraIp;
        return false;
    }

    return m_sessions[cameraIp]->sendCommand(command);
}

bool HikSmartCameraTcpServer::sendStartCaptureToCamera(const QString& cameraIp)
{
    return sendCommandToCamera(cameraIp, "start");
}

void HikSmartCameraTcpServer::onNewConnection()
{
    while (m_tcpServer->hasPendingConnections()) {
        QTcpSocket* socket = m_tcpServer->nextPendingConnection();
        if (!socket) {
            continue;
        }

        QString cameraIp = socket->peerAddress().toString();
        quint16 cameraPort = socket->peerPort();

        // 如果已存在连接，先断开旧的
        if (m_sessions.contains(cameraIp)) {
            qWarning(hikTcpLog) << "Camera already connected, replacing old session:" << cameraIp;
            m_sessions[cameraIp]->deleteLater();
            m_sessions.remove(cameraIp);
        }

        // 创建新会话
        auto* session = new HikSmartCameraSession(socket, this);
        m_sessions[cameraIp] = session;

        // 连接信号
        connect(session, &HikSmartCameraSession::heartbeatReceived,
                this, &HikSmartCameraTcpServer::heartbeatReceived);
        connect(session, &HikSmartCameraSession::commandReceived,
                this, &HikSmartCameraTcpServer::commandReceived);
        connect(session, &HikSmartCameraSession::imageDataReceived,
                this, &HikSmartCameraTcpServer::imageDataReceived);
        connect(session, &HikSmartCameraSession::disconnected,
                this, &HikSmartCameraTcpServer::onSessionDisconnected);
        connect(session, &HikSmartCameraSession::error,
                [this](const QString& cameraIp, const QString& errorMsg) {
                    emit error(QStringLiteral("相机 %1 错误: %2").arg(cameraIp, errorMsg));
                });

        qInfo(hikTcpLog) << "Camera connected:" << cameraIp << ":" << cameraPort;
        emit cameraConnected(cameraIp, cameraPort);
    }
}

void HikSmartCameraTcpServer::onSessionDisconnected(QString cameraIp)
{
    if (m_sessions.contains(cameraIp)) {
        m_sessions[cameraIp]->deleteLater();
        m_sessions.remove(cameraIp);
        qInfo(hikTcpLog) << "Session removed:" << cameraIp;
        emit cameraDisconnected(cameraIp);
    }
}

void HikSmartCameraTcpServer::onHeartbeatTimeout()
{
    qint64 now = QDateTime::currentMSecsSinceEpoch();
    QStringList timeoutCameras;

    for (auto it = m_sessions.begin(); it != m_sessions.end(); ++it) {
        qint64 elapsed = now - it.value()->lastHeartbeatTime();
        if (elapsed > m_heartbeatTimeoutMs) {
            qWarning(hikTcpLog) << "Camera heartbeat timeout:" << it.key()
                                << "elapsed:" << elapsed << "ms";
            timeoutCameras.append(it.key());
        }
    }

    // 断开超时的相机
    for (const QString& cameraIp : timeoutCameras) {
        if (m_sessions.contains(cameraIp)) {
            m_sessions[cameraIp]->deleteLater();
            m_sessions.remove(cameraIp);
            emit cameraDisconnected(cameraIp);
            emit error(QStringLiteral("相机 %1 心跳超时").arg(cameraIp));
        }
    }
}

void HikSmartCameraTcpServer::startHeartbeatMonitor()
{
    if (!m_heartbeatTimer->isActive()) {
        m_heartbeatTimer->start();
        qInfo(hikTcpLog) << "心跳监控已启动";
    }
}

void HikSmartCameraTcpServer::stopHeartbeatMonitor()
{
    if (m_heartbeatTimer->isActive()) {
        m_heartbeatTimer->stop();
        qInfo(hikTcpLog) << "心跳监控已停止";
    }
}

}  // namespace vision
}  // namespace scan_tracking
