/**
 * @file hmi_session.cpp
 * @brief HMI 单客户端 TCP 会话管理实现
 *
 * 实现基于 [4字节大端长度头 + JSON] 的帧协议解析，
 * 处理 TCP 流式传输中的粘包和半包问题。
 */

#include "scan_tracking/hmi_server/hmi_session.h"
#include "scan_tracking/hmi_server/hmi_protocol.h"

#include <QtCore/QJsonDocument>
#include <QtCore/QLoggingCategory>
#include <QtNetwork/QTcpSocket>
#include <QtEndian>
#include<qtcpserver.h>

namespace scan_tracking {
namespace hmi_server {

Q_LOGGING_CATEGORY(LOG_SESSION, "hmi.session")

HmiSession::HmiSession(QTcpSocket* socket, QObject* parent)
    : QObject(parent)
    , m_socket(socket)
    , m_heartbeatTimer(new QTimer(this))
{
    // 将 socket 的父对象设置为当前 session，确保生命周期管理
    m_socket->setParent(this);

    // 连接 socket 信号
    connect(m_socket, &QTcpSocket::readyRead, this, &HmiSession::onReadyRead);
    connect(m_socket, &QTcpSocket::disconnected, this, &HmiSession::onSocketDisconnected);

    // 配置心跳超时定时器
    m_heartbeatTimer->setSingleShot(true);
    m_heartbeatTimer->setInterval(kHeartbeatTimeoutMs);
    connect(m_heartbeatTimer, &QTimer::timeout, this, &HmiSession::onHeartbeatTimeout);
    m_heartbeatTimer->start();

    qInfo(LOG_SESSION) << "HMI 会话已建立，客户端:" << peerDescription();
}

HmiSession::~HmiSession()
{
    qInfo(LOG_SESSION) << "HMI 会话已销毁，客户端:" << peerDescription();
}

bool HmiSession::isConnected() const
{
    return m_socket != nullptr && m_socket->state() == QAbstractSocket::ConnectedState;
}

QString HmiSession::peerDescription() const
{
    if (m_socket == nullptr) {
        return QStringLiteral("(null)");
    }
    return QStringLiteral("%1:%2")
        .arg(m_socket->peerAddress().toString())
        .arg(m_socket->peerPort());
}

void HmiSession::sendMessage(const QJsonObject& envelope)
{
    if (!isConnected()) {
        return;
    }

    const QByteArray frame = serializeFrame(envelope);
    m_socket->write(frame);
}

void HmiSession::disconnect()
{
    if (m_socket != nullptr) {
        m_socket->disconnectFromHost();
    }
}

void HmiSession::resetHeartbeatTimer()
{
    if (m_heartbeatTimer != nullptr) {
        m_heartbeatTimer->start(kHeartbeatTimeoutMs);
    }
}

void HmiSession::onReadyRead()
{
    // 将所有可用数据追加到内部缓冲区
    m_buffer.append(m_socket->readAll());

    // 循环解帧：处理缓冲区中可能存在的多条完整消息（粘包）
    while (m_buffer.size() >= 4) {
        // 读取 4 字节大端长度头
        const quint32 jsonLength = qFromBigEndian<quint32>(
            reinterpret_cast<const uchar*>(m_buffer.constData()));

        // 安全检查：帧大小不超过最大限制
        if (jsonLength > kMaxFrameSize) {
            qWarning(LOG_SESSION) << "收到异常帧，长度=" << jsonLength
                                  << "超过最大限制，断开连接";
            disconnect();
            return;
        }

        // 如果缓冲区数据不足一帧，等待更多数据（半包处理）
        const int totalFrameSize = 4 + static_cast<int>(jsonLength);
        if (m_buffer.size() < totalFrameSize) {
            break;
        }

        // 提取 JSON 正文并从缓冲区中移除已消费的数据
        const QByteArray jsonBytes = m_buffer.mid(4, static_cast<int>(jsonLength));
        m_buffer.remove(0, totalFrameSize);

        // 解析 JSON
        QJsonParseError parseError;
        const QJsonDocument doc = QJsonDocument::fromJson(jsonBytes, &parseError);
        if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
            qWarning(LOG_SESSION) << "JSON 解析失败:" << parseError.errorString();
            continue;
        }

        // 收到有效消息，重置心跳计时器
        resetHeartbeatTimer();

        // 发出消息接收信号，交由 HmiTcpServer 处理
        emit messageReceived(doc.object());
    }
}

void HmiSession::onSocketDisconnected()
{
    qInfo(LOG_SESSION) << "HMI 客户端断开连接:" << peerDescription();
    m_heartbeatTimer->stop();
    emit disconnected();
}

void HmiSession::onHeartbeatTimeout()
{
    qWarning(LOG_SESSION) << "HMI 客户端心跳超时:" << peerDescription();
    emit heartbeatTimeout();
}

}  // namespace hmi_server
}  // namespace scan_tracking
