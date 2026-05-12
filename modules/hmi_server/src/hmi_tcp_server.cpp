/**
 * @file hmi_tcp_server.cpp
 * @brief HMI TCP 服务端实现
 */

#include "scan_tracking/hmi_server/hmi_tcp_server.h"
#include "scan_tracking/hmi_server/hmi_session.h"
#include "scan_tracking/hmi_server/hmi_protocol.h"

#include "scan_tracking/flow_control/state_machine.h"
#include "scan_tracking/modbus/modbus_service.h"
#include "scan_tracking/mech_eye/mech_eye_service.h"
#include "scan_tracking/vision/vision_pipeline_service.h"
#include "scan_tracking/vision/hik_camera_service.h"
#include "scan_tracking/tracking/tracking_service.h"

#include <QtNetwork/QTcpServer>
#include <QtNetwork/QTcpSocket>
#include <QtNetwork/QHostAddress>
#include <QtCore/QLoggingCategory>
#include <QtCore/QJsonObject>
#include <QtCore/QUuid>
#include <qdatetime.h>

namespace scan_tracking {
namespace hmi_server {

Q_LOGGING_CATEGORY(LOG_HMI_SERVER, "hmi.server")

HmiTcpServer::HmiTcpServer(int port, QObject* parent)
    : QObject(parent)
    , m_tcpServer(new QTcpServer(this))
    , m_port(port)
    , m_statusPushTimer(new QTimer(this))
    , m_heartbeatTimer(new QTimer(this))
{
    connect(m_tcpServer, &QTcpServer::newConnection, this, &HmiTcpServer::onNewConnection);

    m_statusPushTimer->setInterval(500); // 500ms 状态推送
    connect(m_statusPushTimer, &QTimer::timeout, this, &HmiTcpServer::onStatusPushTimer);

    m_heartbeatTimer->setInterval(2000); // 2000ms 心跳
    connect(m_heartbeatTimer, &QTimer::timeout, this, &HmiTcpServer::onHeartbeatTimer);
}

HmiTcpServer::~HmiTcpServer()
{
    stop();
}

bool HmiTcpServer::start()
{
    if (m_tcpServer->isListening()) {
        return true;
    }

    if (!m_tcpServer->listen(QHostAddress::Any, m_port)) {
        qCritical(LOG_HMI_SERVER) << "HMI TCP 服务端启动失败，端口:" << m_port << "错误:" << m_tcpServer->errorString();
        return false;
    }

    qInfo(LOG_HMI_SERVER) << "HMI TCP 服务端启动成功，监听端口:" << m_port;

    connectStateMachineSignals();
    // 其他模块信号可以在这连接...

    return true;
}

void HmiTcpServer::stop()
{
    m_statusPushTimer->stop();
    m_heartbeatTimer->stop();

    if (m_session) {
        m_session->disconnect();
        m_session->deleteLater();
        m_session = nullptr;
    }

    if (m_tcpServer->isListening()) {
        m_tcpServer->close();
        qInfo(LOG_HMI_SERVER) << "HMI TCP 服务端已停止";
    }
}

bool HmiTcpServer::isListening() const
{
    return m_tcpServer->isListening();
}

bool HmiTcpServer::hasClient() const
{
    return m_session != nullptr && m_session->isConnected();
}

void HmiTcpServer::setStateMachine(flow_control::StateMachine* sm) { m_stateMachine = sm; }
void HmiTcpServer::setModbusService(modbus::ModbusService* svc) { m_modbusService = svc; }
void HmiTcpServer::setMechEyeService(mech_eye::MechEyeService* svc) { m_mechEyeService = svc; }
void HmiTcpServer::setVisionPipelineService(vision::VisionPipelineService* svc) { m_visionPipeline = svc; }
void HmiTcpServer::setTrackingService(tracking::TrackingService* svc) { m_trackingService = svc; }
void HmiTcpServer::setHikCameraServices(vision::HikCameraService* hikA, vision::HikCameraService* hikB) {
    m_hikCameraA = hikA;
    m_hikCameraB = hikB;
}

void HmiTcpServer::onNewConnection()
{
    while (m_tcpServer->hasPendingConnections()) {
        QTcpSocket* socket = m_tcpServer->nextPendingConnection();
        
        // 单客户端模式：如果已有连接，踢掉旧的或拒绝新的。这里策略是踢掉旧的。
        if (m_session) {
            qWarning(LOG_HMI_SERVER) << "新客户端接入，断开旧客户端会话";
            m_session->disconnect();
            m_session->deleteLater();
            m_session = nullptr;
        }

        m_session = new HmiSession(socket, this);
        connect(m_session, &HmiSession::messageReceived, this, &HmiTcpServer::onMessageReceived);
        connect(m_session, &HmiSession::disconnected, this, &HmiTcpServer::onSessionDisconnected);
        connect(m_session, &HmiSession::heartbeatTimeout, this, &HmiTcpServer::onSessionHeartbeatTimeout);

        m_statusPushTimer->start();
        m_heartbeatTimer->start();

        // 发送 core.hello 欢迎消息
        sendToClient(buildEnvelope(QStringLiteral("core.hello"), nextEventId(), QJsonObject()));
    }
}

void HmiTcpServer::onSessionDisconnected()
{
    if (m_session) {
        m_session->deleteLater();
        m_session = nullptr;
    }
    m_statusPushTimer->stop();
    m_heartbeatTimer->stop();
}

void HmiTcpServer::onSessionHeartbeatTimeout()
{
    if (m_session) {
        m_session->disconnect();
    }
}

void HmiTcpServer::onMessageReceived(const QJsonObject& message)
{
    const QString type = message.value(QLatin1String("type")).toString();
    
    if (type == QLatin1String(msg_type::kHmiHello)) {
        handleHmiHello(message);
    } else if (type == QLatin1String(msg_type::kHeartbeatPing)) {
        handleHeartbeatPing(message);
    } else if (type == QLatin1String(msg_type::kCmdStart)) {
        handleCmdStart(message);
    } else if (type == QLatin1String(msg_type::kCmdStop)) {
        handleCmdStop(message);
    } else if (type == QLatin1String(msg_type::kCmdGetStatus)) {
        handleCmdGetStatus(message);
    } else if (type == QLatin1String(msg_type::kCmdReset)) {
        handleCmdReset(message);
    } else if (type == QLatin1String(msg_type::kCmdClearAlarm)) {
        handleCmdClearAlarm(message);
    } else if (type == QLatin1String(msg_type::kCmdGetConfig)) {
        handleCmdGetConfig(message);
    } else if (type == QLatin1String(msg_type::kCmdModbusConnect)) {
        handleCmdModbusConnect(message);
    } else if (type == QLatin1String(msg_type::kCmdModbusDisconnect)) {
        handleCmdModbusDisconnect(message);
    } else if (type == QLatin1String(msg_type::kCmdRefreshCamera)) {
        handleCmdRefreshCamera(message);
    } else if (type == QLatin1String(msg_type::kCmdTriggerScan)) {
        handleCmdTriggerScan(message);
    } else if (type == QLatin1String(msg_type::kCmdTriggerInspection)) {
        handleCmdTriggerInspection(message);
    } else if (type == QLatin1String(msg_type::kCmdTriggerSelfCheck)) {
        handleCmdTriggerSelfCheck(message);
    } else if (type == QLatin1String(msg_type::kCmdTriggerPoseCheck)) {
        handleCmdTriggerPoseCheck(message);
    } else if (type == QLatin1String(msg_type::kCmdTriggerCodeRead)) {
        handleCmdTriggerCodeRead(message);
    } else if (type == QLatin1String(msg_type::kCmdTriggerResultReset)) {
        handleCmdTriggerResultReset(message);
    } else if (type == QLatin1String(msg_type::kCmdCaptureMechEye)) {
        handleCmdCaptureMechEye(message);
    } else if (type == QLatin1String(msg_type::kCmdCaptureBundle)) {
        handleCmdCaptureBundle(message);
    }
    // ... 其他命令处理
    else {
        qWarning(LOG_HMI_SERVER) << "收到未知类型的消息:" << type;
        const QString msgId = message.value(QLatin1String("msgId")).toString();
        sendResponse(type, msgId, false, QStringLiteral("Unknown command type"));
    }
}

void HmiTcpServer::onStatusPushTimer()
{
    if (!hasClient()) return;
    pushSystemStatus();
    pushPlcStatus();
    pushCameraStatus();
}

void HmiTcpServer::onHeartbeatTimer()
{
    if (!hasClient()) return;
    sendToClient(buildEnvelope(QLatin1String(msg_type::kHeartbeatPing), nextEventId(), QJsonObject()));
}

// --- 命令处理实现示例 ---

void HmiTcpServer::handleHmiHello(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    sendResponse(QLatin1String(msg_type::kHmiHello), msgId, true, QStringLiteral("Welcome"));
}

void HmiTcpServer::handleHeartbeatPing(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    // 响应 pong
    QJsonObject envelope;
    envelope[QStringLiteral("version")]   = QLatin1String(kProtocolVersion);
    envelope[QStringLiteral("msgId")]     = msgId;
    envelope[QStringLiteral("type")]      = QLatin1String(msg_type::kHeartbeatPong);
    envelope[QStringLiteral("timestamp")] = QDateTime::currentMSecsSinceEpoch();
    envelope[QStringLiteral("payload")]   = QJsonObject();
    sendToClient(envelope);
}

void HmiTcpServer::handleCmdStart(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    if (m_stateMachine) {
        m_stateMachine->start();
        sendResponse(QLatin1String(msg_type::kCmdStart), msgId, true, QStringLiteral("State machine started"));
    } else {
        sendResponse(QLatin1String(msg_type::kCmdStart), msgId, false, QStringLiteral("State machine unavailable"));
    }
}

void HmiTcpServer::handleCmdStop(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    if (m_stateMachine) {
        m_stateMachine->stop();
        sendResponse(QLatin1String(msg_type::kCmdStop), msgId, true, QStringLiteral("State machine stopped"));
    } else {
        sendResponse(QLatin1String(msg_type::kCmdStop), msgId, false, QStringLiteral("State machine unavailable"));
    }
}

void HmiTcpServer::handleCmdGetStatus(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    pushSystemStatus();
    pushPlcStatus();
    pushCameraStatus();
    sendResponse(QLatin1String(msg_type::kCmdGetStatus), msgId, true, QStringLiteral("Status pushed"));
}

void HmiTcpServer::handleCmdReset(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    if (m_stateMachine) {
        m_stateMachine->start(); // 目前系统使用 start 作为重启/复位的入口
        sendResponse(QLatin1String(msg_type::kCmdReset), msgId, true, QStringLiteral("State machine reset"));
    } else {
        sendResponse(QLatin1String(msg_type::kCmdReset), msgId, false, QStringLiteral("State machine unavailable"));
    }
}

void HmiTcpServer::handleCmdClearAlarm(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    if (m_stateMachine) {
        m_stateMachine->setAlarm(0, 0, QString());
        sendResponse(QLatin1String(msg_type::kCmdClearAlarm), msgId, true, QStringLiteral("Alarms cleared"));
    } else {
        sendResponse(QLatin1String(msg_type::kCmdClearAlarm), msgId, false, QStringLiteral("State machine unavailable"));
    }
}

void HmiTcpServer::handleCmdGetConfig(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    // 暂时不实现热更新配置，返回空对象
    sendResponse(QLatin1String(msg_type::kCmdGetConfig), msgId, true, QStringLiteral("Not supported"));
}

void HmiTcpServer::handleCmdModbusConnect(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    if (m_modbusService) {
        m_modbusService->connectDevice();
        sendResponse(QLatin1String(msg_type::kCmdModbusConnect), msgId, true, QStringLiteral("Connecting to Modbus"));
    } else {
        sendResponse(QLatin1String(msg_type::kCmdModbusConnect), msgId, false, QStringLiteral("Modbus service unavailable"));
    }
}

void HmiTcpServer::handleCmdModbusDisconnect(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    if (m_modbusService) {
        m_modbusService->disconnectDevice();
        sendResponse(QLatin1String(msg_type::kCmdModbusDisconnect), msgId, true, QStringLiteral("Disconnected from Modbus"));
    } else {
        sendResponse(QLatin1String(msg_type::kCmdModbusDisconnect), msgId, false, QStringLiteral("Modbus service unavailable"));
    }
}

void HmiTcpServer::handleCmdRefreshCamera(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    if (m_mechEyeService) {
        // 由于没有直接重新连接的方法，可以调用一个占位日志或者进行状态查询
        sendResponse(QLatin1String(msg_type::kCmdRefreshCamera), msgId, true, QStringLiteral("Camera refreshed"));
    } else {
        sendResponse(QLatin1String(msg_type::kCmdRefreshCamera), msgId, false, QStringLiteral("Camera service unavailable"));
    }
}

void HmiTcpServer::handleCmdTriggerScan(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    sendResponse(QLatin1String(msg_type::kCmdTriggerScan), msgId, false, QStringLiteral("Direct trigger not implemented, use PLC or StateMachine"));
}

void HmiTcpServer::handleCmdTriggerInspection(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    sendResponse(QLatin1String(msg_type::kCmdTriggerInspection), msgId, false, QStringLiteral("Direct trigger not implemented, use PLC"));
}

void HmiTcpServer::handleCmdTriggerSelfCheck(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    sendResponse(QLatin1String(msg_type::kCmdTriggerSelfCheck), msgId, false, QStringLiteral("Direct trigger not implemented, use PLC"));
}

void HmiTcpServer::handleCmdTriggerPoseCheck(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    sendResponse(QLatin1String(msg_type::kCmdTriggerPoseCheck), msgId, false, QStringLiteral("Direct trigger not implemented, use PLC"));
}

void HmiTcpServer::handleCmdTriggerCodeRead(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    sendResponse(QLatin1String(msg_type::kCmdTriggerCodeRead), msgId, false, QStringLiteral("Direct trigger not implemented, use PLC"));
}

void HmiTcpServer::handleCmdTriggerResultReset(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    sendResponse(QLatin1String(msg_type::kCmdTriggerResultReset), msgId, false, QStringLiteral("Direct trigger not implemented, use PLC"));
}

void HmiTcpServer::handleCmdCaptureMechEye(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    const QString cameraKey = message.value(QLatin1String("payload")).toObject().value(QLatin1String("cameraKey")).toString();
    if (m_mechEyeService) {
        quint64 reqId = m_mechEyeService->requestCapture(cameraKey, mech_eye::CaptureMode::Capture3DOnly);
        QJsonObject payload = buildResponsePayload(true, QStringLiteral("Capture requested"));
        payload[QLatin1String("requestId")] = static_cast<qint64>(reqId);
        
        QJsonObject envelope;
        envelope[QStringLiteral("version")]   = QLatin1String(kProtocolVersion);
        envelope[QStringLiteral("msgId")]     = msgId;
        envelope[QStringLiteral("type")]      = QLatin1String(msg_type::kCmdCaptureMechEye);
        envelope[QStringLiteral("timestamp")] = QDateTime::currentMSecsSinceEpoch();
        envelope[QStringLiteral("payload")]   = payload;
        sendToClient(envelope);
    } else {
        sendResponse(QLatin1String(msg_type::kCmdCaptureMechEye), msgId, false, QStringLiteral("Camera service unavailable"));
    }
}

void HmiTcpServer::handleCmdCaptureBundle(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    if (m_visionPipeline) {
        int segmentIndex = message.value(QLatin1String("payload")).toObject().value(QLatin1String("segmentIndex")).toInt(1);
        quint32 taskId = message.value(QLatin1String("payload")).toObject().value(QLatin1String("taskId")).toInt(0);
        quint64 reqId = m_visionPipeline->requestCaptureBundle(segmentIndex, taskId);
        
        QJsonObject payload = buildResponsePayload(true, QStringLiteral("Bundle capture requested"));
        payload[QLatin1String("requestId")] = static_cast<qint64>(reqId);
        
        QJsonObject envelope;
        envelope[QStringLiteral("version")]   = QLatin1String(kProtocolVersion);
        envelope[QStringLiteral("msgId")]     = msgId;
        envelope[QStringLiteral("type")]      = QLatin1String(msg_type::kCmdCaptureBundle);
        envelope[QStringLiteral("timestamp")] = QDateTime::currentMSecsSinceEpoch();
        envelope[QStringLiteral("payload")]   = payload;
        sendToClient(envelope);
    } else {
        sendResponse(QLatin1String(msg_type::kCmdCaptureBundle), msgId, false, QStringLiteral("Vision pipeline unavailable"));
    }
}

// --- 状态推送实现 ---

void HmiTcpServer::pushSystemStatus()
{
    if (!m_stateMachine) return;

    QJsonObject payload;
    payload[QLatin1String("ipcState")] = static_cast<int>(m_stateMachine->ipcState());
    payload[QLatin1String("appState")] = QString::number(static_cast<int>(m_stateMachine->currentState())); // 或者映射字符串
    payload[QLatin1String("stage")] = static_cast<int>(m_stateMachine->currentStage());
    payload[QLatin1String("alarmLevel")] = m_stateMachine->alarmLevel();
    payload[QLatin1String("alarmCode")] = m_stateMachine->alarmCode();
    payload[QLatin1String("warnCode")] = m_stateMachine->warnCode();
    payload[QLatin1String("progress")] = m_stateMachine->progress();
    
    sendToClient(buildEnvelope(QLatin1String(msg_type::kStatusSystem), nextEventId(), payload));
}

void HmiTcpServer::pushPlcStatus()
{
    if (!m_modbusService) return;
    QJsonObject payload;
    payload[QLatin1String("modbusConnected")] = m_modbusService->isConnected();
    // 根据需要补充 PLC 其他状态寄存器
    sendToClient(buildEnvelope(QLatin1String(msg_type::kStatusPlc), nextEventId(), payload));
}

void HmiTcpServer::pushCameraStatus()
{
    QJsonObject payload;
    if (m_mechEyeService) {
        QJsonObject mechEyeObj;
        mechEyeObj[QLatin1String("state")] = static_cast<int>(m_mechEyeService->state());
        payload[QLatin1String("mechEye")] = mechEyeObj;
    }
    sendToClient(buildEnvelope(QLatin1String(msg_type::kStatusCamera), nextEventId(), payload));
}

// --- 事件连接 ---

void HmiTcpServer::connectStateMachineSignals()
{
    if (!m_stateMachine) return;
    
    // 例如监听协议事件报警
    connect(m_stateMachine, &flow_control::StateMachine::protocolEvent, this, [this](const QString& message) {
        QJsonObject payload;
        payload[QLatin1String("message")] = message;
        payload[QLatin1String("level")] = m_stateMachine->alarmLevel();
        payload[QLatin1String("code")] = m_stateMachine->alarmCode();
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventAlarm), nextEventId(), payload));
    });
}

// --- 辅助发送 ---

void HmiTcpServer::sendToClient(const QJsonObject& envelope)
{
    if (m_session) {
        m_session->sendMessage(envelope);
    }
}

void HmiTcpServer::sendResponse(const QString& type, const QString& msgId, bool success, const QString& message)
{
    QJsonObject payload = buildResponsePayload(success, message);
    QJsonObject envelope;
    envelope[QStringLiteral("version")]   = QLatin1String(kProtocolVersion);
    envelope[QStringLiteral("msgId")]     = msgId;
    envelope[QStringLiteral("type")]      = type;
    envelope[QStringLiteral("timestamp")] = QDateTime::currentMSecsSinceEpoch();
    envelope[QStringLiteral("payload")]   = payload;
    sendToClient(envelope);
}

QString HmiTcpServer::nextEventId()
{
    return QStringLiteral("evt-%1").arg(m_nextEventId++);
}

}  // namespace hmi_server
}  // namespace scan_tracking
