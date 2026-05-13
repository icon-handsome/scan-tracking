/**
 * @file hmi_tcp_server.cpp
 * @brief HMI TCP 服务端实现
 */

#include "scan_tracking/hmi_server/hmi_tcp_server.h"
#include "scan_tracking/hmi_server/hmi_session.h"
#include "scan_tracking/hmi_server/hmi_protocol.h"

#include "scan_tracking/flow_control/state_machine.h"
#include "scan_tracking/flow_control/plc_protocol.h"
#include "scan_tracking/modbus/modbus_service.h"
#include "scan_tracking/mech_eye/mech_eye_service.h"
#include "scan_tracking/vision/vision_pipeline_service.h"
#include "scan_tracking/vision/hik_camera_service.h"
#include "scan_tracking/tracking/tracking_service.h"
#include "scan_tracking/common/config_manager.h"

#include <QtNetwork/QTcpServer>
#include <QtNetwork/QTcpSocket>
#include <QtNetwork/QHostAddress>
#include <QtCore/QLoggingCategory>
#include <QtCore/QJsonArray>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonDocument>
#include <QtCore/QUuid>
#include <qdatetime.h>

namespace scan_tracking {
namespace hmi_server {

Q_LOGGING_CATEGORY(LOG_HMI_SERVER, "hmi.server")

// 静态成员初始化：全局日志拦截器回调所需的单例指针和原始日志处理器
HmiTcpServer* HmiTcpServer::s_instance = nullptr;
QtMessageHandler HmiTcpServer::s_previousHandler = nullptr;

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
    
    // 初始化消息处理函数映射表
    initializeMessageHandlers();
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
    connectModbusSignals();
    connectMechEyeSignals();
    connectVisionPipelineSignals();
    
    // 安装全局日志拦截器，将 qDebug/qInfo/qWarning/qCritical 输出转发为 event.log 推送到远端 Qt 显控
    installLogForwarder();

    return true;
}

void HmiTcpServer::stop()
{
    // 停止远端日志转发，恢复原始日志处理器
    uninstallLogForwarder();
    
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
        qInfo(LOG_HMI_SERVER) << "[TCPIP] 新客户端已接入，IP:" << socket->peerAddress().toString() << "端口:" << socket->peerPort();
        
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
    qInfo(LOG_HMI_SERVER) << "[TCPIP] 客户端连接已断开";
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

// 初始化消息处理函数映射表
void HmiTcpServer::initializeMessageHandlers()
{
    // 连接管理消息
    m_messageHandlers[QString::fromLatin1(msg_type::kHmiHello)]       = &HmiTcpServer::handleHmiHello;
    m_messageHandlers[QString::fromLatin1(msg_type::kHeartbeatPing)]  = &HmiTcpServer::handleHeartbeatPing;
    m_messageHandlers[QString::fromLatin1(msg_type::kHeartbeatPong)]  = &HmiTcpServer::handleHeartbeatPong;
    
    // 基础控制命令
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdStart)]       = &HmiTcpServer::handleCmdStart;
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdStop)]        = &HmiTcpServer::handleCmdStop;
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdReset)]       = &HmiTcpServer::handleCmdReset;
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdClearAlarm)]  = &HmiTcpServer::handleCmdClearAlarm;
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdGetStatus)]   = &HmiTcpServer::handleCmdGetStatus;
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdGetConfig)]   = &HmiTcpServer::handleCmdGetConfig;
    
    // Modbus 控制命令
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdModbusConnect)]    = &HmiTcpServer::handleCmdModbusConnect;
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdModbusDisconnect)] = &HmiTcpServer::handleCmdModbusDisconnect;
    
    // 相机控制命令
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdRefreshCamera)]    = &HmiTcpServer::handleCmdRefreshCamera;
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdCaptureMechEye)]   = &HmiTcpServer::handleCmdCaptureMechEye;
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdCaptureBundle)]    = &HmiTcpServer::handleCmdCaptureBundle;
    
    // 直接触发命令（占位实现）
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdTriggerScan)]         = &HmiTcpServer::handleCmdTriggerScan;
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdTriggerInspection)]   = &HmiTcpServer::handleCmdTriggerInspection;
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdTriggerSelfCheck)]    = &HmiTcpServer::handleCmdTriggerSelfCheck;
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdTriggerPoseCheck)]    = &HmiTcpServer::handleCmdTriggerPoseCheck;
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdTriggerCodeRead)]     = &HmiTcpServer::handleCmdTriggerCodeRead;
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdTriggerResultReset)]  = &HmiTcpServer::handleCmdTriggerResultReset;
}

// 处理接收到的客户端消息，根据 type 字段分发到不同的处理函数
void HmiTcpServer::onMessageReceived(const QJsonObject& message)
{
    // 1. 从 JSON 信封中解析出关键字段
    const QString type = message.value(QLatin1String("type")).toString();
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    
    // [TCPIP关键打印] 将收到的完整 JSON 序列化为字符串，用于存档和追溯
    QByteArray rawJson = QJsonDocument(message).toJson(QJsonDocument::Compact);
    
    // 区分高频消息和关键消息的日志级别
    if (type == QLatin1String(msg_type::kHeartbeatPing) || type == QLatin1String(msg_type::kHeartbeatPong)) {
        qDebug(LOG_HMI_SERVER).noquote() << "[TCPIP] [RX_TRACE] " << rawJson;
    } else {
        qInfo(LOG_HMI_SERVER).noquote() << "[TCPIP] [RX] " << rawJson;
    }
    
    // ------------------------------------------------------------------
    // 2. 通信协议命令分发路由 (Dispatcher)
    // 使用 QHash 映射表进行 O(1) 时间复杂度的快速查找和分发。
    // 说明：所有 handleXxx 函数内部都会在执行完具体业务后，通过 sendResponse() 
    // 回传与该请求 msgId 完全一致的响应结果给显控界面，形成请求-响应的闭环。
    // ------------------------------------------------------------------
    auto it = m_messageHandlers.find(type);
    if (it != m_messageHandlers.end()) {
        // 找到对应的处理函数，调用成员函数指针
        (this->*it.value())(message);
    } else {
        // 未知消息类型，返回错误响应
        qWarning(LOG_HMI_SERVER) << "收到未知类型的消息:" << type;
        sendResponse(type, msgId, false, QStringLiteral("Unknown command type"));
    }
}

void HmiTcpServer::onStatusPushTimer()
{
    if (!hasClient()) return;
    pushSystemStatus();
    pushPlcStatus();
    pushCameraStatus();
    pushDeviceStatus();  // 周期性推送设备在线状态字和故障状态字
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

void HmiTcpServer::handleHeartbeatPong(const QJsonObject& message)
{
    Q_UNUSED(message)
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
    QJsonObject payload = buildResponsePayload(true, QStringLiteral("Status returned"));
    payload[QLatin1String("system")] = buildSystemStatusPayload();
    payload[QLatin1String("plc")] = buildPlcStatusPayload();
    payload[QLatin1String("camera")] = buildCameraStatusPayload();
    payload[QLatin1String("device")] = buildDeviceStatusPayload();
    sendToClient(buildEnvelope(QLatin1String(msg_type::kCmdGetStatus), msgId, payload));
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
    
    // 从 ConfigManager 单例读取所有配置并序列化为 JSON
    auto* cfgMgr = common::ConfigManager::instance();
    if (!cfgMgr) {
        sendResponse(QLatin1String(msg_type::kCmdGetConfig), msgId, false, QStringLiteral("ConfigManager not initialized"));
        return;
    }
    
    QJsonObject configPayload;
    
    // 1. App 配置
    QJsonObject appObj;
    appObj[QLatin1String("version")] = cfgMgr->appConfig().version;
    appObj[QLatin1String("environment")] = cfgMgr->appConfig().environment;
    configPayload[QLatin1String("app")] = appObj;
    
    // 2. Logger 配置
    QJsonObject loggerObj;
    loggerObj[QLatin1String("level")] = cfgMgr->loggerConfig().level;
    loggerObj[QLatin1String("rotateDays")] = cfgMgr->loggerConfig().rotateDays;
    configPayload[QLatin1String("logger")] = loggerObj;
    
    // 3. Modbus 配置
    QJsonObject modbusObj;
    modbusObj[QLatin1String("host")] = cfgMgr->modbusConfig().host;
    modbusObj[QLatin1String("port")] = cfgMgr->modbusConfig().port;
    modbusObj[QLatin1String("unitId")] = cfgMgr->modbusConfig().unitId;
    modbusObj[QLatin1String("timeoutMs")] = cfgMgr->modbusConfig().timeoutMs;
    modbusObj[QLatin1String("reconnectIntervalMs")] = cfgMgr->modbusConfig().reconnectIntervalMs;
    configPayload[QLatin1String("modbus")] = modbusObj;
    
    // 4. Camera 配置
    QJsonObject cameraObj;
    cameraObj[QLatin1String("defaultCamera")] = cfgMgr->cameraConfig().defaultCamera;
    cameraObj[QLatin1String("scanTimeoutMs")] = cfgMgr->cameraConfig().scanTimeoutMs;
    configPayload[QLatin1String("camera")] = cameraObj;
    
    // 5. Vision 配置
    QJsonObject visionObj;
    visionObj[QLatin1String("mechEyeCameraKey")] = cfgMgr->visionConfig().mechEyeCameraKey;
    visionObj[QLatin1String("mechCaptureTimeoutMs")] = cfgMgr->visionConfig().mechCaptureTimeoutMs;
    visionObj[QLatin1String("hikConnectTimeoutMs")] = cfgMgr->visionConfig().hikConnectTimeoutMs;
    visionObj[QLatin1String("hikCaptureTimeoutMs")] = cfgMgr->visionConfig().hikCaptureTimeoutMs;
    visionObj[QLatin1String("hikSdkRoot")] = cfgMgr->visionConfig().hikSdkRoot;
    
    QJsonObject hikAObj;
    hikAObj[QLatin1String("logicalName")] = cfgMgr->visionConfig().hikCameraA.logicalName;
    hikAObj[QLatin1String("cameraKey")] = cfgMgr->visionConfig().hikCameraA.cameraKey;
    hikAObj[QLatin1String("ipAddress")] = cfgMgr->visionConfig().hikCameraA.ipAddress;
    hikAObj[QLatin1String("serialNumber")] = cfgMgr->visionConfig().hikCameraA.serialNumber;
    visionObj[QLatin1String("hikCameraA")] = hikAObj;
    
    QJsonObject hikBObj;
    hikBObj[QLatin1String("logicalName")] = cfgMgr->visionConfig().hikCameraB.logicalName;
    hikBObj[QLatin1String("cameraKey")] = cfgMgr->visionConfig().hikCameraB.cameraKey;
    hikBObj[QLatin1String("ipAddress")] = cfgMgr->visionConfig().hikCameraB.ipAddress;
    hikBObj[QLatin1String("serialNumber")] = cfgMgr->visionConfig().hikCameraB.serialNumber;
    visionObj[QLatin1String("hikCameraB")] = hikBObj;
    
    configPayload[QLatin1String("vision")] = visionObj;
    
    // 6. FlowControl 配置
    QJsonObject flowControlObj;
    flowControlObj[QLatin1String("pollIntervalMs")] = cfgMgr->flowControlConfig().pollIntervalMs;
    flowControlObj[QLatin1String("heartbeatIntervalMs")] = cfgMgr->flowControlConfig().heartbeatIntervalMs;
    flowControlObj[QLatin1String("simulatedProcessingMs")] = cfgMgr->flowControlConfig().simulatedProcessingMs;
    configPayload[QLatin1String("flowControl")] = flowControlObj;
    
    // 7. Tracking 配置
    QJsonObject trackingObj;
    trackingObj[QLatin1String("firstStationOuterSegmentIndex")] = cfgMgr->trackingConfig().firstStationOuterSegmentIndex;
    trackingObj[QLatin1String("firstStationInnerSegmentIndex")] = cfgMgr->trackingConfig().firstStationInnerSegmentIndex;
    trackingObj[QLatin1String("firstStationHoleSegmentIndex")] = cfgMgr->trackingConfig().firstStationHoleSegmentIndex;
    configPayload[QLatin1String("tracking")] = trackingObj;
    
    // 8. LbPose 配置
    QJsonObject lbPoseObj;
    lbPoseObj[QLatin1String("dataRoot")] = cfgMgr->lbPoseConfig().dataRoot;
    lbPoseObj[QLatin1String("leftPattern")] = cfgMgr->lbPoseConfig().leftPattern;
    lbPoseObj[QLatin1String("rightPattern")] = cfgMgr->lbPoseConfig().rightPattern;
    lbPoseObj[QLatin1String("templateFile")] = cfgMgr->lbPoseConfig().templateFile;
    lbPoseObj[QLatin1String("minDistance")] = static_cast<double>(cfgMgr->lbPoseConfig().minDistance);
    lbPoseObj[QLatin1String("maxDistance")] = static_cast<double>(cfgMgr->lbPoseConfig().maxDistance);
    lbPoseObj[QLatin1String("cosTolerance")] = static_cast<double>(cfgMgr->lbPoseConfig().cosTolerance);
    lbPoseObj[QLatin1String("minPercent")] = static_cast<double>(cfgMgr->lbPoseConfig().minPercent);
    configPayload[QLatin1String("lbPose")] = lbPoseObj;
    
    // 构建响应
    QJsonObject payload = buildResponsePayload(true, QStringLiteral("Configuration retrieved"));
    payload[QLatin1String("config")] = configPayload;
    
    QJsonObject envelope;
    envelope[QStringLiteral("version")]   = QLatin1String(kProtocolVersion);
    envelope[QStringLiteral("msgId")]     = msgId;
    envelope[QStringLiteral("type")]      = QLatin1String(msg_type::kCmdGetConfig);
    envelope[QStringLiteral("timestamp")] = QDateTime::currentMSecsSinceEpoch();
    envelope[QStringLiteral("payload")]   = payload;
    sendToClient(envelope);
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
    
    // 刷新 MechEye 3D 相机连接状态
    if (m_mechEyeService) {
        qInfo(LOG_HMI_SERVER) << "[TCPIP] 收到相机刷新请求，正在刷新 MechEye 相机状态...";
        m_mechEyeService->requestRefreshStatus();
        sendResponse(QLatin1String(msg_type::kCmdRefreshCamera), msgId, true, QStringLiteral("Camera refresh requested"));
    } else {
        qWarning(LOG_HMI_SERVER) << "[TCPIP] 相机刷新失败：MechEye 服务不可用";
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
    sendToClient(buildEnvelope(QLatin1String(msg_type::kStatusSystem), nextEventId(), buildSystemStatusPayload()));
}

QJsonObject HmiTcpServer::buildSystemStatusPayload() const
{
    QJsonObject payload;
    if (!m_stateMachine) return payload;

    QString appStateStr;
    switch (m_stateMachine->currentState()) {
    case flow_control::AppState::Init:     appStateStr = QStringLiteral("Init"); break;
    case flow_control::AppState::Ready:    appStateStr = QStringLiteral("Ready"); break;
    case flow_control::AppState::Scanning: appStateStr = QStringLiteral("Scanning"); break;
    case flow_control::AppState::Error:    appStateStr = QStringLiteral("Error"); break;
    default:                               appStateStr = QStringLiteral("Unknown"); break;
    }

    payload[QLatin1String("ipcState")] = static_cast<int>(m_stateMachine->ipcState());
    payload[QLatin1String("appState")] = appStateStr;
    payload[QLatin1String("stage")] = static_cast<int>(m_stateMachine->currentStage());
    payload[QLatin1String("alarmLevel")] = m_stateMachine->alarmLevel();
    payload[QLatin1String("alarmCode")] = m_stateMachine->alarmCode();
    payload[QLatin1String("warnCode")] = m_stateMachine->warnCode();
    payload[QLatin1String("ipcReady")] = (m_stateMachine->currentState() == flow_control::AppState::Ready) ? 1 : 0;
    payload[QLatin1String("progress")] = m_stateMachine->progress();
    return payload;
}

void HmiTcpServer::pushPlcStatus()
{
    if (!m_modbusService) return;
    sendToClient(buildEnvelope(QLatin1String(msg_type::kStatusPlc), nextEventId(), buildPlcStatusPayload()));
}

QJsonObject HmiTcpServer::buildPlcStatusPayload() const
{
    QJsonObject payload;
    if (!m_modbusService) return payload;

    payload[QLatin1String("modbusConnected")] = m_modbusService->isConnected();

    if (m_stateMachine) {
        namespace regs = flow_control::protocol::registers;
        const auto& cb = m_stateMachine->lastCommandBlock();
        if (cb.size() > regs::kScanSegmentTotal) {
            payload[QLatin1String("plcHeartbeat")]    = cb.value(regs::kPlcHeartbeat);
            payload[QLatin1String("plcSystemState")]  = cb.value(regs::kPlcSystemState);
            payload[QLatin1String("workMode")]        = cb.value(regs::kStationWorkMode);
            payload[QLatin1String("flowEnable")]      = cb.value(regs::kFlowEnable);
            payload[QLatin1String("safetyWord")]      = cb.value(regs::kSafetyStatusWord);
            payload[QLatin1String("taskId")]          = static_cast<int>(
                (static_cast<quint32>(cb.value(regs::kTaskIdHigh)) << 16) |
                static_cast<quint32>(cb.value(regs::kTaskIdLow)));
            payload[QLatin1String("productType")]     = cb.value(regs::kProductType);
            payload[QLatin1String("recipeId")]        = cb.value(regs::kRecipeId);
            payload[QLatin1String("scanSegmentIndex")] = cb.value(regs::kScanSegmentIndex);
            payload[QLatin1String("scanSegmentTotal")] = cb.value(regs::kScanSegmentTotal);
        }
    }
    return payload;
}

void HmiTcpServer::pushCameraStatus()
{
    sendToClient(buildEnvelope(QLatin1String(msg_type::kStatusCamera), nextEventId(), buildCameraStatusPayload()));
}

QJsonObject HmiTcpServer::buildCameraStatusPayload() const
{
    QJsonObject payload;
    
    if (m_mechEyeService) {
        QJsonObject mechEyeObj;
        mechEyeObj[QLatin1String("state")] = static_cast<int>(m_mechEyeService->state());
        mechEyeObj[QLatin1String("connected")] = (m_mechEyeService->state() != mech_eye::CameraRuntimeState::Idle
                                                   && m_mechEyeService->state() != mech_eye::CameraRuntimeState::Error);
        payload[QLatin1String("mechEye")] = mechEyeObj;
    }
    
    if (m_hikCameraA) {
        QJsonObject hikAObj;
        hikAObj[QLatin1String("roleName")] = m_hikCameraA->roleName();
        hikAObj[QLatin1String("connected")] = m_hikCameraA->isConnected();
        payload[QLatin1String("hikA")] = hikAObj;
    }
    
    if (m_hikCameraB) {
        QJsonObject hikBObj;
        hikBObj[QLatin1String("roleName")] = m_hikCameraB->roleName();
        hikBObj[QLatin1String("connected")] = m_hikCameraB->isConnected();
        payload[QLatin1String("hikB")] = hikBObj;
    }
    
    if (m_visionPipeline) {
        QJsonObject pipelineObj;
        pipelineObj[QLatin1String("state")] = static_cast<int>(m_visionPipeline->state());
        payload[QLatin1String("pipeline")] = pipelineObj;
    }
    return payload;
}

void HmiTcpServer::pushDeviceStatus()
{
    sendToClient(buildEnvelope(QLatin1String(msg_type::kStatusDevice), nextEventId(), buildDeviceStatusPayload()));
}

QJsonObject HmiTcpServer::buildDeviceStatusPayload() const
{
    quint16 onlineWord0 = 0;
    quint16 faultWord0 = 0;
    
    onlineWord0 |= (1 << 0);
    
    if (hasClient()) {
        onlineWord0 |= (1 << 1);
    }
    
    if (m_mechEyeService && m_mechEyeService->state() != mech_eye::CameraRuntimeState::Idle 
        && m_mechEyeService->state() != mech_eye::CameraRuntimeState::Error) {
        onlineWord0 |= (1 << 2);
    }
    
    // 任意一台 2D 相机在线即视为 Bit4 在线
    if ((m_hikCameraA && m_hikCameraA->isConnected()) || 
        (m_hikCameraB && m_hikCameraB->isConnected())) {
        onlineWord0 |= (1 << 4);
    }
    
    if (m_trackingService) {
        onlineWord0 |= (1 << 5);
    }
    
    if (m_modbusService && m_modbusService->isConnected()) {
        onlineWord0 |= (1 << 6);
    }

    QJsonObject payload;
    payload[QLatin1String("onlineWord0")] = onlineWord0;
    payload[QLatin1String("faultWord0")] = faultWord0;
    return payload;
}

// --- 事件连接 ---

void HmiTcpServer::connectStateMachineSignals()
{
    if (!m_stateMachine) return;

    // 绑定核心业务报警事件：将状态机发出的协议级错误拦截并封装为 event.alarm 向远端推送
    connect(m_stateMachine, &flow_control::StateMachine::protocolEvent, this, [this](const QString& message) {
        QJsonObject payload;
        payload[QLatin1String("message")] = message;
        payload[QLatin1String("level")] = m_stateMachine->alarmLevel();
        payload[QLatin1String("code")] = m_stateMachine->alarmCode();
        payload[QLatin1String("timestamp")] = QDateTime::currentMSecsSinceEpoch();
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventAlarm), nextEventId(), payload));
    });

    // 绑定扫描分段启动事件：告知显控界面哪一段扫描正在拍摄
    connect(m_stateMachine, &flow_control::StateMachine::scanStarted, this,
        [this](int segmentIndex, quint32 taskId) {
        QJsonObject payload;
        payload[QLatin1String("segmentIndex")] = segmentIndex;
        payload[QLatin1String("taskId")] = static_cast<int>(taskId);
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventScanStarted), nextEventId(), payload));
    });

    // 绑定扫描分段完成事件：推送当前段是否采图成功及有效帧数
    connect(m_stateMachine, &flow_control::StateMachine::scanFinished, this,
        [this](int segmentIndex, quint16 resultCode, int imageCount, int cloudFrameCount) {
        QJsonObject payload;
        payload[QLatin1String("segmentIndex")] = segmentIndex;
        payload[QLatin1String("resultCode")] = resultCode; // 1 表示成功
        payload[QLatin1String("imageCount")] = imageCount;
        payload[QLatin1String("cloudFrameCount")] = cloudFrameCount;
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventScanFinished), nextEventId(), payload));
    });

    // 绑定综合检测算法结束事件：推送所有工艺偏移量计算结果及缺陷诊断信息（如划痕、裂纹等特征字）
    connect(m_stateMachine, &flow_control::StateMachine::inspectionFinished, this,
        [this](quint16 resultCode, quint16 ngReasonWord0, quint16 ngReasonWord1,
               quint16 measureItemCount, float offsetXmm, float offsetYmm, float offsetZmm,
               float stableOffsetXmm, float stableOffsetYmm, float stableOffsetZmm,
               const QString& outlinerErrorLog, const QString& inlinerErrorLog,
               const QString& message) {
        QJsonObject payload;
        payload[QLatin1String("resultCode")] = resultCode;
        payload[QLatin1String("ngReasonWord0")] = ngReasonWord0;    // 主要 NG 原因位图
        payload[QLatin1String("ngReasonWord1")] = ngReasonWord1;    // 辅助 NG 原因位图
        payload[QLatin1String("measureItemCount")] = measureItemCount;
        payload[QLatin1String("offsetXmm")] = static_cast<double>(offsetXmm); // 相对基础模板的绝对偏移量
        payload[QLatin1String("offsetYmm")] = static_cast<double>(offsetYmm);
        payload[QLatin1String("offsetZmm")] = static_cast<double>(offsetZmm);
        payload[QLatin1String("stableOffsetXmm")] = static_cast<double>(stableOffsetXmm); // 平滑滤波后的偏移量
        payload[QLatin1String("stableOffsetYmm")] = static_cast<double>(stableOffsetYmm);
        payload[QLatin1String("stableOffsetZmm")] = static_cast<double>(stableOffsetZmm);
        payload[QLatin1String("outlinerErrorLog")] = outlinerErrorLog; // 外部轮廓瑕疵诊断日志
        payload[QLatin1String("inlinerErrorLog")] = inlinerErrorLog;   // 内部孔径瑕疵诊断日志
        payload[QLatin1String("message")] = message;
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventInspectionFinished), nextEventId(), payload));
    });

    // 位姿校验完成
    connect(m_stateMachine, &flow_control::StateMachine::poseCheckFinished, this,
        [this](bool success, quint16 resultCode, double poseDeviationMm, const QVector<double>& rt, const QString& message) {
        QJsonObject payload;
        QJsonArray rtArray;
        for (double value : rt) {
            rtArray.append(value);
        }
        payload[QLatin1String("success")] = success;
        payload[QLatin1String("resultCode")] = resultCode;
        payload[QLatin1String("poseDeviationMm")] = poseDeviationMm;
        payload[QLatin1String("rt")] = rtArray;
        payload[QLatin1String("message")] = message;
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventPoseCheckFinished), nextEventId(), payload));
    });

    // 上料抓取完成
    connect(m_stateMachine, &flow_control::StateMachine::loadGraspFinished, this,
        [this](quint16 resultCode, float x, float y, float z, float rx, float ry, float rz) {
        QJsonObject payload;
        payload[QLatin1String("resultCode")] = resultCode;
        payload[QLatin1String("x")] = static_cast<double>(x);
        payload[QLatin1String("y")] = static_cast<double>(y);
        payload[QLatin1String("z")] = static_cast<double>(z);
        payload[QLatin1String("rx")] = static_cast<double>(rx);
        payload[QLatin1String("ry")] = static_cast<double>(ry);
        payload[QLatin1String("rz")] = static_cast<double>(rz);
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventLoadGraspFinished), nextEventId(), payload));
    });

    // 卸料计算完成
    connect(m_stateMachine, &flow_control::StateMachine::unloadCalcFinished, this,
        [this](quint16 resultCode, float x, float y, float z, float rx, float ry, float rz) {
        QJsonObject payload;
        payload[QLatin1String("resultCode")] = resultCode;
        payload[QLatin1String("x")] = static_cast<double>(x);
        payload[QLatin1String("y")] = static_cast<double>(y);
        payload[QLatin1String("z")] = static_cast<double>(z);
        payload[QLatin1String("rx")] = static_cast<double>(rx);
        payload[QLatin1String("ry")] = static_cast<double>(ry);
        payload[QLatin1String("rz")] = static_cast<double>(rz);
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventUnloadCalcFinished), nextEventId(), payload));
    });

    // 自检完成
    connect(m_stateMachine, &flow_control::StateMachine::selfCheckFinished, this,
        [this](quint16 resultCode, quint16 failWord0) {
        QJsonObject payload;
        payload[QLatin1String("resultCode")] = resultCode;
        payload[QLatin1String("failWord0")] = failWord0;
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventSelfCheckFinished), nextEventId(), payload));
    });

    // 条码读取完成
    connect(m_stateMachine, &flow_control::StateMachine::codeReadFinished, this,
        [this](quint16 resultCode, const QString& codeValue) {
        QJsonObject payload;
        payload[QLatin1String("resultCode")] = resultCode;
        payload[QLatin1String("codeValue")] = codeValue;
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventCodeReadFinished), nextEventId(), payload));
    });

    // 结果复位完成
    connect(m_stateMachine, &flow_control::StateMachine::resultResetFinished, this,
        [this](quint16 resultCode) {
        QJsonObject payload;
        payload[QLatin1String("resultCode")] = resultCode;
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventResultResetFinished), nextEventId(), payload));
    });
}

void HmiTcpServer::connectModbusSignals()
{
    if (!m_modbusService) return;

    connect(m_modbusService, &modbus::ModbusService::connected, this, [this]() {
        QJsonObject payload;
        payload[QLatin1String("message")] = QStringLiteral("Modbus connected");
        payload[QLatin1String("level")] = 0;
        payload[QLatin1String("code")] = 0;
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventAlarm), nextEventId(), payload));
    });

    connect(m_modbusService, &modbus::ModbusService::disconnected, this, [this]() {
        QJsonObject payload;
        payload[QLatin1String("message")] = QStringLiteral("Modbus disconnected");
        payload[QLatin1String("level")] = 3;
        payload[QLatin1String("code")] = 900;
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventAlarm), nextEventId(), payload));
    });

    connect(m_modbusService, &modbus::ModbusService::errorOccurred, this, [this](const QString& errorString) {
        QJsonObject payload;
        payload[QLatin1String("message")] = errorString;
        payload[QLatin1String("level")] = 2;
        payload[QLatin1String("code")] = 901;
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventAlarm), nextEventId(), payload));
    });
}

void HmiTcpServer::connectMechEyeSignals()
{
    if (!m_mechEyeService) return;

    connect(m_mechEyeService, &mech_eye::MechEyeService::captureFinished, this,
        [this](scan_tracking::mech_eye::CaptureResult result) {
        QJsonObject payload;
        payload[QLatin1String("requestId")] = static_cast<qint64>(result.requestId);
        payload[QLatin1String("cameraKey")] = result.cameraKey;
        payload[QLatin1String("pointCount")] = static_cast<int>(result.pointCloud.pointCount);
        payload[QLatin1String("width")] = result.pointCloud.width;
        payload[QLatin1String("height")] = result.pointCloud.height;
        payload[QLatin1String("elapsedMs")] = static_cast<int>(result.elapsedMs);
        payload[QLatin1String("errorCode")] = static_cast<int>(result.errorCode);
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventImageCaptured), nextEventId(), payload));
    });

    connect(m_mechEyeService, &mech_eye::MechEyeService::fatalError, this,
        [this](scan_tracking::mech_eye::CaptureErrorCode code, QString message) {
        QJsonObject payload;
        payload[QLatin1String("message")] = message;
        payload[QLatin1String("level")] = 3;
        payload[QLatin1String("code")] = static_cast<int>(code);
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventAlarm), nextEventId(), payload));
    });
}

void HmiTcpServer::connectVisionPipelineSignals()
{
    if (!m_visionPipeline) return;

    connect(m_visionPipeline, &vision::VisionPipelineService::bundleCaptureFinished, this,
        [this](scan_tracking::vision::MultiCameraCaptureBundle bundle) {
        QJsonObject payload;
        payload[QLatin1String("segmentIndex")] = bundle.request.segmentIndex;
        payload[QLatin1String("taskId")] = static_cast<int>(bundle.request.taskId);
        payload[QLatin1String("mechOk")] = bundle.mechEyeResult.success();
        payload[QLatin1String("hikAOk")] = bundle.hikCameraAResult.success();
        payload[QLatin1String("hikBOk")] = bundle.hikCameraBResult.success();
        payload[QLatin1String("lbOk")] = bundle.lbPoseResult.success;
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventBundleCaptured), nextEventId(), payload));
    });
}

// --- 日志转发实现 ---

void HmiTcpServer::installLogForwarder()
{
    // 保存当前实例指针，供全局回调函数使用
    s_instance = this;
    
    // 拦截全局 Qt 日志输出，并保存原有处理器以支持链式调用
    s_previousHandler = qInstallMessageHandler([](QtMsgType type, const QMessageLogContext& context, const QString& msg) {
        if (s_instance && s_instance->hasClient()) {
            QJsonObject payload;
            int severity = 0;
            switch (type) {
            case QtDebugMsg: severity = 0; break;
            case QtInfoMsg: severity = 1; break;
            case QtWarningMsg: severity = 2; break;
            case QtCriticalMsg: severity = 3; break;
            case QtFatalMsg: severity = 4; break;
            }
            payload[QLatin1String("severity")] = severity;
            payload[QLatin1String("category")] = QString::fromLatin1(context.category ? context.category : "default");
            payload[QLatin1String("message")] = msg;
            payload[QLatin1String("file")] = QString::fromLatin1(context.file ? context.file : "");
            payload[QLatin1String("line")] = context.line;
            payload[QLatin1String("timestamp")] = QDateTime::currentMSecsSinceEpoch();
            
            s_instance->sendToClient(buildEnvelope(QLatin1String(msg_type::kEventLog), s_instance->nextEventId(), payload));
        }
        
        // 链式调用：保证本地控制台或本地文件日志依然能够正常输出
        if (s_previousHandler) {
            s_previousHandler(type, context, msg);
        }
    });
}

void HmiTcpServer::uninstallLogForwarder()
{
    if (s_previousHandler) {
        qInstallMessageHandler(s_previousHandler);
        s_previousHandler = nullptr;
    }
    s_instance = nullptr;
}

// --- 辅助发送 ---

void HmiTcpServer::sendToClient(const QJsonObject& envelope)
{
    if (m_session) {
        // [TCPIP关键打印] 将发送的完整 JSON 序列化为单行字符串，用于存档
        QByteArray rawJson = QJsonDocument(envelope).toJson(QJsonDocument::Compact);
        QString type = envelope.value(QLatin1String("type")).toString();
        
        // 区分高频消息和关键消息：
        // 状态推送、心跳、日志等高频且数据量大的报文使用 Debug 级别（避免撑爆 Info 级别的主日志文件）
        // 核心控制命令、业务事件的回执使用 Info 级别（永久存档）
        if (type == QLatin1String(msg_type::kStatusSystem) || 
            type == QLatin1String(msg_type::kStatusPlc) || 
            type == QLatin1String(msg_type::kStatusCamera) || 
            type == QLatin1String(msg_type::kStatusDevice) || 
            type == QLatin1String(msg_type::kHeartbeatPing) || 
            type == QLatin1String(msg_type::kHeartbeatPong) ||
            type == QLatin1String(msg_type::kEventLog)) {
            
            qDebug(LOG_HMI_SERVER).noquote() << "[TCPIP] [TX_TRACE] " << rawJson;
        } else {
            qInfo(LOG_HMI_SERVER).noquote() << "[TCPIP] [TX] " << rawJson;
        }
        
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
