/**
 * @file hmi_tcp_server.cpp
 * @brief HMI TCP 服务端实现
 */

#include "scan_tracking/hmi_server/hmi_tcp_server.h"
#include "scan_tracking/hmi_server/hmi_session.h"
#include "scan_tracking/hmi_server/hmi_protocol.h"

#include "scan_tracking/flow_control/state_machine.h"
#include "scan_tracking/flow_control/plc_protocol.h"
#include "scan_tracking/flow_control/station_trigger_policy.h"
#include "scan_tracking/modbus/modbus_service.h"
#include "scan_tracking/mech_eye/mech_eye_service.h"
#include "scan_tracking/vision/vision_pipeline_service.h"
#include "scan_tracking/vision/hik_cxp_camera_service.h"
#include "scan_tracking/vision/hik_camera_service.h"
#include "scan_tracking/vision/hik_camera_c_controller.h"
#include "scan_tracking/tracking/tracking_service.h"  // InspectionMeasurement, appendInspectionMeasurementFields
#include "scan_tracking/common/config_manager.h"

#include <cmath>

#include <QtNetwork/QTcpServer>
#include <QtNetwork/QTcpSocket>
#include <QtNetwork/QHostAddress>
#include <QtCore/QLoggingCategory>
#include <QtCore/QJsonArray>
#include <QtCore/QJsonObject>
#include <QtCore/QVector>
#include <QtCore/QJsonDocument>
#include <QtCore/QUuid>
#include <qdatetime.h>

namespace scan_tracking {
namespace hmi_server {

Q_LOGGING_CATEGORY(LOG_HMI_SERVER, "hmi.server")

// 静态成员初始化：全局日志拦截器回调所需的单例指针和原始日志处理器
HmiTcpServer* HmiTcpServer::s_instance = nullptr;
QtMessageHandler HmiTcpServer::s_previousHandler = nullptr;
bool HmiTcpServer::s_logForwarderInstalled = false;
namespace {

bool g_inTcpSend = false;
bool g_inLogForward = false;

bool isHighFrequencyTcpType(const QString& type)
{
    using namespace msg_type;
    return type == QLatin1String(kStatusSystem)
        || type == QLatin1String(kStatusPlc)
        || type == QLatin1String(kStatusCamera)
        || type == QLatin1String(kStatusDevice)
        || type == QLatin1String(kHeartbeatPing)
        || type == QLatin1String(kHeartbeatPong)
        || type == QLatin1String(kEventLog);
}

bool isStatusPushType(const QString& type)
{
    using namespace msg_type;
    return type == QLatin1String(kStatusSystem)
        || type == QLatin1String(kStatusPlc)
        || type == QLatin1String(kStatusCamera)
        || type == QLatin1String(kStatusDevice);
}

void logStatusPushTx(const QString& type, const QString& msgId, const QJsonObject& payload)
{
    qInfo(LOG_HMI_SERVER).noquote()
        << QStringLiteral("[TCPIP] → TX") << type << msgId
        << summarizeHmiTracePayload(type, payload);
}

// TODO(hmi): 远程 event.log 转发默认关闭，优先保证 TCP 简洁与主业务打通。
// 若显控需要「远程日志页」，将此处改为 true，并确认 install/uninstall 成对调用。
constexpr bool kForwardQtLogsToHmi = false;

// 相机连/断 event.alarm 专用 code（与 Modbus 900 段区分）
constexpr int kAlarmCodeMechEyeConnect = 910;
constexpr int kAlarmCodeMechEyeDisconnect = 911;
constexpr int kAlarmCodeHikConnect = 912;
constexpr int kAlarmCodeHikDisconnect = 913;
constexpr int kAlarmCodeTelescopicRodFault = 920;
constexpr int kAlarmCodeElectromagnetFault = 921;

bool shouldForwardLogToHmi(QtMsgType type, const QString& category, const QString& msg)
{
    if (g_inTcpSend || g_inLogForward) {
        return false;
    }
    // HMI 模块日志仅本地可见，不包装成 event.log 发到显控
    if (category == QLatin1String("hmi.server") || category == QLatin1String("hmi.session")) {
        return false;
    }
    if (msg.contains(QLatin1String("[TCPIP]"))) {
        return false;
    }
    return type >= QtWarningMsg;
}

}  // namespace

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

    // TODO(hmi): 需要把 IPC 侧 qWarning+ 推到显控 event.log 时，设 kForwardQtLogsToHmi=true
    if (kForwardQtLogsToHmi) {
        installLogForwarder();
    }

    return true;
}

void HmiTcpServer::stop()
{
    if (kForwardQtLogsToHmi) {
        uninstallLogForwarder(); // 卸载日志转发器
    }

    disconnectServiceSignals(); // 断开业务模块信号连接

    m_statusPushTimer->stop(); // 停止状态推送定时器
    m_heartbeatTimer->stop(); // 停止心跳定时器

    if (m_session) {
        m_session->disconnect(); // 断开客户端连接
        m_session->deleteLater();
        m_session = nullptr; // 释放客户端会话指针
    }

    if (m_tcpServer->isListening()) {
        m_tcpServer->close(); // 关闭 TCP 服务器
        qInfo(LOG_HMI_SERVER) << "HMI TCP 服务端已停止";
    }
}

bool HmiTcpServer::isListening() const
{
    return m_tcpServer->isListening(); // 检查 TCP 服务器是否正在监听
}

bool HmiTcpServer::hasClient() const
{
    return m_session != nullptr && m_session->isConnected();
}

void HmiTcpServer::setStateMachine(flow_control::StateMachine* sm) { m_stateMachine = sm; }
void HmiTcpServer::setModbusService(modbus::ModbusService* svc) { m_modbusService = svc; }

void HmiTcpServer::bindServiceSignals()
{
    if (m_serviceSignalsBound) {
        return;
    }
    connectStateMachineSignals();
    connectModbusSignals();
    connectMechEyeSignals();
    connectVisionPipelineSignals();
    connectStatusRefreshSignals();
    m_serviceSignalsBound = true;
}

void HmiTcpServer::disconnectServiceSignals()
{
    if (!m_serviceSignalsBound) {
        return;
    }
    if (m_stateMachine) {
        disconnect(m_stateMachine, nullptr, this, nullptr);
    }
    if (m_modbusService) {
        disconnect(m_modbusService, nullptr, this, nullptr);
    }
    if (m_mechEyeService) {
        disconnect(m_mechEyeService, nullptr, this, nullptr);
    }
    if (m_visionPipeline) {
        disconnect(m_visionPipeline, nullptr, this, nullptr);
    }
    m_serviceSignalsBound = false;
}
void HmiTcpServer::setMechEyeService(mech_eye::MechEyeService* svc) { m_mechEyeService = svc; }
void HmiTcpServer::setVisionPipelineService(vision::VisionPipelineService* svc) { m_visionPipeline = svc; }
void HmiTcpServer::setTrackingService(tracking::TrackingService* svc) { m_trackingService = svc; }
void HmiTcpServer::setHikCameraServices(vision::HikCxpCameraService* hikA, vision::HikCxpCameraService* hikB,
                                        vision::HikCameraService* hikC) {
    m_hikCameraA = hikA;
    m_hikCameraB = hikB;
    m_hikCameraC = hikC;
}

void HmiTcpServer::setHikCameraCController(vision::HikCameraCController* controller) {
    m_hikCameraCController = controller;
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

        invalidateStatusPushCache();
        m_statusPushTimer->start();
        m_heartbeatTimer->start();

        sendToClient(buildEnvelope(QStringLiteral("core.hello"), nextEventId(), QJsonObject()));
        pushAllStatusToClient();
        publishInitialInspectionDisplay();
        syncCameraConnectivityCache();
        if (m_stateMachine) {
            syncPlcAuxDeviceAlarmCache(m_stateMachine->lastCommandBlock());
        }
    }
}

void HmiTcpServer::onSessionDisconnected()
{
    qInfo(LOG_HMI_SERVER) << "[TCPIP] 客户端连接已断开";
    if (m_session) {
        m_session->deleteLater();
        m_session = nullptr;
    }
    invalidateStatusPushCache();
    m_personZoneAlarmCacheValid = false;
    m_statusPushTimer->stop();
    m_heartbeatTimer->stop();
}

void HmiTcpServer::onSessionHeartbeatTimeout()
{
    qWarning(LOG_HMI_SERVER).noquote()
        << QStringLiteral("[TCPIP] 显控心跳超时（约 6s 无有效报文），断开会话");
    if (m_session) {
        m_session->disconnect();
    }
}

// 初始化消息处理函数映射表
void HmiTcpServer::initializeMessageHandlers()
{
    // 连接管理消息
    m_messageHandlers[QString::fromLatin1(msg_type::kHmiHello)]       = &HmiTcpServer::handleHmiHello;  // 欢迎消息
    m_messageHandlers[QString::fromLatin1(msg_type::kHeartbeatPing)]  = &HmiTcpServer::handleHeartbeatPing;  // 心跳请求
    m_messageHandlers[QString::fromLatin1(msg_type::kHeartbeatPong)]  = &HmiTcpServer::handleHeartbeatPong;  // 心跳响应
    
    // 基础控制命令
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdStart)]       = &HmiTcpServer::handleCmdStart;  // 启动
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdStop)]        = &HmiTcpServer::handleCmdStop;  // 停止
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdReset)]       = &HmiTcpServer::handleCmdReset;  // 复位
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdClearAlarm)]  = &HmiTcpServer::handleCmdClearAlarm;  // 清除报警
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdGetStatus)]   = &HmiTcpServer::handleCmdGetStatus;  // 获取状态
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdGetConfig)]   = &HmiTcpServer::handleCmdGetConfig;  // 获取配置
    
    // Modbus 控制命令
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdModbusConnect)]    = &HmiTcpServer::handleCmdModbusConnect;  // 连接 Modbus
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdModbusDisconnect)] = &HmiTcpServer::handleCmdModbusDisconnect;  // 断开 Modbus
    
    // 相机控制命令
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdRefreshCamera)]    = &HmiTcpServer::handleCmdRefreshCamera;  // 刷新相机
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdCaptureMechEye)]   = &HmiTcpServer::handleCmdCaptureMechEye;  // 捕获 MechEye
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdCaptureBundle)]    = &HmiTcpServer::handleCmdCaptureBundle;  // 捕获 Bundle
    
    // 直接触发命令（占位实现）
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdTriggerScan)]         = &HmiTcpServer::handleCmdTriggerScan;  // 触发扫描
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdTriggerInspection)]   = &HmiTcpServer::handleCmdTriggerInspection;  // 触发综合检测
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdTriggerSelfCheck)]    = &HmiTcpServer::handleCmdTriggerSelfCheck;  // 触发自检
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdTriggerPoseCheck)]    = &HmiTcpServer::handleCmdTriggerPoseCheck;  // 触发位姿校验
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdTriggerCodeRead)]     = &HmiTcpServer::handleCmdTriggerCodeRead;  // 触发条码读取
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdTriggerResultReset)]  = &HmiTcpServer::handleCmdTriggerResultReset;  // 触发结果复位

    // 调试命令（需 config.ini [Hmi] allowDebugTriggerInspection=true）
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdDebugTriggerInspection)] =
        &HmiTcpServer::handleCmdDebugTriggerInspection;
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdSetBevelRecipe)] =
        &HmiTcpServer::handleCmdSetBevelRecipe;
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdReportPersonZoneAlarm)] =
        &HmiTcpServer::handleCmdReportPersonZoneAlarm;
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdSetUnloadAreaConfig)] =
        &HmiTcpServer::handleCmdSetUnloadAreaConfig;
    // 兼容显控侧 zone 误拼为 zome
    m_messageHandlers[QString::fromLatin1(msg_type::kCmdReportPersonZoneAlarmTypo)] =
        &HmiTcpServer::handleCmdReportPersonZoneAlarm;
}

// 处理接收到的客户端消息，根据 type 字段分发到不同的处理函数
void HmiTcpServer::onMessageReceived(const QJsonObject& message)
{
    // 1. 从 JSON 信封中解析出关键字段
    const QString type = message.value(QLatin1String("type")).toString();
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    
    if (kHmiTcpVerboseTrace) {
        const QJsonObject payload = message.value(QLatin1String("payload")).toObject();
        qInfo(LOG_HMI_SERVER).noquote()
            << QStringLiteral("[TCPIP] ← RX") << type << msgId
            << summarizeHmiTracePayload(type, payload);
    } else if (!isHighFrequencyTcpType(type)) {
        qDebug(LOG_HMI_SERVER).noquote()
            << QStringLiteral("[TCPIP] 接收") << type << msgId;
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
        qWarning(LOG_HMI_SERVER).noquote()
            << QStringLiteral("[TCPIP] 未知消息类型:") << type << msgId;
        sendResponse(type, msgId, false, QStringLiteral("未知命令类型"));
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
    sendResponse(QLatin1String(msg_type::kHmiHello), msgId, true, QStringLiteral("欢迎"));
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
        sendResponse(QLatin1String(msg_type::kCmdStart), msgId, true, QStringLiteral("状态机已启动"));
    } else {
        sendResponse(QLatin1String(msg_type::kCmdStart), msgId, false, QStringLiteral("状态机不可用"));
    }
}

void HmiTcpServer::handleCmdStop(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    if (m_stateMachine) {
        m_stateMachine->stop();
        sendResponse(QLatin1String(msg_type::kCmdStop), msgId, true, QStringLiteral("状态机已停止"));
    } else {
        sendResponse(QLatin1String(msg_type::kCmdStop), msgId, false, QStringLiteral("状态机不可用"));
    }
}

void HmiTcpServer::handleCmdGetStatus(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    QJsonObject payload = buildResponsePayload(true, QStringLiteral("状态已返回"));
    payload[QLatin1String("system")] = buildSystemStatusPayload();
    payload[QLatin1String("plc")] = buildPlcStatusPayload();
    payload[QLatin1String("camera")] = buildCameraStatusPayload();
    payload[QLatin1String("device")] = buildDeviceStatusPayload();
    sendToClient(buildEnvelope(QLatin1String(msg_type::kCmdGetStatus), msgId, payload));
    syncStatusPushCacheFromCurrent();
}

void HmiTcpServer::handleCmdReset(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    if (m_stateMachine) {
        m_stateMachine->start(); // 目前系统使用 start 作为重启/复位的入口
        sendResponse(QLatin1String(msg_type::kCmdReset), msgId, true, QStringLiteral("状态机已复位"));
    } else {
        sendResponse(QLatin1String(msg_type::kCmdReset), msgId, false, QStringLiteral("状态机不可用"));
    }
}

void HmiTcpServer::handleCmdClearAlarm(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    if (m_stateMachine) {
        m_stateMachine->setAlarm(0, 0, QString());
        sendResponse(QLatin1String(msg_type::kCmdClearAlarm), msgId, true, QStringLiteral("报警已清除"));
    } else {
        sendResponse(QLatin1String(msg_type::kCmdClearAlarm), msgId, false, QStringLiteral("状态机不可用"));
    }
}

void HmiTcpServer::handleCmdGetConfig(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    
    // 从 ConfigManager 单例读取所有配置并序列化为 JSON
    auto* cfgMgr = common::ConfigManager::instance();
    if (!cfgMgr) {
        sendResponse(QLatin1String(msg_type::kCmdGetConfig), msgId, false, QStringLiteral("配置管理器未初始化"));
        return;
    }
    
    QJsonObject configPayload;
    
    // 1. App 配置
    QJsonObject appObj;
    appObj[QLatin1String("version")] = cfgMgr->appConfig().version; // 版本号
    appObj[QLatin1String("environment")] = cfgMgr->appConfig().environment; // 环境
    configPayload[QLatin1String("app")] = appObj; 
    
    // 2. Logger 配置
    QJsonObject loggerObj;
    loggerObj[QLatin1String("level")] = cfgMgr->loggerConfig().level; // 日志级别
    loggerObj[QLatin1String("rotateDays")] = cfgMgr->loggerConfig().rotateDays; // 日志滚动天数
    configPayload[QLatin1String("logger")] = loggerObj;
    
    // 3. Modbus 配置
    QJsonObject modbusObj;
    modbusObj[QLatin1String("host")] = cfgMgr->modbusConfig().host; // 主机
    modbusObj[QLatin1String("port")] = cfgMgr->modbusConfig().port; // 端口
    modbusObj[QLatin1String("unitId")] = cfgMgr->modbusConfig().unitId; // 单元 ID
    modbusObj[QLatin1String("timeoutMs")] = cfgMgr->modbusConfig().timeoutMs; // 超时时间
    modbusObj[QLatin1String("reconnectIntervalMs")] = cfgMgr->modbusConfig().reconnectIntervalMs; // 重连间隔
    configPayload[QLatin1String("modbus")] = modbusObj;
    
    // 4. Camera 配置
    QJsonObject cameraObj;
    cameraObj[QLatin1String("defaultCamera")] = cfgMgr->cameraConfig().defaultCamera; // 默认相机
    cameraObj[QLatin1String("scanTimeoutMs")] = cfgMgr->cameraConfig().scanTimeoutMs; // 扫描超时时间
    configPayload[QLatin1String("camera")] = cameraObj;
    
    // 5. Vision 配置
    QJsonObject visionObj;
    visionObj[QLatin1String("mechEyeCameraKey")] = cfgMgr->visionConfig().mechEyeCameraKey; // MechEye 相机键
    visionObj[QLatin1String("mechCaptureTimeoutMs")] = cfgMgr->visionConfig().mechCaptureTimeoutMs; // MechEye 捕获超时时间
    visionObj[QLatin1String("hikConnectTimeoutMs")] = cfgMgr->visionConfig().hikConnectTimeoutMs; // Hik 连接超时时间
    visionObj[QLatin1String("hikCaptureTimeoutMs")] = cfgMgr->visionConfig().hikCaptureTimeoutMs; // Hik 捕获超时时间
    visionObj[QLatin1String("hikSdkRoot")] = cfgMgr->visionConfig().hikSdkRoot; // Hik SDK 根目录

    visionObj[QLatin1String("hikCxpEnabled")] = cfgMgr->visionConfig().hikCxpEnabled;
    visionObj[QLatin1String("hikCxpCaptureTimeoutMs")] = cfgMgr->visionConfig().hikCxpCaptureTimeoutMs;

    QJsonObject hikAObj;
    hikAObj[QLatin1String("logicalName")] = cfgMgr->visionConfig().hikCxpCameraA.logicalName;
    hikAObj[QLatin1String("cameraKey")] = cfgMgr->visionConfig().hikCxpCameraA.cameraKey;
    hikAObj[QLatin1String("ipAddress")] = cfgMgr->visionConfig().hikCxpCameraA.ipAddress;
    hikAObj[QLatin1String("serialNumber")] = cfgMgr->visionConfig().hikCxpCameraA.serialNumber;
    hikAObj[QLatin1String("cameraType")] = QStringLiteral("cxp");
    visionObj[QLatin1String("hikCameraA")] = hikAObj;

    QJsonObject hikBObj;
    hikBObj[QLatin1String("logicalName")] = cfgMgr->visionConfig().hikCxpCameraB.logicalName;
    hikBObj[QLatin1String("cameraKey")] = cfgMgr->visionConfig().hikCxpCameraB.cameraKey;
    hikBObj[QLatin1String("ipAddress")] = cfgMgr->visionConfig().hikCxpCameraB.ipAddress;
    hikBObj[QLatin1String("serialNumber")] = cfgMgr->visionConfig().hikCxpCameraB.serialNumber;
    hikBObj[QLatin1String("cameraType")] = QStringLiteral("cxp");
    visionObj[QLatin1String("hikCameraB")] = hikBObj;

    QJsonObject hikCObj;
    hikCObj[QLatin1String("logicalName")] = cfgMgr->visionConfig().hikCameraC.logicalName;
    hikCObj[QLatin1String("cameraKey")] = cfgMgr->visionConfig().hikCameraC.cameraKey;
    hikCObj[QLatin1String("ipAddress")] = cfgMgr->visionConfig().hikCameraC.ipAddress;
    hikCObj[QLatin1String("serialNumber")] = cfgMgr->visionConfig().hikCameraC.serialNumber;
    hikCObj[QLatin1String("accessMode")] = cfgMgr->visionConfig().hikCameraC.accessMode;
    visionObj[QLatin1String("hikCameraC")] = hikCObj;
    
    configPayload[QLatin1String("vision")] = visionObj;
    
    // 6. FlowControl 配置
    QJsonObject flowControlObj;
    flowControlObj[QLatin1String("pollIntervalMs")] = cfgMgr->flowControlConfig().pollIntervalMs; // 轮询间隔
    flowControlObj[QLatin1String("heartbeatIntervalMs")] = cfgMgr->flowControlConfig().heartbeatIntervalMs; // 心跳间隔
    flowControlObj[QLatin1String("simulatedProcessingMs")] = cfgMgr->flowControlConfig().simulatedProcessingMs; // 模拟处理间隔
    configPayload[QLatin1String("flowControl")] = flowControlObj;
    
    // 7. Tracking / Bevel 配置
    QJsonObject trackingObj;
    trackingObj[QLatin1String("scanSegmentTotal")] = cfgMgr->trackingConfig().scanSegmentTotal;
    configPayload[QLatin1String("tracking")] = trackingObj;

    QJsonObject bevelObj;
    bevelObj[QLatin1String("configPath")] = cfgMgr->bevelConfig().configPath;
    bevelObj[QLatin1String("templateDir")] = cfgMgr->bevelConfig().templateDir;
    bevelObj[QLatin1String("angleTolDeg")] = static_cast<double>(cfgMgr->bevelConfig().angleTolDeg);
    bevelObj[QLatin1String("lengthTolMm")] = static_cast<double>(cfgMgr->bevelConfig().lengthTolMm);
    const common::BevelRecipe recipe = cfgMgr->bevelRecipe();
    QJsonObject recipeObj;
    recipeObj[QLatin1String("active")] = recipe.active;
    recipeObj[QLatin1String("bevel_type")] = recipe.bevelType;
    recipeObj[QLatin1String("angle_deg")] = static_cast<double>(recipe.angleDeg);
    recipeObj[QLatin1String("length")] = static_cast<double>(recipe.lengthMm);
    recipeObj[QLatin1String("has_hole")] = recipe.hasHole;
    bevelObj[QLatin1String("recipe")] = recipeObj;
    QJsonArray presetArray;
    for (const common::BevelRecipePreset& preset : common::standardBevelRecipePresets()) {
        QJsonObject presetObj;
        presetObj[QLatin1String("bevel_type")] = preset.bevelType;
        presetObj[QLatin1String("name")] = preset.name;
        presetObj[QLatin1String("angle_deg")] = static_cast<double>(preset.angleDeg);
        presetObj[QLatin1String("length")] = static_cast<double>(preset.lengthMm);
        presetArray.append(presetObj);
    }
    bevelObj[QLatin1String("presets")] = presetArray;
    configPayload[QLatin1String("bevel")] = bevelObj;

    QJsonObject hmiObj;
    hmiObj[QLatin1String("enabled")] = cfgMgr->hmiConfig().enabled;
    hmiObj[QLatin1String("tcpPort")] = cfgMgr->hmiConfig().tcpPort;
    hmiObj[QLatin1String("allowDebugTriggerInspection")] = cfgMgr->hmiConfig().allowDebugTriggerInspection;
    configPayload[QLatin1String("hmi")] = hmiObj;
    
    // 8. LbPose 配置
    QJsonObject lbPoseObj;
    lbPoseObj[QLatin1String("trackConfigFile")] = cfgMgr->lbPoseConfig().trackConfigFile;
    lbPoseObj[QLatin1String("dataRoot")] = cfgMgr->lbPoseConfig().dataRoot;
    lbPoseObj[QLatin1String("leftPattern")] = cfgMgr->lbPoseConfig().leftPattern;
    lbPoseObj[QLatin1String("rightPattern")] = cfgMgr->lbPoseConfig().rightPattern;
    lbPoseObj[QLatin1String("templateFile")] = cfgMgr->lbPoseConfig().templateFile;
    configPayload[QLatin1String("lbPose")] = lbPoseObj;
    
    // 构建响应
    QJsonObject payload = buildResponsePayload(true, QStringLiteral("配置已获取")); // 构建响应Payload
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
        sendResponse(QLatin1String(msg_type::kCmdModbusConnect), msgId, true, QStringLiteral("正在连接 Modbus"));
    } else {
        sendResponse(QLatin1String(msg_type::kCmdModbusConnect), msgId, false, QStringLiteral("Modbus 服务不可用"));
    }
}

void HmiTcpServer::handleCmdModbusDisconnect(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    if (m_modbusService) {
        m_modbusService->disconnectDevice();
        sendResponse(QLatin1String(msg_type::kCmdModbusDisconnect), msgId, true, QStringLiteral("已断开 Modbus 连接"));
    } else {
        sendResponse(QLatin1String(msg_type::kCmdModbusDisconnect), msgId, false, QStringLiteral("Modbus 服务不可用"));
    }
}

void HmiTcpServer::handleCmdRefreshCamera(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    
    // 刷新 MechEye 3D 相机连接状态
    if (m_mechEyeService) {
        qInfo(LOG_HMI_SERVER) << "[TCPIP] 收到相机刷新请求，正在刷新 MechEye 相机状态...";
        m_mechEyeService->requestRefreshStatus();
        sendResponse(QLatin1String(msg_type::kCmdRefreshCamera), msgId, true, QStringLiteral("相机刷新请求已发送"));
    } else {
        qWarning(LOG_HMI_SERVER) << "[TCPIP] 相机刷新失败：MechEye 服务不可用";
        sendResponse(QLatin1String(msg_type::kCmdRefreshCamera), msgId, false, QStringLiteral("相机服务不可用"));
    }
}

void HmiTcpServer::handleCmdTriggerScan(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    sendResponse(QLatin1String(msg_type::kCmdTriggerScan), msgId, false, QStringLiteral("直接触发未实现，请使用 PLC 或状态机"));
}

void HmiTcpServer::handleCmdTriggerInspection(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    sendResponse(QLatin1String(msg_type::kCmdTriggerInspection), msgId, false, QStringLiteral("直接触发未实现，请使用 PLC"));
}

void HmiTcpServer::handleCmdTriggerSelfCheck(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    if (m_stateMachine) {
        m_stateMachine->beginSelfCheckScanSession();
    }
    qInfo(LOG_HMI_SERVER).noquote()
        << QStringLiteral("[TCPIP] 显控请求自检，已通知 PLC（IPC_CurrentStage=SelfCheck）") << msgId;
    sendResponse(
        QLatin1String(msg_type::kCmdTriggerSelfCheck),
        msgId,
        true,
        QStringLiteral("已通知 PLC 开始自检流程；PLC 调度两点扫描后下发 Trig_SelfCheck"));
}

void HmiTcpServer::handleCmdTriggerPoseCheck(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    sendResponse(QLatin1String(msg_type::kCmdTriggerPoseCheck), msgId, false, QStringLiteral("直接触发未实现，请使用 PLC"));
}

void HmiTcpServer::handleCmdTriggerCodeRead(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    sendResponse(QLatin1String(msg_type::kCmdTriggerCodeRead), msgId, false, QStringLiteral("直接触发未实现，请使用 PLC"));
}

void HmiTcpServer::handleCmdTriggerResultReset(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    sendResponse(QLatin1String(msg_type::kCmdTriggerResultReset), msgId, false, QStringLiteral("直接触发未实现，请使用 PLC"));
}

void HmiTcpServer::handleCmdDebugTriggerInspection(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();

    // 1. 配置开关：演示环境在 config.ini 打开，生产环境务必保持 false
    const auto* cfgMgr = common::ConfigManager::instance();
    if (cfgMgr == nullptr || !cfgMgr->hmiConfig().allowDebugTriggerInspection) {
        sendResponse(
            QLatin1String(msg_type::kCmdDebugTriggerInspection),
            msgId,
            false,
            QStringLiteral("调试综合检测未启用，请在 config.ini [Hmi] 设置 allowDebugTriggerInspection=true 后重启 Core"));
        return;
    }

    if (m_stateMachine == nullptr) {
        sendResponse(
            QLatin1String(msg_type::kCmdDebugTriggerInspection),
            msgId,
            false,
            QStringLiteral("状态机不可用，无法读取点云缓存"));
        return;
    }

    // 2. 读取当前缓存段位（多路径场景下 cache 可能含多段，见 multiPathNote）
    const QVector<int> cachedSegments = m_stateMachine->cachedScanSegmentIndices();

    // 3. 用缓存跑综合检测（按路径 inspectionType 分流坡口/Hole；不写 PLC、不清缓存）
    const tracking::InspectionResult inspectionResult = m_stateMachine->runDebugInspectionOnCachedSegments();

    // 4. 无论合格/不合格都推 event.inspection.finished 给显控
    publishInspectionResult(inspectionResult);

    // 5. 命令响应：附带缓存摘要，便于演示排障
    QJsonObject payload = buildResponsePayload(
        true,
        QStringLiteral("调试综合检测已完成，event.inspection.finished 已推送（resultCode=%1）")
            .arg(inspectionResult.resultCode));
    payload[QLatin1String("resultCode")] = inspectionResult.resultCode;
    payload[QLatin1String("ngReasonWord0")] = inspectionResult.ngReasonWord0;
    payload[QLatin1String("cachedSegmentCount")] = cachedSegments.size();
    QJsonArray segmentArray;
    for (int segmentIndex : cachedSegments) {
        segmentArray.append(segmentIndex);
    }
    payload[QLatin1String("cachedSegmentIndices")] = segmentArray;
    payload[QLatin1String("inspectionMessage")] = inspectionResult.message;
    payload[QLatin1String("multiPathNote")] = QStringLiteral(
        "综合检测会将当前可检测路径上的分段点云合并为单云，并按 scan_paths 配置的 inspectionType 调用坡口（Po_Kou）或 Hole（HeadMeasure）算法。");

    QJsonObject envelope;
    envelope[QStringLiteral("version")] = QLatin1String(kProtocolVersion);
    envelope[QStringLiteral("msgId")] = msgId;
    envelope[QStringLiteral("type")] = QLatin1String(msg_type::kCmdDebugTriggerInspection);
    envelope[QStringLiteral("timestamp")] = QDateTime::currentMSecsSinceEpoch();
    envelope[QStringLiteral("payload")] = payload;
    sendToClient(envelope);

    qInfo(LOG_HMI_SERVER).noquote()
        << QStringLiteral("[TCPIP] 调试综合检测完成")
        << QStringLiteral(" cachedSegments=") << cachedSegments
        << QStringLiteral(" resultCode=") << inspectionResult.resultCode
        << QStringLiteral(" message=") << inspectionResult.message;
}

void HmiTcpServer::handleCmdSetBevelRecipe(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    const QJsonObject payload = message.value(QLatin1String("payload")).toObject();

    auto* cfgMgr = common::ConfigManager::instance();
    if (cfgMgr == nullptr) {
        sendResponse(
            QLatin1String(msg_type::kCmdSetBevelRecipe),
            msgId,
            false,
            QStringLiteral("配置管理器未初始化"));
        return;
    }

    if (!payload.contains(QLatin1String("bevel_type"))) {
        sendResponse(
            QLatin1String(msg_type::kCmdSetBevelRecipe),
            msgId,
            false,
            QStringLiteral("缺少必填字段：bevel_type"));
        return;
    }

    const int bevelType = payload.value(QLatin1String("bevel_type")).toInt(-1);
    if (bevelType < 0) {
        sendResponse(
            QLatin1String(msg_type::kCmdSetBevelRecipe),
            msgId,
            false,
            QStringLiteral("bevel_type 不能为负数"));
        return;
    }

    double angleDeg = payload.value(QLatin1String("angle_deg")).toDouble(0.0);
    double lengthMm = payload.value(QLatin1String("length")).toDouble(0.0);
    const bool hasAngle = payload.contains(QLatin1String("angle_deg"));
    const bool hasLength = payload.contains(QLatin1String("length"));

    if (!hasAngle || !hasLength) {
        const common::BevelRecipe preset = common::bevelRecipePresetForType(bevelType);
        if (!preset.active) {
            sendResponse(
                QLatin1String(msg_type::kCmdSetBevelRecipe),
                msgId,
                false,
                QStringLiteral("未知 bevel_type=%1，或未同时提供 angle_deg 与 length").arg(bevelType));
            return;
        }
        if (!hasAngle) {
            angleDeg = preset.angleDeg;
        }
        if (!hasLength) {
            lengthMm = preset.lengthMm;
        }
    }

    if (!(angleDeg > 0.0) || !std::isfinite(angleDeg)) {
        sendResponse(
            QLatin1String(msg_type::kCmdSetBevelRecipe),
            msgId,
            false,
            QStringLiteral("angle_deg 必须为大于 0 的有效数值"));
        return;
    }
    if (!(lengthMm > 0.0) || !std::isfinite(lengthMm)) {
        sendResponse(
            QLatin1String(msg_type::kCmdSetBevelRecipe),
            msgId,
            false,
            QStringLiteral("length 必须为大于 0 的有效数值"));
        return;
    }

    common::BevelRecipe recipe;
    recipe.active = true;
    recipe.bevelType = bevelType;
    recipe.angleDeg = static_cast<float>(angleDeg);
    recipe.lengthMm = static_cast<float>(lengthMm);
    recipe.hasHole = payload.value(QLatin1String("has_hole")).toBool(false);
    cfgMgr->setBevelRecipe(recipe);

    QJsonObject responsePayload = buildResponsePayload(
        true, QStringLiteral("坡口工艺配方已更新"));
    responsePayload[QLatin1String("bevel_type")] = recipe.bevelType;
    responsePayload[QLatin1String("angle_deg")] = static_cast<double>(recipe.angleDeg);
    responsePayload[QLatin1String("length")] = static_cast<double>(recipe.lengthMm);
    responsePayload[QLatin1String("has_hole")] = recipe.hasHole;
    responsePayload[QLatin1String("angleTolDeg")] =
        static_cast<double>(cfgMgr->bevelConfig().angleTolDeg);
    responsePayload[QLatin1String("lengthTolMm")] =
        static_cast<double>(cfgMgr->bevelConfig().lengthTolMm);

    QJsonObject envelope;
    envelope[QStringLiteral("version")] = QLatin1String(kProtocolVersion);
    envelope[QStringLiteral("msgId")] = msgId;
    envelope[QStringLiteral("type")] = QLatin1String(msg_type::kCmdSetBevelRecipe);
    envelope[QStringLiteral("timestamp")] = QDateTime::currentMSecsSinceEpoch();
    envelope[QStringLiteral("payload")] = responsePayload;
    sendToClient(envelope);

    qInfo(LOG_HMI_SERVER).noquote()
        << QStringLiteral("[TCPIP] 坡口配方已更新")
        << QStringLiteral(" bevel_type=") << recipe.bevelType
        << QStringLiteral(" angle_deg=") << recipe.angleDeg
        << QStringLiteral(" length=") << recipe.lengthMm
        << QStringLiteral(" has_hole=") << recipe.hasHole;
}

void HmiTcpServer::handleCmdReportPersonZoneAlarm(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    const QJsonObject payload = message.value(QLatin1String("payload")).toObject();

    const QString requestType = message.value(QLatin1String("type")).toString();

    if (!payload.contains(QLatin1String("alarm"))) {
        qWarning(LOG_HMI_SERVER).noquote()
            << QStringLiteral("[TCPIP] 人员区域上报缺少 alarm 字段 type=") << requestType
            << QStringLiteral(" msgId=") << msgId;
        sendResponse(
            requestType.isEmpty() ? QLatin1String(msg_type::kCmdReportPersonZoneAlarm) : requestType,
            msgId,
            false,
            QStringLiteral("缺少必填字段：alarm"));
        return;
    }

    const bool alarm = payload.value(QLatin1String("alarm")).toBool();
    const bool stateChanged =
        !m_personZoneAlarmCacheValid || alarm != m_personZoneAlarm;

    qInfo(LOG_HMI_SERVER).noquote()
        << QStringLiteral("[TCPIP] 人员区域上报")
        << QStringLiteral(" alarm=") << (alarm ? QStringLiteral("true(有人)") : QStringLiteral("false(无人)"))
        << QStringLiteral(" msgId=") << msgId
        << (stateChanged ? QStringLiteral(" [状态变化]") : QStringLiteral(" [重复上报]"));

    m_personZoneAlarm = alarm;
    m_personZoneAlarmCacheValid = true;

    // 安全信号：不按状态去重 PLC 转发——每次上报都写 PLC。状态机层
    // reportPersonZoneAlarm 对重复值幂等（仅刷新 IPC_SafetyAction_Word 同一 bit，
    // 不重复进故障态），可在首次写入失败或 PLC 自行清位后补写，避免漏报。
    // stateChanged 仅用于日志标注与响应文案，不阻断转发。
    bool plcWritten = true;
    if (m_stateMachine) {
        plcWritten = m_stateMachine->reportPersonZoneAlarm(alarm);
    } else {
        qWarning(LOG_HMI_SERVER).noquote()
            << QStringLiteral("[TCPIP] 人员区域上报：状态机不可用，无法写 PLC");
        sendResponse(
            requestType.isEmpty() ? QLatin1String(msg_type::kCmdReportPersonZoneAlarm) : requestType,
            msgId,
            false,
            QStringLiteral("状态机不可用"));
        return;
    }

    QString responseMessage;
    if (stateChanged) {
        responseMessage = alarm
            ? QStringLiteral("人员区域报警已上报 PLC")
            : QStringLiteral("人员区域报警已解除");
    } else {
        responseMessage = alarm
            ? QStringLiteral("人员区域报警已刷新上报 PLC")
            : QStringLiteral("人员区域无报警状态已刷新");
    }
    if (!plcWritten) {
        responseMessage += QStringLiteral("（PLC 未连接或写入失败）");
    }

    sendResponse(
        requestType.isEmpty() ? QLatin1String(msg_type::kCmdReportPersonZoneAlarm) : requestType,
        msgId,
        true,
        responseMessage);
}

namespace {

quint16 clampUnloadAreaCount(int value, int maxValue)
{
    return static_cast<quint16>(qBound(0, value, maxValue));
}

bool parseOptionalUnloadAreaUInt(
    const QJsonObject& payload,
    const char* key,
    int maxValue,
    quint16& outValue,
    bool& hasField,
    QString& errorMessage)
{
    if (!payload.contains(QLatin1String(key))) {
        return true;
    }
    hasField = true;
    const QJsonValue raw = payload.value(QLatin1String(key));
    if (!raw.isDouble()) {
        errorMessage = QStringLiteral("字段 %1 必须为整数").arg(QString::fromLatin1(key));
        return false;
    }
    const int intValue = raw.toInt(-1);
    if (intValue < 0) {
        errorMessage = QStringLiteral("字段 %1 不能为负数").arg(QString::fromLatin1(key));
        return false;
    }
    outValue = clampUnloadAreaCount(intValue, maxValue);
    return true;
}

}  // namespace

void HmiTcpServer::handleCmdSetUnloadAreaConfig(const QJsonObject& message)
{
    namespace limits = flow_control::protocol::unload_area;

    const QString msgId = message.value(QLatin1String("msgId")).toString();
    const QJsonObject payload = message.value(QLatin1String("payload")).toObject();

    bool hasMaxStack = false;
    bool hasOkCount = false;
    bool hasNgCount = false;
    bool hasAutoClear = false;
    QString parseError;

    flow_control::StateMachine::UnloadAreaConfig config;
    if (m_stateMachine) {
        config = m_stateMachine->unloadAreaConfig();
    }

    if (!parseOptionalUnloadAreaUInt(
            payload, "maxStackCount", limits::kMaxStackCountLimit, config.maxStackCount, hasMaxStack, parseError)
        || !parseOptionalUnloadAreaUInt(
            payload, "okCount", limits::kHeadCountLimit, config.okCount, hasOkCount, parseError)
        || !parseOptionalUnloadAreaUInt(
            payload, "ngCount", limits::kHeadCountLimit, config.ngCount, hasNgCount, parseError)) {
        sendResponse(QLatin1String(msg_type::kCmdSetUnloadAreaConfig), msgId, false, parseError);
        return;
    }

    if (payload.contains(QLatin1String("autoClear"))) {
        hasAutoClear = true;
        config.autoClear = payload.value(QLatin1String("autoClear")).toBool(false)
            ? limits::kAutoClearOn
            : limits::kAutoClearOff;
    }

    if (!hasMaxStack && !hasOkCount && !hasNgCount && !hasAutoClear) {
        sendResponse(
            QLatin1String(msg_type::kCmdSetUnloadAreaConfig),
            msgId,
            false,
            QStringLiteral("至少提供一个字段：maxStackCount / okCount / ngCount / autoClear"));
        return;
    }

    if (!m_stateMachine) {
        sendResponse(
            QLatin1String(msg_type::kCmdSetUnloadAreaConfig),
            msgId,
            false,
            QStringLiteral("状态机不可用"));
        return;
    }

    const bool plcWritten = m_stateMachine->updateUnloadAreaConfig(config);

    QJsonObject responsePayload = buildResponsePayload(
        true,
        plcWritten ? QStringLiteral("下料区封头配置已转发 PLC")
                   : QStringLiteral("下料区封头配置已缓存（PLC 未连接或写入失败）"));
    responsePayload[QLatin1String("maxStackCount")] = static_cast<int>(config.maxStackCount);
    responsePayload[QLatin1String("okCount")] = static_cast<int>(config.okCount);
    responsePayload[QLatin1String("ngCount")] = static_cast<int>(config.ngCount);
    responsePayload[QLatin1String("autoClear")] = config.autoClear != 0;
    responsePayload[QLatin1String("plcWritten")] = plcWritten;

    QJsonObject envelope;
    envelope[QStringLiteral("version")] = QLatin1String(kProtocolVersion);
    envelope[QStringLiteral("msgId")] = msgId;
    envelope[QStringLiteral("type")] = QLatin1String(msg_type::kCmdSetUnloadAreaConfig);
    envelope[QStringLiteral("timestamp")] = QDateTime::currentMSecsSinceEpoch();
    envelope[QStringLiteral("payload")] = responsePayload;
    sendToClient(envelope);

    m_plcStatusCache.isValid = false;
    pushPlcStatus();

    qInfo(LOG_HMI_SERVER).noquote()
        << QStringLiteral("[TCPIP] 下料区封头配置")
        << QStringLiteral(" maxStack=") << config.maxStackCount
        << QStringLiteral(" ok=") << config.okCount
        << QStringLiteral(" ng=") << config.ngCount
        << QStringLiteral(" autoClear=") << config.autoClear
        << QStringLiteral(" plcWritten=") << plcWritten;
}

void HmiTcpServer::handleCmdCaptureMechEye(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    const QString cameraKey = message.value(QLatin1String("payload")).toObject().value(QLatin1String("cameraKey")).toString();
    if (m_mechEyeService) {
        quint64 reqId = m_mechEyeService->requestCapture(cameraKey, mech_eye::CaptureMode::Capture3DOnly);
        QJsonObject payload = buildResponsePayload(true, QStringLiteral("采集请求已发送"));
        payload[QLatin1String("requestId")] = static_cast<qint64>(reqId);
        
        QJsonObject envelope;
        envelope[QStringLiteral("version")]   = QLatin1String(kProtocolVersion);
        envelope[QStringLiteral("msgId")]     = msgId;
        envelope[QStringLiteral("type")]      = QLatin1String(msg_type::kCmdCaptureMechEye);
        envelope[QStringLiteral("timestamp")] = QDateTime::currentMSecsSinceEpoch();
        envelope[QStringLiteral("payload")]   = payload;
        sendToClient(envelope);
    } else {
        sendResponse(QLatin1String(msg_type::kCmdCaptureMechEye), msgId, false, QStringLiteral("相机服务不可用"));
    }
}

void HmiTcpServer::handleCmdCaptureBundle(const QJsonObject& message)
{
    const QString msgId = message.value(QLatin1String("msgId")).toString();
    if (m_visionPipeline) {
        const QJsonObject payloadObj = message.value(QLatin1String("payload")).toObject();
        const int segmentIndex = payloadObj.value(QLatin1String("segmentIndex")).toInt(1);
        const quint32 taskId = static_cast<quint32>(payloadObj.value(QLatin1String("taskId")).toInt(0));
        bool needMechEye2D = payloadObj.value(QLatin1String("needMechEye2D")).toBool(false);
        if (!payloadObj.contains(QLatin1String("needMechEye2D"))) {
            const auto* configMgr = scan_tracking::common::ConfigManager::instance();
            if (configMgr != nullptr) {
                for (const auto& path : configMgr->scanPathsConfig().scanPaths) {
                    if (!path.enabled) {
                        continue;
                    }
                    for (const auto& point : path.points) {
                        if (point.pointIndex == segmentIndex) {
                            needMechEye2D = point.needRotation;
                            break;
                        }
                    }
                    if (needMechEye2D) {
                        break;
                    }
                }
            }
        }
        const auto mechCaptureMode = needMechEye2D
            ? scan_tracking::mech_eye::CaptureMode::Capture2DAnd3D
            : scan_tracking::mech_eye::CaptureMode::Capture3DOnly;
        quint64 reqId = m_visionPipeline->requestCaptureBundle(segmentIndex, taskId, mechCaptureMode);
        
        QJsonObject payload = buildResponsePayload(true, QStringLiteral("组合采集请求已发送"));
        payload[QLatin1String("requestId")] = static_cast<qint64>(reqId);
        
        QJsonObject envelope;
        envelope[QStringLiteral("version")]   = QLatin1String(kProtocolVersion);
        envelope[QStringLiteral("msgId")]     = msgId;
        envelope[QStringLiteral("type")]      = QLatin1String(msg_type::kCmdCaptureBundle);
        envelope[QStringLiteral("timestamp")] = QDateTime::currentMSecsSinceEpoch();
        envelope[QStringLiteral("payload")]   = payload;
        sendToClient(envelope);
    } else {
        sendResponse(QLatin1String(msg_type::kCmdCaptureBundle), msgId, false, QStringLiteral("视觉流水线不可用"));
    }
}

// --- 状态推送实现 ---

void HmiTcpServer::invalidateStatusPushCache()
{
    m_systemStatusCache = {};
    m_plcStatusCache = {};
    m_cameraStatusCache = {};
    m_deviceStatusCache = {};
    m_cameraConnectivityCache = {};
    m_plcAuxDeviceAlarmCache = {};
}

bool HmiTcpServer::pushStatusIfChanged(const QString& type, const QJsonObject& payload,
                                       HmiStatusPushCache& slot, bool forcePush)
{
    if (!forcePush && slot.isValid && payload == slot.payload) {
        return false;
    }
    slot.payload = payload;
    slot.isValid = true;
    const QString msgId = nextEventId();
    sendToClient(buildEnvelope(type, msgId, payload));
    if (hasClient() && isStatusPushType(type)) {
        logStatusPushTx(type, msgId, payload);
    }
    return true;
}

void HmiTcpServer::pushAllStatusToClient()
{
    if (!hasClient()) {
        return;
    }
    pushStatusIfChanged(QLatin1String(msg_type::kStatusSystem), buildSystemStatusPayload(),
                        m_systemStatusCache, true);
    pushStatusIfChanged(QLatin1String(msg_type::kStatusPlc), buildPlcStatusPayload(),
                        m_plcStatusCache, true);
    pushStatusIfChanged(QLatin1String(msg_type::kStatusCamera), buildCameraStatusPayload(),
                        m_cameraStatusCache, true);
    pushStatusIfChanged(QLatin1String(msg_type::kStatusDevice), buildDeviceStatusPayload(),
                        m_deviceStatusCache, true);
}

void HmiTcpServer::syncStatusPushCacheFromCurrent()
{
    if (m_stateMachine) {
        m_systemStatusCache.payload = buildSystemStatusPayload();
        m_systemStatusCache.isValid = true;
    }
    if (m_modbusService) {
        m_plcStatusCache.payload = buildPlcStatusPayload();
        m_plcStatusCache.isValid = true;
    }
    m_cameraStatusCache.payload = buildCameraStatusPayload();
    m_cameraStatusCache.isValid = true;
    m_deviceStatusCache.payload = buildDeviceStatusPayload();
    m_deviceStatusCache.isValid = true;
    syncCameraConnectivityCache();
    if (m_stateMachine) {
        syncPlcAuxDeviceAlarmCache(m_stateMachine->lastCommandBlock());
    }
}

void HmiTcpServer::pushSystemStatus()
{
    if (!m_stateMachine) return;
    const QJsonObject payload = buildSystemStatusPayload();
    pushStatusIfChanged(QLatin1String(msg_type::kStatusSystem), payload, m_systemStatusCache);
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
    if (const auto* cfgMgr = scan_tracking::common::ConfigManager::instance()) {
        const auto& profile = cfgMgr->stationProfile();
        // stage1: station metadata extension
        payload[QLatin1String("stationId")] = scan_tracking::common::stationIdToInt(profile.stationId);
        payload[QLatin1String("stationName")] = profile.stationName;
        payload[QLatin1String("workMode")] =
            scan_tracking::common::workModeIdToString(profile.defaultWorkMode);
        QJsonArray enabledTriggers;
        for (const auto& trigger : scan_tracking::flow_control::protocol::triggerDefinitions()) {
            if (scan_tracking::flow_control::isTriggerEnabledForProfile(profile, trigger.trigOffset)) {
                enabledTriggers.append(QString::fromLatin1(trigger.name));
            }
        }
        payload[QLatin1String("enabledTriggers")] = enabledTriggers;
    }
    return payload;
}

void HmiTcpServer::pushPlcStatus()
{
    const QJsonObject payload = buildPlcStatusPayload();
    pushStatusIfChanged(QLatin1String(msg_type::kStatusPlc), payload, m_plcStatusCache);
}

QJsonObject HmiTcpServer::buildPlcStatusPayload() const
{
    QJsonObject payload;
    payload[QLatin1String("modbusConnected")] = m_modbusService && m_modbusService->isConnected();
    if (const auto* cfgMgr = scan_tracking::common::ConfigManager::instance()) {
        const auto& profile = cfgMgr->stationProfile();
        // stage1: station metadata extension
        payload[QLatin1String("stationId")] = scan_tracking::common::stationIdToInt(profile.stationId);
        payload[QLatin1String("stationName")] = profile.stationName;
        payload[QLatin1String("stationWorkMode")] =
            scan_tracking::common::workModeIdToString(profile.defaultWorkMode);
    }
    if (!m_modbusService) {
        return payload;
    }

    if (m_stateMachine) {
        namespace regs = flow_control::protocol::registers;
        const auto& cb = m_stateMachine->lastCommandBlock();
        if (cb.size() > regs::kScanSegmentIndex) {
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
            payload[QLatin1String("scanSegmentIndex")] = static_cast<int>(
                regs::resolveScanSegmentIndexFromBlock(cb));
            // scanSegmentTotal 从配置获取，不再从 PLC 读取
            const auto* cfgMgr = scan_tracking::common::ConfigManager::instance();
            payload[QLatin1String("scanSegmentTotal")] = cfgMgr ? cfgMgr->trackingConfig().scanSegmentTotal : 0;
            if (cb.size() > regs::kRobotStatusWord) {
                payload[QLatin1String("robotStatusWord")] = cb.value(regs::kRobotStatusWord);
            }
            if (cb.size() > regs::kEstopButtonStatus) {
                payload[QLatin1String("telescopicRodStatus")] = cb.value(regs::kTelescopicRodStatus);
                payload[QLatin1String("rollerSetFreqHz")] = cb.value(regs::kRollerSetFreqHz);
                payload[QLatin1String("rollerRunFreqHz")] = cb.value(regs::kRollerRunFreqHz);
                payload[QLatin1String("electromagnetStatus")] = cb.value(regs::kElectromagnetStatus);
                payload[QLatin1String("estopButtonStatus")] = cb.value(regs::kEstopButtonStatus);
            }
            if (cb.size() > regs::kUnloadOkAreaCount) {
                payload[QLatin1String("loadVisionStatusCode")] = cb.value(regs::kLoadVisionStatusCode);
                payload[QLatin1String("unloadNgAreaFull")] = cb.value(regs::kUnloadNgAreaFull);
                payload[QLatin1String("unloadOkAreaFull")] = cb.value(regs::kUnloadOkAreaFull);
                payload[QLatin1String("plcUnloadNgAreaCount")] = cb.value(regs::kUnloadNgAreaCount);
                payload[QLatin1String("plcUnloadOkAreaCount")] = cb.value(regs::kUnloadOkAreaCount);
            }
        }

        const auto unloadCfg = m_stateMachine->unloadAreaConfig();
        payload[QLatin1String("unloadAreaMaxStackCount")] = static_cast<int>(unloadCfg.maxStackCount);
        payload[QLatin1String("unloadAreaOkCount")] = static_cast<int>(unloadCfg.okCount);
        payload[QLatin1String("unloadAreaNgCount")] = static_cast<int>(unloadCfg.ngCount);
        payload[QLatin1String("unloadAreaAutoClear")] = unloadCfg.autoClear != 0;
    }
    return payload;
}

void HmiTcpServer::syncPlcAuxDeviceAlarmCache(const QVector<quint16>& commandBlock)
{
    namespace regs = flow_control::protocol::registers;
    if (commandBlock.size() <= regs::kEstopButtonStatus) {
        m_plcAuxDeviceAlarmCache = {};
        return;
    }
    m_plcAuxDeviceAlarmCache.telescopicRodStatus = commandBlock.value(regs::kTelescopicRodStatus);
    m_plcAuxDeviceAlarmCache.electromagnetStatus = commandBlock.value(regs::kElectromagnetStatus);
    m_plcAuxDeviceAlarmCache.valid = true;
}

void HmiTcpServer::emitPlcAuxDeviceAlarm(const QString& message, int code, int level)
{
    if (!hasClient()) {
        return;
    }
    QJsonObject payload;
    payload[QLatin1String("message")] = message;
    payload[QLatin1String("level")] = level;
    payload[QLatin1String("code")] = code;
    payload[QLatin1String("timestamp")] = QDateTime::currentMSecsSinceEpoch();
    sendToClient(buildEnvelope(QLatin1String(msg_type::kEventAlarm), nextEventId(), payload));
}

void HmiTcpServer::checkPlcAuxDeviceAlarms(const QVector<quint16>& commandBlock)
{
    namespace regs = flow_control::protocol::registers;
    if (commandBlock.size() <= regs::kEstopButtonStatus) {
        return;
    }

    const int rodStatus = commandBlock.value(regs::kTelescopicRodStatus);
    const int magnetStatus = commandBlock.value(regs::kElectromagnetStatus);

    const auto emitIfFault = [this](int current, int previous, bool cacheValid, int code, const QString& label) {
        if (current != 2) {
            return;
        }
        if (!cacheValid || previous != 2) {
            emitPlcAuxDeviceAlarm(QStringLiteral("PLC辅机报警：%1").arg(label), code, 2);
        }
    };

    emitIfFault(rodStatus, m_plcAuxDeviceAlarmCache.telescopicRodStatus,
                m_plcAuxDeviceAlarmCache.valid, kAlarmCodeTelescopicRodFault,
                QStringLiteral("伸缩杆故障"));
    emitIfFault(magnetStatus, m_plcAuxDeviceAlarmCache.electromagnetStatus,
                m_plcAuxDeviceAlarmCache.valid, kAlarmCodeElectromagnetFault,
                QStringLiteral("电磁吸盘报警"));

    m_plcAuxDeviceAlarmCache.telescopicRodStatus = rodStatus;
    m_plcAuxDeviceAlarmCache.electromagnetStatus = magnetStatus;
    m_plcAuxDeviceAlarmCache.valid = true;
}

void HmiTcpServer::pushCameraStatus()
{
    const QJsonObject payload = buildCameraStatusPayload();
    pushStatusIfChanged(QLatin1String(msg_type::kStatusCamera), payload, m_cameraStatusCache);
    checkCameraConnectivityEdges();
}

bool HmiTcpServer::mechEyeConnected() const
{
    if (!m_mechEyeService) {
        return false;
    }
    const auto state = m_mechEyeService->state();
    return state != mech_eye::CameraRuntimeState::Idle
        && state != mech_eye::CameraRuntimeState::Error;
}

void HmiTcpServer::syncCameraConnectivityCache()
{
    if (m_mechEyeService) {
        m_cameraConnectivityCache.mechEye = mechEyeConnected();
    }
    if (m_hikCameraA) {
        m_cameraConnectivityCache.hikA = m_hikCameraA->isConnected();
    }
    if (m_hikCameraB) {
        m_cameraConnectivityCache.hikB = m_hikCameraB->isConnected();
    }
    if (m_hikCameraC || m_hikCameraCController) {
        m_cameraConnectivityCache.hikC = hikCameraCConnected();
    }
    m_cameraConnectivityCache.valid = true;
}

void HmiTcpServer::emitCameraConnectivityAlarm(const QString& deviceLabel, bool connected, int code)
{
    if (!hasClient()) {
        return;
    }
    QJsonObject payload;
    payload[QLatin1String("message")] = connected
        ? QStringLiteral("%1已连接").arg(deviceLabel)
        : QStringLiteral("%1已断开").arg(deviceLabel);
    payload[QLatin1String("level")] = connected ? 1 : 2;
    payload[QLatin1String("code")] = code;
    payload[QLatin1String("timestamp")] = QDateTime::currentMSecsSinceEpoch();
    sendToClient(buildEnvelope(QLatin1String(msg_type::kEventAlarm), nextEventId(), payload));
}

void HmiTcpServer::checkCameraConnectivityEdges()
{
    if (!hasClient()) {
        return;
    }
    if (!m_cameraConnectivityCache.valid) {
        syncCameraConnectivityCache();
        return;
    }

    if (m_mechEyeService) {
        const bool nowConnected = mechEyeConnected();
        if (nowConnected != m_cameraConnectivityCache.mechEye) {
            emitCameraConnectivityAlarm(
                QStringLiteral("梅卡相机"),
                nowConnected,
                nowConnected ? kAlarmCodeMechEyeConnect : kAlarmCodeMechEyeDisconnect);
            m_cameraConnectivityCache.mechEye = nowConnected;
        }
    }

    if (m_hikCameraA) {
        const bool nowConnected = m_hikCameraA->isConnected();
        if (nowConnected != m_cameraConnectivityCache.hikA) {
            const QString label = QStringLiteral("海康相机 [%1]").arg(m_hikCameraA->roleName());
            emitCameraConnectivityAlarm(
                label,
                nowConnected,
                nowConnected ? kAlarmCodeHikConnect : kAlarmCodeHikDisconnect);
            m_cameraConnectivityCache.hikA = nowConnected;
        }
    }

    if (m_hikCameraB) {
        const bool nowConnected = m_hikCameraB->isConnected();
        if (nowConnected != m_cameraConnectivityCache.hikB) {
            const QString label = QStringLiteral("海康相机 [%1]").arg(m_hikCameraB->roleName());
            emitCameraConnectivityAlarm(
                label,
                nowConnected,
                nowConnected ? kAlarmCodeHikConnect : kAlarmCodeHikDisconnect);
            m_cameraConnectivityCache.hikB = nowConnected;
        }
    }

    if (m_hikCameraC || m_hikCameraCController) {
        const bool nowConnected = hikCameraCConnected();
        if (nowConnected != m_cameraConnectivityCache.hikC) {
            QString roleName = QStringLiteral("hikC");
            if (m_hikCameraC) {
                roleName = m_hikCameraC->roleName();
            }
            const QString label = QStringLiteral("海康相机 [%1]").arg(roleName);
            emitCameraConnectivityAlarm(
                label,
                nowConnected,
                nowConnected ? kAlarmCodeHikConnect : kAlarmCodeHikDisconnect);
            m_cameraConnectivityCache.hikC = nowConnected;
        }
    }
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

    if (m_hikCameraC) {
        QJsonObject hikCObj;
        hikCObj[QLatin1String("roleName")] = m_hikCameraC->roleName();
        hikCObj[QLatin1String("connected")] = hikCameraCConnected();
        payload[QLatin1String("hikC")] = hikCObj;
    }
    
    if (m_visionPipeline) {
        QJsonObject pipelineObj;
        pipelineObj[QLatin1String("state")] = static_cast<int>(m_visionPipeline->state());
        payload[QLatin1String("pipeline")] = pipelineObj;
    }
    return payload;
}

bool HmiTcpServer::hikCameraCConnected() const
{
    if (m_hikCameraCController && m_hikCameraCController->isStarted()) {
        if (m_hikCameraCController->isCameraConnectedToTcp()) {
            return true;
        }
    }
    return m_hikCameraC && m_hikCameraC->isConnected();
}

void HmiTcpServer::pushDeviceStatus()
{
    const QJsonObject payload = buildDeviceStatusPayload();
    pushStatusIfChanged(QLatin1String(msg_type::kStatusDevice), payload, m_deviceStatusCache);
}

QJsonObject HmiTcpServer::buildDeviceStatusPayload() const
{
    // onlineWord0 / faultWord0 位定义与 docs/protocols/封头检测工位_TCP_IP显控通信协议_v1.0.md §2.4 一致
    constexpr int kBitIpcCore = 0;
    constexpr int kBitHmiClient = 1;
    constexpr int kBitMechEye = 2;
    constexpr int kBitVisionPipeline = 3;
    constexpr int kBitHik2d = 4;
    constexpr int kBitTracking = 5;
    constexpr int kBitModbus = 6;
    constexpr int kBitAlarmWarn = 7;

    quint16 onlineWord0 = 0;
    quint16 faultWord0 = 0;

    onlineWord0 |= (1u << kBitIpcCore);

    if (hasClient()) {
        onlineWord0 |= (1u << kBitHmiClient);
    }

    const bool mechEyeOnline = m_mechEyeService
        && m_mechEyeService->state() != mech_eye::CameraRuntimeState::Idle
        && m_mechEyeService->state() != mech_eye::CameraRuntimeState::Error;
    if (mechEyeOnline) {
        onlineWord0 |= (1u << kBitMechEye);
    }
    if (m_mechEyeService && m_mechEyeService->state() == mech_eye::CameraRuntimeState::Error) {
        faultWord0 |= (1u << kBitMechEye);
    }

    if (m_visionPipeline && m_visionPipeline->state() == vision::VisionPipelineState::Ready) {
        onlineWord0 |= (1u << kBitVisionPipeline);
    }
    if (m_visionPipeline && m_visionPipeline->state() == vision::VisionPipelineState::Error) {
        faultWord0 |= (1u << kBitVisionPipeline);
    }

    const bool hikAOnline = m_hikCameraA && m_hikCameraA->isConnected();
    const bool hikBOnline = m_hikCameraB && m_hikCameraB->isConnected();
    const bool hikCOnline = hikCameraCConnected();
    const bool hasAnyHikService = m_hikCameraA || m_hikCameraB || m_hikCameraC;
    if (hikAOnline || hikBOnline || hikCOnline) {
        onlineWord0 |= (1u << kBitHik2d);
    }
    if (hasAnyHikService && !hikAOnline && !hikBOnline && !hikCOnline) {
        faultWord0 |= (1u << kBitHik2d);
    }

    if (m_trackingService) {
        onlineWord0 |= (1u << kBitTracking);
    }

    const bool modbusOnline = m_modbusService && m_modbusService->isConnected();
    if (modbusOnline) {
        onlineWord0 |= (1u << kBitModbus);
    }
    if (m_modbusService && !modbusOnline) {
        faultWord0 |= (1u << kBitModbus);
    }

    if (m_stateMachine) {
        const auto ipcState = m_stateMachine->ipcState();
        const auto appState = m_stateMachine->currentState();
        const quint16 alarmLevel = m_stateMachine->alarmLevel();

        if (ipcState == flow_control::protocol::IpcState::Fault
            || appState == flow_control::AppState::Error) {
            faultWord0 |= (1u << kBitIpcCore);
        }
        if (alarmLevel >= 3) {
            faultWord0 |= (1u << kBitIpcCore);
        }
        if (alarmLevel >= 2) {
            faultWord0 |= (1u << kBitAlarmWarn);
        }
    }

    QJsonObject payload;
    payload[QLatin1String("onlineWord0")] = onlineWord0;
    payload[QLatin1String("faultWord0")] = faultWord0;
    return payload;
}

// --- 事件连接 ---

void HmiTcpServer::connectStateMachineSignals()
{
    if (!m_stateMachine) {
        return;
    }

    qRegisterMetaType<QVector<double>>("QVector<double>");

    // 绑定核心业务报警事件：将状态机发出的协议级错误拦截并封装为 event.alarm 向远端推送
    const bool okProtocol = connect(m_stateMachine, &flow_control::StateMachine::protocolEvent, this,
        [this](const QString& message) {
        QJsonObject payload;
        payload[QLatin1String("message")] = message;
        payload[QLatin1String("level")] = m_stateMachine->alarmLevel();
        payload[QLatin1String("code")] = m_stateMachine->alarmCode();
        payload[QLatin1String("timestamp")] = QDateTime::currentMSecsSinceEpoch();
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventAlarm), nextEventId(), payload));
    }, Qt::UniqueConnection);
    if (!okProtocol) {
        qCritical(LOG_HMI_SERVER) << "connect protocolEvent 失败，请完整重编 flow_control 与 hmi_server";
    }

    // 绑定扫描分段启动事件：告知显控界面哪一段扫描正在拍摄
    connect(m_stateMachine, &flow_control::StateMachine::scanStarted, this,
        [this](int segmentIndex, quint32 taskId) {
        QJsonObject payload;
        payload[QLatin1String("segmentIndex")] = segmentIndex;
        payload[QLatin1String("taskId")] = static_cast<int>(taskId);
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventScanStarted), nextEventId(), payload));
    }, Qt::UniqueConnection);

    // 绑定扫描分段完成事件：推送当前段是否采图成功及有效帧数
    connect(m_stateMachine, &flow_control::StateMachine::scanFinished, this,
        [this](int segmentIndex, quint16 resultCode, int imageCount, int cloudFrameCount) {
        QJsonObject payload;
        payload[QLatin1String("segmentIndex")] = segmentIndex;
        payload[QLatin1String("resultCode")] = resultCode; // 1 表示成功
        payload[QLatin1String("imageCount")] = imageCount;
        payload[QLatin1String("cloudFrameCount")] = cloudFrameCount;
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventScanFinished), nextEventId(), payload));
    }, Qt::UniqueConnection);

    // TODO(hmi-demo): 综合检测结果改由 TrackingService::deliverInspectionResult → publishInspectionResult 直推，
    // 不再绑定 inspectionFinished，避免与显控收到重复 event.inspection.finished。

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
    }, Qt::UniqueConnection);

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
    }, Qt::UniqueConnection);

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
    }, Qt::UniqueConnection);

    // 自检完成
    connect(m_stateMachine, &flow_control::StateMachine::selfCheckFinished, this,
        [this](quint16 resultCode, quint16 failWord0) {
        QJsonObject payload;
        payload[QLatin1String("resultCode")] = resultCode;
        payload[QLatin1String("failWord0")] = failWord0;
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventSelfCheckFinished), nextEventId(), payload));
    }, Qt::UniqueConnection);

    // 条码读取完成
    connect(m_stateMachine, &flow_control::StateMachine::codeReadFinished, this,
        [this](quint16 resultCode, const QString& codeValue) {
        QJsonObject payload;
        payload[QLatin1String("resultCode")] = resultCode;
        payload[QLatin1String("codeValue")] = codeValue;
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventCodeReadFinished), nextEventId(), payload));
    }, Qt::UniqueConnection);

    // 结果复位完成
    connect(m_stateMachine, &flow_control::StateMachine::resultResetFinished, this,
        [this](quint16 resultCode) {
        QJsonObject payload;
        payload[QLatin1String("resultCode")] = resultCode;
        sendToClient(buildEnvelope(QLatin1String(msg_type::kEventResultResetFinished), nextEventId(), payload));
    }, Qt::UniqueConnection);
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
        if (hasClient()) {
            pushDeviceStatus();
        }
    });
}

void HmiTcpServer::connectStatusRefreshSignals()
{
    if (m_stateMachine) {
        connect(m_stateMachine, &flow_control::StateMachine::stateChanged, this, [this]() {
            if (hasClient()) {
                pushSystemStatus();
                pushDeviceStatus();
            }
        }, Qt::UniqueConnection);
    }

    if (m_modbusService) {
        connect(m_modbusService, &modbus::ModbusService::connected, this, [this]() {
            if (hasClient()) {
                pushPlcStatus();
                pushDeviceStatus();
            }
        }, Qt::UniqueConnection);
        connect(m_modbusService, &modbus::ModbusService::disconnected, this, [this]() {
            if (hasClient()) {
                pushPlcStatus();
                pushDeviceStatus();
            }
        }, Qt::UniqueConnection);
        connect(m_modbusService, &modbus::ModbusService::registersRead, this,
                [this](int, const QVector<quint16>& values) {
            checkPlcAuxDeviceAlarms(values);
        }, Qt::UniqueConnection);
    }

    if (m_mechEyeService) {
        connect(m_mechEyeService, &mech_eye::MechEyeService::stateChanged, this,
                [this](mech_eye::CameraRuntimeState, QString) {
            if (hasClient()) {
                pushCameraStatus();
                pushDeviceStatus();
            }
        }, Qt::UniqueConnection);
    }

    if (m_visionPipeline) {
        connect(m_visionPipeline, &vision::VisionPipelineService::stateChanged, this,
                [this](vision::VisionPipelineState, const QString&) {
            if (hasClient()) {
                pushCameraStatus();
                pushDeviceStatus();
            }
        }, Qt::UniqueConnection);
    }

    auto refreshCameraOnHik = [this](QString, QString, QString) {
        if (hasClient()) {
            pushCameraStatus();
            pushDeviceStatus();
        }
    };
    if (m_hikCameraA) {
        connect(m_hikCameraA, &vision::HikCxpCameraService::stateChanged, this, refreshCameraOnHik, Qt::UniqueConnection);
    }
    if (m_hikCameraB) {
        connect(m_hikCameraB, &vision::HikCxpCameraService::stateChanged, this, refreshCameraOnHik, Qt::UniqueConnection);
    }
    if (m_hikCameraC) {
        connect(m_hikCameraC, &vision::HikCameraService::stateChanged, this, refreshCameraOnHik, Qt::UniqueConnection);
    }

    if (m_hikCameraCController) {
        connect(m_hikCameraCController, &vision::HikCameraCController::stateChanged, this,
                [this](vision::HikCameraCState, const QString&) {
            if (hasClient()) {
                pushCameraStatus();
                pushDeviceStatus();
            }
        }, Qt::UniqueConnection);
    }
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

// --- 日志转发实现（默认关闭，见 kForwardQtLogsToHmi）---

void HmiTcpServer::installLogForwarder()
{
    if (s_logForwarderInstalled) {
        return;
    }

    // 保存当前实例指针，供全局回调函数使用
    s_instance = this;
    
    // 拦截全局 Qt 日志输出，并保存原有处理器以支持链式调用
    s_previousHandler = qInstallMessageHandler([](QtMsgType type, const QMessageLogContext& context, const QString& msg) {
        const QString category = QString::fromLatin1(context.category ? context.category : "default");

        if (s_instance && s_instance->hasClient() && shouldForwardLogToHmi(type, category, msg)) {
            QJsonObject payload;
            int severity = 2;
            switch (type) {
            case QtWarningMsg: severity = 2; break;
            case QtCriticalMsg: severity = 3; break;
            case QtFatalMsg: severity = 4; break;
            default: break;
            }
            payload[QLatin1String("severity")] = severity;
            payload[QLatin1String("category")] = category;
            payload[QLatin1String("message")] = msg;
            payload[QLatin1String("timestamp")] = QDateTime::currentMSecsSinceEpoch();

            g_inLogForward = true;
            s_instance->sendToClient(buildEnvelope(QLatin1String(msg_type::kEventLog), s_instance->nextEventId(), payload));
            g_inLogForward = false;
        }

        if (s_previousHandler) {
            s_previousHandler(type, context, msg);
        }
    });
    s_logForwarderInstalled = true;
}

void HmiTcpServer::uninstallLogForwarder()
{
    if (!s_logForwarderInstalled) {
        return;
    }
    if (s_previousHandler) {
        qInstallMessageHandler(s_previousHandler);
        s_previousHandler = nullptr;
    }
    s_instance = nullptr;
    s_logForwarderInstalled = false;
}

// --- 综合检测结果推送（演示初版）---

QJsonObject HmiTcpServer::buildInspectionFinishedPayload(const tracking::InspectionResult& result)
{
    QJsonObject payload;
    payload[QLatin1String("resultCode")] = result.resultCode;
    payload[QLatin1String("ngReasonWord0")] = result.ngReasonWord0;
    payload[QLatin1String("ngReasonWord1")] = result.ngReasonWord1;
    payload[QLatin1String("measureItemCount")] = result.measureItemCount;
    tracking::appendInspectionMeasurementFields(payload, result.measurement);
    tracking::appendHeadDisplayMetricsFields(payload, result.measurement);
    payload[QLatin1String("message")] = result.message;
    payload[QLatin1String("sourcePointCount")] = result.sourcePointCount;

    if (const auto* cfgMgr = common::ConfigManager::instance()) {
        const common::BevelRecipe recipe = cfgMgr->bevelRecipe();
        const common::BevelConfig& bevelConfig = cfgMgr->bevelConfig();
        payload[QLatin1String("expected_bevel_type")] = recipe.bevelType;
        payload[QLatin1String("expected_angle_deg")] = static_cast<double>(recipe.angleDeg);
        payload[QLatin1String("expected_length")] = static_cast<double>(recipe.lengthMm);
        payload[QLatin1String("has_hole")] = recipe.hasHole;
        payload[QLatin1String("angle_tol_deg")] = static_cast<double>(bevelConfig.angleTolDeg);
        payload[QLatin1String("length_tol_mm")] = static_cast<double>(bevelConfig.lengthTolMm);
    }

    return payload;
}

void HmiTcpServer::publishInitialInspectionDisplay()
{
    if (!hasClient()) {
        return;
    }

    tracking::InspectionResult idle;
    idle.resultCode = 0;
    idle.measureItemCount = 0;
    idle.sourcePointCount = 0;
    idle.message = QStringLiteral("等待检测");
    idle.measurement.bevelType = 0;
    idle.measurement.qualityCode = 0;

    const QJsonObject payload = buildInspectionFinishedPayload(idle);
    sendToClient(buildEnvelope(QLatin1String(msg_type::kEventInspectionFinished), nextEventId(), payload));

    qInfo(LOG_HMI_SERVER).noquote()
        << QStringLiteral("[TCPIP] 显控连接后已推送初始检测展示帧 event.inspection.finished（全零占位）");
}

void HmiTcpServer::publishInspectionResult(const tracking::InspectionResult& result)
{
    if (!hasClient()) {
        // TODO(hmi-demo): 无显控连接时缓存最后一帧，连接后补发
        qInfo(LOG_HMI_SERVER).noquote()
            << QStringLiteral("[TCPIP] 坡口测量结果未推送（无显控连接）")
            << QStringLiteral(" resultCode=") << result.resultCode
            << QStringLiteral(" message=") << result.message;
        return;
    }

    const QJsonObject payload = buildInspectionFinishedPayload(result);
    sendToClient(buildEnvelope(QLatin1String(msg_type::kEventInspectionFinished), nextEventId(), payload));

    qInfo(LOG_HMI_SERVER).noquote()
        << QStringLiteral("[TCPIP] 坡口测量结果已推送 event.inspection.finished")
        << QStringLiteral(" resultCode=") << result.resultCode
        << QStringLiteral(" ngWord0=") << result.ngReasonWord0
        << QStringLiteral(" measureItems=") << result.measureItemCount
        << QStringLiteral(" angleDeg=") << result.measurement.headAngleTol
        << QStringLiteral(" lengthMm=") << result.measurement.bluntHeightTol
        << QStringLiteral(" message=") << result.message;
}

// --- 辅助发送 ---

void HmiTcpServer::sendToClient(const QJsonObject& envelope)
{
    if (!m_session) {
        return;
    }

    const QString type = envelope.value(QLatin1String("type")).toString();
    const QString msgId = envelope.value(QLatin1String("msgId")).toString();

    g_inTcpSend = true;
    if (kHmiTcpVerboseTrace) {
        const QJsonObject payload = envelope.value(QLatin1String("payload")).toObject();
        qInfo(LOG_HMI_SERVER).noquote()
            << QStringLiteral("[TCPIP] → TX") << type << msgId
            << summarizeHmiTracePayload(type, payload);
    }
    m_session->sendMessage(envelope);
    g_inTcpSend = false;
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
