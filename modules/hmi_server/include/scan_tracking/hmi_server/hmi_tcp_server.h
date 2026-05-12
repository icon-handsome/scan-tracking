#pragma once

/**
 * @file hmi_tcp_server.h
 * @brief HMI TCP 服务端，负责与 Qt 显控界面的通信
 *
 * 本模块是核心控制程序与 Qt 显控界面之间的通信桥梁，主要功能：
 * 1. 监听 TCP 端口，接受单个 Qt 客户端连接
 * 2. 周期性推送系统状态（IPC/PLC/相机/设备）
 * 3. 转发业务事件（扫描/检测/报警等）
 * 4. 接收并处理 Qt 端的控制命令
 *
 * 注意：本模块不直接控制 PLC，所有命令都通过 StateMachine 间接调度。
 */

#include <QtCore/QObject>
#include <QtCore/QTimer>
#include <QtCore/QString>

QT_BEGIN_NAMESPACE
class QTcpServer;
QT_END_NAMESPACE

namespace scan_tracking {
namespace modbus { class ModbusService; }
namespace mech_eye { class MechEyeService; }
namespace tracking { class TrackingService; }
namespace flow_control { class StateMachine; }
namespace vision {
class HikCameraService;
class VisionPipelineService;
}
namespace hmi_server {

class HmiSession;

class HmiTcpServer : public QObject {
    Q_OBJECT

public:
    /**
     * @brief 构造函数
     * @param port 监听端口号
     * @param parent 父对象
     */
    explicit HmiTcpServer(int port = 9900, QObject* parent = nullptr);
    ~HmiTcpServer() override;

    /// 启动 TCP 服务器
    bool start();

    /// 停止 TCP 服务器
    void stop();

    /// 检查服务器是否正在监听
    bool isListening() const;

    /// 检查是否有客户端已连接
    bool hasClient() const;

    // --- 设置服务依赖（在 start() 之前调用） ---
    void setStateMachine(flow_control::StateMachine* sm);
    void setModbusService(modbus::ModbusService* svc);
    void setMechEyeService(mech_eye::MechEyeService* svc);
    void setVisionPipelineService(vision::VisionPipelineService* svc);
    void setTrackingService(tracking::TrackingService* svc);
    void setHikCameraServices(vision::HikCameraService* hikA, vision::HikCameraService* hikB);

private slots:
    /// 处理新的客户端连接
    void onNewConnection();

    /// 处理客户端断开
    void onSessionDisconnected();

    /// 处理客户端心跳超时
    void onSessionHeartbeatTimeout();

    /// 处理接收到的客户端消息
    void onMessageReceived(const QJsonObject& message);

    /// 周期性推送状态
    void onStatusPushTimer();

    /// 周期性发送心跳
    void onHeartbeatTimer();

private:
    // --- 命令处理函数 ---
    
    /// 处理 Qt 端的 hmi.hello 握手请求
    void handleHmiHello(const QJsonObject& message);
    
    /// 处理 Qt 端发来的心跳 ping 包，返回 pong
    void handleHeartbeatPing(const QJsonObject& message);
    
    /// 处理启动指令，触发核心状态机进入启动状态
    void handleCmdStart(const QJsonObject& message);
    
    /// 处理停止指令，触发核心状态机停止
    void handleCmdStop(const QJsonObject& message);
    
    /// 处理复位指令，重置核心系统异常状态
    void handleCmdReset(const QJsonObject& message);
    
    /// 处理清除报警指令，清空当前系统的报警码
    void handleCmdClearAlarm(const QJsonObject& message);
    
    /// 处理获取全量状态指令，立刻推送系统、PLC、相机等全量状态
    void handleCmdGetStatus(const QJsonObject& message);
    
    /// 处理获取配置指令 (暂未开放热更配置，返回空)
    void handleCmdGetConfig(const QJsonObject& message);
    
    /// 处理连接 Modbus 指令，手动重连 PLC
    void handleCmdModbusConnect(const QJsonObject& message);
    
    /// 处理断开 Modbus 指令，手动断开 PLC
    void handleCmdModbusDisconnect(const QJsonObject& message);
    
    /// 处理刷新相机连接指令
    void handleCmdRefreshCamera(const QJsonObject& message);
    
    /// 处理触发扫描指令 (安全限制：拦截 Qt 端的直接触发，防止撞机，必须由 PLC 触发)
    void handleCmdTriggerScan(const QJsonObject& message);
    
    /// 处理触发综合检测指令 (安全限制：拦截 Qt 端触发，须由 PLC 触发)
    void handleCmdTriggerInspection(const QJsonObject& message);
    
    /// 处理触发自检指令 (安全限制：拦截 Qt 端触发，须由 PLC 触发)
    void handleCmdTriggerSelfCheck(const QJsonObject& message);
    
    /// 处理触发位姿校验指令 (安全限制：拦截 Qt 端触发，须由 PLC 触发)
    void handleCmdTriggerPoseCheck(const QJsonObject& message);
    
    /// 处理触发扫码指令 (安全限制：拦截 Qt 端触发，须由 PLC 触发)
    void handleCmdTriggerCodeRead(const QJsonObject& message);
    
    /// 处理触发结果复位指令 (安全限制：拦截 Qt 端触发，须由 PLC 触发)
    void handleCmdTriggerResultReset(const QJsonObject& message);
    
    /// 处理单独通过 Mech-Eye 进行采图的指令（多用于独立标定或调试测试）
    void handleCmdCaptureMechEye(const QJsonObject& message);
    
    /// 处理发起多相机联合采集的点云/图像 Bundle 指令
    void handleCmdCaptureBundle(const QJsonObject& message);


    // --- 状态推送函数 ---
    
    /// 向 Qt 端周期性推送系统整体运行状态（如 IPCState、报警级别、整体进度等）
    void pushSystemStatus();
    
    /// 向 Qt 端周期性推送 PLC 连接状态及内部部分寄存器运行状态
    void pushPlcStatus();
    
    /// 向 Qt 端周期性推送各相机的连接状态与运行阶段
    void pushCameraStatus();


    // --- 事件上报连接 ---
    
    /// 绑定状态机发出的各种业务及异常信号，组装 JSON 转发到 Qt 端
    void connectStateMachineSignals();
    
    /// 绑定 Modbus 底层通信抛出的断连、超时等信号，组装 JSON 转发到 Qt 端
    void connectModbusSignals();
    
    /// 绑定 MechEye 3D 相机抛出的采集完成或故障信号
    void connectMechEyeSignals();
    
    /// 绑定视觉流水线抛出的多相机 Bundle 采集完成信号
    void connectVisionPipelineSignals();


    // --- 发送辅助函数 ---
    
    /// 将封装好的完整 JSON 信封通过 TCP Session 发送给客户端
    /// @param envelope 带有 version, msgId, type, timestamp, payload 的完整 JSON 对象
    void sendToClient(const QJsonObject& envelope);
    
    /// 辅助构造并发送标准格式的响应报文 (Response) 给客户端
    /// @param type    响应对应的命令 type
    /// @param msgId   对应请求传入的 msgId
    /// @param success 指令是否执行成功
    /// @param message 附带的结果描述信息
    void sendResponse(const QString& type, const QString& msgId, bool success, const QString& message);
    
    /// 生成递增的唯一事件 ID (如: evt-1, evt-2)，用于 Core 主动上报的 event
    QString nextEventId();

    QTcpServer* m_tcpServer = nullptr;      ///< Qt TCP 服务器
    HmiSession* m_session = nullptr;        ///< 当前客户端会话（单客户端模式）
    int m_port = 9900;                      ///< 监听端口
    QTimer* m_statusPushTimer = nullptr;    ///< 状态推送定时器
    QTimer* m_heartbeatTimer = nullptr;     ///< 心跳发送定时器
    quint64 m_nextEventId = 1;             ///< 事件 ID 自增序号

    // --- 服务依赖指针 ---
    flow_control::StateMachine* m_stateMachine = nullptr;
    modbus::ModbusService* m_modbusService = nullptr;
    mech_eye::MechEyeService* m_mechEyeService = nullptr;
    vision::VisionPipelineService* m_visionPipeline = nullptr;
    tracking::TrackingService* m_trackingService = nullptr;
    vision::HikCameraService* m_hikCameraA = nullptr;
    vision::HikCameraService* m_hikCameraB = nullptr;
};

}  // namespace hmi_server
}  // namespace scan_tracking
