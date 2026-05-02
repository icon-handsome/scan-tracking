#include "scan_tracking/flow_control/state_machine.h"

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/common/logger.h"
#include "scan_tracking/mech_eye/mech_eye_service.h"

#include <QtCore/QLoggingCategory>
#include <QtCore/QThread>
#include <cstring>

namespace scan_tracking::flow_control {

// 定义流程控制模块的日志分类
Q_LOGGING_CATEGORY(LOG_FLOW, "flow_control")

namespace {

/**
 * @brief 设备在线状态字位掩码
 * 
 * 用于标识 IPC 设备在线状态的各个比特位，包括：
 * - Bit 0: 系统就绪
 * - Bit 1: 通信正常
 * - Bit 2: 传感器在线
 * - Bit 4: 相机连接
 * - Bit 5: 跟踪服务
 * - Bit 6: Modbus 连接
 */
constexpr quint16 kDeviceOnlineWord0 =
    (1u << 0) |
    (1u << 1) |
    (1u << 2) |
    (1u << 4) |
    (1u << 5) |
    (1u << 6);

/// 最大扫描分段索引（从1开始计数）
constexpr int kMaxScanSegmentIndex = 10;

/// 默认扫描分段采集超时时间（毫秒）
constexpr int kDefaultScanSegmentCaptureTimeoutMs = 30000;

/// 允许的最大连续 Modbus 通信失败次数，超过此值将进入故障状态
constexpr int kMaxConsecutiveModbusFailures = 3;

/**
 * @brief 将浮点数转换为 CDAB 字节序的两个寄存器值
 * 
 * Modbus 协议中 float 类型通常占用两个 16 位寄存器，采用 CDAB 字节序：
 * - 低16位在前（Little Endian 的低位字）
 * - 高16位在后（Little Endian 的高位字）
 * 
 * @param value 要转换的浮点数值
 * @return 包含两个寄存器值的向量 [low_word, high_word]
 */
QVector<quint16> floatToCdabRegisters(float value)
{
    quint32 raw = 0;
    static_assert(sizeof(raw) == sizeof(value), "Unexpected float width");
    std::memcpy(&raw, &value, sizeof(raw));

    const quint16 high = static_cast<quint16>((raw >> 16) & 0xFFFFu);
    const quint16 low = static_cast<quint16>(raw & 0xFFFFu);
    return {low, high};
}

}  // namespace

/**
 * @brief 状态机构造函数
 * 
 * 初始化流程控制状态机，配置定时器、连接信号槽，并设置初始状态。
 * 
 * @param modbusService Modbus 通信服务指针
 * @param mechEyeService Mech-Eye 相机服务指针
 * @param trackingService 跟踪检测服务指针
 * @param parent Qt 父对象指针
 */
StateMachine::StateMachine(
    modbus::ModbusService* modbusService,
    mech_eye::MechEyeService* mechEyeService,
    tracking::TrackingService* trackingService,
    QObject* parent)
    : QObject(parent)
    , m_modbus(modbusService)
    , m_mechEye(mechEyeService)
    , m_tracking(trackingService)
    , m_pollTimer(new QTimer(this))
    , m_heartbeatTimer(new QTimer(this))
    , m_timeoutTimer(new QTimer(this))
    , m_state(AppState::Init)
{
    // 从配置管理器获取流程控制配置，如果配置不存在则使用默认值
    const auto* configMgr = common::ConfigManager::instance();
    const auto flowConfig = configMgr ? configMgr->flowControlConfig()
                                      : common::FlowControlConfig{100, 1000, 300};

    // 配置定时器间隔
    m_pollTimer->setInterval(flowConfig.pollIntervalMs);      // PLC 轮询间隔
    m_heartbeatTimer->setInterval(flowConfig.heartbeatIntervalMs);  // 心跳发送间隔
    m_timeoutTimer->setSingleShot(true);  // 超时定时器为单次触发

    // 连接定时器信号到对应的槽函数
    connect(m_pollTimer, &QTimer::timeout, this, &StateMachine::pollPlcState);
    connect(m_heartbeatTimer, &QTimer::timeout, this, &StateMachine::publishHeartbeat);
    connect(m_timeoutTimer, &QTimer::timeout, this, &StateMachine::onProcessTimeout);

    // 如果 Modbus 服务可用，连接其信号
    if (m_modbus) {
        connect(m_modbus, &modbus::ModbusService::connected, this, &StateMachine::onModbusConnected);
        connect(m_modbus, &modbus::ModbusService::disconnected, this, &StateMachine::onModbusDisconnected);
        connect(m_modbus, &modbus::ModbusService::errorOccurred, this, &StateMachine::onModbusError);
        connect(m_modbus, &modbus::ModbusService::registersRead, this, &StateMachine::handleRegistersRead);
        connect(m_modbus, &modbus::ModbusService::registerReadFailed, this, &StateMachine::onRegisterReadFailed);
        // P1修复：连接写失败信号，便于追踪具体哪个寄存器写入失败
        connect(m_modbus, &modbus::ModbusService::registerWriteFailed, this, &StateMachine::onRegisterWriteFailed);
    }

    // 如果 Mech-Eye 相机服务可用，连接其信号
    if (m_mechEye) {
        connect(
            m_mechEye,
            &mech_eye::MechEyeService::stateChanged,
            this,
            [](mech_eye::CameraRuntimeState state, QString desc) {
                qInfo(LOG_FLOW) << "[MechEye] 相机状态变更:" << static_cast<int>(state) << desc;
            });
        connect(
            m_mechEye,
            &mech_eye::MechEyeService::captureFinished,
            this,
            &StateMachine::onCaptureFinished,
            Qt::QueuedConnection);
        connect(
            m_mechEye,
            &mech_eye::MechEyeService::fatalError,
            this,
            &StateMachine::onMechEyeFatalError,
            Qt::QueuedConnection);
    }
}

/**
 * @brief 析构函数，停止所有定时器并清理资源
 */
StateMachine::~StateMachine()
{
    stop();
}

/**
 * @brief 启动状态机
 * 
 * 重置所有状态变量，发布初始 IPC 状态，如果 Modbus 已连接则直接进入就绪状态。
 * 此方法通常在系统初始化或从错误恢复后调用。
 */
void StateMachine::start()
{
    qInfo(LOG_FLOW) << "Starting state machine.";
    clearActiveTask();           // 清除当前活动任务
    resetPointCloudCache();      // 清空点云缓存
    m_isPollingPlc = false;      // 重置 PLC 轮询标志
    m_ipcState = protocol::IpcState::Initializing;  // 设置 IPC 状态为初始化中
    m_currentStage = protocol::Stage::Idle;         // 设置当前阶段为空闲
    m_alarmLevel = 0;            // 清除报警级别
    m_alarmCode = 0;             // 清除报警代码
    m_warnCode = 0;              // 清除警告代码
    m_progress = 0;              // 重置进度
    m_dataValid = false;         // 标记数据无效
    m_consecutiveModbusFailures = 0;  // 重置 Modbus 失败计数器
    setState(AppState::Init);    // 设置应用状态为初始化
    publishIpcStatus();          // 发布 IPC 状态到 PLC

    // 如果 Modbus 已经连接，直接触发连接成功处理
    if (m_modbus && m_modbus->isConnected()) {
        onModbusConnected();
    }
}

/**
 * @brief 停止状态机
 * 
 * 停止所有定时器，清除活动任务，重置所有状态变量。
 * 此方法通常在系统关闭或需要完全重置时调用。
 */
void StateMachine::stop()
{
    m_pollTimer->stop();         // 停止 PLC 轮询定时器
    m_heartbeatTimer->stop();    // 停止心跳定时器
    m_timeoutTimer->stop();      // 停止超时定时器
    m_isPollingPlc = false;      // 重置 PLC 轮询标志
    clearActiveTask();           // 清除当前活动任务
    resetPointCloudCache();      // 清空点云缓存
    m_consecutiveModbusFailures = 0;  // 重置 Modbus 失败计数器
    m_ipcState = protocol::IpcState::Uninitialized;  // 设置 IPC 状态为未初始化
    m_currentStage = protocol::Stage::Idle;         // 设置当前阶段为空闲
    publishIpcStatus();          // 发布 IPC 状态到 PLC
    setState(AppState::Init);    // 设置应用状态为初始化
}

/**
 * @brief 设置应用状态并发出状态变更信号
 * 
 * @param newState 新的应用状态
 */
void StateMachine::setState(AppState newState)
{
    if (m_state != newState) {
        m_state = newState;
        emit stateChanged(newState);  // 发出状态变更信号，通知外部监听者
        qInfo(LOG_FLOW) << "State transitioned to:" << static_cast<int>(newState);
    }
}

/**
 * @brief Modbus 连接成功时的回调处理
 * 
 * 重置失败计数器，设置 IPC 状态为就绪，启动定时器和心跳。
 */
void StateMachine::onModbusConnected()
{
    qInfo(LOG_FLOW) << "Modbus connected. Flow control ready.";
    
    // P2改进：重连后清理可能的残留状态，确保系统处于干净的初始状态
    if (m_activeTask.definition != nullptr) {
        qWarning(LOG_FLOW).noquote()
            << "Clearing stale active task after Modbus reconnect:"
            << protocol::triggerName(*m_activeTask.definition);
        clearActiveTask();
        resetPointCloudCache();
    }
    
    m_isPollingPlc = false;           // 重置 PLC 轮询标志
    resetModbusFailureCounter();      // 重置 Modbus 失败计数器
    m_consecutiveModbusFailures = 0;  // 显式清零失败计数器
    m_ipcState = protocol::IpcState::Ready;   // 设置 IPC 状态为就绪
    m_currentStage = protocol::Stage::Idle;   // 设置当前阶段为空闲
    m_alarmLevel = 0;                 // 清除报警级别
    m_alarmCode = 0;                  // 清除报警代码
    m_warnCode = 0;                   // 清除警告代码
    m_progress = 0;                   // 重置进度
    m_dataValid = false;              // 标记数据无效
    setState(AppState::Ready);        // 设置应用状态为就绪
    publishIpcStatus();               // 发布 IPC 状态到 PLC
    publishHeartbeat();               // 立即发送一次心跳
    m_pollTimer->start();             // 启动 PLC 轮询定时器
    m_heartbeatTimer->start();        // 启动心跳定时器
    
    qInfo(LOG_FLOW) << "Modbus reconnection recovery completed, system restored to clean Ready state.";
}

/**
 * @brief Modbus 断开连接时的回调处理
 * 
 * 停止所有定时器，进入故障状态，等待重新连接。
 */
void StateMachine::onModbusDisconnected()
{
    qWarning(LOG_FLOW) << "Modbus disconnected. Flow control paused.";
    m_pollTimer->stop();       // 停止 PLC 轮询
    m_heartbeatTimer->stop();  // 停止心跳
    m_timeoutTimer->stop();    // 停止超时定时器
    m_isPollingPlc = false;    // 重置 PLC 轮询标志
    // 进入故障状态，报警代码 900，不中止当前任务（因为没有活跃任务），不通知 PLC
    enterFaultState(900, QStringLiteral("Modbus disconnected"), true, false);
}

/**
 * @brief Modbus 发生错误时的回调处理
 * 
 * 记录错误并增加失败计数器，连续失败达到阈值时将进入故障状态。
 * 
 * @param errorString 错误描述信息
 */
void StateMachine::onModbusError(const QString& errorString)
{
    qWarning(LOG_FLOW).noquote() << "Modbus error propagated to flow control:" << errorString;
    recordModbusFailure(901, errorString);  // 记录 Modbus 失败，报警代码 901
}

/**
 * @brief 轮询 PLC 状态
 * 
 * 定期从 PLC 读取命令块寄存器，检测是否有新的触发请求。
 * 如果上一次读取尚未完成，则跳过本次轮询以避免请求堆积。
 */
void StateMachine::pollPlcState()
{
    if (!m_modbus || !m_modbus->isConnected()) {
        return;  // Modbus 未连接，直接返回
    }

    if (m_isPollingPlc) {
        qWarning(LOG_FLOW) << "Skipping PLC poll because previous read is still pending.";
        return;  // 上次读取仍在进行中，跳过本次轮询
    }

    m_isPollingPlc = true;  // 标记正在轮询
    // 异步读取命令块寄存器（从 kCommandBlockStart 开始，共 kCommandBlockSize 个寄存器）
    const bool readStarted = m_modbus->readRegisters(protocol::registers::kCommandBlockStart, protocol::registers::kCommandBlockSize);
    if (!readStarted) {
        qWarning(LOG_FLOW).noquote() << "Failed to start PLC polling read request";
        m_isPollingPlc = false;  // 重置轮询标志，允许下次重试
    }
}

/**
 * @brief 处理从 PLC 读取的寄存器数据
 * 
 * 解析命令块，检测触发信号，处理任务完成确认等。
 * 
 * @param startAddress 起始寄存器地址
 * @param values 读取到的寄存器值向量
 */
void StateMachine::handleRegistersRead(int startAddress, const QVector<quint16>& values)
{
    // 如果是命令块的读取完成，重置轮询标志
    if (startAddress == protocol::registers::kCommandBlockStart) {
        m_isPollingPlc = false;
    }

    // 验证是否是预期的命令块读取，且数据长度足够
    if (startAddress != protocol::registers::kCommandBlockStart ||
        values.size() < protocol::registers::kCommandBlockSize) {
        return;
    }

    m_lastCommandBlock = values;       // 保存最新的命令块数据
    resetModbusFailureCounter();       // 通信成功，重置失败计数器

    // 如果当前有活动任务且已完成宣告，检查 PLC 是否已释放触发信号
    if (m_activeTask.definition != nullptr && m_activeTask.completionAnnounced) {
        finalizeCompletedTaskIfTriggerReleased(values);
        return;
    }

    // 如果有活动任务但未完成宣告，等待任务执行完毕
    if (m_activeTask.definition != nullptr) {
        return;
    }

    // 遍历所有触发定义，查找哪个触发位被置为 1
    for (const auto& trigger : protocol::triggerDefinitions()) {
        if (trigger.trigOffset < values.size() && values[trigger.trigOffset] == 1) {
            processTrigger(trigger, values);  // 处理找到的触发
            break;  // 一次只处理一个触发
        }
    }
}

/**
 * @brief 处理寄存器读取失败的回调
 * 
 * 记录错误并重置轮询标志，允许下一次轮询继续进行。
 * 
 * @param startAddress 读取失败的起始地址
 * @param errorString 错误描述信息
 */
void StateMachine::onRegisterReadFailed(int startAddress, const QString& errorString)
{
    // 只关心命令块的读取失败
    if (startAddress != protocol::registers::kCommandBlockStart) {
        return;
    }

    if (m_isPollingPlc) {
        qWarning(LOG_FLOW).noquote() << "PLC poll request failed:" << errorString;
    }
    m_isPollingPlc = false;  // 重置轮询标志，允许下次轮询
}

/**
 * @brief 处理寄存器写入失败事件
 * 
 * P1修复：当 Modbus 写操作失败时记录详细日志，便于追踪具体哪个寄存器写入失败。
 * 此槽函数连接到 ModbusService 的 registerWriteFailed 信号。
 * 
 * @param startAddress 写入失败的起始地址
 * @param errorString 错误描述信息
 */
void StateMachine::onRegisterWriteFailed(int startAddress, const QString& errorString)
{
    qWarning(LOG_FLOW).noquote()
        << "Register write failed at address" << startAddress
        << "(0x" << QString::number(startAddress, 16) << ")"
        << ":" << errorString;
    
    // 可以根据地址范围判断是哪个业务的写入失败，采取不同的恢复策略
    // 例如：如果是结果区写入失败，可能需要重新发送结果
}

/**
 * @brief 处理 PLC 触发信号
 * 
 * 当检测到某个触发位被置为 1 时，验证触发条件，初始化任务状态，
 * 发送 ACK 响应，启动超时定时器，并执行对应的任务逻辑。
 * 
 * @param trigger 触发定义结构，包含触发偏移、阶段、超时等信息
 * @param commandBlock 完整的命令块寄存器数据
 */
void StateMachine::processTrigger(const protocol::TriggerDefinition& trigger, const QVector<quint16>& commandBlock)
{
    if (!m_modbus || !m_modbus->isConnected()) {
        return;  // Modbus 未连接，无法处理触发
    }

    // 除了卸载计算和结果复位外，其他触发都需要 Flow_Enable=1 才能执行
    if (trigger.stage != protocol::Stage::UnloadCalc &&
        trigger.stage != protocol::Stage::ResultReset &&
        commandBlock.value(protocol::registers::kFlowEnable) == 0) {
        qWarning(LOG_FLOW).noquote() << "Rejecting trigger while Flow_Enable=0:"
                                     << protocol::triggerName(trigger);
        sendRes(trigger, 9);                          // 返回错误码 9（参数错误）
        sendAck(trigger, protocol::AckState::Failed); // 发送失败 ACK
        return;
    }

    // 初始化活动任务状态
    m_activeTask.definition = &trigger;                                    // 保存触发定义指针
    m_activeTask.taskId = readTaskId(commandBlock);                        // 读取任务 ID
    // 如果 PLC 指定了超时时间则使用，否则使用触发定义的默认超时
    m_activeTask.timeoutSeconds =
        commandBlock.value(protocol::registers::kRequestTimeoutSeconds) > 0
        ? commandBlock.value(protocol::registers::kRequestTimeoutSeconds)
        : static_cast<quint16>(trigger.defaultTimeoutSeconds);
    m_activeTask.scanSegmentIndex = resolveScanSegmentIndex(commandBlock); // 解析扫描分段索引
    m_activeTask.scanSegmentTotal = commandBlock.value(protocol::registers::kScanSegmentTotal); // 总分段数
    m_activeTask.completionAnnounced = false;  // 重置完成宣告标志
    m_activeTask.captureRequestId = 0;         // 重置采集请求 ID

    qInfo(LOG_FLOW).noquote()
        << "Accepted trigger" << protocol::triggerName(trigger)
        << "timeout=" << m_activeTask.timeoutSeconds << "s"
        << "segmentIndex=" << m_activeTask.scanSegmentIndex;

    // 清除之前的报警信息
    setAlarm(0, 0, QString());
    // 切换到扫描状态
    setState(AppState::Scanning);
    m_ipcState = protocol::IpcState::Busy;   // IPC 状态设为忙碌
    m_currentStage = trigger.stage;          // 设置当前阶段
    m_progress = 5;                          // 进度设为 5%（刚开始）
    m_dataValid = false;                     // 数据无效
    publishIpcStatus();                      // 发布 IPC 状态到 PLC

    sendAck(trigger, protocol::AckState::Running);  // 发送运行中 ACK

    // 如果任务 ID 不为 0，回写到 PLC 的任务 ID 回声寄存器
    if (m_activeTask.taskId != 0) {
        const bool taskIdWritten = m_modbus->writeRegisters(protocol::registers::kTaskIdEchoHigh, {
            static_cast<quint16>((m_activeTask.taskId >> 16) & 0xFFFFu),
            static_cast<quint16>(m_activeTask.taskId & 0xFFFFu),
        });
        if (!taskIdWritten) {
            qWarning(LOG_FLOW).noquote() << "Failed to write task ID echo registers";
        }
    }

    // 启动超时定时器（秒转毫秒）
    m_timeoutTimer->start(static_cast<int>(m_activeTask.timeoutSeconds) * 1000);
    // 执行具体的任务逻辑
    executeActiveTask();
}

/**
 * @brief 执行当前活动任务
 * 
 * 根据触发定义的 trigOffset 分发到具体的任务执行函数。
 * 每个 trigOffset 对应一个特定的 PLC 触发信号。
 */
void StateMachine::executeActiveTask()
{
    if (m_activeTask.definition == nullptr) {
        return;  // 没有活动任务，直接返回
    }

    // TODO: [flow_control] 为 Trig_StationMaterialCheck(20)/Trig_PoseCheck(21)/
    // Trig_SelfCheck(25)/Trig_CodeRead(26) 增加专用处理函数。
    // 输入：m_lastCommandBlock、当前触发定义与任务上下文；
    // 输出：对应 Res/Ack 与结果寄存器区（含失败码与报警信息）。

    switch (m_activeTask.definition->trigOffset) {
    case 19:  // Trig_LoadGrasp - 加载抓取任务
        executeLoadGraspTask();
        return;
    case 21:  // Trig_PoseCheck - 位姿检查
        executePoseCheckTask();
        return;
    case 22:  // Trig_ScanSegment - 扫描分段任务
        executeScanSegmentTask();
        return;
    case 23:  // Trig_Inspection - 综合检测任务
        executeInspectionTask();
        return;
    case 24:  // Trig_UnloadCalc - 卸载计算任务
        executeUnloadCalcTask();
        return;
    case 27:  // Trig_ResultReset - 结果复位任务
        executeResultResetTask();
        return;
    default:  // 未知触发类型，使用默认响应码完成任务
        // TODO: [flow_control] default 分支不应长期返回默认成功码。
        // 输入：未覆盖触发的 trigOffset 与命令参数；
        // 输出：协议约定的真实结果码（或明确失败码）并写回对应结果寄存器。
        completeActiveTask(protocol::defaultResCodeFor(*m_activeTask.definition));
        return;
    }
}

/**
 * @brief 执行加载抓取任务（Trig_LoadGrasp）
 * 
 * 向 PLC 写入模拟的加载位姿数据，然后立即完成任务。
 * 这是一个占位实现，实际应用中可能需要调用视觉定位或机器人接口。
 */
void StateMachine::executeLoadGraspTask()
{
    // TODO: [vision/grasp_provider] 接入外部抓取位姿提供模块。
    // 输入：产品/配方/任务上下文（TaskId、RecipeId、ProductType等）；
    // 输出：Load_X~Load_Rz 真实位姿 + 对应 Res/Ack。
    writeLoadGraspResult();   // 写入加载位姿结果到 PLC 寄存器
    completeActiveTask(1);    // 完成任务，返回成功码 1
}

/**
 * @brief 执行卸载计算任务（Trig_UnloadCalc）
 * 
 * 向 PLC 写入模拟的卸料位姿数据，然后立即完成任务。
 * 这是一个占位实现，实际应用中可能需要进行路径规划或碰撞检测。
 */
void StateMachine::executeUnloadCalcTask()
{
    // TODO: [planner/unload_provider] 接入外部卸料位姿计算模块。
    // 输入：当前工件状态、下料区约束与任务上下文；
    // 输出：Unload_X~Unload_Rz 真实位姿 + 对应 Res/Ack。
    writeUnloadCalcResult();  // 写入卸料位姿结果到 PLC 寄存器
    completeActiveTask(1);    // 完成任务，返回成功码 1
}

/**
 * @brief 执行扫描分段任务（Trig_ScanSegment）
 * 
 * 这是整个流程中最复杂的任务，负责控制 Mech-Eye 相机进行 3D 点云采集。
 * 包括参数验证、相机状态检查、异步采集请求发起等步骤。
 * 
 * 关键流程：
 * 1. 验证扫描分段请求的合法性（段号范围、重复检测）
 * 2. 检查相机是否就绪且空闲
 * 3. 发起异步采集请求
 * 4. 等待 onCaptureFinished 回调处理采集结果
 */
void StateMachine::executeScanSegmentTask()
{
    // TODO: [mech_eye] 扩展法向量采集策略（如 capture3DWithNormal / capture2DAnd3DWithNormal）。
    // 输入：segmentIndex/segmentTotal/timeout/cameraKey；
    // 输出：含法向量的点云结果并进入 onCaptureFinished 缓存与回写链路。
    // 注意: 与算法沟通，最好可以转换成PCL数据，mech可调用：ConvertPointCloudWithNormalsToPcl函数。
    // 检查相机服务是否可用
    if (m_mechEye == nullptr) {
        // 没有相机服务时不能进入真实采集，直接按异常握手返回，避免 PLC 等待超时。
        finishScanSegmentFailure(
            5,                    // Res 码：5 = 设备未就绪
            3,                    // 报警级别：3 = 严重错误
            720,                  // 报警代码：720 = 相机服务不可用
            QStringLiteral("MechEye service unavailable"),
            QStringLiteral("MechEye service unavailable"));
        return;
    }

    // 验证扫描分段请求的参数合法性
    QString validationError;
    if (!validateScanSegmentRequest(m_lastCommandBlock, &validationError)) {
        // 段号错误或重复触发会污染分段缓存，因此在拍照前就拒绝本次业务。
        finishScanSegmentFailure(9, 2, 724, validationError, validationError);
        return;
    }

    // 检查相机是否处于就绪状态且当前没有正在进行的采集
    if (m_mechEye->state() != mech_eye::CameraRuntimeState::Ready || m_mechEye->isBusy()) {
        // 相机未就绪或仍在执行上一帧采集时，返回设备未就绪，防止后台请求堆积。
        finishScanSegmentFailure(
            5,                    // Res 码：5 = 设备未就绪
            2,                    // 报警级别：2 = 警告
            721,                  // 报警代码：721 = 相机忙或未就绪
            QStringLiteral("MechEye busy or not ready for capture"),
            QStringLiteral("MechEye busy or not ready for capture"));
        return;
    }

    // 计算采集超时时间：优先使用任务指定的超时，否则使用默认值
    const int captureTimeoutMs = m_activeTask.timeoutSeconds > 0
        ? static_cast<int>(m_activeTask.timeoutSeconds) * 1000
        : kDefaultScanSegmentCaptureTimeoutMs;

    // 向相机服务发起异步采集请求
    const quint64 requestId = m_mechEye->requestCapture(
        QString(),                        // 不使用自定义文件名
        mech_eye::CaptureMode::Capture3DOnly,  // 仅采集 3D 点云
        captureTimeoutMs);                // 超时时间（毫秒）

    if (requestId == 0) {
        // requestCapture 返回 0 表示 facade 拒绝请求，按设备未就绪处理。
        finishScanSegmentFailure(
            5,                    // Res 码：5 = 设备未就绪
            2,                    // 报警级别：2 = 警告
            721,                  // 报警代码：721 = 相机忙或未就绪
            QStringLiteral("MechEye rejected capture request"),
            QStringLiteral("MechEye busy or not ready for capture"));
        return;
    }

    // 保存采集请求 ID，用于在回调中匹配响应
    m_activeTask.captureRequestId = requestId;
    m_progress = 30;              // 更新进度为 30%（采集中）
    publishIpcStatus();           // 发布更新的 IPC 状态

    qInfo(LOG_FLOW).noquote()
        << "Trig_ScanSegment started Mech-Eye capture"
        << "segmentIndex=" << m_activeTask.scanSegmentIndex
        << "segmentTotal=" << m_activeTask.scanSegmentTotal
        << "timeoutMs=" << captureTimeoutMs;
}

/**
 * @brief 执行位姿检查任务（Trig_PoseCheck）
 *
 * 直接调用 LB 位姿算法，输出位姿偏差值并按正式 PLC 结果码回写。
 */
void StateMachine::executePoseCheckTask()
{
    if (m_tracking == nullptr) {
        qWarning(LOG_FLOW).noquote() << "Tracking service unavailable for pose check.";
        writeFloatPlaceholder(protocol::registers::kPoseDeviationMm, 0.0f);
        completeActiveTask(7, protocol::AckState::Failed, false);
        return;
    }

    const tracking::PoseCheckResult poseResult = m_tracking->checkPose();
    writeFloatPlaceholder(
        protocol::registers::kPoseDeviationMm,
        static_cast<float>(poseResult.poseDeviationMm));

    if (!poseResult.invoked) {
        qWarning(LOG_FLOW).noquote()
            << "Pose check did not invoke LB algorithm:"
            << poseResult.message;
        completeActiveTask(7, protocol::AckState::Failed, false);
        return;
    }

    if (!poseResult.success) {
        const quint16 resultCode = poseResult.resultCode == 0 ? 7 : poseResult.resultCode;
        qWarning(LOG_FLOW).noquote()
            << "Pose check failed:"
            << poseResult.message
            << "resultCode=" << resultCode
            << "deviationMm=" << poseResult.poseDeviationMm;
        completeActiveTask(resultCode, protocol::AckState::Failed, false);
        return;
    }

    qInfo(LOG_FLOW).noquote()
        << "Pose check succeeded"
        << "inputPoints=" << poseResult.inputPointCount
        << "deviationMm=" << poseResult.poseDeviationMm
        << "rt00=" << poseResult.rt[0];
    completeActiveTask(1, protocol::AckState::Completed, true);
}

/**
 * @brief 执行综合检测任务（Trig_Inspection）
 * 
 * 调用跟踪检测服务对之前采集的所有分段点云进行综合分析，
 * 计算工件的偏移量和检测结果，并将结果写入 PLC 寄存器。
 * 
 * 关键步骤：
 * 1. 检查跟踪服务是否可用
 * 2. 调用 inspectSegments 进行点云分析
 * 3. 将检测结果写入 PLC
 * 4. 根据检测结果决定任务成功或失败
 * 5. 清空点云缓存（检测完成后不再需要原始点云）
 */
void StateMachine::executeInspectionTask()
{
    // 检查跟踪服务是否可用
    if (m_tracking == nullptr) {
        qWarning(LOG_FLOW) << "Tracking service unavailable for inspection.";
        // 写入默认的检测失败结果
        writeInspectionResult({2, 1u << 4, 0, 0});
        // 完成任务，标记为失败，数据无效
        completeActiveTask(7, protocol::AckState::Failed, false);
        resetPointCloudCache();  // 清空点云缓存
        return;
    }

    // Step 3 正式通过 tracking 模块消费前序缓存的分段点云，避免综合检测继续依赖本地 mock。
    const tracking::InspectionResult trackingResult =
        m_tracking->inspectSegments(m_segmentCaptureResults);

    // 将跟踪服务的检测结果转换为内部摘要结构
    InspectionSummary summary;
    summary.resultCode = trackingResult.resultCode;           // 结果码：1=合格，其他=不合格
    summary.ngReasonWord0 = trackingResult.ngReasonWord0;     // NG 原因字 0
    summary.ngReasonWord1 = trackingResult.ngReasonWord1;     // NG 原因字 1
    summary.measureItemCount = trackingResult.measureItemCount;  // 测量项数量
    summary.offsetXmm = trackingResult.offsetXmm;             // X 方向偏移量（mm）
    summary.offsetYmm = trackingResult.offsetYmm;             // Y 方向偏移量（mm）
    summary.offsetZmm = trackingResult.offsetZmm;             // Z 方向偏移量（mm）

    qInfo(LOG_FLOW).noquote()
        << "Trig_Inspection finished"
        << "segmentCount=" << trackingResult.segmentCount      // 参与检测的分段数量
        << "totalPointCount=" << trackingResult.totalPointCount  // 总点数
        << "offsetXmm=" << trackingResult.offsetXmm
        << "offsetYmm=" << trackingResult.offsetYmm
        << "offsetZmm=" << trackingResult.offsetZmm
        << "message=" << trackingResult.message;               // 检测消息描述

    // 将检测结果写入 PLC 寄存器
    writeInspectionResult(summary);
    // 完成任务：如果 resultCode==1 则数据有效，否则无效
    completeActiveTask(
        summary.resultCode,
        protocol::AckState::Completed,
        summary.resultCode == 1);
    resetPointCloudCache();  // 检测完成，清空点云缓存释放内存
}

/**
 * @brief 执行结果复位任务（Trig_ResultReset）
 * 
 * 清空所有累积的点云缓存和检测结果，将相关寄存器归零，
 * 为下一轮扫描周期做准备。
 */
void StateMachine::executeResultResetTask()
{
    resetPointCloudCache();  // 清空点云缓存
    // 将扫描分段完成索引寄存器清零
    const bool segmentIndexCleared = m_modbus->writeRegisters(protocol::registers::kScanSegmentDoneIndex, {0, 0, 0});
    if (!segmentIndexCleared) {
        qWarning(LOG_FLOW).noquote() << "Failed to clear scan segment done index";
    }
    // 写入空的检测结果（全零）
    writeInspectionResult({});
    // 清除 IPC 安全动作字
    const bool safetyActionCleared = m_modbus->writeRegisters(protocol::registers::kIpcSafetyActionWord, {0});
    if (!safetyActionCleared) {
        qWarning(LOG_FLOW).noquote() << "Failed to clear IPC safety action word";
    }
    // 完成任务，返回成功
    completeActiveTask(1);
}

/**
 * @brief 向 PLC 发送 ACK（应答）信号
 * 
 * 将当前的应答状态写入触发定义中指定的 ACK 寄存器地址。
 * ACK 状态包括：Idle(0)、Running(1)、Completed(2)、Failed(3)。
 * 
 * @param definition 触发定义，包含 ACK 寄存器的偏移地址
 * @param ackState 要写入的应答状态
 */
void StateMachine::sendAck(const protocol::TriggerDefinition& definition, protocol::AckState ackState)
{
    if (!m_modbus) {
        return;  // Modbus 不可用，无法发送
    }

    qDebug(LOG_FLOW).noquote() << "Writing Ack" << static_cast<int>(ackState)
                               << "to" << definition.ackOffset
                               << "for" << protocol::triggerName(definition);
    const bool ackWritten = m_modbus->writeRegister(definition.ackOffset, static_cast<quint16>(ackState));
    if (!ackWritten) {
        qWarning(LOG_FLOW).noquote() << "Failed to write Ack state";
    }
}

/**
 * @brief 向 PLC 发送 Res（结果）信号
 * 
 * 将任务执行的结果码写入触发定义中指定的 Res 寄存器地址。
 * 常见的结果码：1=成功，5=设备未就绪，6=超时，7=处理失败，9=参数错误。
 * 
 * @param definition 触发定义，包含 Res 寄存器的偏移地址
 * @param resultCode 要写入的结果码
 */
void StateMachine::sendRes(const protocol::TriggerDefinition& definition, quint16 resultCode)
{
    if (!m_modbus) {
        return;  // Modbus 不可用，无法发送
    }

    qDebug(LOG_FLOW).noquote() << "Writing Res" << resultCode
                               << "to" << definition.resOffset
                               << "for" << protocol::triggerName(definition);
    const bool resWritten = m_modbus->writeRegister(definition.resOffset, resultCode);
    if (!resWritten) {
        qWarning(LOG_FLOW).noquote() << "Failed to write Res code";
    }
}

/**
 * @brief 发布 IPC 状态到 PLC
 * 
 * 将当前的 IPC 运行状态、报警信息、进度等写入一组连续的 Modbus 寄存器，
 * 供 PLC 实时监控 IPC 的运行情况。
 * 
 * 写入的寄存器包括：
 * - 心跳计数器
 * - IPC 状态（Ready/Busy/Fault）
 * - 当前阶段（Idle/ScanSegment/Inspection 等）
 * - 报警级别和代码
 * - 警告代码
 * - 系统就绪标志
 * - 数据有效标志
 * - 任务进度百分比
 * - 设备在线状态字
 * - 当前任务 ID（高16位和低16位）
 */
void StateMachine::publishIpcStatus()
{
    if (!m_modbus || !m_modbus->isConnected()) {
        return;  // Modbus 未连接，无法发布状态
    }

    QVector<quint16> status = {
        m_heartbeatCounter,                                          // 0: 心跳计数器
        static_cast<quint16>(m_ipcState),                            // 1: IPC 状态
        static_cast<quint16>(m_currentStage),                        // 2: 当前阶段
        m_alarmLevel,                                                // 3: 报警级别
        m_alarmCode,                                                 // 4: 报警代码
        m_warnCode,                                                  // 5: 警告代码
        static_cast<quint16>(m_state == AppState::Ready ? 1 : 0),   // 6: 系统就绪标志
        static_cast<quint16>(m_dataValid ? 1 : 0),                  // 7: 数据有效标志
        m_progress,                                                  // 8: 任务进度（0-100）
        kDeviceOnlineWord0,                                          // 9: 设备在线状态字
        0,                                                           // 10: 保留
        0,                                                           // 11: 保留
        0,                                                           // 12: 保留
        static_cast<quint16>((m_activeTask.taskId >> 16) & 0xFFFFu), // 13: 任务 ID 高16位
        static_cast<quint16>(m_activeTask.taskId & 0xFFFFu),         // 14: 任务 ID 低16位
    };

    // 批量写入状态寄存器（从 kIpcHeartbeat 开始）
    const bool heartbeatWritten = m_modbus->writeRegisters(protocol::registers::kIpcHeartbeat, status);
    if (!heartbeatWritten) {
        qWarning(LOG_FLOW).noquote() << "Failed to write IPC heartbeat status";
    }
}

/**
 * @brief 发布心跳信号
 * 
 * 递增心跳计数器并发布 IPC 状态，用于向 PLC 证明 IPC 仍在正常运行。
 * PLC 可以通过监控心跳计数器的变化来判断 IPC 是否死机或通信中断。
 */
void StateMachine::publishHeartbeat()
{
    if (!m_modbus || !m_modbus->isConnected()) {
        return;  // Modbus 未连接，无法发送心跳
    }

    ++m_heartbeatCounter;  // 递增心跳计数器
    publishIpcStatus();    // 发布包含新心跳计数的状态
}

/**
 * @brief 处理任务超时事件
 * 
 * 当任务执行时间超过设定的超时时间时触发此回调。
 * 设置超时报警，并根据任务类型采取不同的处理策略：
 * - 扫描分段任务：写入失败结果并标记为失败
 * - 其他任务：直接标记为完成但数据无效
 */
void StateMachine::onProcessTimeout()
{
    if (m_activeTask.definition == nullptr) {
        return;  // 没有活动任务，忽略超时
    }

    qWarning(LOG_FLOW).noquote() << "Task timed out:" << protocol::triggerName(*m_activeTask.definition);
    setAlarm(2, 610, QStringLiteral("Task timeout"));  // 设置警告级别报警，代码 610
    m_activeTask.captureRequestId = 0;  // 清除采集请求 ID

    // P0修复：超时时清理已缓存的点云数据，防止内存泄漏
    if (m_activeTask.definition->stage == protocol::Stage::ScanSegment) {
        qWarning(LOG_FLOW) << "Clearing point cloud cache due to task timeout";
        resetPointCloudCache();
    }

    // 根据任务类型采取不同的超时处理策略
    if (m_activeTask.definition->stage == protocol::Stage::ScanSegment) {
        // 扫描分段超时：写入空结果（0 图像数，0 点云帧数）
        writeScanSegmentResult(m_activeTask.scanSegmentIndex, 0, 0);
        completeActiveTask(6, protocol::AckState::Failed, false);  // Res=6 表示超时
        return;
    }
    // 其他任务超时：直接完成，标记为失败
    completeActiveTask(6, protocol::AckState::Completed, false);
}

/**
 * @brief 处理相机采集完成的回调
 * 
 * 当 Mech-Eye 相机完成一次 3D 点云采集后，通过此回调接收结果。
 * 这是扫描分段任务的核心处理逻辑。
 * 
 * 处理流程：
 * 1. 验证回调是否对应当前的活动任务和请求 ID
 * 2. 检查采集是否成功以及点云数据是否有效
 * 3. 如果成功，将点云结果存入缓存，更新进度，完成任务
 * 4. 如果失败，记录错误信息并完成失败的任务
 * 
 * @param result 采集结果，包含点云数据、状态码、错误信息等
 */
void StateMachine::onCaptureFinished(mech_eye::CaptureResult result)
{
    // 只在扫描分段阶段且存在活动任务时才处理
    if (m_activeTask.definition == nullptr ||
        m_activeTask.definition->stage != protocol::Stage::ScanSegment) {
        return;
    }

    // 过滤掉过期的回调（任务已完成或请求 ID 不匹配）
    if (m_activeTask.completionAnnounced || result.requestId != m_activeTask.captureRequestId) {
        qInfo(LOG_FLOW).noquote()
            << "Ignoring stale capture callback for segmentIndex=" << m_activeTask.scanSegmentIndex;
        return;
    }

    // 检查采集是否成功、点云数据是否有效，以及法向量是否齐全
    if (!result.success() || !result.pointCloud.isValid() || !result.pointCloud.hasNormals()) {
        const QString failureMessage =
            result.errorMessage.isEmpty() ? QStringLiteral("Capture failed") : result.errorMessage;
        finishScanSegmentFailure(
            mapCaptureErrorToResCode(result.errorCode),  // 映射错误码到 Res 码
            2,                                           // 报警级别：2 = 警告
            722,                                         // 报警代码：722 = 采集失败
            QStringLiteral("Trig_ScanSegment capture failed: %1").arg(failureMessage),
            failureMessage);
        return;
    }

    // CaptureResult 内部点云使用 shared_ptr 承载大数组，QMap 只保存结果对象和共享指针，
    // 不会在主线程里深拷贝整块点云数据。
    // 将成功的采集结果按段索引存入缓存，供后续综合检测使用
    m_segmentCaptureResults.insert(m_activeTask.scanSegmentIndex, result);
    m_progress = 100;  // 更新进度为 100%
    // 向 PLC 写入扫描分段结果：段索引、图像数=1、点云帧数=1
    writeScanSegmentResult(m_activeTask.scanSegmentIndex, 1, 1);
    // 完成任务，标记为成功且数据有效
    completeActiveTask(1, protocol::AckState::Completed, true);

    qInfo(LOG_FLOW).noquote()
        << "Trig_ScanSegment capture saved"
        << "segmentIndex=" << m_activeTask.scanSegmentIndex
        << "cacheSize=" << m_segmentCaptureResults.size()   // 当前缓存中的分段数量
        << "pointCount=" << result.pointCloud.pointCount     // 本次采集的点数
        << "normalCount=" << result.pointCloud.normalCount()  // 本次采集法向量数量
        << "elapsedMs=" << result.elapsedMs;                 // 采集耗时（毫秒）
}

/**
 * @brief 处理 Mech-Eye 相机的致命错误
 * 
 * 当相机发生无法恢复的错误（如硬件故障、驱动崩溃等）时触发此回调。
 * 如果当前正在执行扫描分段任务，立即终止任务并进入故障状态。
 * 
 * @param code 错误代码
 * @param message 错误描述信息
 */
void StateMachine::onMechEyeFatalError(mech_eye::CaptureErrorCode code, QString message)
{
    qCritical(LOG_FLOW) << "[MechEye] 致命错误:" << message;
    emit protocolEvent(QStringLiteral("Mech-Eye: %1").arg(message));  // 发出协议事件通知

    // 只在扫描分段阶段且任务未完成时才处理
    if (m_activeTask.definition == nullptr ||
        m_activeTask.definition->stage != protocol::Stage::ScanSegment ||
        m_activeTask.completionAnnounced) {
        return;
    }

    // P0修复：相机致命错误时清理已缓存的点云数据，防止内存泄漏
    qWarning(LOG_FLOW) << "Clearing point cloud cache due to Mech-Eye fatal error";
    resetPointCloudCache();

    // 相机在扫描中途发生致命错误时，需要第一时间拉高报警并强制结束当前扫描触发。
    finishScanSegmentFailure(
        mapCaptureErrorToResCode(code),  // 映射错误码到 Res 码
        3,                               // 报警级别：3 = 严重错误
        723,                             // 报警代码：723 = 相机致命错误
        QStringLiteral("Mech-Eye fatal error during scan"),
        message);
}

/**
 * @brief 完成当前活动任务
 * 
 * 这是任务完成的统一出口，负责：
 * 1. 停止超时定时器
 * 2. 更新进度和数据有效性标志
 * 3. 向 PLC 发送 Res（结果码）和 ACK（应答状态）
 * 4. 标记任务已完成宣告
 * 5. 发布更新的 IPC 状态
 * 
 * @param resultCode 任务结果码（1=成功，其他=各种失败原因）
 * @param finalAckState 最终的 ACK 状态（Completed 或 Failed）
 * @param dataValid 数据是否有效（true 表示检测结果可用）
 */
/**
 * @brief 完成活动任务并发送结果和确认
 * 
 * P1修复：使用批量写入 Res 和 Ack，确保原子性，避免 PLC 读到中间状态。
 * 由于所有触发器的 Ack 和 Res 地址都是连续的（Ack 在前，Res 在后），
 * 可以使用一次 writeRegisters 调用同时写入两个寄存器。
 * 
 * @param resultCode 结果代码
 * @param finalAckState 最终 ACK 状态（Completed=2 或 Failed=3）
 * @param dataValid 数据是否有效标志
 */
bool StateMachine::completeActiveTask(
    quint16 resultCode,
    protocol::AckState finalAckState,
    bool dataValid)
{
    if (m_activeTask.definition == nullptr || !m_modbus) {
        qWarning(LOG_FLOW).noquote() << "Cannot complete task: definition or modbus is null";
        return false;  // 没有活动任务或 Modbus 不可用，无法完成
    }

    // P3改进：封装带重试的 Modbus 写入 lambda
    auto executeWithRetry = [this](auto&& writeOperation, const QString& operationName) -> bool {
        constexpr int kMaxRetries = 3;
        constexpr int kRetryIntervalMs = 100;
        
        for (int attempt = 1; attempt <= kMaxRetries; ++attempt) {
            // P3改进：每次重试前检查必要前提条件
            if (!m_modbus) {
                qWarning(LOG_FLOW).noquote()
                    << operationName << "failed: Modbus became null at attempt" << attempt;
                return false;
            }
            
            if (m_activeTask.definition == nullptr) {
                qWarning(LOG_FLOW).noquote()
                    << operationName << "failed: Task definition became null at attempt" << attempt;
                return false;
            }
            
            // 执行写入操作
            writeOperation();
            
            // P3改进：如果是最后一次尝试，直接返回成功（已发起写入请求）
            if (attempt == kMaxRetries) {
                return true;
            }
            
            // P3改进：非最后一次尝试，等待后继续重试
            QThread::msleep(kRetryIntervalMs);
        }
        
        return true;  // 理论上不会到达这里
    };
    
    // P1修复：批量写入 Res 和 Ack，保证原子性
    // P2改进：虽然批量写入是原子的，但为了确保语义清晰，我们明确标注写入顺序
    // 在 Modbus 批量写入中，所有寄存器在同一事务中原子更新，PLC 会同时看到新值
    // 地址布局：ackOffset (低位) -> resOffset (高位)，两者必须连续
    const int ackOffset = m_activeTask.definition->ackOffset;
    const int resOffset = m_activeTask.definition->resOffset;
    
    bool writeSuccess = false;
    auto failCompletionWrite = [this](const QString& reason) -> bool {
        qWarning(LOG_FLOW).noquote() << reason;
        enterFaultState(902, reason, false, false);
        return false;
    };
    
    // 验证地址连续性（防御性编程）
    if (resOffset == ackOffset + 1) {
        // 地址连续，使用批量写入
        QVector<quint16> batchValues = {
            static_cast<quint16>(finalAckState),  // Ack 值
            resultCode                             // Res 值
        };
        
        // P3改进：带重试的批量写入
        writeSuccess = executeWithRetry(
            [&]() {
                m_modbus->writeRegisters(ackOffset, batchValues);
            },
            QStringLiteral("Batch writeRegisters"));
        
        if (!writeSuccess) {
            return failCompletionWrite(QStringLiteral(
                "Failed to complete batch write after retries: Ack=%1 Res=%2 addresses %3-%4")
                .arg(static_cast<int>(finalAckState))
                .arg(resultCode)
                .arg(ackOffset)
                .arg(resOffset));
        }
        
        qInfo(LOG_FLOW).noquote()
            << "Atomic batch wrote Ack=" << static_cast<int>(finalAckState)
            << "and Res=" << resultCode
            << "to addresses" << ackOffset << "-" << resOffset;
    } else {
        // 地址不连续（异常情况），降级为单独写入
        // P2改进：确保先写 Res 再写 Ack，避免 PLC 读到中间状态
        qWarning(LOG_FLOW).noquote()
            << "Ack and Res addresses are not consecutive:"
            << "ackOffset=" << ackOffset << "resOffset=" << resOffset
            << ". Falling back to individual writes with retry mechanism.";
        
        // P3改进：先写 Res（结果数据），带重试
        bool resSuccess = executeWithRetry(
            [&]() {
                sendRes(*m_activeTask.definition, resultCode);
            },
            QStringLiteral("sendRes"));
        
        if (!resSuccess) {
            return failCompletionWrite(QStringLiteral(
                "Failed to send Res after retries: resultCode=%1")
                .arg(resultCode));
        }
        
        // P3改进：再写 Ack（完成标志），带重试
        bool ackSuccess = executeWithRetry(
            [&]() {
                sendAck(*m_activeTask.definition, finalAckState);
            },
            QStringLiteral("sendAck"));
        
        if (!ackSuccess) {
            return failCompletionWrite(QStringLiteral(
                "Failed to send Ack after retries: ackState=%1")
                .arg(static_cast<int>(finalAckState)));
        }
    }
    
    // P3改进：只有 Modbus 操作成功发起后，才更新状态
    m_timeoutTimer->stop();   // 停止超时定时器
    m_progress = 100;         // 进度设为 100%
    m_dataValid = dataValid;  // 设置数据有效性标志
    m_activeTask.completionAnnounced = true;                    // 标记已完成宣告
    m_activeTask.captureRequestId = 0;                          // 清除采集请求 ID
    publishIpcStatus();                                         // 发布更新的 IPC 状态

    qInfo(LOG_FLOW).noquote()
        << "Completed trigger" << protocol::triggerName(*m_activeTask.definition)
        << "with Res=" << resultCode
        << "Ack=" << static_cast<int>(finalAckState);
    
    return true;
}

/**
 * @brief 在 PLC 释放触发信号后 finalize 已完成的任务
 * 
 * 当任务完成后，PLC 需要将对应的 Trig 位清零以确认收到结果。
 * 此方法检测 Trig 位是否已清零，如果是则发送 Idle ACK 并清除活动任务状态，
 * 使系统回到就绪状态等待下一个触发。
 * 
 * @param commandBlock 最新的命令块寄存器数据
 */
void StateMachine::finalizeCompletedTaskIfTriggerReleased(const QVector<quint16>& commandBlock)
{
    if (m_activeTask.definition == nullptr || !m_activeTask.completionAnnounced) {
        return;  // 没有活动任务或任务未完成宣告，无需处理
    }

    const int trigOffset = m_activeTask.definition->trigOffset;
    // 检查 Trig 位是否已清零（PLC 确认收到结果）
    if (trigOffset >= commandBlock.size() || commandBlock[trigOffset] != 0) {
        return;  // Trig 位仍为 1，PLC 尚未释放
    }

    qInfo(LOG_FLOW).noquote() << "PLC released trigger for"
                              << protocol::triggerName(*m_activeTask.definition);
    sendAck(*m_activeTask.definition, protocol::AckState::Idle);  // 发送 Idle ACK
    clearActiveTask();                                            // 清除活动任务
    m_ipcState = protocol::IpcState::Ready;                       // IPC 状态回到就绪
    m_currentStage = protocol::Stage::Idle;                       // 当前阶段回到空闲
    m_progress = 0;                                               // 进度归零
    setState(AppState::Ready);                                    // 应用状态回到就绪
    publishIpcStatus();                                           // 发布更新的 IPC 状态
}

/**
 * @brief 清除当前活动任务的所有状态
 * 
 * 将活动任务结构体重置为默认值，释放所有相关资源。
 */
void StateMachine::clearActiveTask()
{
    m_activeTask = {};  // 重置为默认构造的空任务
}

/**
 * @brief 设置报警信息
 * 
 * 更新报警级别、报警代码和警告代码，并发出协议事件通知。
 * 
 * @param level 报警级别（0=无报警，1=提示，2=警告，3=严重错误）
 * @param code 报警代码
 * @param message 报警描述信息
 */
void StateMachine::setAlarm(quint16 level, quint16 code, const QString& message)
{
    m_alarmLevel = level;
    m_alarmCode = code;
    // 警告代码仅在报警级别为 1 或 2 时有效
    m_warnCode = level > 0 && level < 3 ? code : 0;
    if (!message.isEmpty()) {
        emit protocolEvent(message);  // 发出协议事件通知外部监听者
    }
}

/**
 * @brief 向 PLC 写入浮点数占位符（CDAB 字节序）
 * 
 * 将单个 float 值转换为两个 16 位寄存器并写入指定的起始地址。
 * 用于向 PLC 传递坐标、角度等浮点数据。
 * 
 * @param startOffset 起始寄存器偏移地址
 * @param value 要写入的浮点数值
 */
void StateMachine::writeFloatPlaceholder(int startOffset, float value)
{
    if (!m_modbus) {
        return;  // Modbus 不可用，无法写入
    }

    const bool floatWritten = m_modbus->writeRegisters(startOffset, floatToCdabRegisters(value));
    if (!floatWritten) {
        qWarning(LOG_FLOW).noquote() << "Failed to write float placeholder at offset" << startOffset;
    }
}

/**
 * @brief 向 PLC 写入 ASCII 字符串占位符
 * 
 * 将字符串按每两个字符打包为一个 16 位寄存器的方式写入 PLC。
 * 如果字符串长度不足，用空格填充；如果超长，则截断。
 * 
 * @param startOffset 起始寄存器偏移地址
 * @param registerCount 要写入的寄存器数量
 * @param text 要写入的文本字符串
 */
void StateMachine::writeAsciiPlaceholder(int startOffset, int registerCount, const QString& text)
{
    if (!m_modbus) {
        return;  // Modbus 不可用，无法写入
    }

    // 限制字符串长度并用空格右对齐填充
    const QString padded = text.left(registerCount * 2).leftJustified(registerCount * 2, QLatin1Char(' '));
    QVector<quint16> values;
    values.reserve(registerCount);
    for (int i = 0; i < registerCount; ++i) {
        // 每两个字符打包为一个 16 位寄存器：高8位为第一个字符，低8位为第二个字符
        const QChar first = padded.at(i * 2);
        const QChar second = padded.at(i * 2 + 1);
        const quint16 packed = (static_cast<quint16>(first.unicode()) << 8) |
                               static_cast<quint16>(second.unicode() & 0xFF);
        values.push_back(packed);
    }
    const bool asciiWritten = m_modbus->writeRegisters(startOffset, values);
    if (!asciiWritten) {
        qWarning(LOG_FLOW).noquote() << "Failed to write ASCII placeholder at offset" << startOffset;
    }
}

/**
 * @brief 写入加载抓取任务的模拟结果
 * 
 * 向 PLC 写入预设的加载位姿数据（位置和姿态角）。
 * 这是一个占位实现，实际应用中应替换为真实的视觉定位结果。
 */
void StateMachine::writeLoadGraspResult()
{
    // TODO: [vision/grasp_provider] 用外部模块返回的真实位姿替换当前固定值。
    // 输入：外部定位结果（X/Y/Z/Rx/Ry/Rz）；
    // 输出：写入 kLoadX~kLoadRz 寄存器。
    writeFloatPlaceholder(protocol::registers::kLoadX, 125.0f);   // X 坐标（mm）
    writeFloatPlaceholder(protocol::registers::kLoadY, 250.0f);   // Y 坐标（mm）
    writeFloatPlaceholder(protocol::registers::kLoadZ, 375.0f);   // Z 坐标（mm）
    writeFloatPlaceholder(protocol::registers::kLoadRx, 0.0f);    // Rx 旋转角（度）
    writeFloatPlaceholder(protocol::registers::kLoadRy, 90.0f);   // Ry 旋转角（度）
    writeFloatPlaceholder(protocol::registers::kLoadRz, 180.0f);  // Rz 旋转角（度）
}

/**
 * @brief 写入卸载计算任务的模拟结果
 * 
 * 向 PLC 写入预设的卸料位姿数据（位置和姿态角）。
 * 这是一个占位实现，实际应用中应替换为真实的路径规划结果。
 */
void StateMachine::writeUnloadCalcResult()
{
    // TODO: [planner/unload_provider] 用外部模块返回的真实位姿替换当前固定值。
    // 输入：外部规划结果（X/Y/Z/Rx/Ry/Rz）；
    // 输出：写入 kUnloadX~kUnloadRz 寄存器。
    writeFloatPlaceholder(protocol::registers::kUnloadX, 500.0f);   // X 坐标（mm）
    writeFloatPlaceholder(protocol::registers::kUnloadY, 600.0f);   // Y 坐标（mm）
    writeFloatPlaceholder(protocol::registers::kUnloadZ, 700.0f);   // Z 坐标（mm）
    writeFloatPlaceholder(protocol::registers::kUnloadRx, 0.0f);    // Rx 旋转角（度）
    writeFloatPlaceholder(protocol::registers::kUnloadRy, 0.0f);    // Ry 旋转角（度）
    writeFloatPlaceholder(protocol::registers::kUnloadRz, 90.0f);   // Rz 旋转角（度）
}

/**
 * @brief 写入扫描分段结果到 PLC
 * 
 * 将当前分段的索引、采集的图像数量和点云帧数量写入 PLC 寄存器，
 * 供 PLC 跟踪扫描进度。
 * 
 * @param segmentIndex 分段索引（从1开始）
 * @param imageCount 采集的图像数量（通常为1）
 * @param cloudFrameCount 采集的点云帧数量（通常为1）
 */
void StateMachine::writeScanSegmentResult(int segmentIndex, int imageCount, int cloudFrameCount)
{
    if (!m_modbus) {
        return;  // Modbus 不可用，无法写入
    }

    const bool progressWritten = m_modbus->writeRegisters(protocol::registers::kScanSegmentDoneIndex, {
        static_cast<quint16>(segmentIndex),     // 已完成的分段索引
        static_cast<quint16>(imageCount),       // 该分段的图像数量
        static_cast<quint16>(cloudFrameCount),  // 该分段的点云帧数量
    });
    if (!progressWritten) {
        qWarning(LOG_FLOW).noquote() << "Failed to write scan segment progress";
    }
}

/**
 * @brief 写入综合检测结果到 PLC
 * 
 * 将检测结果的 NG 原因字、测量项数量等写入 PLC 寄存器，
 * 供 PLC 判断工件是否合格以及获取详细的缺陷信息。
 * 
 * @param summary 检测结果摘要结构
 */
void StateMachine::writeInspectionResult(const InspectionSummary& summary)
{
    if (!m_modbus) {
        return;  // Modbus 不可用，无法写入
    }

    // 写入 NG 原因字和测量项数量
    const bool inspectionWritten = m_modbus->writeRegisters(protocol::registers::kNgReasonWord0, {
        summary.ngReasonWord0,      // NG 原因字 0（位掩码表示各种缺陷类型）
        summary.ngReasonWord1,      // NG 原因字 1（扩展缺陷类型）
        summary.measureItemCount,   // 实际测量的项目数量
    });
    if (!inspectionWritten) {
        qWarning(LOG_FLOW).noquote() << "Failed to write inspection result";
        return;
    }

    qInfo(LOG_FLOW).noquote()
        << "Inspection result registers written"
        << "ngReasonWord0=" << summary.ngReasonWord0
        << "ngReasonWord1=" << summary.ngReasonWord1
        << "measureItemCount=" << summary.measureItemCount
        << "offsetXmm=" << summary.offsetXmm   // X 方向偏移（未在寄存器中写入，仅记录日志）
        << "offsetYmm=" << summary.offsetYmm   // Y 方向偏移
        << "offsetZmm=" << summary.offsetZmm;  // Z 方向偏移
}

/**
 * @brief 重置点云缓存
 * 
 * 清空所有累积的扫描分段点云数据，释放内存。
 * 在以下情况下调用：
 * - 综合检测完成后（点云已被消费）
 * - 结果复位任务执行时
 * - 系统启动或停止时
 * - 发生故障需要清理状态时
 */
void StateMachine::resetPointCloudCache()
{
    const int cacheSize = m_segmentCaptureResults.size();
    // 遍历所有缓存的点云，主动释放 shared_ptr 引用以确保内存及时回收
    for (auto it = m_segmentCaptureResults.begin(); it != m_segmentCaptureResults.end(); ++it) {
        // 主动释放 shared_ptr 引用，确保 ResultReset 或检测完成后不继续持有大点云内存。
        it->pointCloud.pointsXYZ.reset();
        it->pointCloud.normalsXYZ.reset();
    }
    m_segmentCaptureResults.clear();  // 清空 QMap

    if (cacheSize > 0) {
        qInfo(LOG_FLOW).noquote() << "Cleared scan segment point cloud cache, count=" << cacheSize;
    }
}

/**
 * @brief 记录 Modbus 通信失败
 * 
 * 递增连续失败计数器，当达到阈值时自动进入故障状态。
 * 这种机制可以容忍偶发的通信干扰，但对持续故障做出快速响应。
 * 
 * @param alarmCode 报警代码
 * @param message 错误描述信息
 */
void StateMachine::recordModbusFailure(quint16 alarmCode, const QString& message)
{
    ++m_consecutiveModbusFailures;
    qWarning(LOG_FLOW).noquote()
        << "Recorded Modbus failure"
        << m_consecutiveModbusFailures << "/" << kMaxConsecutiveModbusFailures
        << "alarmCode=" << alarmCode
        << "reason=" << message;

    // 如果连续失败次数达到阈值，进入故障状态
    if (m_consecutiveModbusFailures >= kMaxConsecutiveModbusFailures) {
        enterFaultState(alarmCode, message, true, true);
    }
}

/**
 * @brief 重置 Modbus 失败计数器
 * 
 * 在成功通信后调用，将连续失败计数器归零。
 */
void StateMachine::resetModbusFailureCounter()
{
    if (m_consecutiveModbusFailures > 0) {
        qInfo(LOG_FLOW) << "Resetting Modbus failure counter after successful communication.";
    }
    m_consecutiveModbusFailures = 0;
}

/**
 * @brief 进入故障状态
 * 
 * 设置报警信息，切换应用状态为 Error，并根据参数决定是否中止当前任务。
 * 这是系统处理严重错误的统一入口。
 * 
 * @param alarmCode 报警代码
 * @param message 错误描述信息
 * @param abortCurrentTask 是否中止当前活动任务
 * @param notifyPlc 是否通知 PLC（通过发送 Res/Ack）
 */
void StateMachine::enterFaultState(
    quint16 alarmCode,
    const QString& message,
    bool abortCurrentTask,
    bool notifyPlc)
{
    setAlarm(3, alarmCode, message);       // 设置严重错误级别报警
    m_ipcState = protocol::IpcState::Fault; // IPC 状态设为故障
    setState(AppState::Error);              // 应用状态设为错误

    if (abortCurrentTask) {
        abortActiveTaskForFault(7);  // 中止当前任务，Res=7 表示处理失败
    } else {
        m_timeoutTimer->stop();      // 停止超时定时器
        m_progress = 0;              // 进度归零
        m_currentStage = protocol::Stage::Idle;  // 阶段回到空闲
        publishIpcStatus();          // 发布故障状态
    }

    if (!notifyPlc) {
        clearActiveTask();           // 清除活动任务
        m_currentStage = protocol::Stage::Idle;
    }
}

/**
 * @brief 因故障中止当前活动任务
 * 
 * 根据任务类型采取不同的中止策略：
 * - 扫描分段任务：先写入空结果再完成
 * - 其他任务：直接清理状态
 * 如果 Modbus 可用，通过正常的 completeActiveTask 流程通知 PLC；
 * 否则直接清理本地状态。
 * 
 * @param resultCode 任务结果码（通常为 7 = 处理失败）
 */
void StateMachine::abortActiveTaskForFault(quint16 resultCode)
{
    if (m_activeTask.definition == nullptr) {
        // 没有活动任务，只需清理基本状态
        m_timeoutTimer->stop();
        m_progress = 0;
        m_dataValid = false;
        m_currentStage = protocol::Stage::Idle;
        publishIpcStatus();
        return;
    }

    // 如果是扫描分段任务，先写入空结果
    if (m_activeTask.definition->stage == protocol::Stage::ScanSegment) {
        writeScanSegmentResult(m_activeTask.scanSegmentIndex, 0, 0);
    }

    // 如果 Modbus 可用，通过正常流程通知 PLC
    if (m_modbus && m_modbus->isConnected()) {
        completeActiveTask(resultCode, protocol::AckState::Failed, false);
        return;
    }

    // Modbus 不可用时，直接清理本地状态
    m_timeoutTimer->stop();
    m_progress = 0;
    m_dataValid = false;
    m_activeTask.captureRequestId = 0;
    m_activeTask.completionAnnounced = false;
    clearActiveTask();
    m_currentStage = protocol::Stage::Idle;
    publishIpcStatus();
}

/**
 * @brief 将相机采集错误码映射为 PLC 结果码
 * 
 * 将 Mech-Eye 相机的内部错误码转换为 PLC 能理解的结果码：
 * - 1: 成功
 * - 5: 设备未就绪（连接失败、忙等）
 * - 6: 超时
 * - 7: 处理失败（未知错误）
 * - 9: 参数错误（无效请求）
 * 
 * @param errorCode 相机采集错误码
 * @return 对应的 PLC 结果码
 */
quint16 StateMachine::mapCaptureErrorToResCode(mech_eye::CaptureErrorCode errorCode) const
{
    switch (errorCode) {
    case mech_eye::CaptureErrorCode::Success:
        return 1;   // 成功
    case mech_eye::CaptureErrorCode::NotStarted:
    case mech_eye::CaptureErrorCode::NotConnected:
    case mech_eye::CaptureErrorCode::Busy:
    case mech_eye::CaptureErrorCode::DiscoverFailed:
    case mech_eye::CaptureErrorCode::ConnectFailed:
    case mech_eye::CaptureErrorCode::DisconnectFailed:
        return 5;   // 设备未就绪
    case mech_eye::CaptureErrorCode::Timeout:
        return 6;   // 超时
    case mech_eye::CaptureErrorCode::InvalidRequest:
        return 9;   // 参数错误
    default:
        return 7;   // 其他错误归为处理失败
    }
}

/**
 * @brief 从命令块中读取任务 ID
 * 
 * 任务 ID 是一个 32 位整数，存储在两个 16 位寄存器中（高16位和低16位）。
 * 
 * @param commandBlock 命令块寄存器数据
 * @return 32 位任务 ID
 */
quint32 StateMachine::readTaskId(const QVector<quint16>& commandBlock) const
{
    const quint32 high = static_cast<quint32>(commandBlock.value(protocol::registers::kTaskIdHigh));
    const quint32 low = static_cast<quint32>(commandBlock.value(protocol::registers::kTaskIdLow));
    return (high << 16) | low;  // 组合高16位和低16位
}

/**
 * @brief 解析扫描分段索引
 * 
 * 从命令块中读取当前请求的扫描分段索引。
 * 注意：地址表中 ScanSegmentIndex 是 40015，即 0 基偏移 14；
 * 40024/偏移23 是 Trig_Inspection。这里统一使用协议常量读取，
 * 避免把触发位误当作段号。
 * 
 * @param commandBlock 命令块寄存器数据
 * @return 扫描分段索引（从1开始）
 */
quint16 StateMachine::resolveScanSegmentIndex(const QVector<quint16>& commandBlock) const
{
    return commandBlock.value(protocol::registers::kScanSegmentIndex);
}

/**
 * @brief 验证扫描分段请求的合法性
 * 
 * 检查分段索引是否在有效范围内，以及是否已经采集过该分段（重复检测）。
 * 
 * @param commandBlock 命令块寄存器数据
 * @param errorMessage 输出参数，如果验证失败则填充错误描述信息
 * @return true 如果请求合法，false 如果请求非法
 */
bool StateMachine::validateScanSegmentRequest(const QVector<quint16>& commandBlock, QString* errorMessage)
{
    const int segmentIndex = resolveScanSegmentIndex(commandBlock);  // 获取分段索引
    const int segmentTotal = commandBlock.value(protocol::registers::kScanSegmentTotal);  // 总分段数
    // 计算允许的最大段号：不超过总段数和系统上限的最小值
    const int maxSegmentIndex = segmentTotal > 0 ? qMin(segmentTotal, kMaxScanSegmentIndex)
                                                 : kMaxScanSegmentIndex;

    // 检查段号是否在有效范围内 [1, maxSegmentIndex]
    if (segmentIndex < 1 || segmentIndex > maxSegmentIndex) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("Invalid scan segment index: index=%1, total=%2, allowed=1-%3")
                .arg(segmentIndex)
                .arg(segmentTotal)
                .arg(maxSegmentIndex);
        }
        qWarning(LOG_FLOW).noquote() << "Rejecting Trig_ScanSegment due to invalid index"
                                     << "index=" << segmentIndex
                                     << "total=" << segmentTotal
                                     << "allowedMax=" << maxSegmentIndex;
        return false;
    }

    // 检查是否已经采集过该分段（防止重复触发污染缓存）
    if (m_segmentCaptureResults.contains(segmentIndex)) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("Duplicate scan segment index: %1").arg(segmentIndex);
        }
        qWarning(LOG_FLOW).noquote() << "Rejecting Trig_ScanSegment due to duplicate cache entry"
                                     << "segmentIndex=" << segmentIndex
                                     << "cacheSize=" << m_segmentCaptureResults.size();
        return false;
    }

    // P2改进：检查缓存大小是否达到上限，防止内存无限增长
    if (m_segmentCaptureResults.size() >= kMaxPointCloudCacheSize) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("Point cloud cache is full: current size=%1, max allowed=%2")
                .arg(m_segmentCaptureResults.size())
                .arg(kMaxPointCloudCacheSize);
        }
        qWarning(LOG_FLOW).noquote() << "Rejecting Trig_ScanSegment due to cache size limit exceeded"
                                     << "currentSize=" << m_segmentCaptureResults.size()
                                     << "maxAllowed=" << kMaxPointCloudCacheSize;
        return false;
    }

    return true;  // 验证通过
}

/**
 * @brief 完成扫描分段失败的处理
 * 
 * 统一的扫描分段失败出口，确保按照正确的顺序写入结果：
 * 1. 设置报警信息
 * 2. 写入空结果（0 图像数，0 点云帧数）
 * 3. 清除采集请求 ID
 * 4. 完成任务并标记为失败
 * 
 * 这样可以保证 PLC 不会读到旧数据或 inconsistent 的状态。
 * 
 * @param resultCode 结果码（5=设备未就绪，6=超时，7=处理失败，9=参数错误）
 * @param alarmLevel 报警级别（2=警告，3=严重错误）
 * @param alarmCode 报警代码
 * @param logMessage 日志消息（详细的技术描述）
 * @param alarmMessage 报警消息（显示给用户的简洁描述）
 */
void StateMachine::finishScanSegmentFailure(
    quint16 resultCode,
    quint16 alarmLevel,
    quint16 alarmCode,
    const QString& logMessage,
    const QString& alarmMessage)
{
    // 失败闭环也必须先写结果区，再写 Res，最后写 Ack=3，保证 PLC 不会读到旧数据。
    qWarning(LOG_FLOW).noquote()
        << "Trig_ScanSegment failed"
        << "segmentIndex=" << m_activeTask.scanSegmentIndex
        << "res=" << resultCode
        << "reason=" << logMessage;
    setAlarm(alarmLevel, alarmCode, alarmMessage);              // 设置报警
    writeScanSegmentResult(m_activeTask.scanSegmentIndex, 0, 0); // 写入空结果
    
    // P0修复：严重错误时清理已缓存的点云数据，防止内存泄漏
    if (resultCode >= 5) {
        qWarning(LOG_FLOW) << "Clearing point cloud cache due to scan failure (res=" << resultCode << ")";
        resetPointCloudCache();
    }
    
    m_activeTask.captureRequestId = 0;                          // 清除采集请求 ID
    completeActiveTask(resultCode, protocol::AckState::Failed, false);  // 完成失败任务
}

}  // namespace scan_tracking::flow_control
