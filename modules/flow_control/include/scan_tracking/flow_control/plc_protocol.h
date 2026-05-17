/**
 * @file plc_protocol.h
 * @brief PLC与IPC之间的Modbus通讯协议定义
 * 
 * 本文件定义了扫描跟踪系统中PLC与IPC之间
 * 的Modbus TCP通讯协议。协议采用寄存器映射方式，通过读写特定的寄存器地址实现
 * 双向数据交换和状态同步。
 * 
 * 通讯架构：
 * - PLC作为Modbus主站，负责发起读写请求
 * - IPC作为Modbus从站，响应PLC的请求并更新寄存器数据
 */

#pragma once

#include <QtCore/QByteArray>
#include <QtCore/QString>
#include <QtCore/QVector>
#include <QtCore/QtGlobal>

namespace scan_tracking::flow_control::protocol {

/**
 * @enum Stage
 * @brief 工作流程阶段枚举
 * 
 * 定义扫描跟踪系统的完整工作流程阶段，每个阶段对应一个具体的工艺步骤。
 * 阶段按顺序执行，形成完整的扫描检测流程。
 */
enum class Stage : quint16 {
    Idle = 0,                 ///< 空闲状态：系统待机，无任务执行
    SafetyMonitoring = 1,     ///< 安全监控：实时监测安全设备状态（光栅、急停等）
    SelfCheck = 2,            ///< 自检阶段：检查各子系统（相机、传感器等）是否正常
    LoadMaterialCheck = 3,    ///< 上料检测：判断上料工位是否有物料需要抓取
    LoadGrasp = 4,            ///< 上料抓取：计算抓取点坐标并执行抓取动作
    StationMaterialCheck = 5, ///< 工位检材：确认扫描工位是否已放置物料
    PoseCheck = 6,            ///< 位姿校验：验证物料通孔位姿是否符合要求
    ScanSegment = 7,          ///< 分段扫描：执行多段式3D扫描作业
    Inspection = 8,           ///< 综合检测：进行OCR识别、缺陷检测和尺寸测量
    UnloadCalc = 9,           ///< 卸料计算：计算下料区卸料点坐标
    ResultReset = 10,         ///< 结果复位：清空检测结果，准备下一轮循环
};

/**
 * @enum IpcState
 * @brief IPC系统状态枚举
 * 
 * 反映IPC当前的运行状态，PLC根据此状态判断是否可以发送新指令。
 */
enum class IpcState : quint16 {
    Uninitialized = 0,  ///< 未初始化：IPC刚启动，尚未完成初始化
    Initializing = 1,   ///< 初始化中：正在加载配置、连接设备等
    Ready = 2,          ///< 就绪状态：可以接收并执行PLC指令
    Busy = 3,           ///< 忙碌状态：正在执行任务，暂不接收新指令
    Paused = 4,         ///< 暂停状态：任务被临时挂起
    Fault = 5,          ///< 故障状态：发生错误，需要人工干预
};

/**
 * @enum AckState
 * @brief 应答状态枚举
 * 
 * 用于触发-应答机制中的状态反馈，表示IPC对PLC触发信号的处理进度。
 */
enum class AckState : quint16 {
    Idle = 0,       ///< 空闲：未收到触发或已完成复位
    Running = 1,    ///< 执行中：正在处理触发的任务
    Completed = 2,  ///< 已完成：任务成功执行完毕
    Failed = 3,     ///< 失败：任务执行出错
};

/**
 * @struct TriggerDefinition
 * @brief 触发器定义结构
 * 
 * 定义一个完整的触发-应答-结果三元组，用于PLC与IPC之间的任务协调。
 * 每个触发器对应一个具体的工艺动作，包含触发信号、应答信号和结果数据的寄存器偏移量。
 */
struct TriggerDefinition {
    const char* name = "";              ///< 触发器名称标识
    int trigOffset = 0;                 ///< 触发信号寄存器偏移量（PLC写入，IPC读取）
    int ackOffset = 0;                  ///< 应答信号寄存器偏移量（IPC写入，PLC读取）
    int resOffset = 0;                  ///< 结果代码寄存器偏移量（IPC写入，PLC读取）
    Stage stage = Stage::Idle;          ///< 对应的工艺流程阶段
    int defaultTimeoutSeconds = 0;      ///< 默认超时时间（秒），超过此时间未完成则判定为故障
};

/**
 * @namespace registers
 * @brief Modbus寄存器地址定义
 * 
 * 定义PLC与IPC通讯时使用的所有Modbus保持寄存器的地址映射。
 * 寄存器分为两个主要区域：
 * - 命令区（0-39）：PLC向IPC发送控制指令和参数
 * - 结果区（100-183）：IPC向PLC返回执行结果和状态信息
 * 
 * 注意：所有地址均为相对于Modbus起始地址的偏移量，实际使用时需加上基地址。
 */
namespace registers {

constexpr int kPlcAddressBase = 1;   ///< PLC 人机界面常用 1 基地址

inline constexpr int toModbusOffset(int plcAddress)
{
    return plcAddress - kPlcAddressBase;
}

inline constexpr int toPlcAddress(int modbusOffset)
{
    return modbusOffset + kPlcAddressBase;
}

// ==================== 寄存器区块定义 ====================

constexpr int kCommandBlockStart = 0;    ///< 命令区起始地址：PLC→IPC的控制指令区域（0 基偏移）
constexpr int kCommandBlockSize = 40;    ///< 命令区大小：共40个寄存器
constexpr int kResultBlockStart = 100;   ///< 结果区起始地址：IPC→PLC的状态和数据区域
constexpr int kResultBlockSize = 84;     ///< 结果区大小：共84个寄存器

// ==================== 命令区寄存器（PLC → IPC）====================

constexpr int kPlcHeartbeat = 0;         ///< PLC心跳计数器：PLC周期性递增，用于检测通讯链路是否正常
constexpr int kPlcSystemState = 1;       ///< PLC系统状态字：反映PLC侧的整体运行状态
constexpr int kStationWorkMode = 2;      ///< 工位工作模式：选择自动/手动/调试等工作模式
constexpr int kFlowEnable = 3;           ///< 流程使能标志：1=允许自动流程，0=禁止自动流程
constexpr int kSafetyStatusWord = 4;     ///< 安全状态字：各位表示不同安全设备的状态（急停、光栅、门锁等）
                                         ///< 位0: 急停按钮, 位1: 安全光栅, 位2: 安全门, ...
constexpr int kTaskIdHigh = 10;          ///< 任务ID高16位：唯一标识当前生产批次或工单
constexpr int kTaskIdLow = 11;           ///< 任务ID低16位：与高位组合成32位任务ID
constexpr int kProductType = 12;         ///< 产品类型代码：区分不同规格的产品型号
constexpr int kRecipeId = 13;            ///< 配方ID：选择对应的工艺参数配方
constexpr int kScanSegmentIndex = 14;    ///< 当前扫描段索引（32位整数，占2个寄存器：offset 14 高16位 + offset 15 低16位）
                                         ///< PLC 使用模拟量格式，高位在前（Big-Endian word order）
constexpr int kScanSegmentIndexLow = 15; ///< 扫描段索引低16位（与 kScanSegmentIndex 组成32位值）
constexpr int kRequestTimeoutSeconds = 16;///< 请求超时时间：覆盖默认超时值的自定义超时设定（秒）

// ==================== 结果区寄存器（IPC → PLC）====================

// --- IPC系统状态 ---
constexpr int kIpcHeartbeat = 100;       ///< IPC心跳计数器：IPC周期性递增，与PLC心跳配合检测通讯
constexpr int kIpcSystemState = 101;     ///< IPC系统状态：对应IpcState枚举值
constexpr int kIpcCurrentStage = 102;    ///< IPC当前阶段：对应Stage枚举值，显示当前执行到的工艺阶段
constexpr int kIpcAlarmLevel = 103;      ///< 报警级别：0=无报警, 1=警告, 2=一般报警, 3=严重报警
constexpr int kIpcAlarmCode = 104;       ///< 报警代码：具体报警类型的编码
constexpr int kIpcWarnCode = 105;        ///< 警告代码：具体警告类型的编码
constexpr int kIpcReady = 106;           ///< IPC就绪标志：1=就绪可接收指令，0=不可用
constexpr int kIpcDataValid = 107;       ///< 数据有效标志：1=结果区数据有效，0=数据无效或过期
constexpr int kIpcProgress = 108;        ///< 任务进度百分比：0-100表示当前任务的完成进度

// --- 设备状态监控 ---
constexpr int kDeviceOnlineWord0 = 109;  ///< 设备在线状态字0：每位代表一个设备的在线状态
                                         ///< 位0: Mech-Eye相机, 位1: Modbus设备, 位2: 机器人, ...
constexpr int kDeviceOnlineWord1 = 110;  ///< 设备在线状态字1：扩展设备在线状态
constexpr int kDeviceFaultWord0 = 111;   ///< 设备故障状态字0：每位代表一个设备的故障状态
constexpr int kDeviceFaultWord1 = 112;   ///< 设备故障状态字1：扩展设备故障状态

// --- 任务ID回显 ---
constexpr int kTaskIdEchoHigh = 113;     ///< 任务ID回显高16位：IPC回显收到的任务ID，用于校验
constexpr int kTaskIdEchoLow = 114;      ///< 任务ID回显低16位：与高位组合验证任务ID一致性

// ==================== 触发器应答与结果寄存器 ====================

// --- 上料抓取（LoadGrasp）---
constexpr int kAckLoadGrasp = 115;       ///< 上料抓取应答：对应AckState枚举，PLC触发后IPC的应答状态
constexpr int kResLoadGrasp = 116;       ///< 上料抓取结果码：1=成功，其他值表示失败原因
constexpr int kLoadX = 117;              ///< 抓取点X坐标（mm）：浮点数占用2个寄存器（IEEE 754格式）
constexpr int kLoadY = 119;              ///< 抓取点Y坐标（mm）：浮点数占用2个寄存器
constexpr int kLoadZ = 121;              ///< 抓取点Z坐标（mm）：浮点数占用2个寄存器
constexpr int kLoadRx = 123;             ///< 抓取点绕X轴旋转角度（度）：浮点数占用2个寄存器
constexpr int kLoadRy = 125;             ///< 抓取点绕Y轴旋转角度（度）：浮点数占用2个寄存器
constexpr int kLoadRz = 127;             ///< 抓取点绕Z轴旋转角度（度）：浮点数占用2个寄存器

// --- 工位检材（StationMaterialCheck）---
constexpr int kAckStationMaterialCheck = 129;  ///< 工位检材应答：对应AckState枚举
constexpr int kResStationMaterialCheck = 130;  ///< 工位检材结果码：1=有料，0=无料，其他=异常

// --- 位姿校验（PoseCheck）---
constexpr int kAckPoseCheck = 131;       ///< 位姿校验应答：对应AckState枚举
constexpr int kResPoseCheck = 132;       ///< 位姿校验结果码：1=合格，0=不合格，其他=异常
constexpr int kPoseDeviationMm = 133;    ///< 位姿偏差值（mm）：实际位姿与标准位姿的最大偏差

// --- 分段扫描（ScanSegment）---
constexpr int kAckScanSegment = 134;     ///< 分段扫描应答：对应AckState枚举
constexpr int kResScanSegment = 135;     ///< 分段扫描结果码：1=成功，其他=失败原因
constexpr int kScanSegmentDoneIndex = 136;///< 已完成扫描段索引：当前已成功完成的段落编号
constexpr int kScanImageCount = 137;     ///< 采集图像数量：本次扫描采集的2D图像帧数
constexpr int kScanCloudFrameCount = 138;///< 点云帧数量：本次扫描生成的3D点云帧数

// --- 综合检测（Inspection）---
constexpr int kAckInspection = 139;      ///< 综合检测应答：对应AckState枚举
constexpr int kResInspection = 140;      ///< 综合检测结果码：1=OK合格，0=NG不合格，其他=异常
constexpr int kNgReasonWord0 = 141;      ///< NG原因字0：各位表示不同的缺陷类型
                                         ///< 位0: 表面划痕, 位1: 凹坑, 位2: 裂纹, ...
constexpr int kNgReasonWord1 = 142;      ///< NG原因字1：扩展NG原因
constexpr int kMeasureItemCount = 143;   ///< 测量项数量：完成的尺寸测量项目总数

// --- 卸料计算（UnloadCalc）---
constexpr int kAckUnloadCalc = 144;      ///< 卸料计算应答：对应AckState枚举
constexpr int kResUnloadCalc = 145;      ///< 卸料计算结果码：1=成功，其他=失败原因
constexpr int kUnloadX = 146;            ///< 卸料点X坐标（mm）：浮点数占用2个寄存器
constexpr int kUnloadY = 148;            ///< 卸料点Y坐标（mm）：浮点数占用2个寄存器
constexpr int kUnloadZ = 150;            ///< 卸料点Z坐标（mm）：浮点数占用2个寄存器
constexpr int kUnloadRx = 152;           ///< 卸料点绕X轴旋转角度（度）：浮点数占用2个寄存器
constexpr int kUnloadRy = 154;           ///< 卸料点绕Y轴旋转角度（度）：浮点数占用2个寄存器
constexpr int kUnloadRz = 156;           ///< 卸料点绕Z轴旋转角度（度）：浮点数占用2个寄存器

// --- 自检（SelfCheck）---
constexpr int kAckSelfCheck = 158;       ///< 自检应答：对应AckState枚举
constexpr int kResSelfCheck = 159;       ///< 自检结果码：1=全部通过，0=存在故障
constexpr int kSelfCheckFailWord0 = 160; ///< 自检失败字0：各位表示不同子系统的自检结果
                                         ///< 位0: 相机系统, 位1: Modbus通讯, 位2: 机器人接口, ...
constexpr int kSelfCheckFailWord1 = 161; ///< 自检失败字1：扩展自检失败状态

// --- 条码读取（CodeRead）---
constexpr int kAckCodeRead = 162;        ///< 条码读取应答：对应AckState枚举
constexpr int kResCodeRead = 163;        ///< 条码读取结果码：1=成功，0=失败，其他=异常
constexpr int kCodeValueAscii = 164;     ///< 条码ASCII值起始地址：连续多个寄存器存储条码字符串
constexpr int kCodeValueRegisterCount = 8;///< 条码值占用寄存器数量：最多支持16个字符（每寄存器存2字节）

// --- 安全与复位 ---
constexpr int kIpcSafetyActionWord = 172;///< IPC安全动作字：IPC主动触发的安全相关动作
                                         ///< 位0: 紧急停止, 位1: 暂停流程, 位2: 释放夹具, ...
constexpr int kAckResultReset = 173;     ///< 结果复位应答：对应AckState枚举
constexpr int kResResultReset = 174;     ///< 结果复位结果码：1=复位成功，其他=失败

}  // namespace registers

/**
 * @brief 获取所有触发器定义的引用
 * 
 * 返回系统中定义的所有触发器的静态数组，包含触发信号、应答信号、结果信号的
 * 寄存器偏移量以及对应的工艺阶段和超时时间。
 * 
 * @return const QVector<TriggerDefinition>& 触发器定义列表
 */
inline const QVector<TriggerDefinition>& triggerDefinitions()
{
    static const QVector<TriggerDefinition> definitions = {
        {"Trig_LoadGrasp", 19, registers::kAckLoadGrasp, registers::kResLoadGrasp, Stage::LoadGrasp, 10},
        {"Trig_StationMaterialCheck", 20, registers::kAckStationMaterialCheck, registers::kResStationMaterialCheck, Stage::StationMaterialCheck, 5},
        {"Trig_PoseCheck", 21, registers::kAckPoseCheck, registers::kResPoseCheck, Stage::PoseCheck, 5},
        {"Trig_ScanSegment", 22, registers::kAckScanSegment, registers::kResScanSegment, Stage::ScanSegment, 60},
        {"Trig_Inspection", 23, registers::kAckInspection, registers::kResInspection, Stage::Inspection, 60},
        {"Trig_UnloadCalc", 24, registers::kAckUnloadCalc, registers::kResUnloadCalc, Stage::UnloadCalc, 10},
        {"Trig_SelfCheck", 25, registers::kAckSelfCheck, registers::kResSelfCheck, Stage::SelfCheck, 10},
        {"Trig_CodeRead", 26, registers::kAckCodeRead, registers::kResCodeRead, Stage::Inspection, 10},
        {"Trig_ResultReset", 27, registers::kAckResultReset, registers::kResResultReset, Stage::ResultReset, 10},
    };
    return definitions;
}

/**
 * @brief 根据触发信号偏移量查找触发器定义
 * 
 * 遍历所有触发器定义，找到与给定触发信号寄存器偏移量匹配的触发器。
 * 
 * @param trigOffset 触发信号寄存器偏移量
 * @return const TriggerDefinition* 找到的触发器定义指针，未找到返回nullptr
 */
inline const TriggerDefinition* triggerByOffset(int trigOffset)
{
    const auto& definitions = triggerDefinitions();
    for (const auto& definition : definitions) {
        if (definition.trigOffset == trigOffset) {
            return &definition;
        }
    }
    return nullptr;
}

/**
 * @brief 获取触发器的默认结果码
 * 
 * 对于给定的触发器定义，返回其成功执行时的默认结果代码。
 * 目前统一返回1表示成功，可根据实际需求扩展。
 * 
 * @param definition 触发器定义引用
 * @return quint16 默认结果码
 */
inline quint16 defaultResCodeFor(const TriggerDefinition& definition)
{
    Q_UNUSED(definition);
    return 1;
}

/**
 * @brief 获取触发器的名称字符串
 * 
 * 将触发器定义中的C风格字符串转换为Qt字符串，便于日志输出和界面显示。
 * 
 * @param definition 触发器定义引用
 * @return QString 触发器名称
 */
inline QString triggerName(const TriggerDefinition& definition)
{
    return QString::fromLatin1(definition.name);
}

}  // namespace scan_tracking::flow_control::protocol
