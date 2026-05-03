#pragma once

#include <QtCore/QMap>
#include <QtCore/QElapsedTimer>
#include <QtCore/QObject>
#include <QtCore/QTimer>
#include <QtCore/QVector>
#include <QtCore/QtGlobal>

#include "scan_tracking/flow_control/plc_protocol.h"
#include "scan_tracking/mech_eye/mech_eye_types.h"
#include "scan_tracking/modbus/modbus_service.h"
#include "scan_tracking/tracking/tracking_service.h"
#include "scan_tracking/vision/vision_types.h"

namespace scan_tracking {
namespace mech_eye {
class MechEyeService;
}
namespace vision {
class VisionPipelineService;
}
namespace tracking {
struct PoseCheckResult;
}
namespace flow_control {

// 应用状态枚举
enum class AppState {
    Init,      // 初始化
    Ready,     // 就绪
    Scanning,  // 扫描中
    Error,     // 错误
};

// 状态机类，负责管理整个扫描跟踪流程的状态转换和业务逻辑
class StateMachine : public QObject {
    Q_OBJECT

public:
    // 构造函数
    // @param modbusService Modbus 通信服务指针
    // @param mechEyeService Mech-Eye 相机服务指针（可选）
    // @param trackingService 跟踪服务指针（可选）
    // @param parent Qt 父对象指针
    explicit StateMachine(
        modbus::ModbusService* modbusService,
        mech_eye::MechEyeService* mechEyeService = nullptr,
        vision::VisionPipelineService* visionPipelineService = nullptr,
        tracking::TrackingService* trackingService = nullptr,
        QObject* parent = nullptr);
    ~StateMachine();

    // 启动状态机
    void start();

    // 停止状态机
    void stop();

    // 获取当前状态
    // @return 当前应用状态
    AppState currentState() const { return m_state; }

signals:
    // 状态改变信号
    // @param newState 新的状态
    void stateChanged(AppState newState);

    // 协议信号
    // @param message 事件消息
    void protocolEvent(const QString& message);

private slots:
    // 轮询 PLC 状态
    void pollPlcState();

    // 处理寄存器读取结果
    // @param startAddress 起始地址
    // @param values 读取到的寄存器值
    void handleRegistersRead(int startAddress, const QVector<quint16>& values);

    // 处理寄存器读取失败
    // @param startAddress 起始地址
    // @param errorString 错误信息
    void onRegisterReadFailed(int startAddress, const QString& errorString);

    // 处理寄存器写入失败
    // @param startAddress 起始地址
    // @param errorString 错误信息
    void onRegisterWriteFailed(int startAddress, const QString& errorString);

    // Modbus 连接成功回调
    void onModbusConnected();

    // Modbus 断开连接回调
    void onModbusDisconnected();

    // Modbus 错误回调
    // @param errorString 错误信息
    void onModbusError(const QString& errorString);

    // 采集完成回调
    // @param result 采集结果
    void onCaptureFinished(scan_tracking::mech_eye::CaptureResult result);

    // 视觉编排采集完成回调
    // @param bundle 多相机采集结果
    void onVisionBundleCaptureFinished(scan_tracking::vision::MultiCameraCaptureBundle bundle);

    // Mech-Eye 致命错误回调
    // @param code 错误码
    // @param message 错误信息
    void onMechEyeFatalError(scan_tracking::mech_eye::CaptureErrorCode code, QString message);

    // 处理超时回调
    void onProcessTimeout();

private:
    // 活动任务结构体，记录当前正在执行的任务信息
    struct ActiveTask {
        const protocol::TriggerDefinition* definition = nullptr;  // 触发定义指针
        quint32 taskId = 0;                                        // 任务 ID
        quint16 timeoutSeconds = 0;                                // 超时时间（秒）
        bool completionAnnounced = false;                          // 是否已宣布完成
        int scanSegmentIndex = 0;                                  // 扫描段索引
        int scanSegmentTotal = 0;                                  // 扫描段总数
        quint64 captureRequestId = 0;                              // 采集请求 ID
    };

public:
    struct PoseSourceResult {
        bool available = false;
        bool success = false;
        QString sourceName;
        QString message;
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        float rx = 0.0f;
        float ry = 0.0f;
        float rz = 0.0f;
    };

private:
    // 检测结果汇总结构体
    struct InspectionSummary {
        quint16 resultCode = 1;       // 结果码
        quint16 ngReasonWord0 = 0;    // NG 原因字 0
        quint16 ngReasonWord1 = 0;    // NG 原因字 1
        quint16 measureItemCount = 0; // 测量项数量
        float offsetXmm = 0.0f;       // X 方向偏移量（mm）
        float offsetYmm = 0.0f;       // Y 方向偏移量（mm）
        float offsetZmm = 0.0f;       // Z 方向偏移量（mm）
    };

    // 设置新状态
    // @param newState 新状态
    void setState(AppState newState);

    // 处理触发器
    // @param trigger 触发器定义
    // @param commandBlock 命令块数据
    void processTrigger(
        const protocol::TriggerDefinition& trigger,
        const QVector<quint16>& commandBlock);

    // 执行当前活动任务
    void executeActiveTask();

    // 执行上料抓取任务
    void executeLoadGraspTask();

    // 执行工位检材任务
    void executeStationMaterialCheckTask();

    // 执行卸料计算任务
    void executeUnloadCalcTask();

    // 执行扫描分段任务
    void executeScanSegmentTask();

    // 执行检测任务
    void executeInspectionTask();

    // 执行位姿校验任务
    void executePoseCheckTask();

    // 执行自检任务
    void executeSelfCheckTask();

    // 执行条码读取任务
    void executeCodeReadTask();

    // 执行结果复位任务
    void executeResultResetTask();

    // 发送 ACK 确认
    // @param definition 触发器定义
    // @param ackState ACK 状态
    void sendAck(const protocol::TriggerDefinition& definition, protocol::AckState ackState);

    // 发送 RES 响应
    // @param definition 触发器定义
    // @param resultCode 结果码
    void sendRes(const protocol::TriggerDefinition& definition, quint16 resultCode);

    // 发布 IPC 状态
    void publishIpcStatus();

    // 发布心跳
    void publishHeartbeat();

    // 完成活动任务
    // @param resultCode 结果码
    // @param finalAckState 最终 ACK 状态
    // @param dataValid 数据是否有效
    bool completeActiveTask(
        quint16 resultCode,
        protocol::AckState finalAckState = protocol::AckState::Completed,
        bool dataValid = true);

    // 如果触发器已释放，则完成已完成任务的收尾
    // @param commandBlock 命令块数据
    void finalizeCompletedTaskIfTriggerReleased(const QVector<quint16>& commandBlock);

    // 清除活动任务
    void clearActiveTask();

    // 清空扫描分段缓存（点云 + 视觉 bundle）
    void resetScanSegmentCache();

    // 设置报警
    // @param level 报警级别
    // @param code 报警代码
    // @param message 报警信息
    void setAlarm(quint16 level, quint16 code, const QString& message);

    // 记录 Modbus 故障
    // @param alarmCode 报警代码
    // @param message 故障信息
    void recordModbusFailure(quint16 alarmCode, const QString& message);

    // 重置 Modbus 故障计数器
    void resetModbusFailureCounter();

    // 进入故障状态
    // @param alarmCode 报警代码
    // @param message 故障信息
    // @param abortCurrentTask 是否中止当前任务
    // @param notifyPlc 是否通知 PLC
    void enterFaultState(
        quint16 alarmCode,
        const QString& message,
        bool abortCurrentTask,
        bool notifyPlc);

    // 因故障中止活动任务
    // @param resultCode 结果码
    void abortActiveTaskForFault(quint16 resultCode);

    // 写入浮点数占位符
    // @param startOffset 起始偏移量
    // @param value 浮点值
    void writeFloatPlaceholder(int startOffset, float value);

    // 读取加载抓取位姿源
    PoseSourceResult resolveLoadGraspPoseSource() const;

    // 读取卸料计算位姿源
    PoseSourceResult resolveUnloadCalcPoseSource() const;

    // 写入 ASCII 字符串占位符
    // @param startOffset 起始偏移量
    // @param registerCount 寄存器数量
    // @param text 文本内容
    void writeAsciiPlaceholder(int startOffset, int registerCount, const QString& text);

    // 写入上料抓取结果
    void writeLoadGraspResult();

    // 写入卸料计算结果
    void writeUnloadCalcResult();

    // 写入扫描分段结果
    // @param segmentIndex 分段索引
    // @param imageCount 图像数量
    // @param cloudFrameCount 点云帧数
    void writeScanSegmentResult(int segmentIndex, int imageCount, int cloudFrameCount);

    // 写入检测结果
    // @param summary 检测汇总信息
    void writeInspectionResult(const InspectionSummary& summary);

    // 重置点云缓存
    void resetPointCloudCache();

    // 将采集错误码映射为 RES 结果码
    // @param errorCode 采集错误码
    // @return RES 结果码
    quint16 mapCaptureErrorToResCode(scan_tracking::mech_eye::CaptureErrorCode errorCode) const;

    // 从命令块中读取任务 ID
    // @param commandBlock 命令块数据
    // @return 任务 ID
    quint32 readTaskId(const QVector<quint16>& commandBlock) const;

    // 解析扫描分段索引
    // @param commandBlock 命令块数据
    // @return 扫描分段索引
    quint16 resolveScanSegmentIndex(const QVector<quint16>& commandBlock) const;

    // 验证扫描分段请求
    // @param commandBlock 命令块数据
    // @param errorMessage 错误信息输出
    // @return 是否通过验证
    bool validateScanSegmentRequest(const QVector<quint16>& commandBlock, QString* errorMessage);

    // 完成扫描分段失败处理
    // @param resultCode 结果码
    // @param alarmLevel 报警级别
    // @param alarmCode 报警代码
    // @param logMessage 日志消息
    // @param alarmMessage 报警消息
    void finishScanSegmentFailure(
        quint16 resultCode,
        quint16 alarmLevel,
        quint16 alarmCode,
        const QString& logMessage,
        const QString& alarmMessage);

    modbus::ModbusService* m_modbus = nullptr;              // Modbus 服务对象指针
    mech_eye::MechEyeService* m_mechEye = nullptr;          // Mech-Eye 相机服务对象指针
    vision::VisionPipelineService* m_visionPipeline = nullptr;  // 视觉编排服务对象指针
    tracking::TrackingService* m_tracking = nullptr;        // 跟踪服务对象指针
    QTimer* m_pollTimer = nullptr;                          // PLC 轮询定时器
    QTimer* m_heartbeatTimer = nullptr;                     // 心跳定时器
    QTimer* m_timeoutTimer = nullptr;                       // 超时定时器
    AppState m_state = AppState::Init;                      // 当前应用状态
    protocol::IpcState m_ipcState = protocol::IpcState::Uninitialized;  // IPC 状态
    protocol::Stage m_currentStage = protocol::Stage::Idle;             // 当前阶段
    ActiveTask m_activeTask;                                // 当前活动任务
    quint16 m_heartbeatCounter = 0;                         // 心跳计数器
    quint16 m_alarmLevel = 0;                               // 报警级别
    quint16 m_alarmCode = 0;                                // 报警代码
    quint16 m_warnCode = 0;                                 // 警告代码
    quint16 m_progress = 0;                                 // 进度百分比
    bool m_dataValid = false;                               // 数据有效标志
    bool m_isPollingPlc = false;                            // 是否正在轮询 PLC
    quint64 m_pollRequestSequence = 0;                      // 轮询请求序号
    quint64 m_activePollRequestSequence = 0;                // 当前未完成轮询请求序号
    QElapsedTimer m_pollRequestTimer;                       // 当前轮询请求耗时计时器
    int m_consecutiveModbusFailures = 0;                    // 连续 Modbus 失败次数
    QVector<quint16> m_lastCommandBlock;                    // 上一次命令块副本
    QMap<int, scan_tracking::mech_eye::CaptureResult> m_segmentCaptureResults;  // 分段采集结果缓存
    QMap<int, scan_tracking::vision::MultiCameraCaptureBundle> m_segmentCaptureBundles;  // 分段视觉 bundle 缓存

    // 允许的最大点云缓存条目数，防止内存无限增长
    static constexpr int kMaxPointCloudCacheSize = 20;
};

}  // namespace flow_control
}  // namespace scan_tracking
