#pragma once

#include <QtCore/QMap>
#include <QtCore/QElapsedTimer>
#include <QtCore/QObject>
#include <QtCore/QSet>
#include <QtCore/QTimer>
#include <QtCore/QVector>
#include <QtCore/QtGlobal>

#include <atomic>
#include <array>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/flow_control/plc_protocol.h"
#include "scan_tracking/flow_control/task_handler_context.h"
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

class CodeReadHandler;
class InspectionHandler;
class LoadGraspHandler;
class PoseCheckHandler;
class ResultResetHandler;
class ScanSegmentHandler;
class SelfCheckHandler;
class StationMaterialCheckHandler;
class TaskHandlerRegistry;
class UnloadCalcHandler;

/// 分段点云后处理 worker 输出（由主线程 applySegmentProcessOutcome 消费）
struct SegmentProcessOutcome {
    bool success = false;
    int pathId = 0;
    int segmentIndex = 0;
    quint32 taskId = 0;
    quint64 captureRequestId = 0;
    qint64 processElapsedMs = 0;       ///< 后台 refinement 总耗时（含排队、克隆、PCL）
    qint64 pclProcessElapsedMs = 0;    ///< processPointCloudFrame 耗时
    int rawPointCount = 0;
    int processedPointCount = 0;
    QString errorMessage;
    scan_tracking::mech_eye::CaptureResult captureResult;
    std::unique_ptr<scan_tracking::vision::MultiCameraCaptureBundle> bundle;
};

/// HMI 路径级进度事件字段（event.path.*）
struct ScanPathEventInfo {
    int pathId = 0;
    QString pathName;
    int pathIndex = 0;       ///< 在 enabledPathIds 中的序号，1-based
    int pathCount = 0;       ///< 本次任务启用路径总数
    QString inspectionType;  ///< snake_case，如 hole / bevel
    int totalPoints = 0;
    quint16 resultCode = 0;  ///< finished 时 1=成功
};

/// HMI status.system.scanPathProgress 快照
struct ScanPathProgressSnapshot {
    QVector<int> enabledPathIds;
    int currentPathId = 0;
    QString currentPathName;
    QVector<int> completedPathIds;
    int pathCount = 0;
    bool allPathsComplete = false;
};

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

    protocol::IpcState ipcState() const { return m_ipcState; }
    protocol::Stage currentStage() const { return m_currentStage; }
    quint16 alarmLevel() const { return m_alarmLevel; }
    quint16 alarmCode() const { return m_alarmCode; }
    quint16 warnCode() const { return m_warnCode; }
    quint16 progress() const { return m_progress; }

    /// 获取最近一次 PLC 命令块快照（供 HMI 状态推送使用）
    const QVector<quint16>& lastCommandBlock() const { return m_lastCommandBlock; }

    /// PLC 下发的机械臂末端中心点位姿（40029~40040，CDAB FLOAT32）
    protocol::registers::Pose6f robotTcpPose() const { return m_robotTcpPose; }

    /// PLC 转发的埃斯顿机械臂状态字（40018，位定义见 protocol::registers::robot_status_bits）
    quint16 robotStatusWord() const
    {
        return m_lastCommandBlock.value(protocol::registers::kRobotStatusWord, 0);
    }

    /// 注册综合检测结果推送回调（tracking 不可用等路径由状态机补发）
    void setInspectionResultPublisher(std::function<void(const tracking::InspectionResult&)> publisher);

    /// 当前点云缓存中已有的扫描分段索引（升序，供 HMI 调试命令展示）
    QVector<int> cachedScanSegmentIndices() const;

    /**
     * @brief HMI 调试：用缓存点云跑蓝友综合检测
     *
     * 不写 PLC、不清缓存、不占用 PLC 任务槽；结果由调用方推送显控。
     * TODO(multipath): 多路径多点位时须按 pathId 与 inspectionType 筛选/融合缓存，再调用对应检测算法
     */
    tracking::InspectionResult runDebugInspectionOnCachedSegments() const;

    /**
     * @brief 一次性联调：从 [Hole] offlineReplaySessionDir 加载落盘点云并跑 Hole 检测
     *
     * 不写 PLC、不占用 PLC 任务槽；结果写入日志。
     */
    tracking::InspectionResult runOfflineHoleInspectionFromSavedSession();

    /**
     * @brief 一次性联调：从 [InternalSurface] offlineReplayPointCloudPath 加载点云并跑内表面检测
     *
     * 不写 PLC、不占用 PLC 任务槽；结果写入日志。
     */
    tracking::InspectionResult runOfflineInternalSurfaceInspectionFromFile(
        bool emitInspectionSignal = true);

    void deliverOfflineInternalSurfaceInspectionResult(
        const tracking::InspectionResult& result);

    /**
     * @brief 在线 Trig_Inspection 内表面算法完成后的主线程收尾（写 PLC / 推 HMI）
     *
     * 由后台线程经 QueuedConnection 回投；generation 不匹配或任务已结束时丢弃。
     */
    void deliverOnlineInternalSurfaceInspectionResult(
        const tracking::InspectionResult& result,
        int segmentCount,
        quint64 generation);

    /**
     * @brief 一次性联调：从 [Bevel] offlineReplayDataDir 加载 PCD/PLY 点云并跑坡口测量
     *
     * 不写 PLC、不占用 PLC 任务槽；结果写入日志并推送显控（若已连接）。
     */
    tracking::InspectionResult runOfflineBevelInspectionFromDataDir();

    void deliverOfflineBevelInspectionResult(const tracking::InspectionResult& result);

    /**
     * @brief 一次性联调：从 [Thickness] offlineReplayDataDir 加载 inner/outer 点云并跑厚度测量
     *
     * 不写 PLC、不占用 PLC 任务槽；结果写入日志并推送显控（若已连接）。
     */
    tracking::InspectionResult runOfflineThicknessInspectionFromDataDir();

    void deliverOfflineThicknessInspectionResult(const tracking::InspectionResult& result);

    // 设置报警
    // @param level 报警级别
    // @param code 报警代码
    // @param message 报警信息
    void setAlarm(quint16 level, quint16 code, const QString& message);

    /**
     * @brief 显控上报监控区域人员状态，写 IPC_SafetyAction_Word 联动 PLC 停线
     * @param alarm true=有人，false=无人
     * @return 是否成功写入 Modbus 安全动作字（未连接时仍更新本地状态，返回 false）
     */
    bool reportPersonZoneAlarm(bool alarm);

    /// 显控下发的下料区封头计数与叠加配置（经 IPC 写入 PLC 40176~40179）
    struct UnloadAreaConfig {
        quint16 maxStackCount = 0;
        quint16 okCount = 0;
        quint16 ngCount = 0;
        quint16 autoClear = 0;
    };

    /**
     * @brief 更新下料区封头配置并写入 PLC 结果区 40176~40179
     * @return 是否成功写入 Modbus（未连接时仍更新本地缓存，返回 false）
     */
    bool updateUnloadAreaConfig(const UnloadAreaConfig& config);

    UnloadAreaConfig unloadAreaConfig() const;

    /// 显控 cmd.trigger_self_check：进入自检会话并写 IPC_CurrentStage 通知 PLC
    void beginSelfCheckScanSession();

    /// 结束扫描仪自检扫描会话
    void endSelfCheckScanSession();

    bool isSelfCheckSessionActive() const { return m_selfCheckSessionActive; }

    /// 供 HMI status.system.scanPathProgress 使用的路径进度快照
    ScanPathProgressSnapshot scanPathProgressSnapshot() const;

signals:
    // 状态改变信号
    // @param newState 新的状态
    void stateChanged(AppState newState);

    // 协议信号
    // @param message 事件消息
    void protocolEvent(const QString& message);

    // --- HMI 业务事件信号（供 HmiTcpServer 绑定并转发给 Qt 显控） ---

    /// 扫描分段开始
    void scanStarted(int segmentIndex, quint32 taskId);

    /// 扫描分段完成
    void scanFinished(int segmentIndex, quint16 resultCode, int imageCount, int cloudFrameCount);

    /// 扫描路径开始（该路径首段 Trig_ScanSegment）
    void pathStarted(const ScanPathEventInfo& info);

    /// 扫描路径完成（该路径全部段缓存成功）
    void pathFinished(const ScanPathEventInfo& info);

    /// 所有启用路径扫描完成
    void scanPathsAllFinished(const QVector<int>& completedPathIds, int pathCount);

    /// 路径进度复位（Trig_ResultReset / 缓存清空）
    void pathProgressReset(const QString& reason);

    /// 综合检测完成
    void inspectionFinished(quint16 resultCode, quint16 ngReasonWord0, quint16 ngReasonWord1,
                            quint16 measureItemCount,
                            const tracking::InspectionMeasurement& measurement,
                            const QString& message);

    /// 位姿校验完成
    void poseCheckFinished(bool success, quint16 resultCode, double poseDeviationMm,
                           const QVector<double>& rt, const QString& message);

    /// 上料抓取完成
    void loadGraspFinished(quint16 resultCode, float x, float y, float z,
                           float rx, float ry, float rz);

    /// 卸料计算完成
    void unloadCalcFinished(quint16 resultCode, float x, float y, float z,
                            float rx, float ry, float rz);

    /// 自检完成
    void selfCheckFinished(quint16 resultCode, quint16 failWord0);

    /// 条码读取完成
    void codeReadFinished(quint16 resultCode, const QString& codeValue);

    /// 结果复位完成
    void resultResetFinished(quint16 resultCode);

    /// 演示：末路径末段 Trig_Inspection 握手完成，请求 HMI 推送预设 headMetrics
    void presetInspectionDemoRequested(int segmentIndex);

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
    friend class CodeReadHandler;
    friend class InspectionHandler;
    friend class LoadGraspHandler;
    friend class PoseCheckHandler;
    friend class ResultResetHandler;
    friend class ScanSegmentHandler;
    friend class SelfCheckHandler;
    friend class StationMaterialCheckHandler;
    friend class UnloadCalcHandler;
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

    struct LastPoseStitchArtifact {
        bool valid = false;
        bool lbTrackingValid = false;
        int pathId = 0;
        int segmentIndex = 0;
        std::array<float, 16> baseCalibrationT0{};
        std::array<float, 16> calibrationT0Prime{};
        std::array<float, 16> stereoTrackingT{};
        std::array<float, 16> combinedOutputRt{};
        scan_tracking::mech_eye::PointCloudFrame stitchedPointCloud;
    };

    /// 每段位姿拼接矩阵快照（供 Trig_Inspection 落盘给算法反馈）
    struct SegmentPoseStitchRecord {
        bool valid = false;
        bool lbTrackingValid = false;
        bool lbRtGlobalValid = false;
        int pathId = 0;
        int segmentIndex = 0;
        std::array<float, 16> baseCalibrationT0{};
        std::array<float, 16> t0PrimeLbn{};
        std::array<float, 16> lbRtGlobal{};
        std::array<float, 16> appliedT0{};
        std::array<float, 16> stereoTrackingT{};
        std::array<float, 16> combinedOutputRt{};
    };

private:
    // 检测结果汇总结构体 
    struct InspectionSummary {
        quint16 resultCode = 1;       // 结果码
        quint16 ngReasonWord0 = 0;    // NG 原因字 0
        quint16 ngReasonWord1 = 0;    // NG 原因字 1
        quint16 measureItemCount = 0; // 测量项数量
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

    // 当前工位 profile 未启用此触发器时，执行统一拒绝握手（Res=8）
    void rejectDisabledTrigger(const protocol::TriggerDefinition& trigger);

    // 执行当前活动任务
    void executeActiveTask();

    // 算法旁路模式：跳过 Handler/算法，直接回写 PLC 成功结果
    void executeBypassActiveTask();

    // 执行上料抓取任务
    void executeLoadGraspTask();

    // 执行工位检材任务
    void executeStationMaterialCheckTask();

    // 执行卸料计算任务
    void executeUnloadCalcTask();

    // 执行扫描分段任务
    void executeScanSegmentTask();

    // path3/code_read：Trig_ScanSegment 仅发海康 C start，不采集梅卡/CXP
    void executeCodeReadScanSegmentTask(int pathIdForCapture);

    void commitCodeReadScanSegmentComplete(int pathId, int segmentIndex);

    bool isPathCodeReadOnly(int pathId) const;

    /// code_read 已扫满但未 Inspection 时，是否应软屏蔽会切入下一路径的 ScanSegment
    bool shouldSoftShieldPrematureNextPathScan(int segmentIndex) const;

    /// 软屏蔽：Ack/Res 成功握手，不切路径、不采点云
    void softCompletePrematureNextPathScan();

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

    // 程序退出时将 IPC→PLC 结果区寄存器全部清零
    void resetPlcOutputRegisters();

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

    // 分段后台 refinement 完成（主线程更新缓存，不再触发 PLC 握手）
    void onSegmentBackgroundRefinementFinished(SegmentProcessOutcome outcome);

    // 原始点云一次深拷贝入内存并立即完成 Trig_ScanSegment 握手
    void commitScanSegmentCaptureImmediate(
        int pathId,
        int segmentIndex,
        const scan_tracking::mech_eye::CaptureResult& result,
        const scan_tracking::vision::MultiCameraCaptureBundle& bundle);

    // 算法旁路：完成 PLC 握手但不缓存点云（采集后立即丢弃）
    void commitBypassScanSegmentCapture(
        int pathId,
        int segmentIndex,
        int capturedPointCount,
        const scan_tracking::vision::MultiCameraCaptureBundle& bundle);

    // 后台 PCL 点云 refinement（与 PLC 握手并行；PCL 在 processPointCloudFrame 内串行）
    void startSegmentBackgroundRefinement(
        int pathId,
        int segmentIndex,
        quint32 taskId,
        const scan_tracking::common::PointCloudProcessingConfig& config);

    // 将 refinement 结果写回内存缓存
    void applySegmentRefinementOutcome(const SegmentProcessOutcome& outcome);

    // maxWaitMs < 0 表示使用默认上限；综合检测会传入「任务超时 - 余量」避免占满 60s
    void joinAllBackgroundRefinementJobs(int maxWaitMs = -1);

    // 综合检测前从内存缓存合并路径点云（会等待后台 refinement，不可 const）
    bool loadMergedPointCloudForInspection(
        scan_tracking::mech_eye::PointCloudFrame* outCloud,
        int* totalPointCount,
        int* segmentCount,
        QString* errorMessage);

    // 坡口检测：从内存缓存按分段取出点云（不合并，供逐段测量取均值）
    // outSegments 与 outMergedInspectionPcdPath 至少填其一；内表面检测仅需后者
    bool loadSegmentPointCloudsForInspection(
        QList<scan_tracking::mech_eye::PointCloudFrame>* outSegments,
        int* totalPointCount,
        int* segmentCount,
        QString* errorMessage,
        QString* outMergedInspectionPcdPath = nullptr);

    /// Hole 检测：从 session 落盘目录收集各段 pointcloud_stitched 文件路径（不加载点云）
    bool loadHoleSegmentPcdPathsForInspection(
        QStringList* outPcdPaths,
        int* totalPointCount,
        int* segmentCount,
        QString* errorMessage);

    // 厚度检测：按路径配置段号加载 inner/outer 两帧点云
    bool loadThicknessPointCloudsForInspection(
        scan_tracking::mech_eye::PointCloudFrame* outInnerCloud,
        scan_tracking::mech_eye::PointCloudFrame* outOuterCloud,
        int* innerPointCount,
        int* outerPointCount,
        int* innerSegmentIndex,
        int* outerSegmentIndex,
        QString* errorMessage);

    /// 所有启用路径的 1..scanSegmentTotal 段均已缓存
    bool hasAllScanSegmentsCached() const;

    /// 指定路径是否已缓存 1..scanSegmentTotal 全部段
    bool isPathScanComplete(int pathId) const;

    /// 是否存在至少一条已扫满、可执行 Trig_Inspection 的路径
    bool hasPathReadyForInspection() const;

    /// 本次检测应使用的路径 ID（取已扫满的最高路径号）
    int resolvePathIdForInspection() const;

    /// 当前路径是否已扫满 scanSegmentTotal 段（内存段号集合）
    bool isCurrentPathSegmentSetComplete() const;

    /// 获取所有路径的总缓存点云数量
    int totalCachedPointCloudCount() const;

    /// 检查指定路径的指定段号是否已缓存
    bool hasSegmentInPath(int pathId, int segmentIndex) const;
    int selfCheckCachePathId() const;
    void clearPathSegmentCache(int pathId, bool preservePathProgress = false);
    void markPathInspectionCompleted(int pathId);
    bool isPathInspectionCompleted(int pathId) const;
    bool hasSelfCheckCaptureReady() const;

    /// 本次 Trig_ScanSegment 应写入的路径 ID（段号重复且当前路径已满则切下一路径）
    int resolvePathIdForIncomingSegment(int segmentIndex) const;

    /// 多路径缓存进度摘要（日志/拒绝过早检测时使用）
    QString multiPathCacheStatusText() const;

    /// 从命令块选取下一个待处理触发（含 Scan/Inspection 同时为 1 时的优先级）
    const protocol::TriggerDefinition* selectPendingTrigger(const QVector<quint16>& commandBlock) const;

    // 从 scan_paths_config.json 重新加载 T0，并重置当前标定矩阵
    void reloadCalibrationMatricesFromConfig();

    // pathId + segmentIndex 与 scan_paths 中点位对齐时返回 needRotation
    bool resolveNeedRotationForSegment(int pathId, int segmentIndex) const;

    // 转动点位：用 LBN 的 Rt 更新当前标定矩阵 T0' = Rt × T0
    void applyLbnCalibrationUpdate(
        int pathId,
        int segmentIndex,
        bool needRotation,
        const scan_tracking::vision::MultiCameraCaptureBundle& bundle);

    // 将段点云变换到统一坐标系：LB 成功时 p' = p × T0（T0=Rt_global）；否则 p' = p × T0'（LBN 链）
    void applySegmentPoseStitching(int pathId, int segmentIndex);

    void updateLastPoseStitchArtifact(
        int pathId,     
        int segmentIndex,       
        bool lbTrackingValid,
        const std::array<float, 16>& baseCalibrationT0,   // 基准 T0    
        const std::array<float, 16>& calibrationT0Prime,  // T0 或 T0'（LB 成功时为 Rt_global）
        const std::array<float, 16>& stereoTrackingT,     // 恒为单位阵（Rt_global 已作为 T0）
        const std::array<float, 16>& combinedOutputRt,    // 与 calibrationT0Prime 相同（T 为单位阵）
        const scan_tracking::mech_eye::PointCloudFrame& stitchedPointCloud);

    void recordSegmentPoseStitch(
        int pathId,
        int segmentIndex,
        bool lbTrackingValid,
        bool lbRtGlobalValid,
        const std::array<float, 16>& baseCalibrationT0,
        const std::array<float, 16>& t0PrimeLbn,
        const std::array<float, 16>& lbRtGlobal,
        const std::array<float, 16>& appliedT0,
        const std::array<float, 16>& stereoTrackingT,
        const std::array<float, 16>& combinedOutputRt);

    /// Trig_Inspection 前落盘：各段矩阵 + 最终矩阵 + 末段拼接点云
    void persistInspectionPoseStitchOutput(int pathId) const;

    void persistLastPoseStitchArtifactToDisk() const;

    /// 综合检测路径级点云落盘：原始合并 + 拼接后合并（二进制 PLY/PCD）
    void persistPathLevelMergedPointCloudsToDisk(
        int pathId,
        int mergedSegmentCount,
        const scan_tracking::mech_eye::PointCloudFrame& rawMergedCloud,
        const scan_tracking::mech_eye::PointCloudFrame& stitchedMergedCloud) const;

    bool ensureSegmentCaptureExportSessionRoot();
    void exportSegmentCxp2dImages(
        int pathId,
        int segmentIndex,
        const scan_tracking::vision::MultiCameraCaptureBundle& bundle);

    int firstEnabledScanPathId() const;
    int configuredFirstPathPauseAfterPoint() const;
    void maybeLatchFirstPathStepPause(int pathId, int segmentIndex);
    void maybeEmitPresetInspectionDemo(int segmentIndex);
    bool isFirstPathStepPauseBlocking(QString* errorMessage) const;
    void clearFirstPathStepPauseLatch();

    ScanPathEventInfo buildScanPathEventInfo(int pathId, quint16 resultCode = 0) const;
    bool isPathCompleteForProgress(int pathId) const;
    void maybeEmitPathStarted(int pathId);
    void maybeEmitPathFinished(int pathId, quint16 resultCode = 1);
    void clearPathProgressTracking(const QString& resetReason);

    void persistSegmentCaptureExportGroup(
        int pathId,
        int segmentIndex,
        const scan_tracking::vision::MultiCameraCaptureBundle& bundle,
        const scan_tracking::mech_eye::PointCloudFrame& rawPointCloud,
        const scan_tracking::mech_eye::PointCloudFrame& comparisonPointCloud,
        const scan_tracking::mech_eye::PointCloudFrame& stitchedPointCloud,
        const SegmentPoseStitchRecord& stitchRecord);

    void initializePoseStitchRunOutputDirectory();
    bool ensurePoseStitchRunRootDirectory();

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

    // 写入 IPC_SafetyAction_Word (40173)
    bool writeIpcSafetyActionWord();

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
    std::unique_ptr<TaskHandlerRegistry> m_handlerRegistry; // PLC 触发任务 Handler 注册表
    AppState m_state = AppState::Init;                      // 当前应用状态
    protocol::IpcState m_ipcState = protocol::IpcState::Uninitialized;  // IPC 状态
    protocol::Stage m_currentStage = protocol::Stage::Idle;             // 当前阶段
    ActiveTaskState m_activeTask;                           // 当前活动任务
    quint16 m_heartbeatCounter = 0;                         // 心跳计数器
    quint16 m_alarmLevel = 0;                               // 报警级别
    quint16 m_alarmCode = 0;                                // 报警代码
    quint16 m_warnCode = 0;                                 // 警告代码
    quint16 m_progress = 0;                                 // 进度百分比
    quint16 m_ipcSafetyActionWord = 0;                      // IPC_SafetyAction_Word 本地缓存
    bool m_personZoneAlarmActive = false;                   // 是否因人员区域报警处于联锁
    UnloadAreaConfig m_unloadAreaConfig;                    // 下料区封头计数本地缓存（40176~40179）
    bool m_unloadAreaConfigReceived = false;                // 显控是否至少下发过一次下料区配置
    bool m_selfCheckSessionActive = false;                  // 显控触发的自检扫描会话
    bool m_dataValid = false;                               // 数据有效标志
    bool m_isPollingPlc = false;                            // 是否正在轮询 PLC
    quint64 m_pollRequestSequence = 0;                      // 轮询请求序号
    quint64 m_activePollRequestSequence = 0;                // 当前未完成轮询请求序号
    QElapsedTimer m_pollRequestTimer;                       // 当前轮询请求耗时计时器
    int m_consecutiveModbusFailures = 0;                    // 连续 Modbus 失败次数
    QVector<quint16> m_lastCommandBlock;                    // 上一次命令块副本
    protocol::registers::Pose6f m_robotTcpPose;              // 机械臂末端中心点位姿（PLC→IPC）
    
    // === 多路径支持：二维缓存结构（路径ID → 段号 → 数据） ===
    QMap<int, QMap<int, scan_tracking::mech_eye::CaptureResult>> m_pathSegmentCaptureResults;  // 多路径点云缓存
    QMap<int, QMap<int, scan_tracking::mech_eye::PointCloudFrame>> m_pathSegmentRawPointClouds;  // 采集原始点云（位姿拼接前）
    QMap<int, QMap<int, scan_tracking::vision::MultiCameraCaptureBundle>> m_pathSegmentCaptureBundles;  // 多路径视觉 bundle
    QMap<int, QSet<int>> m_codeReadCompletedSegments;  // code_read 路径：已完成 Trig_ScanSegment 的段号
    int m_currentPathId = 1;                                // 当前路径ID（自动递增）
    QSet<int> m_currentPathSegments;                        // 当前路径已缓存的段号集合（用于检测重复）
    QSet<int> m_emittedPathStarted;                         // 本工件已推送 path.started 的路径
    QSet<int> m_emittedPathFinished;                        // 本工件已推送 path.finished 的路径
    QSet<int> m_inspectedPathIds;                           // 已完成 Trig_Inspection 的路径（复位前拒绝再扫）
    bool m_emittedAllPathsFinished = false;                 // 本工件已推送 scan_paths.all_finished
    bool m_firstPathStepPauseLatched = false;               // 路径1联调：已达暂停点，拒绝后续扫描
    int m_firstPathStepPauseAtSegment = 0;                  // 路径1联调：已暂停的点位号
    mutable std::mutex m_segmentCacheMutex;
    std::array<float, 16> m_baseCalibrationMatrix{};       // 基准 T0（来自 scan_paths_config.json）
    std::array<float, 16> m_currentCalibrationMatrix{};    // 当前 T0' / T0''（转动点由 LBN 链式更新）
    QMap<int, QMap<int, std::array<float, 16>>> m_pathSegmentCalibrationMatrices;  // 每路径每段 T0' 快照
    QMap<int, QMap<int, SegmentPoseStitchRecord>> m_pathSegmentPoseStitchRecords;
    LastPoseStitchArtifact m_lastPoseStitchArtifact;
    mutable std::mutex m_lastPoseStitchMutex;
    QString m_poseStitchRunRootDirectory;
    QString m_poseStitchOutputTimestamp;
    QString m_segmentCaptureExportSessionRoot;
    QString m_segmentCaptureExportSessionTimestamp;

    /// 综合检测结果推送（与 TrackingService::InspectionResultNotifier 共用同一回调）
    std::function<void(const tracking::InspectionResult&)> m_inspectionResultPublisher;

    /// 在线内表面异步检测代数：超时/新任务时递增，用于丢弃过期后台结果
    quint64 m_internalSurfaceAsyncGeneration = 0;

    static constexpr int kMaxPointCloudCacheSize = 200;   // 多路径缓存上限（6 路径共 183 段，留余量）
    /// 内表面大点云 ICP/网格化可达数分钟；低于此时长会误超时掐断握手
    static constexpr int kInternalSurfaceTimeoutFloorSeconds = 600;
    static constexpr int kMaxReasonableRefinementJobs = 32;  // 并发 refinement 上限（超出视为逻辑错误）
    static constexpr int kShutdownRefinementJoinTimeoutMs = 3000;

    std::atomic_bool m_stopped{false};

    void registerRefinementJob();
    void completeRefinementJob();
    int pendingRefinementJobCount() const;
    /// 计数越界时强制归零，避免 join 死循环；返回复位前的值
    int reconcilePendingRefinementJobCounter(const char* reason);
    void dispatchSegmentRefinementFinished(SegmentProcessOutcome outcome);

    /// 后台 refinement 在途数（原子计数，避免在 processEvents 重入时锁 std::mutex 崩溃）
    std::atomic_int m_pendingRefinementJobs{0};
};

}  // namespace flow_control
}  // namespace scan_tracking
