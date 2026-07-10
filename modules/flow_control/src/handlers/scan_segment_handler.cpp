#include "scan_tracking/flow_control/handlers/scan_segment_handler.h"

#include "scan_tracking/flow_control/state_machine.h"
#include "scan_tracking/tracking/tracking_service.h"
#include "scan_tracking/vision/vision_pipeline_service.h"

#include <QtCore/QDateTime>
#include <QtCore/QLoggingCategory>

namespace scan_tracking {
namespace flow_control {

Q_DECLARE_LOGGING_CATEGORY(LOG_FLOW)

const char* ScanSegmentHandler::triggerName() const { return "Trig_ScanSegment"; }
int ScanSegmentHandler::trigOffset() const { return 23; }
void ScanSegmentHandler::execute(TaskHandlerContext& ctx) { ctx.machine.executeScanSegmentTask(); }

namespace {
constexpr int kDefaultScanSegmentCaptureTimeoutMs = 30000;

bool isPath4BevelOnly3dMode(int pathId)
{
    if (pathId != 4) {
        return false;
    }
    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    return configManager != nullptr &&
           configManager->inspectionTypeForPath(pathId) == scan_tracking::common::InspectionType::Bevel;
}

bool isPathCodeReadOnly(int pathId)
{
    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    return configManager != nullptr &&
           configManager->inspectionTypeForPath(pathId) == scan_tracking::common::InspectionType::CodeRead;
}
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
    QString validationError;
    if (!validateScanSegmentRequest(m_lastCommandBlock, &validationError)) {
        finishScanSegmentFailure(9, 2, 724, validationError, validationError);
        return;
    }

    const int pathIdForCapture = resolvePathIdForIncomingSegment(m_activeTask.scanSegmentIndex);
    if (isPathCodeReadOnly(pathIdForCapture)) {
        executeCodeReadScanSegmentTask(pathIdForCapture);
        return;
    }

    // 优先走视觉编排层：梅卡点云 + 海康双目黑白图同时采集。
    if (m_visionPipeline == nullptr) {
        finishScanSegmentFailure(
            5,                    // Res 码：5 = 设备未就绪
            3,                    // 报警级别：3 = 严重错误
            720,                  // 报警代码：720 = 视觉编排服务不可用
            QStringLiteral("视觉流水线服务不可用"),
            QStringLiteral("视觉流水线服务不可用"));
        return;
    }

    // 检查相机是否处于就绪状态且当前没有正在进行的采集
    const auto pipelineState = m_visionPipeline->state();
    const bool pipelineStarted = m_visionPipeline->isStarted();
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("[ScanSegment] 视觉流水线前置检查")
        << QStringLiteral(" ptr=") << Qt::hex << reinterpret_cast<quintptr>(m_visionPipeline) << Qt::dec
        << QStringLiteral(" started=") << pipelineStarted
        << QStringLiteral(" state=") << static_cast<int>(pipelineState)
        << QStringLiteral(" (0=Idle,1=Ready,2=Capturing,3=Error,4=Stopped)");
    if (m_mechEye != nullptr) {
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("[ScanSegment] MechEye state=") << static_cast<int>(m_mechEye->state())
            << QStringLiteral(" (3=Ready)");
    }
    if (pipelineState != vision::VisionPipelineState::Ready || pipelineStarted == false) {
        finishScanSegmentFailure(
            5,                    // Res 码：5 = 设备未就绪
            2,                    // 报警级别：2 = 警告
            721,                  // 报警代码：721 = 视觉编排忙或未就绪
            QStringLiteral("视觉流水线忙或未就绪 (started=%1 state=%2)")
                .arg(pipelineStarted ? 1 : 0)
                .arg(static_cast<int>(pipelineState)),
            QStringLiteral("视觉流水线忙或未就绪"));
        return;
    }

    // 计算采集超时时间：优先使用任务指定的超时，否则使用默认值
    const int captureTimeoutMs = m_activeTask.timeoutSeconds > 0
        ? static_cast<int>(m_activeTask.timeoutSeconds) * 1000
        : kDefaultScanSegmentCaptureTimeoutMs;

    const bool forceSelfCheckCapture = isSelfCheckSessionActive();
    const bool needMechEye2D = forceSelfCheckCapture
        ? true
        : resolveNeedRotationForSegment(pathIdForCapture, m_activeTask.scanSegmentIndex);
    const bool skipHikPoseCapture =
        !forceSelfCheckCapture && isPath4BevelOnly3dMode(pathIdForCapture);
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("[ScanSync] 触发") << QDateTime::currentMSecsSinceEpoch();
    const auto mechCaptureMode = needMechEye2D
        ? scan_tracking::mech_eye::CaptureMode::Capture2DAnd3D
        : scan_tracking::mech_eye::CaptureMode::Capture3DOnly;
    const auto visionConfig = scan_tracking::common::ConfigManager::instance()
        ? scan_tracking::common::ConfigManager::instance()->visionConfig()
        : scan_tracking::common::VisionConfig{};
    const quint64 requestId = m_visionPipeline->requestCaptureBundle(
        m_activeTask.scanSegmentIndex,
        m_activeTask.taskId,
        mechCaptureMode,
        visionConfig.mechPointCloudProcessingComparisonEnabled,
        skipHikPoseCapture);

    if (requestId == 0) {
        finishScanSegmentFailure(
            5,                    // Res 码：5 = 设备未就绪
            2,                    // 报警级别：2 = 警告
            721,                  // 报警代码：721 = 视觉编排忙或未就绪
            QStringLiteral("视觉流水线拒绝采集请求"),
            QStringLiteral("视觉流水线忙或未就绪"));
        return;
    }

    // 保存采集请求 ID，用于在回调中匹配响应
    m_activeTask.captureRequestId = requestId;
    m_progress = 30;              // 更新进度为 30%（采集中）
    publishIpcStatus();           // 发布更新的 IPC 状态

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("Trig_ScanSegment 已启动组合采集")
        << QStringLiteral(" 路径=") << pathIdForCapture
        << QStringLiteral(" 段号=") << m_activeTask.scanSegmentIndex
        << QStringLiteral(" 段总数=") << m_activeTask.scanSegmentTotal
        << QStringLiteral(" 需梅卡2D=") << needMechEye2D
        << QStringLiteral(" 跳过CXP=") << skipHikPoseCapture
        << QStringLiteral(" 超时ms=") << captureTimeoutMs;
    maybeEmitPathStarted(pathIdForCapture);
    emit scanStarted(m_activeTask.scanSegmentIndex, m_activeTask.taskId);
}

bool StateMachine::isPathCodeReadOnly(int pathId) const
{
    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    return configManager != nullptr &&
           configManager->inspectionTypeForPath(pathId) == scan_tracking::common::InspectionType::CodeRead;
}

void StateMachine::executeCodeReadScanSegmentTask(int pathIdForCapture)
{
    if (m_tracking == nullptr) {
        finishScanSegmentFailure(
            5,
            3,
            720,
            QStringLiteral("编号识别扫描失败：Tracking 服务不可用"),
            QStringLiteral("编号识别扫描失败：Tracking 服务不可用"));
        return;
    }

    QString errorMessage;
    if (!m_tracking->triggerCodeReadCapture(&errorMessage)) {
        finishScanSegmentFailure(
            5,
            2,
            721,
            errorMessage.isEmpty()
                ? QStringLiteral("编号识别扫描失败：海康 C 未就绪")
                : errorMessage,
            errorMessage.isEmpty()
                ? QStringLiteral("编号识别扫描失败：海康 C 未就绪")
                : errorMessage);
        return;
    }

    m_progress = 100;
    publishIpcStatus();

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("Trig_ScanSegment 编号识别：已发 start，跳过梅卡/CXP")
        << QStringLiteral(" 路径=") << pathIdForCapture
        << QStringLiteral(" 段号=") << m_activeTask.scanSegmentIndex;

    maybeEmitPathStarted(pathIdForCapture);
    emit scanStarted(m_activeTask.scanSegmentIndex, m_activeTask.taskId);
    commitCodeReadScanSegmentComplete(pathIdForCapture, m_activeTask.scanSegmentIndex);
}

void StateMachine::commitCodeReadScanSegmentComplete(int pathId, int segmentIndex)
{
    if (pathId != m_currentPathId) {
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("[多路径] 切换路径（编号识别）")
            << QStringLiteral(" 段号=") << segmentIndex
            << QStringLiteral(" 旧路径ID=") << m_currentPathId
            << QStringLiteral(" 新路径ID=") << pathId;
        m_currentPathId = pathId;
        m_currentPathSegments.clear();
    }

    m_currentPathSegments.insert(segmentIndex);
    {
        std::lock_guard<std::mutex> lock(m_segmentCacheMutex);
        m_codeReadCompletedSegments[pathId].insert(segmentIndex);
    }

    writeScanSegmentResult(segmentIndex, 1, 0);
    completeActiveTask(1);
    emit scanFinished(segmentIndex, 1, 1, 0);

    maybeLatchFirstPathStepPause(pathId, segmentIndex);
    maybeEmitPathFinished(pathId);

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("[CodeRead] 扫描段握手完成")
        << QStringLiteral(" [路径") << pathId << QStringLiteral("][段") << segmentIndex << QStringLiteral("]");
}


}  // namespace flow_control
}  // namespace scan_tracking