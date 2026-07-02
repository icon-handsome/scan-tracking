#include "scan_tracking/flow_control/handlers/inspection_handler.h"

#include "scan_tracking/flow_control/state_machine.h"

#include <QtCore/QLoggingCategory>

namespace scan_tracking {
namespace flow_control {

Q_DECLARE_LOGGING_CATEGORY(LOG_FLOW)

const char* InspectionHandler::triggerName() const { return "Trig_Inspection"; }
int InspectionHandler::trigOffset() const { return 24; }
void InspectionHandler::execute(TaskHandlerContext& ctx) { ctx.machine.executeInspectionTask(); }

namespace {
constexpr quint16 kInspectionResOk = 1;
/// IPC 侧处理失败（Tracking 不可用、点云加载失败等），与算法 NG(2) 区分
constexpr quint16 kInspectionResProcessingFail = 7;
}

void StateMachine::executeInspectionTask()
{
    if (m_activeTask.inspectionPathId <= 0) {
        m_activeTask.inspectionPathId = resolvePathIdForInspection();
    }
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("Trig_Inspection 开始，检测路径ID=") << m_activeTask.inspectionPathId
        << multiPathCacheStatusText();

    // 检查跟踪服务是否可用
    if (m_tracking == nullptr) {
        qWarning(LOG_FLOW) << QStringLiteral("Trig_Inspection：Tracking 服务不可用。");
        // 写入默认的检测失败结果
        writeInspectionResult({2, 1u << 4, 0, 0});

        // 演示：tracking 不可用时也向显控推送失败结果（与蓝友出口字段一致）
        if (m_inspectionResultPublisher) {
            tracking::InspectionResult failure;
            failure.resultCode = 2;
            failure.ngReasonWord0 = (1u << 4);
            failure.message = QStringLiteral("综合检测失败：Tracking 服务不可用。");
            m_inspectionResultPublisher(failure);
        }

        completeActiveTask(
            kInspectionResProcessingFail,
            protocol::AckState::Completed,
            false);
        return;
    }

    QString loadError;
    ensurePoseStitchRunRootDirectory();
    persistInspectionPoseStitchOutput(m_activeTask.inspectionPathId);

    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    const auto inspectionType = configManager != nullptr
        ? configManager->inspectionTypeForPath(m_activeTask.inspectionPathId)
        : scan_tracking::common::InspectionType::Bevel;

    tracking::InspectionResult trackingResult;
    int segmentCount = 0;

    if (inspectionType == scan_tracking::common::InspectionType::CodeRead) {
        trackingResult = m_tracking->inspectCodeRead(m_activeTask.inspectionPathId);
    } else if (inspectionType == scan_tracking::common::InspectionType::Defect) {
        trackingResult = m_tracking->inspectSurfaceDefect(m_activeTask.inspectionPathId);
    } else if (inspectionType == scan_tracking::common::InspectionType::Thickness) {
        scan_tracking::mech_eye::PointCloudFrame innerCloud;
        scan_tracking::mech_eye::PointCloudFrame outerCloud;
        int innerPointCount = 0;
        int outerPointCount = 0;
        int innerSegmentIndex = 0;
        int outerSegmentIndex = 0;
        if (!loadThicknessPointCloudsForInspection(
                &innerCloud,
                &outerCloud,
                &innerPointCount,
                &outerPointCount,
                &innerSegmentIndex,
                &outerSegmentIndex,
                &loadError)) {
            qWarning(LOG_FLOW).noquote()
                << QStringLiteral("Trig_Inspection 加载厚度点云失败：") << loadError
                << multiPathCacheStatusText();
            writeInspectionResult({2, 1u << 4, 0, 0});
            if (m_inspectionResultPublisher) {
                tracking::InspectionResult failure;
                failure.resultCode = 2;
                failure.ngReasonWord0 = (1u << 4);
                failure.message = loadError.isEmpty()
                    ? QStringLiteral("综合检测失败：无法加载厚度 inner/outer 点云。")
                    : loadError;
                m_inspectionResultPublisher(failure);
            }
            completeActiveTask(
                kInspectionResProcessingFail,
                protocol::AckState::Completed,
                false);
            clearActiveTask();
            m_ipcState = protocol::IpcState::Ready;
            m_currentStage = protocol::Stage::Idle;
            m_progress = 0;
            setState(AppState::Ready);
            publishIpcStatus();
            return;
        }

        segmentCount = 2;
        trackingResult = m_tracking->inspectThicknessPointClouds(
            innerCloud,
            outerCloud,
            innerPointCount,
            outerPointCount,
            m_activeTask.inspectionPathId);
    } else if (inspectionType == scan_tracking::common::InspectionType::Bevel) {
        QList<scan_tracking::mech_eye::PointCloudFrame> segmentClouds;
        int totalPointCount = 0;
        if (!loadSegmentPointCloudsForInspection(
                &segmentClouds, &totalPointCount, &segmentCount, &loadError)) {
            qWarning(LOG_FLOW).noquote()
                << QStringLiteral("Trig_Inspection 加载坡口分段点云失败：") << loadError
                << multiPathCacheStatusText();
            writeInspectionResult({2, 1u << 4, 0, 0});
            if (m_inspectionResultPublisher) {
                tracking::InspectionResult failure;
                failure.resultCode = 2;
                failure.ngReasonWord0 = (1u << 4);
                failure.message = loadError.isEmpty()
                    ? QStringLiteral("综合检测失败：无法加载坡口分段点云。")
                    : loadError;
                m_inspectionResultPublisher(failure);
            }
            completeActiveTask(
                kInspectionResProcessingFail,
                protocol::AckState::Completed,
                false);
            clearActiveTask();
            m_ipcState = protocol::IpcState::Ready;
            m_currentStage = protocol::Stage::Idle;
            m_progress = 0;
            setState(AppState::Ready);
            publishIpcStatus();
            return;
        }

        trackingResult = m_tracking->inspectBevelPointCloudFramesAveraged(
            segmentClouds, totalPointCount, m_activeTask.inspectionPathId);
    } else if (inspectionType == scan_tracking::common::InspectionType::Hole) {
        QList<scan_tracking::mech_eye::PointCloudFrame> segmentClouds;
        int totalPointCount = 0;
        if (!loadSegmentPointCloudsForInspection(
                &segmentClouds, &totalPointCount, &segmentCount, &loadError)) {
            qWarning(LOG_FLOW).noquote()
                << QStringLiteral("Trig_Inspection 加载 Hole 分段点云失败：") << loadError
                << multiPathCacheStatusText();
            writeInspectionResult({2, 1u << 4, 0, 0});
            if (m_inspectionResultPublisher) {
                tracking::InspectionResult failure;
                failure.resultCode = 2;
                failure.ngReasonWord0 = (1u << 4);
                failure.message = loadError.isEmpty()
                    ? QStringLiteral("综合检测失败：无法加载 Hole 分段点云。")
                    : loadError;
                m_inspectionResultPublisher(failure);
            }
            completeActiveTask(
                kInspectionResProcessingFail,
                protocol::AckState::Completed,
                false);
            clearActiveTask();
            m_ipcState = protocol::IpcState::Ready;
            m_currentStage = protocol::Stage::Idle;
            m_progress = 0;
            setState(AppState::Ready);
            publishIpcStatus();
            return;
        }

        trackingResult = m_tracking->inspectHolePointCloudFrames(
            segmentClouds, totalPointCount, m_activeTask.inspectionPathId);
    } else if (inspectionType == scan_tracking::common::InspectionType::InternalSurface) {
        QList<scan_tracking::mech_eye::PointCloudFrame> segmentClouds;
        int totalPointCount = 0;
        if (!loadSegmentPointCloudsForInspection(
                &segmentClouds, &totalPointCount, &segmentCount, &loadError)) {
            qWarning(LOG_FLOW).noquote()
                << QStringLiteral("Trig_Inspection 加载内表面分段点云失败：") << loadError
                << multiPathCacheStatusText();
            writeInspectionResult({2, 1u << 4, 0, 0});
            if (m_inspectionResultPublisher) {
                tracking::InspectionResult failure;
                failure.resultCode = 2;
                failure.ngReasonWord0 = (1u << 4);
                failure.message = loadError.isEmpty()
                    ? QStringLiteral("综合检测失败：无法加载内表面分段点云。")
                    : loadError;
                m_inspectionResultPublisher(failure);
            }
            completeActiveTask(
                kInspectionResProcessingFail,
                protocol::AckState::Completed,
                false);
            clearActiveTask();
            m_ipcState = protocol::IpcState::Ready;
            m_currentStage = protocol::Stage::Idle;
            m_progress = 0;
            setState(AppState::Ready);
            publishIpcStatus();
            return;
        }

        trackingResult = m_tracking->inspectInternalSurfaceFromSegmentFrames(
            segmentClouds, totalPointCount, m_activeTask.inspectionPathId);
    } else {
        scan_tracking::mech_eye::PointCloudFrame mergedCloud;
        int totalPointCount = 0;
        if (!loadMergedPointCloudForInspection(
                &mergedCloud, &totalPointCount, &segmentCount, &loadError)) {
            qWarning(LOG_FLOW).noquote()
                << QStringLiteral("Trig_Inspection 加载内存点云失败：") << loadError
                << multiPathCacheStatusText();
            writeInspectionResult({2, 1u << 4, 0, 0});
            if (m_inspectionResultPublisher) {
                tracking::InspectionResult failure;
                failure.resultCode = 2;
                failure.ngReasonWord0 = (1u << 4);
                failure.message = loadError.isEmpty()
                    ? QStringLiteral("综合检测失败：无法加载必需分段点云。")
                    : loadError;
                m_inspectionResultPublisher(failure);
            }
            completeActiveTask(
                kInspectionResProcessingFail,
                protocol::AckState::Completed,
                false);
            clearActiveTask();
            m_ipcState = protocol::IpcState::Ready;
            m_currentStage = protocol::Stage::Idle;
            m_progress = 0;
            setState(AppState::Ready);
            publishIpcStatus();
            return;
        }

        trackingResult = m_tracking->inspectPointCloud(
            mergedCloud, totalPointCount, m_activeTask.inspectionPathId);
    }

    InspectionSummary summary;
    summary.resultCode = trackingResult.resultCode;
    summary.ngReasonWord0 = trackingResult.ngReasonWord0;
    summary.ngReasonWord1 = trackingResult.ngReasonWord1;
    summary.measureItemCount = trackingResult.measureItemCount;

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("Trig_Inspection 完成")
        << QStringLiteral(" 参与段数=") << segmentCount
        << QStringLiteral(" 总点数=") << trackingResult.sourcePointCount
        << QStringLiteral(" angleDeg=") << trackingResult.measurement.headAngleTol
        << QStringLiteral(" lengthMm=") << trackingResult.measurement.bluntHeightTol
        << QStringLiteral(" thicknessMm=") << trackingResult.measurement.thicknessMm
        << QStringLiteral(" 说明=") << trackingResult.message;

    writeInspectionResult(summary);

    const quint16 plcRes = summary.resultCode;
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("Trig_Inspection Res_Inspection=") << plcRes
        << QStringLiteral(" ngReasonWord0=") << summary.ngReasonWord0
        << QStringLiteral(" ngReasonWord1=") << summary.ngReasonWord1
        << QStringLiteral(" measureItemCount=") << summary.measureItemCount;
    completeActiveTask(plcRes, protocol::AckState::Completed, plcRes == kInspectionResOk);
    emit inspectionFinished(
        summary.resultCode, summary.ngReasonWord0, summary.ngReasonWord1,
        summary.measureItemCount, trackingResult.measurement, trackingResult.message);
    maybeEmitPresetInspectionDemo(m_activeTask.scanSegmentIndex);
    // 每条路径检测后保留各路径点云，供后续路径继续扫描；整轮结束由 Trig_ResultReset 清空
}


}  // namespace flow_control
}  // namespace scan_tracking
