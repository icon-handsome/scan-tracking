#include "scan_tracking/flow_control/handlers/inspection_handler.h"

#include "scan_tracking/flow_control/state_machine.h"

#include <QtCore/QFileInfo>
#include <QtCore/QLoggingCategory>

#include <future>
#include <thread>

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

tracking::InspectionResult runInternalSurfaceInspectionInWorker(
    scan_tracking::tracking::TrackingService* tracking,
    const QString& mergedPcdPath,
    int inspectionPathId)
{
    if (tracking == nullptr) {
        tracking::InspectionResult failure;
        failure.resultCode = 2;
        failure.ngReasonWord0 = (1u << 4);
        failure.message = QStringLiteral("综合检测失败：Tracking 服务不可用。");
        return failure;
    }

    std::packaged_task<tracking::InspectionResult()> task(
        [tracking, mergedPcdPath, inspectionPathId]() {
            return tracking->inspectInternalSurfaceFromScanFile(
                mergedPcdPath, inspectionPathId, false, false);
        });
    std::future<tracking::InspectionResult> future = task.get_future();
    std::thread worker(std::move(task));
    tracking::InspectionResult result = future.get();
    worker.join();
    return result;
}
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
    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    const auto inspectionType = configManager != nullptr
        ? configManager->inspectionTypeForPath(m_activeTask.inspectionPathId)
        : scan_tracking::common::InspectionType::Bevel;

    if (inspectionType != scan_tracking::common::InspectionType::CodeRead) {
        ensurePoseStitchRunRootDirectory();
        persistInspectionPoseStitchOutput(m_activeTask.inspectionPathId);
    }

    tracking::InspectionResult trackingResult;
    int segmentCount = 0;

    if (inspectionType == scan_tracking::common::InspectionType::CodeRead) {
        trackingResult = m_tracking->finalizeCodeReadInspection(m_activeTask.inspectionPathId);
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
        QString mergedInspectionPcdPath;
        int totalPointCount = 0;
        if (!loadSegmentPointCloudsForInspection(
                nullptr,
                &totalPointCount,
                &segmentCount,
                &loadError,
                &mergedInspectionPcdPath)) {
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

        if (mergedInspectionPcdPath.trimmed().isEmpty()
            || !QFileInfo::exists(mergedInspectionPcdPath)) {
            loadError = QStringLiteral("坡口检测融合点云不存在：%1").arg(mergedInspectionPcdPath);
            qWarning(LOG_FLOW).noquote()
                << QStringLiteral("Trig_Inspection 坡口落盘点云不可用：") << loadError
                << multiPathCacheStatusText();
            writeInspectionResult({2, 1u << 4, 0, 0});
            if (m_inspectionResultPublisher) {
                tracking::InspectionResult failure;
                failure.resultCode = 2;
                failure.ngReasonWord0 = (1u << 4);
                failure.message = loadError;
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

        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("[Bevel] 从落盘融合 PCD 执行测量（避免主线程持有千万级分段点云）")
            << QStringLiteral(" path=") << mergedInspectionPcdPath
            << QStringLiteral(" 总点数=") << totalPointCount
            << QStringLiteral(" 参与段数=") << segmentCount;

        trackingResult = m_tracking->inspectBevelPointCloudFile(
            mergedInspectionPcdPath, m_activeTask.inspectionPathId);
        if (trackingResult.sourcePointCount <= 0 && totalPointCount > 0) {
            trackingResult.sourcePointCount = totalPointCount;
        }
    } else if (inspectionType == scan_tracking::common::InspectionType::Hole) {
        QList<scan_tracking::mech_eye::PointCloudFrame> segmentClouds;
        QStringList segmentPcdPaths;
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

        QString pcdLoadError;
        const bool useSegmentPcdFiles = loadHoleSegmentPcdPathsForInspection(
            &segmentPcdPaths, &totalPointCount, &segmentCount, &pcdLoadError);
        if (useSegmentPcdFiles) {
            trackingResult = m_tracking->inspectHolePointCloudFromSegmentPcdFiles(
                segmentPcdPaths, totalPointCount, m_activeTask.inspectionPathId);
        } else {
            qInfo(LOG_FLOW).noquote()
                << QStringLiteral("[Hole] 落盘分段不可用，回退内存点云：")
                << pcdLoadError;
            trackingResult = m_tracking->inspectHolePointCloudFrames(
                segmentClouds, totalPointCount, m_activeTask.inspectionPathId);
        }
    } else if (inspectionType == scan_tracking::common::InspectionType::InternalSurface) {
        QString mergedInspectionPcdPath;
        int totalPointCount = 0;
        if (!loadSegmentPointCloudsForInspection(
                nullptr,
                &totalPointCount,
                &segmentCount,
                &loadError,
                &mergedInspectionPcdPath)) {
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

        if (mergedInspectionPcdPath.trimmed().isEmpty()
            || !QFileInfo::exists(mergedInspectionPcdPath)) {
            loadError = QStringLiteral("内表面检测融合点云不存在：%1").arg(mergedInspectionPcdPath);
            qWarning(LOG_FLOW).noquote()
                << QStringLiteral("Trig_Inspection 内表面落盘点云不可用：") << loadError
                << multiPathCacheStatusText();
            writeInspectionResult({2, 1u << 4, 0, 0});
            if (m_inspectionResultPublisher) {
                tracking::InspectionResult failure;
                failure.resultCode = 2;
                failure.ngReasonWord0 = (1u << 4);
                failure.message = loadError;
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

        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("[InternalSurface] 从落盘 PCD 执行测量（后台线程，避免主线程大点云堆损坏）")
            << QStringLiteral(" path=") << mergedInspectionPcdPath
            << QStringLiteral(" 总点数=") << totalPointCount
            << QStringLiteral(" 参与段数=") << segmentCount;

        // 落盘完成后释放路径段点云缓存（保留段号进度，避免 PLC 再发段1 被误判为新一轮 path2）。
        clearPathSegmentCache(m_activeTask.inspectionPathId, true);

        trackingResult = runInternalSurfaceInspectionInWorker(
            m_tracking, mergedInspectionPcdPath, m_activeTask.inspectionPathId);
        if (trackingResult.sourcePointCount <= 0 && totalPointCount > 0) {
            trackingResult.sourcePointCount = totalPointCount;
        }
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
        << QStringLiteral(" codeValue=")
        << (trackingResult.codeValue.isEmpty() ? QStringLiteral("<empty>") : trackingResult.codeValue)
        << QStringLiteral(" 说明=") << trackingResult.message;

    writeInspectionResult(summary);

    if (inspectionType == scan_tracking::common::InspectionType::CodeRead
        && m_modbus
        && m_modbus->isConnected()) {
        writeAsciiPlaceholder(
            protocol::registers::kCodeValueAscii,
            protocol::registers::kCodeValueRegisterCount,
            trackingResult.codeValue);
    }

    const quint16 plcRes = summary.resultCode;
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("Trig_Inspection Res_Inspection=") << plcRes
        << QStringLiteral(" ngReasonWord0=") << summary.ngReasonWord0
        << QStringLiteral(" ngReasonWord1=") << summary.ngReasonWord1
        << QStringLiteral(" measureItemCount=") << summary.measureItemCount;
    completeActiveTask(plcRes, protocol::AckState::Completed, plcRes == kInspectionResOk);
    markPathInspectionCompleted(m_activeTask.inspectionPathId);
    if (inspectionType == scan_tracking::common::InspectionType::CodeRead) {
        emit codeReadFinished(summary.resultCode, trackingResult.codeValue);
    }
    emit inspectionFinished(
        summary.resultCode, summary.ngReasonWord0, summary.ngReasonWord1,
        summary.measureItemCount, trackingResult.measurement, trackingResult.message);
    maybeEmitPresetInspectionDemo(m_activeTask.scanSegmentIndex);
    // 每条路径检测后保留各路径点云，供后续路径继续扫描；整轮结束由 Trig_ResultReset 清空
}


}  // namespace flow_control
}  // namespace scan_tracking
