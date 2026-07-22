#include "scan_tracking/flow_control/handlers/inspection_handler.h"

#include "scan_tracking/flow_control/state_machine.h"

#include <QtCore/QCoreApplication>
#include <QtCore/QFileInfo>
#include <QtCore/QLoggingCategory>
#include <QtCore/QPointer>

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

        // 演示：tracking 不可用时也向显控推送失败结果（与出口字段一致）
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
        // 与内表面一致：Res=1/Ack=2 仅放行；加载/解算全进后台，避免主线程等 path2 的 PCL 锁。
        const int pathId = m_activeTask.inspectionPathId;
        const int demoSegmentIndex = m_activeTask.scanSegmentIndex;

        InspectionSummary provisional;
        provisional.resultCode = kInspectionResOk;
        provisional.ngReasonWord0 = 0;
        provisional.ngReasonWord1 = 0;
        provisional.measureItemCount = 0;
        writeInspectionResult(provisional);
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("[Bevel] PLC 假 OK 放行 Res=1/Ack=2")
            << QStringLiteral(" pathId=") << pathId
            << QStringLiteral("（真结果仅推 HMI，不回写 PLC）");
        completeActiveTask(kInspectionResOk, protocol::AckState::Completed, true);
        markPathInspectionCompleted(pathId);
        maybeEmitPathFinished(pathId, kInspectionResOk);

        // PLC 可能在后台仍写千万级 PCD / 解算时重复拉 Trig_Inspection。
        // 再启一个后台任务会并发落盘同一批 pathN_merged_*.pcd，易 OOM / 堆损坏闪退。
        const QString algoStatus = m_pathAlgoStatus.value(pathId);
        if (m_activeBevelPathId == pathId
            || algoStatus == QStringLiteral("running")
            || algoStatus == QStringLiteral("done")) {
            qInfo(LOG_FLOW).noquote()
                << QStringLiteral("[Bevel] 跳过重复后台启动 pathId=") << pathId
                << QStringLiteral(" algoStatus=") << (algoStatus.isEmpty() ? QStringLiteral("<empty>") : algoStatus)
                << QStringLiteral(" activeBevelPathId=") << m_activeBevelPathId;
            return;
        }

        notePathAlgoStatus(pathId, QStringLiteral("running"));
        persistWorkpieceCheckpoint("bevel_algo_start");
        m_activeBevelPathId = pathId;

        const quint64 generation = ++m_bevelAsyncGeneration;
        tracking::TrackingService* tracking = m_tracking;
        QPointer<StateMachine> self(this);

        std::thread([self,
                     tracking,
                     pathId,
                     generation,
                     demoSegmentIndex]() {
            tracking::InspectionResult result;
            int asyncSegmentCount = 0;
            if (!self || self->m_stopped.load(std::memory_order_acquire)) {
                return;
            }
            if (tracking == nullptr) {
                result.resultCode = 2;
                result.ngReasonWord0 = (1u << 4);
                result.message = QStringLiteral("综合检测失败：Tracking 服务不可用。");
            } else {
                QString mergedInspectionPcdPath;
                QString loadError;
                int totalPointCount = 0;
                int segmentCount = 0;
                // 后台加载：不等 refinement join；pathId 显式传入（activeTask 可能已收尾）
                const bool loadOk = self->loadSegmentPointCloudsForInspection(
                    nullptr,
                    &totalPointCount,
                    &segmentCount,
                    &loadError,
                    &mergedInspectionPcdPath,
                    pathId,
                    false);
                const bool pcdReady = loadOk
                    && !mergedInspectionPcdPath.trimmed().isEmpty()
                    && QFileInfo::exists(mergedInspectionPcdPath);
                asyncSegmentCount = segmentCount;

                if (!pcdReady) {
                    if (loadError.trimmed().isEmpty()) {
                        loadError = QStringLiteral("坡口检测融合点云不存在：%1")
                                        .arg(mergedInspectionPcdPath);
                    }
                    result.resultCode = 2;
                    result.ngReasonWord0 = (1u << 4);
                    result.message = loadError;
                } else {
                    QMetaObject::invokeMethod(
                        QCoreApplication::instance(),
                        [self, pathId, mergedInspectionPcdPath]() {
                            if (!self) {
                                return;
                            }
                            self->noteMergedInspectionPcd(pathId, mergedInspectionPcdPath);
                            self->persistWorkpieceCheckpoint("bevel_merged_pcd");
                        },
                        Qt::QueuedConnection);
                    qInfo(LOG_FLOW).noquote()
                        << QStringLiteral("[Bevel] 后台解算开始（可排队于内表面算法之后）")
                        << QStringLiteral(" path=") << mergedInspectionPcdPath
                        << QStringLiteral(" 总点数=") << totalPointCount
                        << QStringLiteral(" 参与段数=") << segmentCount;
                    result = tracking->inspectBevelPointCloudFile(
                        mergedInspectionPcdPath, pathId, false);
                    if (result.sourcePointCount <= 0 && totalPointCount > 0) {
                        result.sourcePointCount = totalPointCount;
                    }
                }
            }

            QCoreApplication* app = QCoreApplication::instance();
            if (app == nullptr) {
                return;
            }
            QMetaObject::invokeMethod(
                app,
                [self, result, asyncSegmentCount, generation, demoSegmentIndex]() {
                    if (!self) {
                        return;
                    }
                    self->deliverOnlineBevelInspectionResult(
                        result, asyncSegmentCount, generation, demoSegmentIndex);
                },
                Qt::QueuedConnection);
        }).detach();

        return;
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
        // 与 PLC 约定：握手外形不变，但 Res=1/Ack=2 仅作放行票；真解算只推 HMI。
        const int pathId = m_activeTask.inspectionPathId;
        const int demoSegmentIndex = m_activeTask.scanSegmentIndex;

        auto provisionalCompleteToPlc = [this, pathId]() {
            constexpr quint16 kProvisionalOk = 1;
            InspectionSummary provisional;
            provisional.resultCode = kProvisionalOk;
            provisional.ngReasonWord0 = 0;
            provisional.ngReasonWord1 = 0;
            provisional.measureItemCount = 0;
            writeInspectionResult(provisional);
            qInfo(LOG_FLOW).noquote()
                << QStringLiteral("[InternalSurface] PLC 假 OK 放行 Res=1/Ack=2")
                << QStringLiteral(" pathId=") << pathId
                << QStringLiteral("（真结果仅推 HMI，不回写 PLC）");
            completeActiveTask(kProvisionalOk, protocol::AckState::Completed, true);
            markPathInspectionCompleted(pathId);
            maybeEmitPathFinished(pathId, kProvisionalOk);
        };

        // 先完成握手，再融合点云/解算，避免机构等分钟级算法。
        provisionalCompleteToPlc();

        QString mergedInspectionPcdPath;
        int totalPointCount = 0;
        const bool loadOk = loadSegmentPointCloudsForInspection(
            nullptr,
            &totalPointCount,
            &segmentCount,
            &loadError,
            &mergedInspectionPcdPath);
        const bool pcdReady = loadOk
            && !mergedInspectionPcdPath.trimmed().isEmpty()
            && QFileInfo::exists(mergedInspectionPcdPath);

        // 落盘完成后释放路径段点云缓存（保留段号进度）。
        clearPathSegmentCache(pathId, true);

        if (!pcdReady) {
            if (loadError.trimmed().isEmpty()) {
                loadError = QStringLiteral("内表面检测融合点云不存在：%1")
                                .arg(mergedInspectionPcdPath);
            }
            qWarning(LOG_FLOW).noquote()
                << QStringLiteral("[InternalSurface] 点云不可用，PLC 已假 OK；仅向 HMI 报失败：")
                << loadError << multiPathCacheStatusText();
            tracking::InspectionResult failure;
            failure.resultCode = 2;
            failure.ngReasonWord0 = (1u << 4);
            failure.message = loadError;
            if (m_inspectionResultPublisher) {
                m_inspectionResultPublisher(failure);
            }
            emit inspectionFinished(
                failure.resultCode,
                failure.ngReasonWord0,
                failure.ngReasonWord1,
                failure.measureItemCount,
                failure.measurement,
                failure.message);
            return;
        }

        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("[InternalSurface] 后台解算开始（PLC 已放行）")
            << QStringLiteral(" path=") << mergedInspectionPcdPath
            << QStringLiteral(" 总点数=") << totalPointCount
            << QStringLiteral(" 参与段数=") << segmentCount;

        noteMergedInspectionPcd(pathId, mergedInspectionPcdPath);
        notePathAlgoStatus(pathId, QStringLiteral("running"));
        persistWorkpieceCheckpoint("internal_surface_algo_start");

        startBackgroundInternalSurfaceFromFile(
            pathId,
            mergedInspectionPcdPath,
            segmentCount,
            totalPointCount,
            demoSegmentIndex);

        return;
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
    // code_read 路径：HMI pathFinished 延后到本处；其它路径此前扫满时已推过，此处为 no-op
    maybeEmitPathFinished(m_activeTask.inspectionPathId, plcRes);
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
