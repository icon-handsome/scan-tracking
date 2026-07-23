#include "scan_tracking/flow_control/handlers/result_reset_handler.h"

#include "scan_tracking/flow_control/state_machine.h"
#include "scan_tracking/mech_eye/mech_eye_service.h"
#include "scan_tracking/vision/vision_pipeline_service.h"

#include <QtCore/QLoggingCategory>

namespace scan_tracking::flow_control {

Q_DECLARE_LOGGING_CATEGORY(LOG_FLOW)

const char* ResultResetHandler::triggerName() const { return "Trig_ResultReset"; }
int ResultResetHandler::trigOffset() const { return 28; }
void ResultResetHandler::execute(TaskHandlerContext& ctx) { ctx.machine.executeResultResetTask(); }

/**
 * @brief 执行结果复位任务（Trig_ResultReset）
 *
 * PLC 用此触发显式告知「新工件 / 重头开始」。IPC 侧清点云缓存、已检测路径、
 * Resume 检查点与落盘会话目录，下一轮 Trig_ScanSegment 段号 1 将从 path1 起算。
 */
void StateMachine::executeResultResetTask()
{
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("Trig_ResultReset：进入新工件复位（清缓存/检查点/路径进度）");

    m_ipcSafetyActionWord = 0;
    m_personZoneAlarmActive = false;

    // 丢弃仍在跑的内表面/坡口后台解算，避免复位后把上一件结果推给 HMI
    ++m_internalSurfaceAsyncGeneration;
    ++m_bevelAsyncGeneration;

    resetScanSegmentCache();  // 清空扫描缓存 + 路径回到 1 + 轮换 session/run 目录
    clearWorkpieceCheckpoint("result_reset");
    // 将扫描分段完成索引寄存器清零
    const bool segmentIndexCleared = m_modbus->writeRegisters(protocol::registers::kScanSegmentDoneIndex, {0, 0, 0});
    if (!segmentIndexCleared) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("清除扫描分段完成索引失败");
    }
    // 写入空的检测结果（全零）
    writeInspectionResult({});
    // 清除 IPC 安全动作字
    const bool safetyActionCleared = m_modbus->writeRegisters(protocol::registers::kIpcSafetyActionWord, {0});
    if (!safetyActionCleared) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("清除 IPC 安全动作字失败");
    }

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("Trig_ResultReset 完成：currentPathId=1，后续段号1按 path1 处理");
    completeActiveTask(1);
    emit resultResetFinished(1);
}


}  // namespace scan_tracking::flow_control
