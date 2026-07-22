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
 * 清空所有累积的点云缓存和检测结果，将相关寄存器归零，
 * 为下一轮扫描周期做准备。
 */
void StateMachine::executeResultResetTask()
{
    m_ipcSafetyActionWord = 0;
    m_personZoneAlarmActive = false;

    // 丢弃仍在跑的内表面/坡口后台解算，避免复位后把上一件结果推给 HMI
    ++m_internalSurfaceAsyncGeneration;
    ++m_bevelAsyncGeneration;

    resetScanSegmentCache();  // 清空扫描缓存
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
    // 完成任务，返回成功
    completeActiveTask(1);
    emit resultResetFinished(1);
}


}  // namespace scan_tracking::flow_control
