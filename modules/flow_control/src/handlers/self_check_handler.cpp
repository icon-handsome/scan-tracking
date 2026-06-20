#include "scan_tracking/flow_control/handlers/self_check_handler.h"

#include "scan_tracking/flow_control/state_machine.h"
#include "scan_tracking/vision/self_check_measurement_adapter.h"

#include <QtCore/QLoggingCategory>

namespace scan_tracking::flow_control {

Q_DECLARE_LOGGING_CATEGORY(LOG_FLOW)

const char* SelfCheckHandler::triggerName() const { return "Trig_SelfCheck"; }
int SelfCheckHandler::trigOffset() const { return 26; }
void SelfCheckHandler::execute(TaskHandlerContext& ctx) { ctx.machine.executeSelfCheckTask(); }

void StateMachine::executeSelfCheckTask()
{
    const int pathId = selfCheckCachePathId();
    const auto* configMgr = common::ConfigManager::instance();
    const int totalPoints = configMgr != nullptr && configMgr->selfCheckConfig().totalPoints > 0
        ? configMgr->selfCheckConfig().totalPoints
        : 2;

    auto finishSelfCheck = [this](quint16 resultCode, quint16 failWord0, quint16 failWord1,
                                  protocol::AckState ackState, const QString& message) {
        if (m_modbus && m_modbus->isConnected()) {
            m_modbus->writeRegisters(protocol::registers::kSelfCheckFailWord0, {failWord0, failWord1});
        }
        if (!message.isEmpty()) {
            if (resultCode == 1) {
                qInfo(LOG_FLOW).noquote() << message;
            } else {
                qWarning(LOG_FLOW).noquote() << message;
            }
        }
        completeActiveTask(resultCode, ackState, resultCode == 1);
        emit selfCheckFinished(resultCode, failWord0);
        endSelfCheckScanSession();
    };

    if (!m_selfCheckSessionActive) {
        finishSelfCheck(
            5,
            static_cast<quint16>(1u << 3),
            0,
            protocol::AckState::Failed,
            QStringLiteral("自检失败：未收到显控自检请求，请先 cmd.trigger_self_check"));
        return;
    }

    if (!hasSelfCheckCaptureReady()) {
        finishSelfCheck(
            5,
            static_cast<quint16>(1u << 3),
            0,
            protocol::AckState::Failed,
            QStringLiteral("自检失败：尚未完成 %1 个自检扫描点（需显控发起后由 PLC Trig_ScanSegment）")
                .arg(totalPoints));
        return;
    }

    scan_tracking::vision::MultiCameraCaptureBundle frame0;
    scan_tracking::vision::MultiCameraCaptureBundle frame1;
    {
        std::lock_guard<std::mutex> lock(m_segmentCacheMutex);
        frame0 = m_pathSegmentCaptureBundles[pathId].value(1);
        frame1 = m_pathSegmentCaptureBundles[pathId].value(totalPoints > 1 ? totalPoints : 2);
    }

    const scan_tracking::vision::self_check::SelfCheckResult result =
        scan_tracking::vision::self_check::runScannerSelfCheck(frame0, frame1);

    const quint16 plcResultCode = result.ok ? static_cast<quint16>(1) : static_cast<quint16>(0);
    finishSelfCheck(
        plcResultCode,
        result.failWord0,
        result.failWord1,
        result.ok ? protocol::AckState::Completed : protocol::AckState::Failed,
        result.message);
}


}  // namespace scan_tracking::flow_control
