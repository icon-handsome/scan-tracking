#include "scan_tracking/flow_control/handlers/code_read_handler.h"

#include "scan_tracking/flow_control/state_machine.h"
#include "scan_tracking/mech_eye/mech_eye_service.h"
#include "scan_tracking/vision/vision_pipeline_service.h"

#include <QtCore/QLoggingCategory>

namespace scan_tracking::flow_control {

Q_DECLARE_LOGGING_CATEGORY(LOG_FLOW)

const char* CodeReadHandler::triggerName() const { return "Trig_CodeRead"; }
int CodeReadHandler::trigOffset() const { return 27; }
void CodeReadHandler::execute(TaskHandlerContext& ctx) { ctx.machine.executeCodeReadTask(); }

void StateMachine::executeCodeReadTask()
{
    tracking::InspectionResult result;
    if (m_tracking == nullptr) {
        result.resultCode = 2;
        result.ngReasonWord0 = (1u << 4);
        result.message = QStringLiteral("Trig_CodeRead 失败：Tracking 服务不可用。");
    } else {
        result = m_tracking->inspectCodeRead(0, false);
    }

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("收到 Trig_CodeRead，resultCode=") << result.resultCode
        << QStringLiteral(" codeValue=")
        << (result.codeValue.isEmpty() ? QStringLiteral("<empty>") : result.codeValue)
        << QStringLiteral(" message=") << result.message;

    if (m_modbus && m_modbus->isConnected()) {
        writeAsciiPlaceholder(
            protocol::registers::kCodeValueAscii,
            protocol::registers::kCodeValueRegisterCount,
            result.codeValue);
    }
    completeActiveTask(
        result.resultCode,
        protocol::AckState::Completed,
        result.resultCode == 1);
    emit codeReadFinished(result.resultCode, result.codeValue);
}


}  // namespace scan_tracking::flow_control
