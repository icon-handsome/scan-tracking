#pragma once

#include <QtCore/QCoreApplication>
#include <memory>

namespace scan_tracking {
namespace modbus { class ModbusService; }
namespace mech_eye { class MechEyeService; }
namespace tracking { class TrackingService; }
namespace flow_control { class StateMachine; }
namespace vision {
class HikCameraService;
class VisionPipelineService;
}
}

namespace scan_tracking::app {

class ConsoleRuntime final {
public:
    explicit ConsoleRuntime(QCoreApplication& application);
    ~ConsoleRuntime();

    int run();

private:
    void printStartupStatus();
    void printShutdownStatus();
    void initModules();

    QCoreApplication& application_;
    std::unique_ptr<scan_tracking::modbus::ModbusService> modbusService_;
    std::unique_ptr<scan_tracking::mech_eye::MechEyeService> mechEyeService_;
    std::unique_ptr<scan_tracking::vision::HikCameraService> hikCameraAService_;
    std::unique_ptr<scan_tracking::vision::HikCameraService> hikCameraBService_;
    std::unique_ptr<scan_tracking::vision::VisionPipelineService> visionPipelineService_;
    std::unique_ptr<scan_tracking::tracking::TrackingService> trackingService_;
    std::unique_ptr<scan_tracking::flow_control::StateMachine> stateMachine_;
};

}
