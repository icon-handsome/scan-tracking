#include "scan_tracking/app/console_runtime.h"

#ifdef _WIN32
#include <windows.h>
#endif

#include <QtCore/QCoreApplication>
#include <QtCore/QLoggingCategory>
#include <QtCore/QMetaObject>
#include <QtCore/QString>

#include "scan_tracking/common/application_info.h"
#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/common/logger.h"
#include "scan_tracking/flow_control/state_machine.h"
#include "scan_tracking/mech_eye/mech_eye_service.h"
#include "scan_tracking/mech_eye/mech_eye_types.h"
#include "scan_tracking/modbus/modbus_service.h"
#include "scan_tracking/tracking/tracking_service.h"
#include "scan_tracking/vision/hik_camera_service.h"
#include "scan_tracking/vision/vision_pipeline_service.h"
#include "scan_tracking/vision/vision_types.h"

Q_LOGGING_CATEGORY(appLog, "app")

namespace scan_tracking::app {

namespace {

#ifdef _WIN32
BOOL WINAPI handleConsoleSignal(DWORD signal)
{
    switch (signal) {
    case CTRL_C_EVENT:
    case CTRL_BREAK_EVENT:
    case CTRL_CLOSE_EVENT:
    case CTRL_SHUTDOWN_EVENT:
        if (QCoreApplication::instance() != nullptr) {
            QMetaObject::invokeMethod(
                QCoreApplication::instance(),
                []() { QCoreApplication::quit(); },
                Qt::QueuedConnection);
            return TRUE;
        }
        return FALSE;
    default:
        return FALSE;
    }
}
#endif

}  // namespace

ConsoleRuntime::ConsoleRuntime(QCoreApplication& application)
    : application_(application)
{
}

ConsoleRuntime::~ConsoleRuntime() = default;

int ConsoleRuntime::run()
{
#ifdef _WIN32
    SetConsoleCtrlHandler(handleConsoleSignal, TRUE);
#endif

    QObject::connect(
        &application_,
        &QCoreApplication::aboutToQuit,
        &application_,
        [this]() { printShutdownStatus(); });

    printStartupStatus();
    initModules();
    return application_.exec();
}

void ConsoleRuntime::initModules()
{
    qInfo(appLog) << "Initializing modules...";
    int startupStage = 5;
    const QByteArray startupStageEnv = qgetenv("SCAN_TRACKING_STARTUP_STAGE");
	bool startupStageOk = false;
    if (!startupStageEnv.isEmpty()) {
        startupStage = startupStageEnv.toInt(&startupStageOk);
        if (!startupStageOk) {
            startupStage = 5;
        }
    }
    qInfo(appLog) << "Startup stage =" << startupStage
                  << "(0=Modbus, 1=+MechEye, 2=+Hik, 3=+VisionPipeline, 4=+Tracking, 5=+StateMachine)";

    modbusService_ = std::make_unique<scan_tracking::modbus::ModbusService>(&application_);
    qInfo(appLog) << "Modbus service created.";

    if (startupStage < 1) {
        qInfo(appLog) << "Startup stage stops at Modbus only.";
        if (!modbusService_->connectDevice()) {
            qWarning(appLog) << "Modbus connection initiation failed.";
        }
        qInfo(appLog) << "All modules initialized.";
        return;
    }
    // MechEye 服务先启动，保证后续状态机和视觉集成层都能复用同一份点云采集入口。
    mechEyeService_ = std::make_unique<scan_tracking::mech_eye::MechEyeService>();

    QObject::connect(
        mechEyeService_.get(),
        &scan_tracking::mech_eye::MechEyeService::stateChanged,
        [](scan_tracking::mech_eye::CameraRuntimeState state, const QString& description) {
            qInfo(appLog) << "[MechEye] state =" << static_cast<int>(state) << description;
        });

    QObject::connect(
        mechEyeService_.get(),
        &scan_tracking::mech_eye::MechEyeService::fatalError,
        [](scan_tracking::mech_eye::CaptureErrorCode code, const QString& message) {
            qCritical(appLog) << "[MechEye] fatal error:" << static_cast<int>(code) << message;
        });

    mechEyeService_->start();
    qInfo(appLog) << "MechEye service started.";

    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    const auto visionConfig = configManager != nullptr
        ? configManager->visionConfig()
        : scan_tracking::common::VisionConfig{};

    // 两台海康相机当前只接入服务骨架，不落具体 SDK 调用。
    hikCameraAService_ = std::make_unique<scan_tracking::vision::HikCameraService>(
        QStringLiteral("hik_camera_a"));
    hikCameraBService_ = std::make_unique<scan_tracking::vision::HikCameraService>(
        QStringLiteral("hik_camera_b"));

    QObject::connect(
        hikCameraAService_.get(),
        &scan_tracking::vision::HikCameraService::stateChanged,
        [](const QString& roleName, const QString& stateText, const QString& description) {
            qInfo(appLog) << "[HikCamera]" << roleName << stateText << description;
        });
    QObject::connect(
        hikCameraBService_.get(),
        &scan_tracking::vision::HikCameraService::stateChanged,
        [](const QString& roleName, const QString& stateText, const QString& description) {
            qInfo(appLog) << "[HikCamera]" << roleName << stateText << description;
        });

    hikCameraAService_->start(visionConfig.hikCameraA, visionConfig.hikCaptureTimeoutMs);
    hikCameraBService_->start(visionConfig.hikCameraB, visionConfig.hikCaptureTimeoutMs);

    // 统一视觉编排层负责把“1 份点云 + 2 份矩阵”收口为一个算法输入包。
    visionPipelineService_ = std::make_unique<scan_tracking::vision::VisionPipelineService>(
        mechEyeService_.get(),
        hikCameraAService_.get(),
        hikCameraBService_.get());

    QObject::connect(
        visionPipelineService_.get(),
        &scan_tracking::vision::VisionPipelineService::stateChanged,
        [](scan_tracking::vision::VisionPipelineState state, const QString& description) {
            qInfo(appLog) << "[VisionPipeline] state =" << static_cast<int>(state) << description;
        });
    QObject::connect(
        visionPipelineService_.get(),
        &scan_tracking::vision::VisionPipelineService::bundleCaptureFinished,
        [](const scan_tracking::vision::MultiCameraCaptureBundle& bundle) {
            qInfo(appLog) << "[VisionPipeline]" << bundle.summary();
        });

    visionPipelineService_->start(visionConfig);
    qInfo(appLog) << "Vision integration framework started.";

    modbusService_ = std::make_unique<scan_tracking::modbus::ModbusService>(&application_);
    trackingService_ = std::make_unique<scan_tracking::tracking::TrackingService>();

    stateMachine_ = std::make_unique<scan_tracking::flow_control::StateMachine>(
        modbusService_.get(),
        mechEyeService_.get(),
        visionPipelineService_.get(),
        trackingService_.get(),
        &application_);
    stateMachine_->start();

    // 等状态机接好信号后再建 Modbus 链路，避免启动期漏掉 connected 事件。
    if (!modbusService_->connectDevice()) {
        qWarning(appLog) << "Modbus connection initiation failed.";
    }
    qInfo(appLog) << "All modules initialized.";
}

void ConsoleRuntime::printStartupStatus()
{
    qInfo(appLog).noquote()
        << "Starting"
        << QString::fromStdString(scan_tracking::common::ApplicationInfo::name())
        << "version"
        << QString::fromStdString(scan_tracking::common::ApplicationInfo::version());
    qInfo(appLog).noquote() << "Event loop active. Press Ctrl+C to exit.";
}

void ConsoleRuntime::printShutdownStatus()
{
    // 关闭顺序按依赖逆序执行，避免退出过程中还有异步请求落到已析构对象上。
    if (stateMachine_) {
        stateMachine_->stop();
    }
    if (visionPipelineService_) {
        visionPipelineService_->stop();
    }
    if (hikCameraAService_) {
        hikCameraAService_->stop();
    }
    if (hikCameraBService_) {
        hikCameraBService_->stop();
    }
    if (mechEyeService_) {
        mechEyeService_->stop();
    }
    if (modbusService_) {
        modbusService_->disconnectDevice();
    }
    qInfo(appLog).noquote() << "Stopping Scan Tracking core framework.";
}

}  // namespace scan_tracking::app

