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
#include "scan_tracking/vision/hik_camera_c_controller.h"
#include "scan_tracking/vision/vision_types.h"
#include "scan_tracking/hmi_server/hmi_tcp_server.h"

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
    qInfo(appLog) << "正在初始化模块...";
    int startupStage = 5;
    const QByteArray startupStageEnv = qgetenv("SCAN_TRACKING_STARTUP_STAGE");
	bool startupStageOk = false;
    if (!startupStageEnv.isEmpty()) {
        startupStage = startupStageEnv.toInt(&startupStageOk);
        if (!startupStageOk) {
            startupStage = 5;
        }
    }
    qInfo(appLog) << "启动阶段 =" << startupStage
                  << "(0=Modbus, 1=+MechEye, 2=+Hik, 3=+VisionPipeline, 4=+Tracking, 5=+StateMachine)";

    modbusService_ = std::make_unique<scan_tracking::modbus::ModbusService>(&application_);
    qInfo(appLog) << "Modbus 服务已创建。";

    if (startupStage < 1) {
        qInfo(appLog) << "启动阶段仅到 Modbus。";
        if (!modbusService_->connectDevice()) {
            qWarning(appLog) << "Modbus 连接初始化失败。";
        }
        qInfo(appLog) << "所有模块已初始化。";
        return;
    }
    // MechEye 服务先启动，保证后续状态机和视觉集成层都能复用同一份点云采集入口。
    mechEyeService_ = std::make_unique<scan_tracking::mech_eye::MechEyeService>();

    QObject::connect(
        mechEyeService_.get(),
        &scan_tracking::mech_eye::MechEyeService::stateChanged,
        [](scan_tracking::mech_eye::CameraRuntimeState state, const QString& description) {
            qInfo(appLog) << "[MechEye] 状态 =" << static_cast<int>(state) << description;
        });

    QObject::connect(
        mechEyeService_.get(),
        &scan_tracking::mech_eye::MechEyeService::fatalError,
        [](scan_tracking::mech_eye::CaptureErrorCode code, const QString& message) {
            qCritical(appLog) << "[MechEye] 致命错误：" << static_cast<int>(code) << message;
        });

    mechEyeService_->start();
    qInfo(appLog) << "MechEye 服务已启动。";

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

    // 第三台海康相机（独立用途，不同型号 - 读码相机）
    hikCameraCService_ = std::make_unique<scan_tracking::vision::HikCameraService>(
        QStringLiteral("hik_camera_c"));

    QObject::connect(
        hikCameraCService_.get(),
        &scan_tracking::vision::HikCameraService::stateChanged,
        [](const QString& roleName, const QString& stateText, const QString& description) {
            qInfo(appLog) << "[HikCamera]" << roleName << stateText << description;
        });
    QObject::connect(
        hikCameraCService_.get(),
        &scan_tracking::vision::HikCameraService::fatalError,
        [](scan_tracking::vision::VisionErrorCode code, const QString& message) {
            qCritical(appLog) << "[HikCamera] hik_camera_c 致命错误：" 
                              << static_cast<int>(code) << message;
        });

    hikCameraCService_->start(visionConfig.hikCameraC, visionConfig.hikCaptureTimeoutMs);
    qInfo(appLog) << "HikCamera C 服务已启动（仅连接，不采集）。";

    // 海康相机 C 控制器（独立管理第三台相机 - 智能相机）
    // 使用 TCP 通信协议进行控制和图像获取
    hikCameraCController_ = std::make_unique<scan_tracking::vision::HikCameraCController>(
        hikCameraCService_.get());

    QObject::connect(
        hikCameraCController_.get(),
        &scan_tracking::vision::HikCameraCController::stateChanged,
        [](scan_tracking::vision::HikCameraCState state, const QString& description) {
            qInfo(appLog) << "[HikCameraCController] 状态 =" << static_cast<int>(state) << description;
        });
    QObject::connect(
        hikCameraCController_.get(),
        &scan_tracking::vision::HikCameraCController::fatalError,
        [](scan_tracking::vision::VisionErrorCode code, const QString& message) {
            qCritical(appLog) << "[HikCameraCController] 致命错误：" 
                              << static_cast<int>(code) << message;
        });
    QObject::connect(
        hikCameraCController_.get(),
        &scan_tracking::vision::HikCameraCController::captureCompleted,
        [](scan_tracking::vision::CaptureType type, const QByteArray& imageData) {
            QString typeStr;
            switch (type) {
                case scan_tracking::vision::CaptureType::SurfaceDefect:
                    typeStr = "SurfaceDefect";
                    break;
                case scan_tracking::vision::CaptureType::WeldDefect:
                    typeStr = "WeldDefect";
                    break;
                case scan_tracking::vision::CaptureType::NumberRecognition:
                    typeStr = "NumberRecognition";
                    break;
                default:
                    typeStr = "Unknown";
                    break;
            }
            qInfo(appLog) << "[HikCameraCController] 采集完成：" << typeStr
                          << imageData.size() << "字节";
        });

    hikCameraCController_->start(visionConfig);
    qInfo(appLog) << "HikCamera C 控制器已启动（TCP 通信模式）。";


    // 统一视觉编排层负责把“1 份点云 + 2 份矩阵”收口为一个算法输入包。
    visionPipelineService_ = std::make_unique<scan_tracking::vision::VisionPipelineService>(
        mechEyeService_.get(),
        hikCameraAService_.get(),
        hikCameraBService_.get());

    QObject::connect(
        visionPipelineService_.get(),
        &scan_tracking::vision::VisionPipelineService::stateChanged,
        [](scan_tracking::vision::VisionPipelineState state, const QString& description) {
            qInfo(appLog) << "[VisionPipeline] 状态 =" << static_cast<int>(state) << description;
        });
    QObject::connect(
        visionPipelineService_.get(),
        &scan_tracking::vision::VisionPipelineService::bundleCaptureFinished,
        [](const scan_tracking::vision::MultiCameraCaptureBundle& bundle) {
            qInfo(appLog) << "[VisionPipeline]" << bundle.summary();
        });

    visionPipelineService_->start(visionConfig);
    qInfo(appLog) << "视觉集成框架已启动。";

    trackingService_ = std::make_unique<scan_tracking::tracking::TrackingService>();

    stateMachine_ = std::make_unique<scan_tracking::flow_control::StateMachine>(
        modbusService_.get(),
        mechEyeService_.get(),
        visionPipelineService_.get(),
        trackingService_.get(),
        &application_);
    stateMachine_->start();

    // HMI TCP 服务端：注入所有依赖后启动监听，放在状态机之后以确保信号绑定正确。
    hmiTcpServer_ = std::make_unique<scan_tracking::hmi_server::HmiTcpServer>(9900, &application_);
    hmiTcpServer_->setStateMachine(stateMachine_.get());
    hmiTcpServer_->setModbusService(modbusService_.get());
    hmiTcpServer_->setMechEyeService(mechEyeService_.get());
    hmiTcpServer_->setVisionPipelineService(visionPipelineService_.get());
    hmiTcpServer_->setTrackingService(trackingService_.get());
    hmiTcpServer_->setHikCameraServices(hikCameraAService_.get(), hikCameraBService_.get());
    if (!hmiTcpServer_->start()) {
        qWarning(appLog) << "HMI TCP 服务器在端口 9900 启动失败。";
    } else {
        qInfo(appLog) << "HMI TCP 服务器已在端口 9900 启动。";
    }

    // 等状态机接好信号后再建 Modbus 链路，避免启动期漏掉 connected 事件。
    if (!modbusService_->connectDevice()) {
        qWarning(appLog) << "Modbus 连接初始化失败。";
    }
    qInfo(appLog) << "所有模块已初始化。";
}

void ConsoleRuntime::printStartupStatus()
{
    qInfo(appLog).noquote()
        << "正在启动"
        << QString::fromStdString(scan_tracking::common::ApplicationInfo::name())
        << "版本"
        << QString::fromStdString(scan_tracking::common::ApplicationInfo::version());
    qInfo(appLog).noquote() << "事件循环已激活。按 Ctrl+C 退出。";
}

void ConsoleRuntime::printShutdownStatus()
{
    // 关闭顺序按依赖逆序执行，避免退出过程中还有异步请求落到已析构对象上。
    // HmiTcpServer 必须最先停止：它持有所有其他服务的裸指针，
    // 必须在那些服务析构之前切断 TCP 连接和定时器，防止悬垂指针访问。
    if (hmiTcpServer_) {
        hmiTcpServer_->stop();
        hmiTcpServer_.reset();
    }
    if (stateMachine_) {
        stateMachine_->stop();
    }
    if (visionPipelineService_) {
        visionPipelineService_->stop();
    }
    if (hikCameraCController_) {
        hikCameraCController_->stop();
    }
    if (hikCameraAService_) {
        hikCameraAService_->stop();
    }
    if (hikCameraBService_) {
        hikCameraBService_->stop();
    }
    if (hikCameraCService_) {
        hikCameraCService_->stop();
    }
    if (mechEyeService_) {
        mechEyeService_->stop();
    }
    if (modbusService_) {
        modbusService_->disconnectDevice();
    }
    qInfo(appLog).noquote() << "正在停止扫描跟踪核心框架。";
}

}  // namespace scan_tracking::app
