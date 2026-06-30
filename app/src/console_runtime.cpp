#include "scan_tracking/orbbec_gemini/orbbec_gemini_service.h"
#include "scan_tracking/livox_mid360/livox_mid360_service.h"
#include "scan_tracking/tfmini_plus/tfmini_plus_service.h"
#include "scan_tracking/app/console_runtime.h"

#ifdef _WIN32
#include <windows.h>
#ifdef round
#undef round
#endif
#ifdef Round
#undef Round
#endif
#endif

#include <QtCore/QCoreApplication>
#include <QtCore/QDateTime>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QLoggingCategory>
#include <QtCore/QMetaObject>
#include <QtCore/QPointer>
#include <QtCore/QSettings>
#include <QtCore/QString>
#include <QtCore/QTimer>

#include <thread>

#include "scan_tracking/common/application_info.h"
#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/common/logger.h"
#include "scan_tracking/flow_control/state_machine.h"
#include "scan_tracking/mech_eye/mech_eye_service.h"
#include "scan_tracking/mech_eye/mech_eye_types.h"
#include "scan_tracking/modbus/modbus_service.h"
#include "scan_tracking/tracking/tracking_service.h"
#include "scan_tracking/vision/hik_cxp_camera_service.h"
#include "scan_tracking/vision/vision_pipeline_service.h"
#include "scan_tracking/vision/hik_camera_c_controller.h"
#include "scan_tracking/vision/hik_mono_io.h"
#include "scan_tracking/vision/vision_types.h"
#include "scan_tracking/hmi_server/hmi_tcp_server.h"
#include "scan_tracking/mech_eye/point_cloud_io.h"
#include "scan_tracking/common/capture_cache_paths.h"

Q_LOGGING_CATEGORY(appLog, "app")

namespace scan_tracking::app {

namespace {

#ifdef _WIN32
// Windows 控制台 Ctrl+C / 关闭事件 → 排队调用 QCoreApplication::quit()
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

    // aboutToQuit 时按逆序 stop 各模块（见 printShutdownStatus）
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

    // SCAN_TRACKING_STARTUP_STAGE 用于分步联调：0=仅 Modbus，5=全量模块
    int startupStage = 5;
    const QByteArray startupStageEnv = qgetenv("SCAN_TRACKING_STARTUP_STAGE");
	bool startupStageOk = false;
    if (!startupStageEnv.isEmpty()) {
        startupStage = startupStageEnv.toInt(&startupStageOk);
        if (!startupStageOk) {
            startupStage = 5;
        }
    }
    qInfo(appLog) << QStringLiteral("启动阶段 =") << startupStage
                  << QStringLiteral(" (0=Modbus, 1=+MechEye, 2=+Hik, 3=+VisionPipeline, 4=+Tracking, 5=+StateMachine)");
    qInfo(appLog) << QStringLiteral("第一工位程序启动... ...");
    modbusService_ = std::make_unique<scan_tracking::modbus::ModbusService>(&application_);
    qInfo(appLog) << "Modbus 服务已创建。";

    // stage 0：仅 Modbus，用于 PLC 通信单独验证
    if (startupStage < 1) {
        qInfo(appLog) << "启动阶段仅到 Modbus。";
        if (!modbusService_->connectDevice()) {
            qWarning(appLog) << "Modbus 连接初始化失败。";
        }
        qInfo(appLog) << "所有模块已初始化。";
        return;
    }
    // MechEye 服务先启动，保证后续状态机和视觉集成层都能复用同一份点云采集入口。
    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    if (configManager != nullptr) {
        const auto& profile = configManager->stationProfile();
        qInfo(appLog).noquote()
            << QStringLiteral("[Station] stationId=") << scan_tracking::common::stationIdToInt(profile.stationId)
            << QStringLiteral(" name=") << profile.stationName
            << QStringLiteral(" scanPaths=") << (profile.scanPathsConfigPath.isEmpty()
                                                    ? QStringLiteral("<fallback scan_paths_config.json>")
                                                    : profile.scanPathsConfigPath)
            << QStringLiteral(" workMode=") << scan_tracking::common::workModeIdToString(profile.defaultWorkMode);
        qInfo(appLog).noquote()
            << QStringLiteral("[Station] enableLoadGrasp=") << profile.enableLoadGrasp
            << QStringLiteral(" enableUnloadCalc=") << profile.enableUnloadCalc
            << QStringLiteral(" enablePoseCheck=") << profile.enablePoseCheck
            << QStringLiteral(" enableTelescopicScan=") << profile.enableTelescopicScan
            << QStringLiteral(" enableHoistAssist=") << profile.enableHoistAssist
            << QStringLiteral(" enableCollisionMonitor=") << profile.enableCollisionMonitor
            << QStringLiteral(" (reserved, not enforced in stage1)");
    }

    mechEyeService_ = std::make_unique<scan_tracking::mech_eye::MechEyeService>();

    QObject::connect(
        mechEyeService_.get(),
        &scan_tracking::mech_eye::MechEyeService::stateChanged,
        [](scan_tracking::mech_eye::CameraRuntimeState state, const QString& description) {
            qInfo(appLog) << QStringLiteral("[梅卡] 状态 =") << static_cast<int>(state) << description;
        });

    QObject::connect(
        mechEyeService_.get(),
        &scan_tracking::mech_eye::MechEyeService::fatalError,
        [](scan_tracking::mech_eye::CaptureErrorCode code, const QString& message) {
            qCritical(appLog) << QStringLiteral("[梅卡] 致命错误：") << static_cast<int>(code) << message;
        });

    mechEyeService_->start();
    qInfo(appLog) << QStringLiteral("梅卡相机服务已启动。");

    if (configManager != nullptr) {
        const auto& orbbecConfig = configManager->orbbecGeminiConfig();
        if (!orbbecConfig.enabled) {
            // qInfo(appLog) << QStringLiteral("[OrbbecGemini] disabled (orbbecGeminiEnabled=false)");
        } else {
            orbbecGeminiService_ = std::make_unique<scan_tracking::orbbec_gemini::OrbbecGeminiService>();
            QObject::connect(
                orbbecGeminiService_.get(),
                &scan_tracking::orbbec_gemini::OrbbecGeminiService::logMessage,
                [](const QString& message) {
                    Q_UNUSED(message);
                    // qInfo(appLog).noquote() << message;
                });
            QObject::connect(
                orbbecGeminiService_.get(),
                &scan_tracking::orbbec_gemini::OrbbecGeminiService::stateChanged,
                [](scan_tracking::orbbec_gemini::OrbbecGeminiRuntimeState state,
                   const QString& description) {
                    Q_UNUSED(state);
                    Q_UNUSED(description);
                    // qInfo(appLog).noquote()
                    //     << QStringLiteral("[OrbbecGemini] state=")
                    //     << static_cast<int>(state)
                    //     << description;
                });
            QObject::connect(
                orbbecGeminiService_.get(),
                &scan_tracking::orbbec_gemini::OrbbecGeminiService::captureFinished,
                [](const scan_tracking::orbbec_gemini::OrbbecCaptureResult& result) {
                    if (result.errorCode
                        != scan_tracking::orbbec_gemini::OrbbecCaptureErrorCode::Success) {
                        // qWarning(appLog).noquote()
                        //     << QStringLiteral("[OrbbecGemini] Capture failed:")
                        //     << result.errorMessage;
                        return;
                    }
                    // qInfo(appLog).noquote()
                    //     << QStringLiteral("[OrbbecGemini] Capture saved req=") << result.requestId
                    //     << QStringLiteral(" depthRaw=") << result.depthRawPngPath
                    //     << QStringLiteral(" depthPreview=") << result.depthPreviewPngPath
                    //     << QStringLiteral(" pointCloud=") << result.pointCloudPlyPath;
                });
            QObject::connect(
                orbbecGeminiService_.get(),
                &scan_tracking::orbbec_gemini::OrbbecGeminiService::openFinished,
                [this, orbbecConfig](bool success,
                   scan_tracking::orbbec_gemini::OrbbecGeminiDeviceSummary,
                   const QString& errorMessage) {
                    if (!success && !errorMessage.isEmpty()) {
                        // qWarning(appLog).noquote()
                        //     << QStringLiteral("[OrbbecGemini] Open failed:")
                        //     << errorMessage;
                        return;
                    }
                    if (!success || !orbbecConfig.captureOnStart || !orbbecConfig.saveCaptureToDisk) {
                        return;
                    }
                    if (orbbecGeminiService_ == nullptr) {
                        return;
                    }
                    const quint64 requestId = orbbecGeminiService_->requestCapture(
                        orbbecConfig.captureTimeoutMs,
                        true);
                    if (requestId == 0) {
                        // qWarning(appLog).noquote()
                        //     << QStringLiteral("[OrbbecGemini] Startup capture request rejected");
                    } else {
                        // qInfo(appLog).noquote()
                        //     << QStringLiteral("[OrbbecGemini] Startup capture requested req=")
                        //     << requestId;
                    }
                });
            orbbecGeminiService_->start();
            // qInfo(appLog) << QStringLiteral("[OrbbecGemini] service started.");
        }

        const auto& livoxConfig = configManager->livoxMid360Config();
        if (!livoxConfig.enabled) {
            // qInfo(appLog) << QStringLiteral("[LivoxMid360] disabled (livoxMid360Enabled=false)");
        } else {
            livoxMid360Service_ = std::make_unique<scan_tracking::livox_mid360::LivoxMid360Service>();
            QObject::connect(
                livoxMid360Service_.get(),
                &scan_tracking::livox_mid360::LivoxMid360Service::logMessage,
                [](const QString& message) {
                    Q_UNUSED(message);
                    // qInfo(appLog).noquote() << message;
                });
            QObject::connect(
                livoxMid360Service_.get(),
                &scan_tracking::livox_mid360::LivoxMid360Service::stateChanged,
                [](scan_tracking::livox_mid360::LivoxMid360RuntimeState state,
                   const QString& description) {
                    Q_UNUSED(state);
                    Q_UNUSED(description);
                    // qInfo(appLog).noquote()
                    //     << QStringLiteral("[LivoxMid360] state=")
                    //     << static_cast<int>(state)
                    //     << description;
                });
            QObject::connect(
                livoxMid360Service_.get(),
                &scan_tracking::livox_mid360::LivoxMid360Service::openFinished,
                [](bool success,
                   scan_tracking::livox_mid360::LivoxMid360DeviceSummary,
                   const QString& errorMessage) {
                    if (!success && !errorMessage.isEmpty()) {
                        // qWarning(appLog).noquote()
                        //     << QStringLiteral("[LivoxMid360] Open failed:")
                        //     << errorMessage;
                    }
                });
            livoxMid360Service_->start();
            // qInfo(appLog) << QStringLiteral("[LivoxMid360] service started.");
        }

        const auto& tfminiConfig = configManager->tfminiPlusConfig();
        if (!tfminiConfig.enabled) {
            // qInfo(appLog) << QStringLiteral("[TfminiPlus] disabled (tfminiPlusEnabled=false)");
        } else {
            // 第二工位吊装/内壁防碰辅助测距；Worker 按协议解析后丢弃，不写 PLC 安全位。
            tfminiPlusService_ = std::make_unique<scan_tracking::tfmini_plus::TfminiPlusService>();
            QObject::connect(
                tfminiPlusService_.get(),
                &scan_tracking::tfmini_plus::TfminiPlusService::logMessage,
                [](const QString& message) {
                    Q_UNUSED(message);
                    // qInfo(appLog).noquote() << message;
                });
            QObject::connect(
                tfminiPlusService_.get(),
                &scan_tracking::tfmini_plus::TfminiPlusService::stateChanged,
                [](scan_tracking::tfmini_plus::TfminiPlusRuntimeState state,
                   const QString& description) {
                    Q_UNUSED(state);
                    Q_UNUSED(description);
                    // qInfo(appLog).noquote()
                    //     << QStringLiteral("[TfminiPlus] state=")
                    //     << static_cast<int>(state)
                    //     << description;
                });
            QObject::connect(
                tfminiPlusService_.get(),
                &scan_tracking::tfmini_plus::TfminiPlusService::openFinished,
                [](bool success, const QString& errorMessage) {
                    if (!success && !errorMessage.isEmpty()) {
                        // qWarning(appLog).noquote()
                        //     << QStringLiteral("[TfminiPlus] Open failed:")
                        //     << errorMessage;
                    }
                });
            // TODO: Worker 恢复 emit distanceUpdated 后，在此接入打印/告警/碰撞阈值过滤。
            tfminiPlusService_->start();
            // qInfo(appLog) << QStringLiteral("[TfminiPlus] service started.");
        }
    }

    const auto visionConfig = configManager != nullptr
        ? configManager->visionConfig()
        : scan_tracking::common::VisionConfig{};

    if (startupStage >= 2) {
        if (!visionConfig.hikCxpEnabled) {
            qCritical(appLog) << QStringLiteral("hikCxpEnabled=false，CXP 双目未启动；请在 config.ini [Vision] 中启用。");
        } else {
            // CXP 左/右目各一个 HikCxpCameraService 实例，roleName 区分 ch250_a / ch250_b
            hikCxpCameraAService_ = std::make_unique<scan_tracking::vision::HikCxpCameraService>(
                QStringLiteral("ch250_a"));
            hikCxpCameraBService_ = std::make_unique<scan_tracking::vision::HikCxpCameraService>(
                QStringLiteral("ch250_b"));

            QObject::connect(
                hikCxpCameraAService_.get(),
                &scan_tracking::vision::HikCxpCameraService::stateChanged,
                [](const QString& roleName, const QString& stateText, const QString& description) {
                    qInfo(appLog) << QStringLiteral("[CXP]") << roleName << stateText << description;
                });
            QObject::connect(
                hikCxpCameraBService_.get(),
                &scan_tracking::vision::HikCxpCameraService::stateChanged,
                [](const QString& roleName, const QString& stateText, const QString& description) {
                    qInfo(appLog) << QStringLiteral("[CXP]") << roleName << stateText << description;
                });

            hikCxpCameraAService_->start(
                visionConfig.hikCxpCameraA,
                visionConfig.hikCxpCaptureTimeoutMs,
                visionConfig.hikCxpExposureTimeUs,
                visionConfig.hikCxpGain,
                visionConfig.hikCxpTriggerMode);
            hikCxpCameraBService_->start(
                visionConfig.hikCxpCameraB,
                visionConfig.hikCxpCaptureTimeoutMs,
                visionConfig.hikCxpExposureTimeUs,
                visionConfig.hikCxpGain,
                visionConfig.hikCxpTriggerMode);
            qInfo(appLog) << QStringLiteral("CXP 双目相机服务已启动。");
        }
    }

    if (startupStage < 2) {
        qInfo(appLog) << QStringLiteral("启动阶段仅到 MechEye/辅助传感器。");
        return;
    }

    // MechEye/Hik MVS SDK 与后续 Qt 网络/目录操作错开：延迟到事件循环稳定后再启动流程模块。
    QTimer::singleShot(200, &application_, [this, startupStage, visionConfig]() {
        initFlowModules(startupStage, visionConfig);
    });
}

void ConsoleRuntime::initFlowModules(
    int startupStage,
    scan_tracking::common::VisionConfig visionConfig)
{
    qInfo(appLog) << QStringLiteral("[启动] initFlowModules 开始，stage=") << startupStage;

    // 海康相机 C（智能相机）：纯 TCP 控制，不通过 MVS SDK 打开设备，避免与 SCMVS 冲突
    qInfo(appLog) << QStringLiteral("[启动] 创建 HikCameraCController...");
    hikCameraCController_ = std::make_unique<scan_tracking::vision::HikCameraCController>();
    qInfo(appLog) << QStringLiteral("[启动] HikCameraCController 对象已创建。");

    hikCameraCController_->start(visionConfig);
    qInfo(appLog) << QStringLiteral("海康 C 相机控制器已启动（TCP 通信模式）。");

    if (startupStage < 3) {
        QObject::connect(
            hikCameraCController_.get(),
            &scan_tracking::vision::HikCameraCController::stateChanged,
            [](scan_tracking::vision::HikCameraCState state, const QString& description) {
                qInfo(appLog) << QStringLiteral("[海康C控制器] 状态 =") << static_cast<int>(state) << description;
            });
        QObject::connect(
            hikCameraCController_.get(),
            &scan_tracking::vision::HikCameraCController::fatalError,
            [](scan_tracking::vision::VisionErrorCode code, const QString& message) {
                qCritical(appLog) << QStringLiteral("[海康C控制器] 致命错误：")
                                  << static_cast<int>(code) << message;
            });
        QObject::connect(
            hikCameraCController_.get(),
            &scan_tracking::vision::HikCameraCController::captureCompleted,
            [](scan_tracking::vision::CaptureType type, const QByteArray& imageData) {
                QString typeStr;
                switch (type) {
                    case scan_tracking::vision::CaptureType::SurfaceDefect:
                        typeStr = QStringLiteral("表面缺陷");
                        break;
                    case scan_tracking::vision::CaptureType::WeldDefect:
                        typeStr = QStringLiteral("焊缝缺陷");
                        break;
                    case scan_tracking::vision::CaptureType::NumberRecognition:
                        typeStr = QStringLiteral("编号识别");
                        break;
                    default:
                        typeStr = QStringLiteral("未知");
                        break;
                }
                qInfo(appLog) << QStringLiteral("[海康C控制器] 采集完成：") << typeStr
                              << imageData.size() << QStringLiteral("字节");
            });
        qInfo(appLog) << QStringLiteral("启动阶段仅到 HikCameraC。");
        return;
    }

    // stage>=3：海康 C 信号与视觉流水线均延后，让 FTP/TCP 的 singleShot(0) 先完成。
    QTimer::singleShot(0, &application_, [this, startupStage]() {
        if (hikCameraCController_ != nullptr) {
            QObject::connect(
                hikCameraCController_.get(),
                &scan_tracking::vision::HikCameraCController::stateChanged,
                [](scan_tracking::vision::HikCameraCState state, const QString& description) {
                    qInfo(appLog) << QStringLiteral("[海康C控制器] 状态 =") << static_cast<int>(state) << description;
                });
            QObject::connect(
                hikCameraCController_.get(),
                &scan_tracking::vision::HikCameraCController::fatalError,
                [](scan_tracking::vision::VisionErrorCode code, const QString& message) {
                    qCritical(appLog) << QStringLiteral("[海康C控制器] 致命错误：")
                                      << static_cast<int>(code) << message;
                });
            QObject::connect(
                hikCameraCController_.get(),
                &scan_tracking::vision::HikCameraCController::captureCompleted,
                [](scan_tracking::vision::CaptureType type, const QByteArray& imageData) {
                    qInfo(appLog) << QStringLiteral("[海康C控制器] 采集完成：")
                                  << static_cast<int>(type) << imageData.size() << QStringLiteral("字节");
                });
        }
        const auto* configManager = scan_tracking::common::ConfigManager::instance();
        const auto visionConfig =
            configManager != nullptr ? configManager->visionConfig() : scan_tracking::common::VisionConfig{};
        initVisionFlowModules(startupStage, visionConfig);
    });
    qInfo(appLog) << QStringLiteral("[启动] initFlowModules 已调度视觉模块初始化。");
}

void ConsoleRuntime::initVisionFlowModules(
    int startupStage,
    scan_tracking::common::VisionConfig visionConfig)
{
    qInfo(appLog) << QStringLiteral("[启动] initVisionFlowModules 开始，stage=") << startupStage;

    // 统一视觉编排层负责把“1 份点云 + 2 份矩阵”收口为一个算法输入包。
    qInfo(appLog) << QStringLiteral("[启动] 即将创建 VisionPipelineService...");
    visionPipelineService_ = std::make_unique<scan_tracking::vision::VisionPipelineService>(
        mechEyeService_.get(),
        hikCxpCameraAService_.get(),
        hikCxpCameraBService_.get(),
        &application_);
    qInfo(appLog) << QStringLiteral("[启动] VisionPipelineService 已创建。");

    qInfo(appLog) << QStringLiteral("[启动] 连接 VisionPipelineService 信号...");
    QObject::connect(
        visionPipelineService_.get(),
        &scan_tracking::vision::VisionPipelineService::stateChanged,
        &application_,
        [](scan_tracking::vision::VisionPipelineState state, const QString& description) {
            qInfo(appLog) << QStringLiteral("[视觉流水线] 状态 =") << static_cast<int>(state) << description;
        },
        Qt::QueuedConnection);
    QObject::connect(
        visionPipelineService_.get(),
        &scan_tracking::vision::VisionPipelineService::bundleCaptureFinished,
        &application_,
        [this](const scan_tracking::vision::MultiCameraCaptureBundle& bundle) {
            qInfo(appLog) << QStringLiteral("[视觉流水线]") << bundle.summary();
            if (m_autoLatencyTestPending) {
                onAutoLatencyBundleFinished(bundle);
            }
        });
    qInfo(appLog) << QStringLiteral("[启动] VisionPipelineService 信号已连接。");

    qInfo(appLog) << QStringLiteral("[启动] 调用 VisionPipelineService::start...");
    visionPipelineService_->start(visionConfig);
    qInfo(appLog) << QStringLiteral("视觉集成框架已启动。");

    if (startupStage < 4) {
        qInfo(appLog) << QStringLiteral("启动阶段仅到 VisionPipeline。");
        return;
    }

    qInfo(appLog) << QStringLiteral("[启动] 即将创建 TrackingService...");
    trackingService_ = std::make_unique<scan_tracking::tracking::TrackingService>();
    qInfo(appLog) << QStringLiteral("[启动] TrackingService 已创建。");

    if (startupStage < 5) {
        qInfo(appLog) << QStringLiteral("启动阶段仅到 Tracking。");
        return;
    }

    // StateMachine 是主流程编排核心，注入 Modbus / 视觉 / 跟踪等依赖
    qInfo(appLog) << QStringLiteral("[启动] 即将创建 StateMachine...");
    stateMachine_ = std::make_unique<scan_tracking::flow_control::StateMachine>(
        modbusService_.get(),
        mechEyeService_.get(),
        visionPipelineService_.get(),
        trackingService_.get(),
        &application_);
    qInfo(appLog) << QStringLiteral("[启动] StateMachine 已创建。");

    // HMI：先注入依赖并绑定信号，再 listen / 启动状态机，避免 start() 内重复 connect 或漏接早期事件
    const auto& hmiConfig = scan_tracking::common::ConfigManager::instance()->hmiConfig();
    if (hmiConfig.enabled) {
        qInfo(appLog) << QStringLiteral("[启动] 即将创建 HmiTcpServer...");
        hmiTcpServer_ = std::make_unique<scan_tracking::hmi_server::HmiTcpServer>(
            static_cast<int>(hmiConfig.tcpPort), &application_);
        qInfo(appLog) << QStringLiteral("[启动] HmiTcpServer 已创建，绑定服务信号...");
        hmiTcpServer_->setStateMachine(stateMachine_.get());
        hmiTcpServer_->setModbusService(modbusService_.get());
        hmiTcpServer_->setMechEyeService(mechEyeService_.get());
        hmiTcpServer_->setVisionPipelineService(visionPipelineService_.get());
        hmiTcpServer_->setTrackingService(trackingService_.get());
        hmiTcpServer_->setHikCameraServices(
            hikCxpCameraAService_.get(), hikCxpCameraBService_.get(), nullptr);
        hmiTcpServer_->setHikCameraCController(hikCameraCController_.get());
        hmiTcpServer_->bindServiceSignals();
        qInfo(appLog) << QStringLiteral("[启动] HmiTcpServer 服务信号已绑定。");

        // 坡口测量 inspectPointCloud 返回后立即经 HMI TCP 推送 event.inspection.finished（含失败）
        // 注意：std::function 不可对同一对象连续 std::move，否则 StateMachine 侧会得到空回调并在析构时崩溃。
        const QPointer<scan_tracking::hmi_server::HmiTcpServer> hmiWeak(hmiTcpServer_.get());
        const scan_tracking::tracking::InspectionResultNotifier publishInspectionToHmi =
            [hmiWeak](const scan_tracking::tracking::InspectionResult& inspectionResult) {
                if (hmiWeak) {
                    hmiWeak->publishInspectionResult(inspectionResult);
                }
            };
        trackingService_->setInspectionResultNotifier(publishInspectionToHmi);
        stateMachine_->setInspectionResultPublisher(publishInspectionToHmi);

        if (!hmiTcpServer_->start()) {
            qWarning(appLog) << "HMI TCP 服务器在端口" << hmiConfig.tcpPort << "启动失败。";
        } else {
            qInfo(appLog) << "HMI TCP 服务器已在端口" << hmiConfig.tcpPort << "启动。";
        }
    } else {
        qInfo(appLog) << "HMI TCP 服务已在 config.ini [Hmi] enabled=false 下禁用。";
    }

    stateMachine_->start();
    qInfo(appLog) << QStringLiteral("[启动] StateMachine 已 start。");

    // 等状态机接好信号后再建 Modbus 链路，避免启动期漏掉 connected 事件。
    if (!modbusService_->connectDevice()) {
        qWarning(appLog) << "Modbus 连接初始化失败。";
    }
    qInfo(appLog) << "所有模块已初始化。";

    // 全模块就绪后，可选启动 [Debug] 自动延时测试
    scheduleAutoLatencyBundleTest();
    scheduleOfflineHoleReplay();
    scheduleOfflineInternalSurfaceReplay();
}

void ConsoleRuntime::stopAutoLatencyBundleTest()
{
    if (m_autoLatencyTimer != nullptr) {
        m_autoLatencyTimer->stop();
    }
}

void ConsoleRuntime::scheduleAutoLatencyBundleTest()
{
    if (visionPipelineService_ == nullptr) {
        return;
    }

    // 优先读 exe 上级目录的 config.ini（开发树），否则读 applicationDirPath
    QString iniPath = QCoreApplication::applicationDirPath() + QStringLiteral("/config.ini");
    QDir rootDir(QCoreApplication::applicationDirPath());
    if (rootDir.cdUp() && rootDir.cdUp() && rootDir.cdUp()) {
        const QString projectIni = rootDir.filePath(QStringLiteral("config.ini"));
        if (QFile::exists(projectIni)) {
            iniPath = projectIni;
        }
    }

    QSettings settings(iniPath, QSettings::IniFormat);
    settings.beginGroup(QStringLiteral("Debug"));
    // autoLatencyBundleTestEnabled / DelayMs / Periodic 控制延时测试行为
    const bool enabled = settings.value(QStringLiteral("autoLatencyBundleTestEnabled"), false).toBool();
    const int delayMs = settings.value(QStringLiteral("autoLatencyBundleTestDelayMs"), 20000).toInt();
    const bool periodic = settings.value(QStringLiteral("autoLatencyBundleTestPeriodic"), false).toBool();
    settings.endGroup();

    if (!enabled) {
        return;
    }

    m_autoLatencyPeriodic = periodic;
    m_autoLatencyIntervalMs = delayMs > 0 ? delayMs : 20000;
    m_latencyBundleSeq = 0;

    if (m_autoLatencyTimer == nullptr) {
        m_autoLatencyTimer = new QTimer(&application_);
        QObject::connect(m_autoLatencyTimer, &QTimer::timeout, &application_, [this]() {
            triggerAutoLatencyBundleTest();
        });
    }

    m_autoLatencyTimer->setInterval(m_autoLatencyIntervalMs);

    if (m_autoLatencyPeriodic) {
        m_autoLatencyTimer->setSingleShot(false);
        m_autoLatencyTimer->start();
        qInfo(appLog).noquote()
            << "[LatencyTest] 周期模式已启动：每" << m_autoLatencyIntervalMs
            << "ms 组合拍一次（Mech 2D + 海康 A/B）；关闭周期请设 autoLatencyBundleTestPeriodic=false";
    } else {
        m_autoLatencyTimer->setSingleShot(true);
        m_autoLatencyTimer->start();
        qInfo(appLog).noquote()
            << "[LatencyTest] 单次模式已调度：" << m_autoLatencyIntervalMs
            << "ms 后触发一次（Mech 2D + 海康 A/B）；关闭请设 autoLatencyBundleTestEnabled=false";
    }
}

void ConsoleRuntime::scheduleOfflineHoleReplay()
{
    if (stateMachine_ == nullptr) {
        return;
    }

    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    if (configManager == nullptr || !configManager->holeConfig().offlineReplayEnabled) {
        return;
    }

    const int delayMs = configManager->holeConfig().offlineReplayDelayMs;
    if (m_offlineHoleReplayTimer == nullptr) {
        m_offlineHoleReplayTimer = new QTimer(&application_);
        QObject::connect(m_offlineHoleReplayTimer, &QTimer::timeout, &application_, [this]() {
            triggerOfflineHoleReplay();
        });
    }

    m_offlineHoleReplayTimer->setSingleShot(true);
    m_offlineHoleReplayTimer->setInterval(delayMs > 0 ? delayMs : 5000);
    m_offlineHoleReplayTimer->start();

    qInfo(appLog).noquote()
        << QStringLiteral("[Hole] 离线回放已调度：") << (delayMs > 0 ? delayMs : 5000)
        << QStringLiteral("ms 后从 session 加载点云并执行 Hole 检测；关闭请设 offlineReplayEnabled=false");
}

void ConsoleRuntime::triggerOfflineHoleReplay()
{
    if (stateMachine_ == nullptr) {
        qWarning(appLog) << QStringLiteral("[Hole] 离线回放跳过：StateMachine 不可用。");
        return;
    }

    const tracking::InspectionResult result = stateMachine_->runOfflineHoleInspectionFromSavedSession();
    qInfo(appLog).noquote()
        << QStringLiteral("[Hole] 离线回放结果 resultCode=") << result.resultCode
        << QStringLiteral(" message=") << result.message;
}

void ConsoleRuntime::scheduleOfflineInternalSurfaceReplay()
{
    if (stateMachine_ == nullptr) {
        return;
    }

    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    if (configManager == nullptr || !configManager->internalSurfaceConfig().offlineReplayEnabled) {
        return;
    }

    const int delayMs = configManager->internalSurfaceConfig().offlineReplayDelayMs;
    if (m_offlineInternalSurfaceReplayTimer == nullptr) {
        m_offlineInternalSurfaceReplayTimer = new QTimer(&application_);
        QObject::connect(
            m_offlineInternalSurfaceReplayTimer,
            &QTimer::timeout,
            &application_,
            [this]() { triggerOfflineInternalSurfaceReplay(); });
    }

    m_offlineInternalSurfaceReplayTimer->setSingleShot(true);
    m_offlineInternalSurfaceReplayTimer->setInterval(delayMs > 0 ? delayMs : 5000);
    m_offlineInternalSurfaceReplayTimer->start();

    qInfo(appLog).noquote()
        << QStringLiteral("[InternalSurface] 离线回放已调度：") << (delayMs > 0 ? delayMs : 5000)
        << QStringLiteral(
               "ms 后加载点云并执行内表面检测；关闭请设 offlineReplayEnabled=false");
}

void ConsoleRuntime::triggerOfflineInternalSurfaceReplay()
{
    if (stateMachine_ == nullptr) {
        qWarning(appLog) << QStringLiteral("[InternalSurface] 离线回放跳过：StateMachine 不可用。");
        return;
    }

    if (m_offlineInternalSurfaceReplayInFlight) {
        qWarning(appLog) << QStringLiteral("[InternalSurface] 离线回放仍在执行，跳过重复触发。");
        return;
    }

    m_offlineInternalSurfaceReplayInFlight = true;
    qInfo(appLog).noquote()
        << QStringLiteral("[InternalSurface] 离线回放后台线程启动（避免主线程大点云处理崩溃）");

    auto* stateMachine = stateMachine_.get();
    std::thread([this, stateMachine]() {
        const tracking::InspectionResult result =
            stateMachine->runOfflineInternalSurfaceInspectionFromFile(false);

        QMetaObject::invokeMethod(
            &application_,
            [this, stateMachine, result]() {
                m_offlineInternalSurfaceReplayInFlight = false;
                stateMachine->deliverOfflineInternalSurfaceInspectionResult(result);
                qInfo(appLog).noquote()
                    << QStringLiteral("[InternalSurface] 离线回放结果 resultCode=")
                    << result.resultCode << QStringLiteral(" message=") << result.message;
            },
            Qt::QueuedConnection);
    }).detach();
}

void ConsoleRuntime::triggerAutoLatencyBundleTest()
{
    if (visionPipelineService_ == nullptr || !visionPipelineService_->isStarted()) {
        qWarning(appLog) << "[LatencyTest] 视觉流水线未就绪，跳过自动采集。";
        return;
    }

    // 上一轮未完成时不重叠触发，避免 m_pending 冲突
    if (m_autoLatencyTestPending) {
        const QString modeText =
            m_autoLatencyPeriodic ? QStringLiteral("周期") : QStringLiteral("单次");
        qWarning(appLog).noquote()
            << QStringLiteral("[LatencyTest] 上一轮采集尚未结束，跳过本次%1触发").arg(modeText);
        return;
    }

    const qint64 triggerMs = QDateTime::currentMSecsSinceEpoch();
    m_autoLatencyTriggerMs = triggerMs;
    m_autoLatencyTestPending = true;

    constexpr int kTestSegmentIndex = 1;
    const quint32 roundIndex = m_latencyBundleSeq;
    const quint32 taskId = 90001u + roundIndex;
    m_latencyBundleSeq = roundIndex + 1u;

    const QString periodicText = m_autoLatencyPeriodic ? QStringLiteral("true") : QStringLiteral("false");

    qInfo(appLog).noquote()
        << QStringLiteral("[ScanSync] 触发 %1（自动延时测试 round=%2）")
               .arg(triggerMs)
               .arg(roundIndex);

    // 仅 Mech 2D 模式，不测 3D 点云；segment=1 为固定测试段号
    const quint64 requestId = visionPipelineService_->requestCaptureBundle(
        kTestSegmentIndex,
        taskId,
        scan_tracking::mech_eye::CaptureMode::Capture2DOnly);

    if (requestId == 0) {
        m_autoLatencyTestPending = false;
        qWarning(appLog) << "[LatencyTest] 组合采集请求被拒绝。";
        return;
    }

    qInfo(appLog).noquote()
        << QStringLiteral(
               "[LatencyTest] 已发送组合采集 requestId=%1 segment=%2 taskId=%3 mechMode=仅2D periodic=%4")
               .arg(requestId)
               .arg(kTestSegmentIndex)
               .arg(taskId)
               .arg(periodicText);
}

void ConsoleRuntime::onAutoLatencyBundleFinished(
    const scan_tracking::vision::MultiCameraCaptureBundle& bundle)
{
    m_autoLatencyTestPending = false;

    const qint64 triggerMs = m_autoLatencyTriggerMs;
    const quint32 roundCount = m_latencyBundleSeq;
    quint32 roundIndex = 0u;
    quint32 taskId = 90001u;
    if (roundCount > 0u) {
        roundIndex = roundCount - 1u;
        taskId = 90000u + roundCount;
    }

    const qint64 finishedMs = QDateTime::currentMSecsSinceEpoch();
    qint64 wallMs = -1;
    if (triggerMs > 0) {
        wallMs = finishedMs - triggerMs; // 从 requestCaptureBundle 到 bundle 完成的墙钟耗时
    }

    // 分别汇总 Mech 2D 与海康 A/B 的成功标志、耗时、分辨率、时间戳
    const bool mechOk = bundle.mechEyeResult.success();
    const qint64 mechElapsedMs = bundle.mechEyeResult.elapsedMs;
    const int mechTexW = bundle.mechEyeResult.texture2D.width;
    const int mechTexH = bundle.mechEyeResult.texture2D.height;

    const bool hikAOk = bundle.hikCameraAResult.success();
    const qint64 hikAElapsedMs = bundle.hikCameraAResult.elapsedMs;
    const int hikAW = bundle.hikCameraAResult.frame.width;
    const int hikAH = bundle.hikCameraAResult.frame.height;
    const qint64 hikATs = bundle.hikCameraAResult.frame.timestampMs;

    const bool hikBOk = bundle.hikCameraBResult.success();
    const qint64 hikBElapsedMs = bundle.hikCameraBResult.elapsedMs;
    const int hikBW = bundle.hikCameraBResult.frame.width;
    const int hikBH = bundle.hikCameraBResult.frame.height;
    const qint64 hikBTs = bundle.hikCameraBResult.frame.timestampMs;

    qInfo(appLog).noquote()
        << QStringLiteral("[LatencyTest] ========== 延时汇总 round=%1 ==========").arg(roundIndex);
    qInfo(appLog).noquote()
        << QStringLiteral("[LatencyTest] 触发时刻 triggerMs=%1 完成时刻 finishedMs=%2 墙钟总耗时 wallMs=%3")
               .arg(triggerMs)
               .arg(finishedMs)
               .arg(wallMs);
    qInfo(appLog).noquote()
        << QStringLiteral("[LatencyTest] Mech 2D：成功=%1 耗时ms=%2 纹理=%3x%4")
               .arg(mechOk)
               .arg(mechElapsedMs)
               .arg(mechTexW)
               .arg(mechTexH);
    qInfo(appLog).noquote()
        << QStringLiteral("[LatencyTest] 海康 A：成功=%1 耗时ms=%2 帧=%3x%4 时间戳ms=%5")
               .arg(hikAOk)
               .arg(hikAElapsedMs)
               .arg(hikAW)
               .arg(hikAH)
               .arg(hikATs);
    qInfo(appLog).noquote()
        << QStringLiteral("[LatencyTest] 海康 B：成功=%1 耗时ms=%2 帧=%3x%4 时间戳ms=%5")
               .arg(hikBOk)
               .arg(hikBElapsedMs)
               .arg(hikBW)
               .arg(hikBH)
               .arg(hikBTs);

    if (triggerMs > 0) {
        if (hikATs > 0) {
            qInfo(appLog).noquote()
                << QStringLiteral("[LatencyTest] Hik A 相对触发 deltaMs=%1").arg(hikATs - triggerMs);
        }
        if (hikBTs > 0) {
            qInfo(appLog).noquote()
                << QStringLiteral("[LatencyTest] Hik B 相对触发 deltaMs=%1").arg(hikBTs - triggerMs);
        }
    }

    qInfo(appLog).noquote()
        << QStringLiteral("[LatencyTest] 日志中搜索 [ScanSync] 可对比触发时刻 / 梅卡 / 海康_a / 海康_b 的 epoch ms");
    qInfo(appLog).noquote() << QStringLiteral("[LatencyTest] ================================");

    // 将 Mech 2D 纹理与海康 A/B 单目帧落盘，便于离线分析延时与图像质量
    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    const QString configuredRoot = configManager != nullptr
        ? configManager->flowControlConfig().scanCacheDirectory
        : QString();
    const QString cacheRoot = scan_tracking::common::resolveCaptureCacheRoot(configuredRoot);
    const QString timestamp = scan_tracking::common::buildCaptureTimestamp();
    constexpr int kTestSegmentIndex = 1;
    QString mechPngPath;
    QString hikAPath;
    QString hikBPath;

    if (bundle.mechEyeResult.texture2D.isValid()) {
        mechPngPath = scan_tracking::mech_eye::buildSegmentMech2DPngPath(
            configuredRoot, kTestSegmentIndex, taskId, timestamp);
        scan_tracking::mech_eye::saveGrayTextureFrameToPng(
            bundle.mechEyeResult.texture2D, mechPngPath);
    }

    if (bundle.hikCameraAResult.success() && bundle.hikCameraAResult.frame.isValid()) {
        hikAPath = scan_tracking::vision::buildSegmentHikMonoPath(
            configuredRoot, kTestSegmentIndex, taskId, QStringLiteral("hikA"), timestamp);
        scan_tracking::vision::saveHikMonoFrameToBmp(
            bundle.hikCameraAResult.frame, hikAPath);
    }

    if (bundle.hikCameraBResult.success() && bundle.hikCameraBResult.frame.isValid()) {
        hikBPath = scan_tracking::vision::buildSegmentHikMonoPath(
            configuredRoot, kTestSegmentIndex, taskId, QStringLiteral("hikB"), timestamp);
        scan_tracking::vision::saveHikMonoFrameToBmp(
            bundle.hikCameraBResult.frame, hikBPath);
    }

    qInfo(appLog).noquote()
        << QStringLiteral("[LatencyTest] 落盘目录 cacheRoot=%1 mech2d=%2 海康A=%3 海康B=%4")
               .arg(cacheRoot, mechPngPath, hikAPath, hikBPath);
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
    stopAutoLatencyBundleTest();

    // 关闭顺序按依赖逆序执行，避免退出过程中还有异步请求落到已析构对象上。
    // HmiTcpServer 必须最先停止：它持有所有其他服务的裸指针，
    // 必须在那些服务析构之前切断 TCP 连接和定时器，防止悬垂指针访问。
    if (hmiTcpServer_) {
        hmiTcpServer_->stop();
        hmiTcpServer_.reset();
    }
    if (stateMachine_) {
        stateMachine_->stop();
        stateMachine_.reset();
    }
    if (visionPipelineService_) {
        visionPipelineService_->stop();
    }
    if (hikCameraCController_) {
        hikCameraCController_->stop();
    }
    if (hikCxpCameraAService_) {
        hikCxpCameraAService_->stop();
    }
    if (hikCxpCameraBService_) {
        hikCxpCameraBService_->stop();
    }
    if (orbbecGeminiService_) {
        orbbecGeminiService_->stop();
    }
    if (livoxMid360Service_) {
        livoxMid360Service_->stop();
    }
    if (tfminiPlusService_) {
        tfminiPlusService_->stop();
    }
    if (mechEyeService_) {
        mechEyeService_->stop();
    }
    if (modbusService_) {
        if (modbusService_->isConnected()) {
            modbusService_->resetIpcResultBlock();
        }
        modbusService_->disconnectDevice();
    }
    qInfo(appLog).noquote() << "正在停止扫描跟踪核心框架。";
}

}  // namespace scan_tracking::app
