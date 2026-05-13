#include "scan_tracking/vision/hik_camera_c_controller.h"

#include <QtCore/QLoggingCategory>

#include "scan_tracking/vision/hik_camera_service.h"

Q_LOGGING_CATEGORY(hikCControllerLog, "vision.hik_camera_c_controller")

namespace scan_tracking {
namespace vision {

void HikCameraCController::registerMetaTypes()
{
    static bool registered = false;
    if (registered) {
        return;
    }
    qRegisterMetaType<scan_tracking::vision::HikCameraCState>("scan_tracking::vision::HikCameraCState");
    registered = true;
}

HikCameraCController::HikCameraCController(
    HikCameraService* hikCameraCService,
    QObject* parent)
    : QObject(parent)
    , m_hikCameraCService(hikCameraCService)
{
    registerMetaTypes();
}

void HikCameraCController::start(const scan_tracking::common::VisionConfig& config)
{
    if (m_started) {
        qWarning(hikCControllerLog) << "HikCameraCController already started.";
        return;
    }

    m_config = config;

    if (m_hikCameraCService == nullptr) {
        setState(HikCameraCState::Error, QStringLiteral("海康相机 C 服务未初始化"));
        emit fatalError(VisionErrorCode::InvalidConfig, QStringLiteral("HikCameraC service is null."));
        return;
    }

    // 连接相机状态变化信号
    connect(
        m_hikCameraCService,
        &HikCameraService::stateChanged,
        this,
        &HikCameraCController::onCameraCStateChanged,
        Qt::QueuedConnection);

    // 连接错误信号
    connect(
        m_hikCameraCService,
        &HikCameraService::fatalError,
        this,
        &HikCameraCController::onCameraError,
        Qt::QueuedConnection);

    m_started = true;
    setState(HikCameraCState::Initializing, QStringLiteral("海康相机 C 控制器正在初始化"));

    qInfo(hikCControllerLog) << "HikCameraCController started with camera:"
                             << m_config.hikCameraC.logicalName
                             << "IP:" << m_config.hikCameraC.ipAddress
                             << "Key:" << m_config.hikCameraC.cameraKey;

    // 检查相机是否已连接
    if (m_hikCameraCService->isConnected()) {
        setState(HikCameraCState::Ready, QStringLiteral("海康相机 C 已就绪"));
    }
}

void HikCameraCController::stop()
{
    if (!m_started) {
        return;
    }

    m_started = false;
    setState(HikCameraCState::Stopped, QStringLiteral("海康相机 C 控制器已停止"));
    qInfo(hikCControllerLog) << "HikCameraCController stopped.";
}

void HikCameraCController::setState(HikCameraCState state, const QString& description)
{
    if (m_state != state) {
        m_state = state;
        emit stateChanged(state, description);
        qInfo(hikCControllerLog) << "State changed to" << static_cast<int>(state) << ":" << description;
    }
}

void HikCameraCController::onCameraCStateChanged(
    QString roleName,
    QString stateText,
    QString description)
{
    qInfo(hikCControllerLog) << "Camera state:" << roleName << stateText << description;

    // 当相机连接成功后，更新控制器状态
    if (stateText == QStringLiteral("ready") && description.contains(QStringLiteral("已连接"))) {
        if (m_state == HikCameraCState::Initializing) {
            setState(HikCameraCState::Ready, QStringLiteral("海康相机 C 已连接并就绪"));
        }
    }
}

void HikCameraCController::onCameraError(
    scan_tracking::vision::VisionErrorCode code,
    QString message)
{
    qWarning(hikCControllerLog) << "Camera error:" << static_cast<int>(code) << message;
    setState(HikCameraCState::Error, QStringLiteral("海康相机 C 错误: %1").arg(message));
    emit fatalError(code, message);
}

}  // namespace vision
}  // namespace scan_tracking
