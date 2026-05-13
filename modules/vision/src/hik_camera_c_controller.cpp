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
    , m_autoTestTimer(new QTimer(this))
{
    registerMetaTypes();
    
    // 配置自动测试定时器
    m_autoTestTimer->setSingleShot(true);
    connect(m_autoTestTimer, &QTimer::timeout, this, &HikCameraCController::onAutoTestCaptureTimer);
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

    // 连接采图完成信号
    connect(
        m_hikCameraCService,
        &HikCameraService::poseCaptureFinished,
        this,
        &HikCameraCController::onCameraCaptureFinished,
        Qt::QueuedConnection);

    // 连接错误信号
    connect(
        m_hikCameraCService,
        &HikCameraService::fatalError,
        this,
        &HikCameraCController::onCameraCaptureFailed,
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

quint64 HikCameraCController::requestTestCapture()
{
    if (!m_started) {
        qWarning(hikCControllerLog) << "Cannot request capture: controller not started.";
        emit fatalError(VisionErrorCode::NotStarted, QStringLiteral("控制器未启动"));
        return 0;
    }

    if (m_hikCameraCService == nullptr) {
        qWarning(hikCControllerLog) << "Cannot request capture: camera service is null.";
        emit fatalError(VisionErrorCode::InvalidConfig, QStringLiteral("相机服务未初始化"));
        return 0;
    }

    setState(HikCameraCState::Capturing, QStringLiteral("海康相机 C 正在采图"));

    const quint64 requestId = m_hikCameraCService->requestPoseCapture(
        m_config.hikCameraC.cameraKey,
        m_config.hikCaptureTimeoutMs);

    if (requestId == 0) {
        setState(HikCameraCState::Error, QStringLiteral("海康相机 C 采图请求失败"));
        qWarning(hikCControllerLog) << "Failed to request capture from camera C.";
        return 0;
    }

    qInfo(hikCControllerLog) << "Test capture requested, requestId:" << requestId;
    return requestId;
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
            
            // 自动触发一次测试采图
            if (m_autoTestEnabled && !m_autoTestTriggered) {
                qInfo(hikCControllerLog) << "Auto test capture will be triggered in 2 seconds...";
                m_autoTestTimer->start(2000);  // 2秒后自动采图
            }
        } else if (m_state == HikCameraCState::Capturing) {
            setState(HikCameraCState::Ready, QStringLiteral("海康相机 C 采图完成，已就绪"));
        }
    }
}

void HikCameraCController::onCameraCaptureFailed(
    scan_tracking::vision::VisionErrorCode code,
    QString message)
{
    qWarning(hikCControllerLog) << "Camera capture failed:" << static_cast<int>(code) << message;
    setState(HikCameraCState::Error, QStringLiteral("海康相机 C 采图失败: %1").arg(message));
    emit fatalError(code, message);
}

void HikCameraCController::onCameraCaptureFinished(
    scan_tracking::vision::HikPoseCaptureResult result)
{
    qInfo(hikCControllerLog) << "Capture finished, requestId:" << result.requestId
                             << "success:" << result.success()
                             << "frame:" << result.frame.width << "x" << result.frame.height
                             << "elapsed:" << result.elapsedMs << "ms";

    if (result.success()) {
        setState(HikCameraCState::Ready, QStringLiteral("海康相机 C 采图成功"));
        qInfo(hikCControllerLog) << "=== 采图成功详情 ===";
        qInfo(hikCControllerLog) << "  相机: " << result.logicalName << "(" << result.cameraKey << ")";
        qInfo(hikCControllerLog) << "  分辨率: " << result.frame.width << "x" << result.frame.height;
        qInfo(hikCControllerLog) << "  帧ID: " << result.frame.frameId;
        qInfo(hikCControllerLog) << "  像素格式: " << result.frame.pixelFormat;
        qInfo(hikCControllerLog) << "  数据大小: " << (result.frame.pixels ? result.frame.pixels->size() : 0) << " bytes";
        qInfo(hikCControllerLog) << "  耗时: " << result.elapsedMs << " ms";
        qInfo(hikCControllerLog) << "==================";
    } else {
        setState(HikCameraCState::Error, 
                 QStringLiteral("海康相机 C 采图失败: %1").arg(result.errorMessage));
        qWarning(hikCControllerLog) << "=== 采图失败 ===";
        qWarning(hikCControllerLog) << "  错误码: " << static_cast<int>(result.errorCode);
        qWarning(hikCControllerLog) << "  错误信息: " << result.errorMessage;
        qWarning(hikCControllerLog) << "================";
    }

    emit testCaptureFinished(result);
}

void HikCameraCController::onAutoTestCaptureTimer()
{
    if (!m_autoTestEnabled || m_autoTestTriggered) {
        return;
    }
    
    m_autoTestTriggered = true;
    qInfo(hikCControllerLog) << "=== 自动触发测试采图 ===";
    
    const quint64 requestId = requestTestCapture();
    if (requestId > 0) {
        qInfo(hikCControllerLog) << "测试采图请求已发送, requestId:" << requestId;
    } else {
        qWarning(hikCControllerLog) << "测试采图请求失败";
    }
}

}  // namespace vision
}  // namespace scan_tracking
