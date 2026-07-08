/**
 * @file mech_eye_service.cpp
 * @brief MechEyeService 实现：线程生命周期管理与请求转发
 *
 * 架构概览：
 * @code
 *   [主线程] MechEyeService
 *       | sig_* (QueuedConnection)
 *       v
 *   [MechEyeWorkerThread] MechEyeWorker  ← mmind::eye::Camera 仅在此线程
 *       | captureFinished / stateChanged / fatalError (QueuedConnection)
 *       v
 *   [主线程] StateMachine / VisionPipelineService
 * @endcode
 *
 * 状态与 busy 标志由 Service 维护，Worker 的 Capturing 状态会同步清除 busy。
 */
#include "scan_tracking/mech_eye/mech_eye_service.h"

#include <QtCore/QLoggingCategory>
#include <QtCore/QThread>

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/mech_eye/mech_eye_worker.h"

Q_LOGGING_CATEGORY(LOG_MECHEYE_SVC, "mech_eye.service")

namespace scan_tracking {
namespace mech_eye {

/**
 * @brief 注册跨线程 QueuedConnection 所需的 Qt 元类型
 *
 * CaptureRequest/CaptureResult 等结构体含 QString 与 shared_ptr，
 * 未注册时信号槽参数无法跨线程 marshal，会在运行时 qWarning。
 * 使用 static bool 保证进程内只注册一次。
 */
void MechEyeService::registerMetaTypes()
{
    static bool registered = false;
    if (registered) {
        return;
    }

    qRegisterMetaType<scan_tracking::mech_eye::CaptureMode>("scan_tracking::mech_eye::CaptureMode");
    qRegisterMetaType<scan_tracking::mech_eye::CaptureErrorCode>("scan_tracking::mech_eye::CaptureErrorCode");
    qRegisterMetaType<scan_tracking::mech_eye::CameraRuntimeState>("scan_tracking::mech_eye::CameraRuntimeState");
    qRegisterMetaType<scan_tracking::mech_eye::CameraInfoSnapshot>("scan_tracking::mech_eye::CameraInfoSnapshot");
    qRegisterMetaType<scan_tracking::mech_eye::PointCloudFrame>("scan_tracking::mech_eye::PointCloudFrame");
    qRegisterMetaType<scan_tracking::mech_eye::CaptureRequest>("scan_tracking::mech_eye::CaptureRequest");
    qRegisterMetaType<scan_tracking::mech_eye::CaptureResult>("scan_tracking::mech_eye::CaptureResult");
    registered = true;
}

MechEyeService::MechEyeService(QObject* parent)
    : QObject(parent)
{
}

MechEyeService::~MechEyeService()
{
    // 析构时同步 stop，避免 worker 线程泄漏或 SDK 未 disconnect
    stop();
}

/**
 * @brief 启动 Mech-Eye 服务
 *
 * 流程：
 * 1. 从 ConfigManager 读取 defaultCamera、scanTimeoutMs
 * 2. 创建 QThread + MechEyeWorker，worker moveToThread
 * 3. 建立双向 QueuedConnection（Service↔Worker）
 * 4. 启动线程并 emit sig_startWorker，Worker 异步连接默认相机
 */
void MechEyeService::start()
{
    if (m_started) {
        return;
    }

    registerMetaTypes();

    // 读取 config.ini [Vision] mechEyeCameraKey；回退 [Camera] defaultCamera
    const auto* configManager = common::ConfigManager::instance();
    if (configManager != nullptr) {
        const auto visionConfig = configManager->visionConfig();
        const auto cameraConfig = configManager->cameraConfig();
        m_defaultCameraKey = !visionConfig.mechEyeCameraKey.trimmed().isEmpty()
            ? visionConfig.mechEyeCameraKey.trimmed()
            : cameraConfig.defaultCamera;
        if (visionConfig.mechCaptureTimeoutMs > 0) {
            m_defaultCaptureTimeoutMs = visionConfig.mechCaptureTimeoutMs;
        } else {
            m_defaultCaptureTimeoutMs =
                cameraConfig.scanTimeoutMs > 0 ? cameraConfig.scanTimeoutMs : 5000;
        }
    } else {
        m_defaultCameraKey.clear();
        m_defaultCaptureTimeoutMs = 5000;
    }

    m_workerThread = new QThread();
    m_worker = new MechEyeWorker();

    // Camera 对象生命周期绑定 worker 线程；主线程禁止直接 touch SDK
    m_worker->moveToThread(m_workerThread);

    // --- Service → Worker：异步投递控制与采集命令 ---
    connect(this, &MechEyeService::sig_startWorker,
            m_worker, &MechEyeWorker::startWorker, Qt::QueuedConnection);
    connect(this, &MechEyeService::sig_stopWorker,
            m_worker, &MechEyeWorker::stopWorker, Qt::QueuedConnection);
    connect(this, &MechEyeService::sig_refreshStatus,
            m_worker, &MechEyeWorker::refreshStatus, Qt::QueuedConnection);
    connect(this, &MechEyeService::sig_performCapture,
            m_worker, &MechEyeWorker::performCapture, Qt::QueuedConnection);

    // --- Worker → Service：结果与状态回传主线程 ---
    connect(m_worker, &MechEyeWorker::captureFinished,
            this, &MechEyeService::onWorkerCaptureFinished, Qt::QueuedConnection);
    connect(m_worker, &MechEyeWorker::stateChanged,
            this, &MechEyeService::onWorkerStateChanged, Qt::QueuedConnection);
    connect(m_worker, &MechEyeWorker::fatalError,
            this, &MechEyeService::onWorkerFatalError, Qt::QueuedConnection);

    m_workerThread->setObjectName(QStringLiteral("MechEyeWorkerThread"));
    m_workerThread->start();

    m_started = true;
    m_busy = false;
    m_stopping = false;
    m_currentState = CameraRuntimeState::Idle;

    // 触发 Worker 侧 discover + connect；连接结果经 stateChanged/fatalError 通知
    emit sig_startWorker(m_defaultCameraKey);
}

/**
 * @brief 停止 Mech-Eye 服务
 *
 * 顺序：sig_stopWorker（Worker 内 disconnect SDK）→ thread quit → wait(10s) → delete。
 * 超时未退出仅打 Critical 日志，仍强制 delete（避免进程 hang）。
 */
void MechEyeService::stop()
{
    if (!m_started) {
        return;
    }

    m_stopping = true;
    m_busy = false;

    if (m_worker != nullptr) {
        emit sig_stopWorker();
    }
    if (m_workerThread != nullptr) {
        m_workerThread->quit();
        if (!m_workerThread->wait(10000)) {
            qCritical(LOG_MECHEYE_SVC) << "Mech-Eye worker 线程未能及时退出。";
        }
    }

    delete m_worker;
    m_worker = nullptr;
    delete m_workerThread;
    m_workerThread = nullptr;

    m_started = false;
    m_stopping = false;
    m_busy = false;
    m_currentState = CameraRuntimeState::Stopped;
}

void MechEyeService::requestRefreshStatus()
{
    if (!m_started || m_stopping || m_worker == nullptr) {
        return;
    }

    emit sig_refreshStatus();
}

/**
 * @brief 发起一次采集请求
 * @return 非零 requestId 表示已受理；0 表示拒绝（未启动/停止中/忙/非 Ready）
 *
 * 拒绝策略（不排队）：
 * - m_busy：上一帧 captureFinished 尚未到达
 * - m_currentState != Ready：连接中、采集中、Error 等
 *
 * 实际 SDK 调用在 Worker::performCapture 中阻塞执行。
 */
quint64 MechEyeService::requestCapture(
    const QString& cameraKey,
    CaptureMode mode,
    int timeoutMs,
    bool comparisonCaptureEnabled)
{
    if (!m_started || m_stopping || m_worker == nullptr) {
        return 0;
    }

    if (m_busy || m_currentState != CameraRuntimeState::Ready) {
        return 0;
    }

    CaptureRequest request;
    request.requestId = m_nextRequestId++;
    request.cameraKey = cameraKey.trimmed().isEmpty() ? m_defaultCameraKey : cameraKey.trimmed();
    request.mode = mode;
    request.timeoutMs = timeoutMs > 0 ? timeoutMs : m_defaultCaptureTimeoutMs;
    request.comparisonCaptureEnabled = comparisonCaptureEnabled;

    m_busy = true;
    emit sig_performCapture(request);
    return request.requestId;
}

/** @brief Worker 采集结束：清除 busy 并向上层转发 CaptureResult（含失败结果） */
void MechEyeService::onWorkerCaptureFinished(scan_tracking::mech_eye::CaptureResult result)
{
    m_busy = false;
    emit captureFinished(result);
}

/**
 * @brief 同步 Worker 状态机到 Service 侧缓存
 *
 * - Capturing 以外的状态均清除 busy（防止 Worker 提前 Error 后 Service 永久 busy）
 * - Ready/Capturing 视为相机在线，供后续 Modbus 状态字等扩展
 */
void MechEyeService::onWorkerStateChanged(
    scan_tracking::mech_eye::CameraRuntimeState newState,
    QString description)
{
    m_currentState = newState;
    if (newState != CameraRuntimeState::Capturing) {
        m_busy = false;
    }

    m_cameraConnected = (newState == CameraRuntimeState::Ready ||
                         newState == CameraRuntimeState::Capturing);

    emit stateChanged(newState, description);
}

/** @brief Worker 不可恢复错误（如启动连接失败、采集抛异常）：清除 busy 并上报 fatalError */
void MechEyeService::onWorkerFatalError(
    scan_tracking::mech_eye::CaptureErrorCode code,
    QString message)
{
    m_busy = false;
    emit fatalError(code, message);
}

}  // namespace mech_eye
}  // namespace scan_tracking
