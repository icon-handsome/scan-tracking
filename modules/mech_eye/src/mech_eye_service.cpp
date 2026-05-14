#include "scan_tracking/mech_eye/mech_eye_service.h"

#include <QtCore/QLoggingCategory>
#include <QtCore/QThread>

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/mech_eye/mech_eye_worker.h"

Q_LOGGING_CATEGORY(LOG_MECHEYE_SVC, "mech_eye.service")

namespace scan_tracking {
namespace mech_eye {

/* 注册跨线程传递所需的 Qt 元类型。
 * queued connection 依赖元类型系统，否则参数无法安全投递到 worker 线程。
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

/* 构造函数 */
MechEyeService::MechEyeService(QObject* parent)
    : QObject(parent)
{
}

/* 析构函数。
 * 退出时会调用 stop()，确保 worker 线程和 SDK 资源被按顺序回收。
 */
MechEyeService::~MechEyeService()
{
    stop();
}

/* 启动 Mech-Eye 服务。
 * 该流程会读取默认配置、创建 worker 线程，并把外部请求转发到后台执行。
 */
void MechEyeService::start()
{
    if (m_started) {
        return;
    }

    registerMetaTypes();

    const auto* configManager = common::ConfigManager::instance();
    if (configManager != nullptr) {
        const auto cameraConfig = configManager->cameraConfig();
        m_defaultCameraKey = cameraConfig.defaultCamera;
        m_defaultCaptureTimeoutMs = cameraConfig.scanTimeoutMs > 0 ? cameraConfig.scanTimeoutMs : 5000;
    } else {
        m_defaultCameraKey.clear();
        m_defaultCaptureTimeoutMs = 5000;
    }

    m_workerThread = new QThread();
    m_worker = new MechEyeWorker();

    // SDK 对象必须停留在 worker 线程，主线程只负责投递请求和接收结果。
    m_worker->moveToThread(m_workerThread);

    connect(this, &MechEyeService::sig_startWorker,
            m_worker, &MechEyeWorker::startWorker, Qt::QueuedConnection);
    connect(this, &MechEyeService::sig_stopWorker,
            m_worker, &MechEyeWorker::stopWorker, Qt::QueuedConnection);
    connect(this, &MechEyeService::sig_refreshStatus,
            m_worker, &MechEyeWorker::refreshStatus, Qt::QueuedConnection);
    connect(this, &MechEyeService::sig_performCapture,
            m_worker, &MechEyeWorker::performCapture, Qt::QueuedConnection);

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

    emit sig_startWorker(m_defaultCameraKey);
}

/* 停止 Mech-Eye 服务。
 * 采用 stop -> quit -> wait 的顺序，确保 worker 先释放 SDK，再退出线程。
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

/* 主动刷新相机状态。
 * 当前服务未启动、正在停止或 worker 不存在时直接忽略。
 */
void MechEyeService::requestRefreshStatus()
{
    if (!m_started || m_stopping || m_worker == nullptr) {
        return;
    }

    emit sig_refreshStatus();
}

/* 发起采集请求。
 * 这里仅做状态与参数检查，真正的采集逻辑在 worker 线程执行。
 */
quint64 MechEyeService::requestCapture(const QString& cameraKey, CaptureMode mode, int timeoutMs)
{
    if (!m_started || m_stopping || m_worker == nullptr) {
        return 0;
    }

    // 单相机单并发策略：忙碌或未就绪时直接拒绝，避免队列堆积拖慢节拍。
    if (m_busy || m_currentState != CameraRuntimeState::Ready) {
        return 0;
    }

    CaptureRequest request;
    request.requestId = m_nextRequestId++;
    // 优先使用请求参数中的相机 key，若无效则退回默认配置。
    request.cameraKey = cameraKey.trimmed().isEmpty() ? m_defaultCameraKey : cameraKey.trimmed();  

    request.mode = mode;    // 采集模式
    request.timeoutMs = timeoutMs > 0 ? timeoutMs : m_defaultCaptureTimeoutMs;  //超时设置

    m_busy = true;
    emit sig_performCapture(request);   // 发出采集请求，worker 线程会接收并执行
    return request.requestId;
}

/* 处理 worker 返回的采集完成结果。 */
void MechEyeService::onWorkerCaptureFinished(scan_tracking::mech_eye::CaptureResult result)
{
    m_busy = false;
    emit captureFinished(result);
}

/* 处理 worker 返回的状态变化。 */
void MechEyeService::onWorkerStateChanged(
    scan_tracking::mech_eye::CameraRuntimeState newState,
    QString description)
{
    m_currentState = newState;
    if (newState != CameraRuntimeState::Capturing) {
        m_busy = false;
    }
    emit stateChanged(newState, description);
}

/* 处理 worker 返回的致命错误。 */
void MechEyeService::onWorkerFatalError(
    scan_tracking::mech_eye::CaptureErrorCode code,
    QString message)
{
    m_busy = false;
    emit fatalError(code, message);
}

}  // namespace mech_eye
}  // namespace scan_tracking