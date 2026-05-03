#include "scan_tracking/mech_eye/mech_eye_worker.h"

#include <QtCore/QDateTime>
#include <QtCore/QElapsedTimer>
#include <QtCore/QLoggingCategory>

#include <algorithm>
#include <exception>
#include <limits>
#include <vector>

#include "ErrorStatus.h"
#include "area_scan_3d_camera/Camera.h"
#include "area_scan_3d_camera/CameraProperties.h"
#include "area_scan_3d_camera/Frame2DAnd3D.h"
#include "area_scan_3d_camera/Frame3D.h"

Q_LOGGING_CATEGORY(LOG_MECHEYE_WORKER, "mech_eye.worker")

namespace scan_tracking {
namespace mech_eye {

namespace {

bool matchesCameraKey(const QString& key, const mmind::eye::CameraInfo& cameraInfo)
{
    if (key.trimmed().isEmpty()) {
        return false;
    }

    const QString normalizedKey = key.trimmed();
    const QString model = QString::fromStdString(cameraInfo.model);
    const QString serialNumber = QString::fromStdString(cameraInfo.serialNumber);
    const QString ipAddress = QString::fromStdString(cameraInfo.ipAddress);
    const QString deviceName = QString::fromStdString(cameraInfo.deviceName);

    return model.compare(normalizedKey, Qt::CaseInsensitive) == 0 ||
           serialNumber.compare(normalizedKey, Qt::CaseInsensitive) == 0 ||
           ipAddress.compare(normalizedKey, Qt::CaseInsensitive) == 0 ||
           deviceName.compare(normalizedKey, Qt::CaseInsensitive) == 0 ||
           model.contains(normalizedKey, Qt::CaseInsensitive) ||
           serialNumber.contains(normalizedKey, Qt::CaseInsensitive) ||
           ipAddress.contains(normalizedKey, Qt::CaseInsensitive) ||
           deviceName.contains(normalizedKey, Qt::CaseInsensitive);
}

/**
 * @brief 将梅卡无纹理点云（含法向量）转换为项目内部点云结构
 *
 * 该函数会把 SDK 返回的 XYZ 与 Normal 向量展平到连续数组，
 * 便于跨线程传输和后续算法模块（例如 PCL）直接消费。
 */
PointCloudFrame ConvertPointCloudWithNormalsToPcl(
    const mmind::eye::PointCloudWithNormals& cloud,
    quint64 frameId)
{
    PointCloudFrame pointCloud;
    const std::size_t width = cloud.width();
    const std::size_t height = cloud.height();
    const std::size_t pointCount = width * height;
    const auto* data = cloud.data();

    pointCloud.width = static_cast<int>(width);
    pointCloud.height = static_cast<int>(height);
    pointCloud.pointCount = static_cast<int>(pointCount);
    pointCloud.frameId = frameId;
    pointCloud.timestampMs = QDateTime::currentMSecsSinceEpoch();
    pointCloud.pointsXYZ = std::make_shared<std::vector<float>>();
    pointCloud.normalsXYZ = std::make_shared<std::vector<float>>();
    pointCloud.pointsXYZ->reserve(pointCount * 3);
    pointCloud.normalsXYZ->reserve(pointCount * 3);

    if (data == nullptr) {
        pointCloud.pointCount = 0;
        return pointCloud;
    }

    const float nanValue = std::numeric_limits<float>::quiet_NaN();
    for (std::size_t index = 0; index < pointCount; ++index) {
        pointCloud.pointsXYZ->push_back(data[index].point.x);
        pointCloud.pointsXYZ->push_back(data[index].point.y);
        pointCloud.pointsXYZ->push_back(data[index].point.z);

        const float nx = data[index].normal.x;
        const float ny = data[index].normal.y;
        const float nz = data[index].normal.z;
        pointCloud.normalsXYZ->push_back(nx);
        pointCloud.normalsXYZ->push_back(ny);
        pointCloud.normalsXYZ->push_back(nz);

        // 防御性处理：若 SDK 返回非法法向量，用 NaN 占位，保持点/法向索引对齐。
        if (!(nx == nx) || !(ny == ny) || !(nz == nz)) {
            (*pointCloud.normalsXYZ)[index * 3] = nanValue;
            (*pointCloud.normalsXYZ)[index * 3 + 1] = nanValue;
            (*pointCloud.normalsXYZ)[index * 3 + 2] = nanValue;
        }
    }

    return pointCloud;
}

/**
 * @brief 将梅卡彩色点云（含法向量）转换为项目内部点云结构
 */
PointCloudFrame ConvertTexturedPointCloudWithNormalsToPcl(
    const mmind::eye::TexturedPointCloudWithNormals& cloud,
    quint64 frameId)
{
    PointCloudFrame pointCloud;
    const std::size_t width = cloud.width();
    const std::size_t height = cloud.height();
    const std::size_t pointCount = width * height;
    const auto* data = cloud.data();

    pointCloud.width = static_cast<int>(width);
    pointCloud.height = static_cast<int>(height);
    pointCloud.pointCount = static_cast<int>(pointCount);
    pointCloud.frameId = frameId;
    pointCloud.timestampMs = QDateTime::currentMSecsSinceEpoch();
    pointCloud.pointsXYZ = std::make_shared<std::vector<float>>();
    pointCloud.normalsXYZ = std::make_shared<std::vector<float>>();
    pointCloud.pointsXYZ->reserve(pointCount * 3);
    pointCloud.normalsXYZ->reserve(pointCount * 3);

    if (data == nullptr) {
        pointCloud.pointCount = 0;
        return pointCloud;
    }

    const float nanValue = std::numeric_limits<float>::quiet_NaN();
    for (std::size_t index = 0; index < pointCount; ++index) {
        pointCloud.pointsXYZ->push_back(data[index].colorPoint.x);
        pointCloud.pointsXYZ->push_back(data[index].colorPoint.y);
        pointCloud.pointsXYZ->push_back(data[index].colorPoint.z);

        const float nx = data[index].normal.x;
        const float ny = data[index].normal.y;
        const float nz = data[index].normal.z;
        pointCloud.normalsXYZ->push_back(nx);
        pointCloud.normalsXYZ->push_back(ny);
        pointCloud.normalsXYZ->push_back(nz);

        if (!(nx == nx) || !(ny == ny) || !(nz == nz)) {
            (*pointCloud.normalsXYZ)[index * 3] = nanValue;
            (*pointCloud.normalsXYZ)[index * 3 + 1] = nanValue;
            (*pointCloud.normalsXYZ)[index * 3 + 2] = nanValue;
        }
    }

    return pointCloud;
}

}  // namespace

class MechEyeWorker::Impl {
public:
    mmind::eye::Camera camera;
    std::vector<mmind::eye::CameraInfo> discoveredCameras;
};

/* 构造函数：创建内部实现对象，但不在这里直接连接相机。 */
MechEyeWorker::MechEyeWorker(QObject* parent)
    : QObject(parent)
    , m_impl(new Impl())
{
}

/* 析构函数：在对象销毁前先断开相机，再释放内部实现。 */
MechEyeWorker::~MechEyeWorker()
{
    QString errorMessage;
    disconnectCamera(&errorMessage);
    delete m_impl;
    m_impl = nullptr;
}

/* 启动 worker：记录默认相机，并尝试建立初始连接。 */
void MechEyeWorker::startWorker(const QString& defaultCameraKey)
{
    m_defaultCameraKey = defaultCameraKey.trimmed();

    QString errorMessage;
    if (connectCamera(m_defaultCameraKey, 5000, &errorMessage)) {
        setRuntimeState(
            CameraRuntimeState::Ready,
            QStringLiteral("相机已连接: %1").arg(m_cameraInfo.serialNumber));
        return;
    }

    setRuntimeState(CameraRuntimeState::Error, errorMessage);
    emit fatalError(CaptureErrorCode::ConnectFailed, errorMessage);
}

/* 停止 worker：发出断开流程并把状态收束到 Stopped。 */
void MechEyeWorker::stopWorker()
{
    setRuntimeState(CameraRuntimeState::Disconnecting, QStringLiteral("正在断开相机连接"));

    QString errorMessage;
    if (!disconnectCamera(&errorMessage) && !errorMessage.isEmpty()) {
        emit fatalError(CaptureErrorCode::DisconnectFailed, errorMessage);
    }

    setRuntimeState(CameraRuntimeState::Stopped, QStringLiteral("相机服务已停止"));
}

/* 刷新状态：必要时重新连接相机，并同步当前在线信息。 */
void MechEyeWorker::refreshStatus()
{
    if (!m_connected) {
        QString errorMessage;
        if (ensureConnected(m_defaultCameraKey, 3000, &errorMessage)) {
            setRuntimeState(
                CameraRuntimeState::Ready,
                QStringLiteral("相机重新连接成功: %1").arg(m_cameraInfo.serialNumber));
        } else {
            setRuntimeState(CameraRuntimeState::Error, errorMessage);
        }
        return;
    }

    mmind::eye::CameraInfo liveInfo;
    const mmind::eye::ErrorStatus status = m_impl->camera.getCameraInfo(liveInfo);
    if (!status.isOK()) {
        QString errorMessage = QStringLiteral("刷新相机状态失败: %1")
            .arg(QString::fromStdString(status.errorDescription));
        if (mapSdkError(status.errorCode) == CaptureErrorCode::NotConnected) {
            m_connected = false;
            m_cameraInfo.connected = false;
        }
        setRuntimeState(CameraRuntimeState::Error, errorMessage);
        return;
    }

    m_cameraInfo = makeSnapshot(liveInfo, true);
    emit stateChanged(
        m_state,
        QStringLiteral("相机在线: %1 @ %2")
            .arg(m_cameraInfo.serialNumber, m_cameraInfo.ipAddress));
}

/* 执行一次采集：先确保连接正常，再按采集模式调用 SDK。 */
void MechEyeWorker::performCapture(const scan_tracking::mech_eye::CaptureRequest& request)
{
    QElapsedTimer timer;
    timer.start();

    CaptureRequest normalized = request;
    if (normalized.timeoutMs <= 0) {
        normalized.timeoutMs = 5000;
    }
    if (normalized.cameraKey.trimmed().isEmpty()) {
        normalized.cameraKey = m_defaultCameraKey;
    }

    if (m_busy) {
        emit captureFinished(makeFailureResult(
            normalized,
            CaptureErrorCode::Busy,
            QStringLiteral("相机忙，拒绝重入"),
            timer.elapsed()));
        return;
    }

    QString errorMessage;
    if (!ensureConnected(normalized.cameraKey, normalized.timeoutMs, &errorMessage)) {
        emit captureFinished(makeFailureResult(
            normalized,
            CaptureErrorCode::NotConnected,
            errorMessage,
            timer.elapsed()));
        return;
    }

    m_busy = true;
    setRuntimeState(
        CameraRuntimeState::Capturing,
        QStringLiteral("开始采集 requestId=%1").arg(normalized.requestId));

    CaptureResult result;
    result.requestId = normalized.requestId;
    result.cameraKey = normalized.cameraKey;
    result.mode = normalized.mode;
    result.cameraInfo = m_cameraInfo;
    result.elapsedMs = 0;

#if defined(__cpp_exceptions) || defined(_CPPUNWIND)
    try {
#endif
        mmind::eye::ErrorStatus status;

        if (normalized.mode == CaptureMode::Capture3DOnly) {
            mmind::eye::Frame3D frame3D;
            // 真实采集路径：直接让相机侧计算法向量，减少 IPC 侧额外计算开销。
            status = m_impl->camera.capture3DWithNormal(
                frame3D,
                static_cast<unsigned int>(normalized.timeoutMs));
            if (status.isOK()) {
                result.pointCloud = buildPointCloud3D(frame3D);
            }
        } else {
            mmind::eye::Frame2DAnd3D frame2DAnd3D;
            status = m_impl->camera.capture2DAnd3DWithNormal(
                frame2DAnd3D,
                static_cast<unsigned int>(normalized.timeoutMs));
            if (status.isOK()) {
                result.pointCloud = buildPointCloud2DAnd3D(frame2DAnd3D);
            }
        }

        result.elapsedMs = timer.elapsed();

        if (!status.isOK()) {
            result.errorCode = mapSdkError(status.errorCode);
            result.errorMessage = QStringLiteral("采集失败: %1")
                .arg(QString::fromStdString(status.errorDescription));
            if (result.errorCode == CaptureErrorCode::NotConnected) {
                m_connected = false;
                m_cameraInfo.connected = false;
                setRuntimeState(CameraRuntimeState::Error, result.errorMessage);
            } else {
                setRuntimeState(CameraRuntimeState::Ready, QStringLiteral("采集结束，等待下一次触发"));
            }
            m_busy = false;
            emit captureFinished(result);
            return;
        }

        if (!result.pointCloud.isValid()) {
            m_busy = false;
            setRuntimeState(CameraRuntimeState::Ready, QStringLiteral("采集结束，但点云为空"));
            emit captureFinished(makeFailureResult(
                normalized,
                CaptureErrorCode::CaptureFailed,
                QStringLiteral("采集成功，但点云为空"),
                result.elapsedMs));
            return;
        }

        result.errorCode = CaptureErrorCode::Success;
        result.errorMessage.clear();
        m_busy = false;
        setRuntimeState(CameraRuntimeState::Ready, QStringLiteral("采集成功，等待下一次触发"));
        qInfo(LOG_MECHEYE_WORKER).noquote()
            << "Capture success"
            << "requestId=" << result.requestId
            << "pointCount=" << result.pointCloud.pointCount
            << "normalCount=" << result.pointCloud.normalCount()
            << "elapsedMs=" << result.elapsedMs;
        emit captureFinished(result);
#if defined(__cpp_exceptions) || defined(_CPPUNWIND)
    } catch (const std::exception& exception) {
        m_busy = false;
        m_connected = false;
        m_cameraInfo.connected = false;
        QString failureMessage = QStringLiteral("采集异常: %1")
            .arg(QString::fromLocal8Bit(exception.what()));
        setRuntimeState(CameraRuntimeState::Error, failureMessage);
        emit fatalError(CaptureErrorCode::UnknownError, failureMessage);
        emit captureFinished(makeFailureResult(
            normalized,
            CaptureErrorCode::UnknownError,
            failureMessage,
            timer.elapsed()));
    } catch (...) {
        m_busy = false;
        m_connected = false;
        m_cameraInfo.connected = false;
        const QString failureMessage = QStringLiteral("采集异常: 未知错误");
        setRuntimeState(CameraRuntimeState::Error, failureMessage);
        emit fatalError(CaptureErrorCode::UnknownError, failureMessage);
        emit captureFinished(makeFailureResult(
            normalized,
            CaptureErrorCode::UnknownError,
            failureMessage,
            timer.elapsed()));
    }
#endif
}

/* 更新运行状态并向外发射通知。 */
void MechEyeWorker::setRuntimeState(CameraRuntimeState newState, const QString& description)
{
    m_state = newState;
    emit stateChanged(newState, description);
}

/* 将 SDK 错误码转换为项目内部错误码，便于上层统一处理。 */
CaptureErrorCode MechEyeWorker::mapSdkError(int sdkErrorCode) const
{
    switch (sdkErrorCode) {
    case mmind::eye::ErrorStatus::MMIND_STATUS_SUCCESS:
        return CaptureErrorCode::Success;
    case mmind::eye::ErrorStatus::MMIND_STATUS_TIMEOUT_ERROR:
        return CaptureErrorCode::Timeout;
    case mmind::eye::ErrorStatus::MMIND_STATUS_DEVICE_OFFLINE:
    case mmind::eye::ErrorStatus::MMIND_STATUS_INVALID_DEVICE:
        return CaptureErrorCode::NotConnected;
    case mmind::eye::ErrorStatus::MMIND_STATUS_DEVICE_BUSY:
        return CaptureErrorCode::Busy;
    case mmind::eye::ErrorStatus::MMIND_STATUS_INVALID_INPUT_ERROR:
    case mmind::eye::ErrorStatus::MMIND_STATUS_OUT_OF_RANGE_ERROR:
    case mmind::eye::ErrorStatus::MMIND_STATUS_PARAMETER_ERROR:
        return CaptureErrorCode::InvalidRequest;
    default:
        return CaptureErrorCode::CaptureFailed;
    }
}

CameraInfoSnapshot MechEyeWorker::makeSnapshot(const mmind::eye::CameraInfo& info, bool connected) const
{
    CameraInfoSnapshot snapshot;
    snapshot.model = QString::fromStdString(info.model);
    snapshot.serialNumber = QString::fromStdString(info.serialNumber);
    snapshot.ipAddress = QString::fromStdString(info.ipAddress);
    snapshot.firmwareVersion = QString::fromStdString(info.firmwareVersion.toString());
    snapshot.connected = connected;
    return snapshot;
}

bool MechEyeWorker::ensureConnected(const QString& cameraKey, int timeoutMs, QString* errorMessage)
{
    if (m_connected) {
        return true;
    }
    return connectCamera(cameraKey, timeoutMs, errorMessage);
}

bool MechEyeWorker::connectCamera(const QString& cameraKey, int timeoutMs, QString* errorMessage)
{
    setRuntimeState(CameraRuntimeState::Discovering, QStringLiteral("正在搜索相机"));

    try {
        m_impl->discoveredCameras = mmind::eye::Camera::discoverCameras(
            static_cast<unsigned int>(timeoutMs > 0 ? timeoutMs : 5000));
    } catch (const std::exception& exception) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("搜索相机异常: %1")
                .arg(QString::fromLocal8Bit(exception.what()));
        }
        m_impl->discoveredCameras.clear();
        m_connected = false;
        m_cameraInfo = {};
        return false;
    } catch (...) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("搜索相机异常: 未知错误");
        }
        m_impl->discoveredCameras.clear();
        m_connected = false;
        m_cameraInfo = {};
        return false;
    }

    if (m_impl->discoveredCameras.empty()) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("未发现可用的 Mech-Eye 相机");
        }
        return false;
    }

    const auto selectedIt = std::find_if(
        m_impl->discoveredCameras.begin(),
        m_impl->discoveredCameras.end(),
        [&cameraKey](const mmind::eye::CameraInfo& info) {
            return cameraKey.trimmed().isEmpty() || matchesCameraKey(cameraKey, info);
        });

    if (selectedIt == m_impl->discoveredCameras.end()) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("未找到匹配的相机: %1").arg(cameraKey);
        }
        return false;
    }

    setRuntimeState(
        CameraRuntimeState::Connecting,
        QStringLiteral("正在连接相机 %1")
            .arg(QString::fromStdString(selectedIt->serialNumber)));

    mmind::eye::ErrorStatus status;
    try {
        status = m_impl->camera.connect(
            *selectedIt,
            static_cast<unsigned int>(timeoutMs > 0 ? timeoutMs : 5000));
    } catch (const std::exception& exception) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("连接相机异常: %1")
                .arg(QString::fromLocal8Bit(exception.what()));
        }
        m_connected = false;
        m_cameraInfo = {};
        return false;
    } catch (...) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("连接相机异常: 未知错误");
        }
        m_connected = false;
        m_cameraInfo = {};
        return false;
    }

    if (!status.isOK()) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("连接相机失败: %1")
                .arg(QString::fromStdString(status.errorDescription));
        }
        m_connected = false;
        m_cameraInfo = {};
        return false;
    }

    m_connected = true;
    m_cameraInfo = makeSnapshot(*selectedIt, true);
    return true;
}

bool MechEyeWorker::disconnectCamera(QString* errorMessage)
{
    if (!m_connected) {
        return true;
    }

    try {
        m_impl->camera.disconnect();
        m_connected = false;
        m_busy = false;
        m_cameraInfo.connected = false;
        return true;
    } catch (const std::exception& exception) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("断开相机失败: %1")
                .arg(QString::fromLocal8Bit(exception.what()));
        }
        return false;
    } catch (...) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("断开相机失败: 未知错误");
        }
        return false;
    }
}

CaptureResult MechEyeWorker::makeFailureResult(
    const CaptureRequest& request,
    CaptureErrorCode errorCode,
    const QString& errorMessage,
    qint64 elapsedMs) const
{
    CaptureResult result; 
    result.requestId = request.requestId;  
    result.cameraKey = request.cameraKey;   
    result.mode = request.mode;             
    result.errorCode = errorCode;           
    result.errorMessage = errorMessage;
    result.cameraInfo = m_cameraInfo;
    result.elapsedMs = elapsedMs;
    return result;
}

PointCloudFrame MechEyeWorker::buildPointCloud3D(const mmind::eye::Frame3D& frame) const
{
    const auto cloudWithNormals = frame.getUntexturedPointCloudWithNormals();
    return ConvertPointCloudWithNormalsToPcl(cloudWithNormals, static_cast<quint64>(frame.frameId()));
}

PointCloudFrame MechEyeWorker::buildPointCloud2DAnd3D(const mmind::eye::Frame2DAnd3D& frame) const
{
    const auto cloudWithNormals = frame.getTexturedPointCloudWithNormals();
    return ConvertTexturedPointCloudWithNormalsToPcl(
        cloudWithNormals,
        static_cast<quint64>(frame.frame3D().frameId()));
}

}  // namespace mech_eye
}  // namespace scan_tracking
