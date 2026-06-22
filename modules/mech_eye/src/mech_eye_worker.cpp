#include "scan_tracking/mech_eye/mech_eye_worker.h"

#include <QtCore/QDateTime>
#include <QtCore/QElapsedTimer>
#include <QtCore/QLoggingCategory>

#include <algorithm>
#include <cmath>
#include <exception>
#include <fstream>
#include <limits>
#include <vector>
#include <qdir.h>
#include <qcoreapplication.h>
#include "scan_tracking/common/config_manager.h"
#include "ErrorStatus.h"
#include "area_scan_3d_camera/Camera.h"
#include "area_scan_3d_camera/CameraProperties.h"
#include "area_scan_3d_camera/CameraProperties.h"
#include "area_scan_3d_camera/Frame2D.h"
#include "area_scan_3d_camera/Frame2DAnd3D.h"
#include "area_scan_3d_camera/Frame3D.h"
#include "area_scan_3d_camera/parameters/PointCloudProcessing.h"
#include "area_scan_3d_camera/parameters/Scanning3D.h"
#include "UserSet.h"

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
 * @brief 将梅卡无纹理点云（仅 XYZ）转换为项目内部点云结构
 */
PointCloudFrame ConvertUntexturedPointCloudToFrame(
    const mmind::eye::UntexturedPointCloud& cloud,
    quint64 frameId,
    std::size_t* validPointCount = nullptr)
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
    pointCloud.normalsXYZ.reset();

    if (data == nullptr || pointCount == 0) {
        pointCloud.pointCount = 0;
        if (validPointCount != nullptr) {
            *validPointCount = 0;
        }
        return pointCloud;
    }

    pointCloud.pointsXYZ->resize(pointCount * 3);
    float* points = pointCloud.pointsXYZ->data();

    std::size_t validCount = 0;
    for (std::size_t index = 0; index < pointCount; ++index) {
        const auto& sample = data[index];
        const float x = sample.x;
        const float y = sample.y;
        const float z = sample.z;
        const std::size_t base = index * 3;
        points[base] = x;
        points[base + 1] = y;
        points[base + 2] = z;

        if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
            ++validCount;
        }
    }

    if (validPointCount != nullptr) {
        *validPointCount = validCount;
    }
    return pointCloud;
}

void applyMech3DCaptureUserSet(mmind::eye::UserSet& userSet)
{
    const auto& visionCfg = common::ConfigManager::instance()->visionConfig();

    const mmind::eye::Range<int> configuredRange(
        visionCfg.mechDepthRangeMin, visionCfg.mechDepthRangeMax);
    const auto depthStatus = userSet.setRangeValue(
        mmind::eye::scanning3d_setting::DepthRange::name, configuredRange);
    qInfo(LOG_MECHEYE_WORKER).noquote()
        << QStringLiteral("[深度范围] 设置为 [") << configuredRange.min << QStringLiteral(",")
        << configuredRange.max << QStringLiteral("] mm，成功=") << depthStatus.isOK();

    const auto edgeStatus = userSet.setBoolValue(
        mmind::eye::pointcloud_processing_setting::EdgeArtifactRemoval::name,
        visionCfg.mechEdgeArtifactRemovalEnabled);
    qInfo(LOG_MECHEYE_WORKER).noquote()
        << QStringLiteral("[边缘伪点去除] EdgeArtifactRemoval=")
        << visionCfg.mechEdgeArtifactRemovalEnabled
        << QStringLiteral("，成功=") << edgeStatus.isOK();
    if (!edgeStatus.isOK()) {
        qWarning(LOG_MECHEYE_WORKER).noquote()
            << QStringLiteral("[边缘伪点去除] 设置失败（当前型号可能不支持）: ")
            << QString::fromStdString(edgeStatus.errorDescription);
    }
}

GrayTextureFrame ConvertGrayTextureFrame(const mmind::eye::Frame2D& frame2d)
{
    GrayTextureFrame texture;
    const mmind::eye::GrayScale2DImage gray = frame2d.getGrayScaleImage();
    if (gray.isEmpty()) {
        return texture;
    }

    const int width = static_cast<int>(gray.width());
    const int height = static_cast<int>(gray.height());
    if (width <= 0 || height <= 0) {
        return texture;
    }

    texture.width = width;
    texture.height = height;
    texture.pixels = std::make_shared<std::vector<uint8_t>>();
    texture.pixels->resize(static_cast<std::size_t>(width * height));
    uint8_t* dst = texture.pixels->data();
    const std::size_t pixelCount = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    for (std::size_t index = 0; index < pixelCount; ++index) {
        const int row = static_cast<int>(index / static_cast<std::size_t>(width));
        const int col = static_cast<int>(index % static_cast<std::size_t>(width));
        dst[index] = gray.at(row, col).gray;
    }
    return texture;
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

        qInfo(LOG_MECHEYE_WORKER).noquote()
            << QStringLiteral("[ScanSync] 梅卡采集时刻 ms=") << QDateTime::currentMSecsSinceEpoch();

        if (normalized.mode == CaptureMode::Capture2DOnly) {
            mmind::eye::Frame2D frame2D;
            status = m_impl->camera.capture2D(
                frame2D,
                static_cast<unsigned int>(normalized.timeoutMs));
            if (status.isOK()) {
                result.texture2D = ConvertGrayTextureFrame(frame2D);
            }
        } else if (normalized.mode == CaptureMode::Capture3DOnly) {
            mmind::eye::Frame3D frame3D;

            applyMech3DCaptureUserSet(m_impl->camera.currentUserSet());

            // 先尝试 capture3D（不含相机侧法向量计算）
            status = m_impl->camera.capture3D(
                frame3D,
                static_cast<unsigned int>(normalized.timeoutMs));
            if (status.isOK()) {
                const auto untexturedCloud = frame3D.getUntexturedPointCloud();
                std::size_t validCount = 0;
                result.pointCloud = ConvertUntexturedPointCloudToFrame(
                    untexturedCloud,
                    static_cast<quint64>(frame3D.frameId()),
                    &validCount);

                if (validCount == 0) {
                    qWarning(LOG_MECHEYE_WORKER)
                        << QStringLiteral("点云全 NaN，请检查 DepthRange 配置和目标物距离；本段将不会写入内存缓存");
                }
            }
        } else {
            mmind::eye::Frame2DAnd3D frame2DAnd3D;
            applyMech3DCaptureUserSet(m_impl->camera.currentUserSet());
            status = m_impl->camera.capture2DAnd3D(
                frame2DAnd3D,
                static_cast<unsigned int>(normalized.timeoutMs));
            if (status.isOK()) {
                result.pointCloud = buildPointCloud2DAnd3D(frame2DAnd3D);
                result.texture2D = ConvertGrayTextureFrame(frame2DAnd3D.frame2D());
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

        if (normalized.mode == CaptureMode::Capture2DOnly) {
            if (!result.texture2D.isValid()) {
                m_busy = false;
                setRuntimeState(CameraRuntimeState::Ready, QStringLiteral("采集结束，但 2D 图像为空"));
                emit captureFinished(makeFailureResult(
                    normalized,
                    CaptureErrorCode::CaptureFailed,
                    QStringLiteral("采集成功，但 2D 图像为空"),
                    result.elapsedMs));
                return;
            }
        } else if (!result.pointCloud.isValid()) {
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
            << QStringLiteral("梅卡采集成功")
            << QStringLiteral(" 请求ID=") << result.requestId
            << QStringLiteral(" 模式=") << static_cast<int>(result.mode)
            << QStringLiteral(" 点数=") << result.pointCloud.pointCount
            << QStringLiteral(" 纹理2D=") << result.texture2D.width << QStringLiteral("x") << result.texture2D.height
            << QStringLiteral(" 耗时ms=") << result.elapsedMs;
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
    // 设置 SDK 心跳间隔为 5 秒（默认 10 秒），用于检测相机网络断连
    m_impl->camera.setHeartbeatInterval(5000);
    // 连接成功后打印相机基础参数
    printCameraParameters();

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
    const auto untexturedCloud = frame.getUntexturedPointCloud();
    return ConvertUntexturedPointCloudToFrame(
        untexturedCloud,
        static_cast<quint64>(frame.frameId()));
}

PointCloudFrame MechEyeWorker::buildPointCloud2DAnd3D(const mmind::eye::Frame2DAnd3D& frame) const
{
    const auto untexturedCloud = frame.frame3D().getUntexturedPointCloud();
    return ConvertUntexturedPointCloudToFrame(
        untexturedCloud,
        static_cast<quint64>(frame.frame3D().frameId()));
}

void MechEyeWorker::printCameraParameters()
{
    if (!m_connected || !m_impl) {
        return;
    }

    auto& userSet = m_impl->camera.currentUserSet();

    // 获取当前用户设置名称
    std::string userSetName;
    userSet.getName(userSetName);

    // 获取相机分辨率
    mmind::eye::CameraResolutions resolutions;
    m_impl->camera.getCameraResolutions(resolutions);

    // 获取相机内参
    mmind::eye::CameraIntrinsics intrinsics;
    m_impl->camera.getCameraIntrinsics(intrinsics);

    // 2D 曝光模式和曝光时间
    std::string scan2DExposureMode;
    userSet.getEnumValue("Scan2DExposureMode", scan2DExposureMode);

    double scan2DExposureTime = 0.0;
    userSet.getFloatValue("Scan2DExposureTime", scan2DExposureTime);

    // 3D 增益
    double scan3DGain = 0.0;
    userSet.getFloatValue("Scan3DGain", scan3DGain);

    // 3D 曝光时间
    std::vector<double> scan3DExposureList;
    userSet.getFloatArrayValue("Scan3DExposureSequence", scan3DExposureList);

    // 深度范围（Range 类型）
    mmind::eye::Range<int> depthRange;
    userSet.getRangeValue("DepthRange", depthRange);

    // 点云处理参数
    std::string surfaceSmoothing;
    userSet.getEnumValue("PointCloudSurfaceSmoothing", surfaceSmoothing);

    std::string noiseRemoval;
    userSet.getEnumValue("PointCloudNoiseRemoval", noiseRemoval);

    std::string outlierRemoval;
    userSet.getEnumValue("PointCloudOutlierRemoval", outlierRemoval);

    bool edgeArtifactRemoval = false;
    userSet.getBoolValue(
        mmind::eye::pointcloud_processing_setting::EdgeArtifactRemoval::name,
        edgeArtifactRemoval);

    qInfo(LOG_MECHEYE_WORKER).noquote()
        << "=== MechEye 相机信息 ===\n"
        << "  型号:" << m_cameraInfo.model << "\n"
        << "  序列号:" << m_cameraInfo.serialNumber << "\n"
        << "  IP:" << m_cameraInfo.ipAddress << "\n"
        << "  固件版本:" << m_cameraInfo.firmwareVersion << "\n"
        << "  当前用户设置:" << QString::fromStdString(userSetName) << "\n"
        << "  2D 分辨率:" << resolutions.texture.width << "x" << resolutions.texture.height << "\n"
        << "  深度图分辨率:" << resolutions.depth.width << "x" << resolutions.depth.height << "\n"
        << "  2D 曝光模式:" << QString::fromStdString(scan2DExposureMode) << "\n"
        << "  2D 曝光时间:" << scan2DExposureTime << "ms\n"
        << "  3D 增益:" << scan3DGain << "\n"
        << "  3D 曝光序列数量:" << scan3DExposureList.size() << "\n"
        << "  深度范围:" << depthRange.min << " - " << depthRange.max << " mm\n"
        << "  表面平滑:" << QString::fromStdString(surfaceSmoothing) << "\n"
        << "  噪声去除:" << QString::fromStdString(noiseRemoval) << "\n"
        << "  离群点去除:" << QString::fromStdString(outlierRemoval) << "\n"
        << "  边缘伪点去除:" << edgeArtifactRemoval;

    // 打印深度相机内参（Nano Ultra 没有独立 2D 纹理相机，只打印深度内参）
    const auto& depIntr = intrinsics.depth;
    qInfo(LOG_MECHEYE_WORKER).noquote()
        << "=== MechEye 深度相机内参 ===\n"
        << "  fx=" << depIntr.cameraMatrix.fx << " fy=" << depIntr.cameraMatrix.fy << "\n"
        << "  cx=" << depIntr.cameraMatrix.cx << " cy=" << depIntr.cameraMatrix.cy << "\n"
        << "  畸变系数: k1=" << depIntr.cameraDistortion.k1
        << " k2=" << depIntr.cameraDistortion.k2
        << " p1=" << depIntr.cameraDistortion.p1
        << " p2=" << depIntr.cameraDistortion.p2
        << " k3=" << depIntr.cameraDistortion.k3;
}

}  // namespace mech_eye
}  // namespace scan_tracking
