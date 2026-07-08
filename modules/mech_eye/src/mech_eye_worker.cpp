/**
 * @file mech_eye_worker.cpp
 * @brief MechEyeWorker 实现：SDK 发现/连接/采图与帧格式转换
 *
 * 线程约束：本文件所有 mmind::eye::* 调用必须在 Worker 所在 QThread 执行。
 * 采集流程（performCapture）：
 *   参数规范化 → 重入检查 → ensureConnected → 按 CaptureMode 分支采图
 *   → 结果校验 → captureFinished
 */
#include "scan_tracking/mech_eye/mech_eye_worker.h"

#include <QtCore/QDateTime>
#include <QtCore/QElapsedTimer>
#include <QtCore/QLoggingCategory>
#include <QtCore/QTimer>

#if defined(_MSC_VER)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#include <eh.h>
#endif

#include <algorithm>
#include <cmath>
#include <exception>
#include <fstream>
#include <limits>
#include <memory>
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

#if defined(_MSC_VER)
void mechEyeSehTranslator(unsigned int code, EXCEPTION_POINTERS* /*pointers*/)
{
    if (code == EXCEPTION_ACCESS_VIOLATION) {
        throw std::runtime_error("Mech-Eye SDK access violation");
    }
    throw std::runtime_error("Mech-Eye SDK structured exception");
}

void installMechEyeSehTranslatorOnce()
{
    static thread_local bool installed = false;
    if (!installed) {
        _set_se_translator(mechEyeSehTranslator);
        installed = true;
    }
}
#endif

/**
 * @brief 包装 discoverCameras，将 Windows 访问冲突转为 C++ 异常以便捕获
 */
bool discoverMechCamerasSafe(
    unsigned int timeoutMs,
    std::vector<mmind::eye::CameraInfo>& cameras,
    QString* errorMessage)
{
    cameras.clear();
#if defined(_MSC_VER)
    installMechEyeSehTranslatorOnce();
#endif
    try {
        cameras = mmind::eye::Camera::discoverCameras(timeoutMs);
        return true;
    } catch (const std::exception& exception) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("搜索相机异常: %1")
                                .arg(QString::fromLocal8Bit(exception.what()));
        }
        return false;
    } catch (...) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("搜索相机异常: 未知错误");
        }
        return false;
    }
}

/**
 * @brief 判断 cameraKey 是否匹配 SDK 返回的 CameraInfo
 *
 * 支持精确匹配（型号/序列号/IP/设备名）与包含匹配（子串），大小写不敏感。
 * cameraKey 为空时始终返回 false。
 */
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

    // 先尝试全字段精确匹配，再尝试子串包含（便于用 IP 段或型号前缀筛选）
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
 * @brief 将梅卡无纹理点云（UntexturedPointCloud）转换为项目内部 PointCloudFrame
 * @param validPointCount 可选输出：有限值（非 NaN/Inf）点的数量，用于判空
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

    // SDK 返回空缓冲时仍保留 width/height 元数据，pointCount 置 0
    if (data == nullptr || pointCount == 0) {
        pointCloud.pointCount = 0;
        if (validPointCount != nullptr) {
            *validPointCount = 0;
        }
        return pointCloud;
    }

    pointCloud.pointsXYZ->resize(pointCount * 3);
    float* points = pointCloud.pointsXYZ->data();

    // 逐点拷贝 XYZ；同时统计有限值点数（Mech 无效深度通常为 NaN）
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

/**
 * @brief 按 visionConfig 写入 Mech-Eye 3D 单曝光（Scan3DExposureSequence）
 *
 * 多曝光 HDR、2D 曝光、增益、持久化到设备等配置项已预留，尚未接入。
 */
void applyMech3DSingleExposureUserSet(mmind::eye::UserSet& userSet)
{
    const auto& visionCfg = common::ConfigManager::instance()->visionConfig();
    if (!visionCfg.mechScan3DSingleExposureEnabled) {
        return;
    }
    if (visionCfg.mechScan3DSingleExposureMs <= 0.0) {
        qWarning(LOG_MECHEYE_WORKER).noquote()
            << QStringLiteral("[3D 曝光] 单曝光已启用但 mechScan3DSingleExposureMs 无效，跳过写入");
        return;
    }

    const std::vector<double> exposureSequence = {visionCfg.mechScan3DSingleExposureMs};
    const auto exposureStatus = userSet.setFloatArrayValue(
        mmind::eye::scanning3d_setting::ExposureSequence::name, exposureSequence);
    qInfo(LOG_MECHEYE_WORKER).noquote()
        << QStringLiteral("[3D 曝光] 单曝光=") << visionCfg.mechScan3DSingleExposureMs
        << QStringLiteral(" ms，成功=") << exposureStatus.isOK();
    if (!exposureStatus.isOK()) {
        qWarning(LOG_MECHEYE_WORKER).noquote()
            << QStringLiteral("[3D 曝光] 写入失败: ")
            << QString::fromStdString(exposureStatus.errorDescription);
    }
}

/**
 * @brief 按 visionConfig 设置 SDK UserSet 中的 3D 采集参数
 *
 * 每次 capture3D / capture2DAnd3D 前调用，确保深度范围与点云滤波与 config.ini 一致：
 * - DepthRange：[mechDepthRangeMin, mechDepthRangeMax] mm
 * - Scan3DExposureSequence：单曝光（mechScan3DSingleExposureEnabled）
 * - OutlierRemoval：Off
 * - NoiseRemoval：Normal（主流程）或 Off（对比采集）
 */
void applyMech3DCaptureUserSet(mmind::eye::UserSet& userSet, bool noiseRemovalNormal)
{
    const auto& visionCfg = common::ConfigManager::instance()->visionConfig();

    applyMech3DSingleExposureUserSet(userSet);

    const mmind::eye::Range<int> configuredRange(
        visionCfg.mechDepthRangeMin, visionCfg.mechDepthRangeMax);
    const auto depthStatus = userSet.setRangeValue(
        mmind::eye::scanning3d_setting::DepthRange::name, configuredRange);
    qInfo(LOG_MECHEYE_WORKER).noquote()
        << QStringLiteral("[深度范围] 设置为 [") << configuredRange.min << QStringLiteral(",")
        << configuredRange.max << QStringLiteral("] mm，成功=") << depthStatus.isOK();

    using Outlier = mmind::eye::pointcloud_processing_setting::OutlierRemoval;
    using Noise = mmind::eye::pointcloud_processing_setting::NoiseRemoval;

    const auto outlierStatus =
        userSet.setEnumValue(Outlier::name, static_cast<int>(Outlier::Value::Off));

    const int noiseLevel = noiseRemovalNormal
        ? static_cast<int>(Noise::Value::Normal)
        : static_cast<int>(Noise::Value::Off);

    const auto noiseStatus = userSet.setEnumValue(Noise::name, noiseLevel);
    qInfo(LOG_MECHEYE_WORKER).noquote()
        << QStringLiteral("[点云滤波] OutlierRemoval=Off NoiseRemoval=")
        << (noiseRemovalNormal ? QStringLiteral("Normal") : QStringLiteral("Off"))
        << QStringLiteral(" 成功=") << outlierStatus.isOK() << noiseStatus.isOK();
    if (!outlierStatus.isOK()) {
        qWarning(LOG_MECHEYE_WORKER).noquote()
            << QStringLiteral("[点云滤波] OutlierRemoval 关闭失败: ")
            << QString::fromStdString(outlierStatus.errorDescription);
    }
    if (!noiseStatus.isOK()) {
        qWarning(LOG_MECHEYE_WORKER).noquote()
            << QStringLiteral("[点云滤波] NoiseRemoval 设置失败: ")
            << QString::fromStdString(noiseStatus.errorDescription);
    }
}

/** @brief 执行单次 capture3D 并转换为 PointCloudFrame；失败时返回空帧并在 errorMessage 中说明 */
PointCloudFrame capturePointCloud3D(
    mmind::eye::Camera& camera,
    int timeoutMs,
    bool noiseRemovalNormal,
    qint64* elapsedMs,
    QString* errorMessage)
{
    QElapsedTimer timer;
    timer.start();

    mmind::eye::Frame3D frame3D;
    // 采图前刷新 UserSet，使深度范围/滤波与 config.ini 同步
    applyMech3DCaptureUserSet(camera.currentUserSet(), noiseRemovalNormal);
    const auto status = camera.capture3D(frame3D, static_cast<unsigned int>(timeoutMs));
    if (!status.isOK()) {
        if (elapsedMs != nullptr) {
            *elapsedMs = timer.elapsed();
        }
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("采集失败: %1")
                                .arg(QString::fromStdString(status.errorDescription));
        }
        return {};
    }

    std::size_t validCount = 0;
    PointCloudFrame cloud = ConvertUntexturedPointCloudToFrame(
        frame3D.getUntexturedPointCloud(),
        static_cast<quint64>(frame3D.frameId()),
        &validCount);
    if (elapsedMs != nullptr) {
        *elapsedMs = timer.elapsed();
    }
    // SDK 返回成功但全 NaN 时仍视为采集失败
    if (validCount == 0) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("点云全 NaN");
        }
        return {};
    }
    if (errorMessage != nullptr) {
        errorMessage->clear();
    }
    return cloud;
}

/** @brief 对比采集封装，与 capturePointCloud3D 相同但 noiseRemovalNormal 由调用方指定 */
PointCloudFrame capturePointCloud3DComparison(
    mmind::eye::Camera& camera,
    int timeoutMs,
    bool noiseRemovalNormal,
    qint64* elapsedMs,
    QString* errorMessage)
{
    return capturePointCloud3D(
        camera,
        timeoutMs,
        noiseRemovalNormal,
        elapsedMs,
        errorMessage);
}

/** @brief 将 SDK Frame2D 灰度图逐像素拷贝为 GrayTextureFrame（行优先 uint8 数组） */
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
    // gray.at(row,col) 为 SDK 二维访问接口；dst 按行优先一维展开
    for (std::size_t index = 0; index < pixelCount; ++index) {
        const int row = static_cast<int>(index / static_cast<std::size_t>(width));
        const int col = static_cast<int>(index % static_cast<std::size_t>(width));
        dst[index] = gray.at(row, col).gray;
    }
    return texture;
}

}  // namespace

/** @brief PIMPL 数据：Camera 延迟在 Worker 线程创建，避免跨线程构造 SDK 对象 */
class MechEyeWorker::Impl {
public:
    std::unique_ptr<mmind::eye::Camera> camera;
    std::vector<mmind::eye::CameraInfo> discoveredCameras;

    mmind::eye::Camera& ensureCamera()
    {
        if (!camera) {
            camera = std::make_unique<mmind::eye::Camera>();
        }
        return *camera;
    }
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
    // 析构可能在 stopWorker 之后触发，此处再次 disconnect 作为兜底
    QString errorMessage;
    disconnectCamera(&errorMessage);
    delete m_impl;
    m_impl = nullptr;
}

/* 启动 worker：记录默认相机，延迟后台连接（不阻塞 IPC 启动） */
void MechEyeWorker::startWorker(const QString& defaultCameraKey)
{
    m_defaultCameraKey = defaultCameraKey.trimmed();
    setRuntimeState(
        CameraRuntimeState::Idle,
        QStringLiteral("Worker 就绪，后台连接相机 key=%1").arg(m_defaultCameraKey));
    QTimer::singleShot(200, this, [this]() { attemptInitialConnect(); });
}

void MechEyeWorker::attemptInitialConnect()
{
    if (m_state == CameraRuntimeState::Stopped || m_state == CameraRuntimeState::Ready) {
        return;
    }

    qInfo(LOG_MECHEYE_WORKER).noquote()
        << QStringLiteral("[启动连接] 开始 discover/connect，key=") << m_defaultCameraKey;

    QString errorMessage;
    const int connectTimeoutMs = []() {
        const auto* configManager = common::ConfigManager::instance();
        if (configManager == nullptr) {
            return 5000;
        }
        const int configuredMs = configManager->visionConfig().mechCaptureTimeoutMs;
        return configuredMs > 0 ? configuredMs : 5000;
    }();

    if (connectCamera(m_defaultCameraKey, connectTimeoutMs, &errorMessage)) {
        setRuntimeState(
            CameraRuntimeState::Ready,
            QStringLiteral("相机已连接: %1").arg(m_cameraInfo.serialNumber));
        return;
    }

    qWarning(LOG_MECHEYE_WORKER).noquote()
        << QStringLiteral("[启动连接] 失败: ") << errorMessage;
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
        // 掉线后尝试用默认相机重连，超时 3s（比启动时更短，适合周期性心跳）
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

    // 已连接时仅查询 live CameraInfo，不重复 discover
    mmind::eye::CameraInfo liveInfo;
    const mmind::eye::ErrorStatus status = m_impl->ensureCamera().getCameraInfo(liveInfo);
    if (!status.isOK()) {
        QString errorMessage = QStringLiteral("刷新相机状态失败: %1")
            .arg(QString::fromStdString(status.errorDescription));
        if (mapSdkError(status.errorCode) == CaptureErrorCode::NotConnected) {
            // getCameraInfo 报离线时同步本地连接标志，下次 refresh 会走重连分支
            m_connected = false;
            m_cameraInfo.connected = false;
        }
        setRuntimeState(CameraRuntimeState::Error, errorMessage);
        return;
    }

    m_cameraInfo = makeSnapshot(liveInfo, true);
    // 状态不变，仅更新描述文本供 HMI 显示 IP/序列号
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

    // --- 1. 请求参数规范化 ---
    CaptureRequest normalized = request;
    if (normalized.timeoutMs <= 0) {
        normalized.timeoutMs = 5000;
    }
    if (normalized.cameraKey.trimmed().isEmpty()) {
        normalized.cameraKey = m_defaultCameraKey;
    }

    // --- 2. 重入与连接前置检查 ---
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
    // --- 3. SDK 采图（可能阻塞数秒，必须在 worker 线程） ---
    try {
#endif
        mmind::eye::ErrorStatus status;

        qInfo(LOG_MECHEYE_WORKER).noquote()
            << QStringLiteral("[ScanSync] 梅卡采集时刻 ms=") << QDateTime::currentMSecsSinceEpoch();

        /* 按采集模式分支：2DOnly / 3DOnly（含对比采集）/ 2DAnd3D */
        if (normalized.mode == CaptureMode::Capture2DOnly) {
            mmind::eye::Frame2D frame2D;
            status = m_impl->ensureCamera().capture2D(
                frame2D,
                static_cast<unsigned int>(normalized.timeoutMs));
            if (status.isOK()) {
                result.texture2D = ConvertGrayTextureFrame(frame2D);
            }
        } else if (normalized.mode == CaptureMode::Capture3DOnly) {
            QString errorMessage;
            result.pointCloud = capturePointCloud3D(
                m_impl->ensureCamera(),
                normalized.timeoutMs,
                true,
                &result.elapsedMs,
                &errorMessage);
            if (!errorMessage.isEmpty()) {
                status = mmind::eye::ErrorStatus(
                    mmind::eye::ErrorStatus::MMIND_STATUS_NO_DATA_ERROR,
                    errorMessage.toStdString());
            }

            if (normalized.comparisonCaptureEnabled && status.isOK()) {
                /* 同次再采一帧 NoiseRemoval=Off，用于与主流程 Normal 滤波效果对比 */
                result.comparisonPointCloud = capturePointCloud3DComparison(
                    m_impl->ensureCamera(),
                    normalized.timeoutMs,
                    false,
                    &result.comparisonElapsedMs,
                    &errorMessage);
                if (!errorMessage.isEmpty()) {
                    qWarning(LOG_MECHEYE_WORKER).noquote()
                        << QStringLiteral("[对比采集] ") << errorMessage;
                }
            }
        } else {
            mmind::eye::Frame2DAnd3D frame2DAnd3D;
            applyMech3DCaptureUserSet(m_impl->ensureCamera().currentUserSet(), true);
            status = m_impl->ensureCamera().capture2DAnd3D(
                frame2DAnd3D,
                static_cast<unsigned int>(normalized.timeoutMs));
            if (status.isOK()) {
                result.pointCloud = buildPointCloud2DAnd3D(frame2DAnd3D);
                result.texture2D = ConvertGrayTextureFrame(frame2DAnd3D.frame2D());
            }
        }

        result.elapsedMs = timer.elapsed();

        // --- 4. SDK 层失败处理 ---
        if (!status.isOK()) {
            result.errorCode = mapSdkError(status.errorCode);
            result.errorMessage = QStringLiteral("采集失败: %1")
                .arg(QString::fromStdString(status.errorDescription));
            if (result.errorCode == CaptureErrorCode::NotConnected) {
                // 采图过程中掉线：置 Error，上层需 refresh 或重启
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

        // --- 5. 结果有效性校验（SDK 成功但数据为空） ---
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

        // --- 6. 成功路径 ---
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
        // SDK 抛异常时标记掉线，同时发 fatalError + 失败 CaptureResult 双通道通知
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
    // 将 mmind::eye::ErrorStatus 错误码映射为项目统一 CaptureErrorCode
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

/** @brief SDK CameraInfo → 可跨线程传递的 QString 快照 */
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

/* 确保相机已连接；若 m_connected 已为 true 则直接返回，否则触发 connectCamera */
bool MechEyeWorker::ensureConnected(const QString& cameraKey, int timeoutMs, QString* errorMessage)
{
    if (m_connected) {
        return true;
    }
    return connectCamera(cameraKey, timeoutMs, errorMessage);
}

/* 发现 → 匹配 cameraKey → connect；成功后设置 5s 心跳并打印相机参数 */
bool MechEyeWorker::connectCamera(const QString& cameraKey, int timeoutMs, QString* errorMessage)
{
    setRuntimeState(CameraRuntimeState::Discovering, QStringLiteral("正在搜索相机"));

    m_impl->ensureCamera();

    // --- 阶段 A：局域网 UDP 广播发现 ---
    if (!discoverMechCamerasSafe(
            static_cast<unsigned int>(timeoutMs > 0 ? timeoutMs : 5000),
            m_impl->discoveredCameras,
            errorMessage)) {
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

    // --- 阶段 B：按 cameraKey 筛选目标相机；key 为空则取 discover 列表第一台 ---
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

    // --- 阶段 C：TCP 连接并初始化心跳 ---
    mmind::eye::ErrorStatus status;
    try {
        status = m_impl->ensureCamera().connect(
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
    m_impl->ensureCamera().setHeartbeatInterval(5000);
    // 连接成功后打印相机基础参数
    printCameraParameters();

    return true;
}

/** @brief 断开当前相机；未连接时视为成功（幂等） */
bool MechEyeWorker::disconnectCamera(QString* errorMessage)
{
    if (!m_connected) {
        return true;
    }
    if (m_impl->camera == nullptr) {
        m_connected = false;
        return true;
    }

    try {
        m_impl->camera->disconnect();
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

/* 构造带错误码的失败 CaptureResult，保留 request 上下文与当前相机快照 */
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

/** @brief Frame3D 专用转换入口（当前由 capturePointCloud3D 内联转换，保留供扩展） */
PointCloudFrame MechEyeWorker::buildPointCloud3D(const mmind::eye::Frame3D& frame) const
{
    const auto untexturedCloud = frame.getUntexturedPointCloud();
    return ConvertUntexturedPointCloudToFrame(
        untexturedCloud,
        static_cast<quint64>(frame.frameId()));
}

/** @brief Frame2DAnd3D 模式：从 frame3D 子帧提取无纹理点云 */
PointCloudFrame MechEyeWorker::buildPointCloud2DAnd3D(const mmind::eye::Frame2DAnd3D& frame) const
{
    const auto untexturedCloud = frame.frame3D().getUntexturedPointCloud();
    return ConvertUntexturedPointCloudToFrame(
        untexturedCloud,
        static_cast<quint64>(frame.frame3D().frameId()));
}

/**
 * @brief 连接成功后 dump 相机参数到日志，便于现场联调核对 UserSet
 *
 * 输出：分辨率、曝光/增益、深度范围、点云滤波开关、深度相机内参与畸变。
 * Nano Ultra 无独立 2D 纹理相机，仅打印 depth intrinsics。
 */
void MechEyeWorker::printCameraParameters()
{
    if (!m_connected || !m_impl) {
        return;
    }

    auto& userSet = m_impl->ensureCamera().currentUserSet();

    // 获取当前用户设置名称
    std::string userSetName;
    userSet.getName(userSetName);

    // 获取相机分辨率
    mmind::eye::CameraResolutions resolutions;
    m_impl->ensureCamera().getCameraResolutions(resolutions);

    // 获取相机内参
    mmind::eye::CameraIntrinsics intrinsics;
    m_impl->ensureCamera().getCameraIntrinsics(intrinsics);

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
        << "  离群点去除:" << QString::fromStdString(outlierRemoval);

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
