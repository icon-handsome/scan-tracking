#include "scan_tracking/vision/hik_cxp_camera_service.h"

#include "scan_tracking/vision/hik_mvs_sdk_runtime.h"

#include <QtCore/QDateTime>
#include <QtCore/QElapsedTimer>
#include <QtCore/QPointer>
#include <QtCore/QMetaObject>
#include <QtCore/QMutex>
#include <QtCore/QMutexLocker>
#include <QtCore/QThread>

#include <algorithm>
#include <cstring>
#include <memory>
#include <thread>
#include <vector>
#include <qdebug.h>
#include "MvCameraControl.h"

namespace scan_tracking {
namespace vision {

namespace {

QMutex g_cxpSdkMutex;

QString trimSdkString(const unsigned char* raw, std::size_t maxLength)
{
    if (raw == nullptr || maxLength == 0) {
        return {};
    }
    std::size_t length = 0;
    while (length < maxLength && raw[length] != '\0') {
        ++length;
    }
    return QString::fromLocal8Bit(reinterpret_cast<const char*>(raw), static_cast<int>(length))
        .trimmed();
}

bool containsKey(const QString& text, const QString& key)
{
    return !text.isEmpty() && !key.isEmpty() && text.contains(key, Qt::CaseInsensitive);
}

void extractCxpDeviceInfo(
    const MV_CC_DEVICE_INFO* deviceInfo,
    QString* serialNumber,
    QString* modelName,
    QString* userDefinedName,
    QString* deviceId,
    QString* interfaceId)
{
    if (deviceInfo == nullptr || deviceInfo->nTLayerType != MV_GENTL_CXP_DEVICE) {
        return;
    }
    const MV_CXP_DEVICE_INFO& info = deviceInfo->SpecialInfo.stCXPInfo;
    if (serialNumber != nullptr) {
        *serialNumber = trimSdkString(info.chSerialNumber, sizeof(info.chSerialNumber));
    }
    if (modelName != nullptr) {
        *modelName = trimSdkString(info.chModelName, sizeof(info.chModelName));
    }
    if (userDefinedName != nullptr) {
        *userDefinedName = trimSdkString(info.chUserDefinedName, sizeof(info.chUserDefinedName));
    }
    if (deviceId != nullptr) {
        *deviceId = trimSdkString(info.chDeviceID, sizeof(info.chDeviceID));
    }
    if (interfaceId != nullptr) {
        *interfaceId = trimSdkString(info.chInterfaceID, sizeof(info.chInterfaceID));
    }
}

bool deviceMatchesKey(
    const MV_CC_DEVICE_INFO* deviceInfo,
    const QString& preferredCameraKey)
{
    if (deviceInfo == nullptr || preferredCameraKey.isEmpty()) {
        return preferredCameraKey.isEmpty();
    }

    QString serialNumber;
    QString modelName;
    QString userDefinedName;
    QString deviceId;
    extractCxpDeviceInfo(deviceInfo, &serialNumber, &modelName, &userDefinedName, &deviceId, nullptr);

    return preferredCameraKey.compare(serialNumber, Qt::CaseInsensitive) == 0 ||
           preferredCameraKey.compare(userDefinedName, Qt::CaseInsensitive) == 0 ||
           preferredCameraKey.compare(modelName, Qt::CaseInsensitive) == 0 ||
           preferredCameraKey.compare(deviceId, Qt::CaseInsensitive) == 0 ||
           containsKey(serialNumber, preferredCameraKey) ||
           containsKey(userDefinedName, preferredCameraKey) ||
           containsKey(modelName, preferredCameraKey) ||
           containsKey(deviceId, preferredCameraKey);
}

void logDiscoveredCxpDevices(const MV_CC_DEVICE_INFO_LIST& deviceList)
{
    qInfo() << QStringLiteral("[CXP] 枚举到 %1 台 CoaXPress 设备").arg(deviceList.nDeviceNum);
    for (unsigned int i = 0; i < deviceList.nDeviceNum; ++i) {
        const MV_CC_DEVICE_INFO* deviceInfo = deviceList.pDeviceInfo[i];
        if (deviceInfo == nullptr) {
            continue;
        }
        QString serialNumber;
        QString modelName;
        QString userDefinedName;
        QString deviceId;
        QString interfaceId;
        extractCxpDeviceInfo(
            deviceInfo, &serialNumber, &modelName, &userDefinedName, &deviceId, &interfaceId);
        qInfo().noquote()
            << QStringLiteral("[CXP]  [%1] model=%2 sn=%3 user=%4 deviceId=%5 iface=%6")
                   .arg(i)
                   .arg(modelName, serialNumber, userDefinedName, deviceId, interfaceId);
    }
}

}  // namespace

class HikCxpCameraService::Impl {
public:
    QMutex mutex;
    void* handle = nullptr;
    MV_CC_DEVICE_INFO deviceInfo{};
    QString serialNumber;
    QString modelName;
    QString userDefinedName;
    QString deviceId;
    QString interfaceId;
    bool sdkReady = false;
    bool connected = false;
    bool grabbing = false;
};

void HikCxpCameraService::registerMetaTypes()
{
    static bool registered = false;
    if (registered) {
        return;
    }
    qRegisterMetaType<scan_tracking::vision::VisionErrorCode>("scan_tracking::vision::VisionErrorCode");
    qRegisterMetaType<scan_tracking::vision::HikMonoFrame>("scan_tracking::vision::HikMonoFrame");
    qRegisterMetaType<scan_tracking::vision::HikPoseCaptureResult>(
        "scan_tracking::vision::HikPoseCaptureResult");
    registered = true;
}

HikCxpCameraService::HikCxpCameraService(const QString& roleName, QObject* parent)
    : QObject(parent)
    , m_roleName(roleName)
    , m_impl(new Impl())
{
    registerMetaTypes();
}

HikCxpCameraService::~HikCxpCameraService()
{
    stop();
    delete m_impl;
    m_impl = nullptr;
}

void HikCxpCameraService::start(
    const scan_tracking::common::VisionCameraEndpointConfig& endpointConfig,
    int defaultCaptureTimeoutMs,
    float exposureTimeUs,
    float gain,
    quint32 triggerMode)
{
    m_endpointConfig = endpointConfig;
    m_defaultCaptureTimeoutMs = defaultCaptureTimeoutMs > 0 ? defaultCaptureTimeoutMs : 5000;
    m_exposureTimeUs = exposureTimeUs > 0.0f ? exposureTimeUs : 50000.0f;
    m_gain = gain;
    m_triggerMode = triggerMode;

    QString sdkError;
    if (!acquireHikMvsSdk(&sdkError)) {
        emit fatalError(VisionErrorCode::SdkInitFailed, sdkError);
        return;
    }
    m_impl->sdkReady = true;

    m_started = true;
    emit stateChanged(
        m_roleName,
        QStringLiteral("ready"),
        QStringLiteral("CXP 相机服务已启动，后台正在尝试连接设备。"));
    startAsyncConnect();
}

void HikCxpCameraService::stop()
{
    m_started = false;

    int connectWait = 0;
    while (m_connectInFlight.load() && connectWait < 300) {
        QThread::msleep(10);
        ++connectWait;
    }

    if (m_impl != nullptr && m_impl->handle != nullptr) {
        MV_CC_StopGrabbing(m_impl->handle);
    }

    int waitCount = 0;
    const int maxWaitCount = 600;
    while (m_captureInFlight.load() && waitCount < maxWaitCount) {
        QThread::msleep(10);
        ++waitCount;
    }

    closeDevice();

    if (m_impl != nullptr && m_impl->sdkReady) {
        releaseHikMvsSdk();
        m_impl->sdkReady = false;
    }

    emit stateChanged(m_roleName, QStringLiteral("stopped"), QStringLiteral("CXP 相机服务已停止。"));
}

bool HikCxpCameraService::isStarted() const
{
    return m_started;
}

bool HikCxpCameraService::isConnected() const
{
    if (m_impl == nullptr) {
        return false;
    }
    QMutexLocker locker(&m_impl->mutex);
    return m_impl->connected;
}

QString HikCxpCameraService::roleName() const
{
    return m_roleName;
}

QString HikCxpCameraService::resolveCameraKey(const QString& preferredCameraKey) const
{
    if (!preferredCameraKey.trimmed().isEmpty()) {
        return preferredCameraKey.trimmed();
    }
    if (!m_endpointConfig.cameraKey.trimmed().isEmpty()) {
        return m_endpointConfig.cameraKey.trimmed();
    }
    if (!m_endpointConfig.serialNumber.trimmed().isEmpty()) {
        return m_endpointConfig.serialNumber.trimmed();
    }
    return m_endpointConfig.logicalName.trimmed();
}

bool HikCxpCameraService::captureMonoFrame(
    int timeoutMs,
    const QString& cameraKey,
    QString* errorMessage,
    HikMonoFrame* outFrame)
{
    QMutexLocker sdkLocker(&g_cxpSdkMutex);
    void* handle = nullptr;
    {
        QMutexLocker locker(&m_impl->mutex);
        if (m_impl->handle == nullptr || !m_impl->connected) {
            if (errorMessage != nullptr) {
                *errorMessage = QStringLiteral("CXP 相机尚未连接，无法采图：%1").arg(cameraKey);
            }
            return false;
        }
        if (!m_started) {
            if (errorMessage != nullptr) {
                *errorMessage = QStringLiteral("CXP 相机服务正在停止，取消采图");
            }
            return false;
        }
        handle = m_impl->handle;
    }

    const unsigned int waitMs = static_cast<unsigned int>(timeoutMs > 0 ? timeoutMs : m_defaultCaptureTimeoutMs);
    const unsigned int actualWaitMs = waitMs < 5000 ? 5000 : waitMs;

    bool alreadyGrabbing = false;
    {
        QMutexLocker locker(&m_impl->mutex);
        alreadyGrabbing = m_impl->grabbing;
    }
    if (!alreadyGrabbing) {
        const int startResult = MV_CC_StartGrabbing(handle);
        if (startResult != MV_OK && startResult != MV_E_CALLORDER) {
            if (errorMessage != nullptr) {
                *errorMessage = QStringLiteral("MV_CC_StartGrabbing 失败，错误码=0x%1")
                                    .arg(static_cast<quint32>(startResult), 8, 16, QLatin1Char('0'));
            }
            return false;
        }
        QMutexLocker locker(&m_impl->mutex);
        m_impl->grabbing = true;
    }

    const int clearResult = MV_CC_ClearImageBuffer(handle);
    if (clearResult != MV_OK) {
        qWarning() << QStringLiteral("[%1] MV_CC_ClearImageBuffer 失败 0x%2")
                          .arg(m_roleName)
                          .arg(static_cast<quint32>(clearResult), 8, 16, QLatin1Char('0'));
    }

    if (m_triggerMode == 1) {
        const int triggerResult = MV_CC_TriggerSoftwareExecute(handle);
        if (triggerResult != MV_OK) {
            if (errorMessage != nullptr) {
                *errorMessage = QStringLiteral("MV_CC_TriggerSoftwareExecute 失败，错误码=0x%1")
                                    .arg(static_cast<quint32>(triggerResult), 8, 16, QLatin1Char('0'));
            }
            return false;
        }
    }

    MV_FRAME_OUT frameOut{};
    const int getBufferResult = MV_CC_GetImageBuffer(handle, &frameOut, actualWaitMs);

    if (!m_started) {
        if (getBufferResult == MV_OK && frameOut.pBufAddr != nullptr) {
            MV_CC_FreeImageBuffer(handle, &frameOut);
        }
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("CXP 相机服务正在停止，采图被中断");
        }
        return false;
    }

    if (getBufferResult == MV_OK && frameOut.pBufAddr != nullptr) {
        const int width = static_cast<int>(frameOut.stFrameInfo.nWidth);
        const int height = static_cast<int>(frameOut.stFrameInfo.nHeight);
        const int frameLen = static_cast<int>(frameOut.stFrameInfo.nFrameLen);

        if (width > 0 && height > 0 && frameLen > 0) {
            auto pixels = std::make_shared<std::vector<std::uint8_t>>();
            pixels->assign(
                static_cast<unsigned char*>(frameOut.pBufAddr),
                static_cast<unsigned char*>(frameOut.pBufAddr) + frameLen);

            HikMonoFrame frame;
            frame.pixels = std::move(pixels);
            frame.width = width;
            frame.height = height;
            frame.stride = width;
            frame.frameId = frameOut.stFrameInfo.nFrameNum;
            frame.timestampMs = QDateTime::currentMSecsSinceEpoch();
            frame.sourceCameraKey = cameraKey;
            frame.pixelFormat = QStringLiteral("Mono8");

            MV_CC_FreeImageBuffer(handle, &frameOut);

            if (frame.isValid()) {
                if (outFrame != nullptr) {
                    *outFrame = frame;
                }
                if (errorMessage != nullptr) {
                    errorMessage->clear();
                }
                qInfo() << QStringLiteral("[%1] CXP 采图成功 %2x%3 len=%4")
                               .arg(m_roleName)
                               .arg(width)
                               .arg(height)
                               .arg(frameLen);
                return true;
            }
        }
        MV_CC_FreeImageBuffer(handle, &frameOut);
    }

    thread_local std::vector<unsigned char> fallbackBuffer;
    if (fallbackBuffer.size() < 48U * 1024U * 1024U) {
        fallbackBuffer.resize(48U * 1024U * 1024U);
    }
    MV_FRAME_OUT_INFO_EX frameInfo{};
    const int grabResult = MV_CC_GetOneFrameTimeout(
        handle,
        fallbackBuffer.data(),
        static_cast<unsigned int>(fallbackBuffer.size()),
        &frameInfo,
        actualWaitMs);

    if (!m_started) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("CXP 相机服务正在停止，采图被中断");
        }
        return false;
    }

    if (grabResult != MV_OK) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("CXP 采图失败(GetImageBuffer=0x%1, GetOneFrame=0x%2)")
                                .arg(static_cast<quint32>(getBufferResult), 8, 16, QLatin1Char('0'))
                                .arg(static_cast<quint32>(grabResult), 8, 16, QLatin1Char('0'));
        }
        return false;
    }

    const int width = static_cast<int>(frameInfo.nWidth);
    const int height = static_cast<int>(frameInfo.nHeight);
    const int frameLen = static_cast<int>(frameInfo.nFrameLen);
    if (width <= 0 || height <= 0 || frameLen <= 0 || frameLen > static_cast<int>(fallbackBuffer.size())) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("CXP 图像帧信息无效 width=%1 height=%2 len=%3")
                                .arg(width)
                                .arg(height)
                                .arg(frameLen);
        }
        return false;
    }

    auto pixels = std::make_shared<std::vector<std::uint8_t>>();
    pixels->assign(fallbackBuffer.begin(), fallbackBuffer.begin() + frameLen);

    HikMonoFrame frame;
    frame.pixels = std::move(pixels);
    frame.width = width;
    frame.height = height;
    frame.stride = width;
    frame.frameId = frameInfo.nFrameNum;
    frame.timestampMs = QDateTime::currentMSecsSinceEpoch();
    frame.sourceCameraKey = cameraKey;
    frame.pixelFormat = QStringLiteral("Mono8");

    if (!frame.isValid()) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("CXP 采图成功但帧无效");
        }
        return false;
    }

    if (outFrame != nullptr) {
        *outFrame = frame;
    }
    if (errorMessage != nullptr) {
        errorMessage->clear();
    }
    qInfo() << QStringLiteral("[%1] CXP 采图成功(GetOneFrame) %2x%3")
                   .arg(m_roleName)
                   .arg(width)
                   .arg(height);
    return true;
}

quint64 HikCxpCameraService::requestMonoCapture(const QString& preferredCameraKey, int timeoutMs)
{
    if (!m_started) {
        emit fatalError(VisionErrorCode::NotStarted, QStringLiteral("CXP 相机服务尚未启动。"));
        return 0;
    }
    if (m_captureInFlight.exchange(true)) {
        emit fatalError(VisionErrorCode::Busy, QStringLiteral("CXP 相机采图进行中，请稍后重试。"));
        return 0;
    }

    HikPoseCaptureResult seedResult;
    seedResult.requestId = m_nextRequestId++;
    seedResult.cameraKey = resolveCameraKey(preferredCameraKey);
    seedResult.logicalName = m_endpointConfig.logicalName;
    const int effectiveTimeoutMs = timeoutMs > 0 ? timeoutMs : m_defaultCaptureTimeoutMs;

    QPointer<HikCxpCameraService> self(this);
    std::thread([self, seedResult, preferredCameraKey, effectiveTimeoutMs]() mutable {
        if (!self) {
            return;
        }
        QElapsedTimer timer;
        timer.start();
        QString errorMessage;

        if (!self->ensureConnected(preferredCameraKey, &errorMessage)) {
            seedResult.errorCode = VisionErrorCode::DeviceOpenFailed;
            seedResult.errorMessage = errorMessage;
            seedResult.elapsedMs = timer.elapsed();
        } else {
            HikMonoFrame capturedFrame;
            if (!self->captureMonoFrame(effectiveTimeoutMs, seedResult.cameraKey, &errorMessage, &capturedFrame)) {
                seedResult.errorCode = VisionErrorCode::CaptureRejected;
                seedResult.errorMessage = errorMessage.isEmpty()
                    ? QStringLiteral("CXP Mono 采图失败。")
                    : errorMessage;
                seedResult.elapsedMs = timer.elapsed();
            } else {
                seedResult.errorCode = VisionErrorCode::Success;
                seedResult.errorMessage = QStringLiteral("CXP Mono 采图完成。");
                seedResult.frame = capturedFrame;
                seedResult.frame.sourceCameraKey = seedResult.cameraKey;
                seedResult.elapsedMs = timer.elapsed();
            }
        }

        if (!self) {
            return;
        }
        QMetaObject::invokeMethod(
            self.data(),
            [self, seedResult]() {
                if (!self) {
                    return;
                }
                self->m_captureInFlight = false;
                emit self->monoCaptureFinished(seedResult);
                emit self->poseCaptureFinished(seedResult);
                emit self->stateChanged(
                    self->m_roleName,
                    QStringLiteral("ready"),
                    seedResult.success() ? QStringLiteral("CXP 采图完成。")
                                         : QStringLiteral("CXP 采图失败。"));
            },
            Qt::QueuedConnection);
    }).detach();

    return seedResult.requestId;
}

bool HikCxpCameraService::ensureConnected(const QString& preferredCameraKey, QString* errorMessage)
{
    {
        QMutexLocker locker(&m_impl->mutex);
        if (m_impl->connected && m_impl->handle != nullptr) {
            return true;
        }
    }
    return openMatchedDevice(resolveCameraKey(preferredCameraKey), errorMessage);
}

bool HikCxpCameraService::openMatchedDevice(const QString& preferredCameraKey, QString* errorMessage)
{
    QMutexLocker sdkLocker(&g_cxpSdkMutex);

    if (m_impl == nullptr || !m_impl->sdkReady) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("MVS SDK 尚未初始化。");
        }
        return false;
    }

    if (m_impl->handle != nullptr) {
        MV_CC_StopGrabbing(m_impl->handle);
        MV_CC_CloseDevice(m_impl->handle);
        MV_CC_DestroyHandle(m_impl->handle);
        m_impl->handle = nullptr;
    }

    MV_CC_DEVICE_INFO_LIST deviceList{};
    const int enumResult = MV_CC_EnumDevices(MV_GENTL_CXP_DEVICE, &deviceList);
    if (enumResult != MV_OK) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("枚举 CXP 相机失败，错误码=0x%1")
                                .arg(static_cast<quint32>(enumResult), 8, 16, QLatin1Char('0'));
        }
        return false;
    }

    logDiscoveredCxpDevices(deviceList);

    MV_CC_DEVICE_INFO* matchedDevice = nullptr;
    QString matchedSerial;
    QString matchedModel;
    QString matchedUser;
    QString matchedDeviceId;
    QString matchedInterfaceId;

    for (unsigned int i = 0; i < deviceList.nDeviceNum; ++i) {
        MV_CC_DEVICE_INFO* deviceInfo = deviceList.pDeviceInfo[i];
        if (deviceInfo == nullptr || deviceInfo->nTLayerType != MV_GENTL_CXP_DEVICE) {
            continue;
        }

        if (!preferredCameraKey.isEmpty() && !deviceMatchesKey(deviceInfo, preferredCameraKey)) {
            continue;
        }

        matchedDevice = deviceInfo;
        extractCxpDeviceInfo(
            deviceInfo,
            &matchedSerial,
            &matchedModel,
            &matchedUser,
            &matchedDeviceId,
            &matchedInterfaceId);
        break;
    }

    if (matchedDevice == nullptr) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("未找到匹配的 CXP 相机：%1").arg(preferredCameraKey);
        }
        return false;
    }

    void* handle = nullptr;
    const int createResult = MV_CC_CreateHandle(&handle, matchedDevice);
    if (createResult != MV_OK || handle == nullptr) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("创建 CXP 相机句柄失败，错误码=0x%1")
                                .arg(static_cast<quint32>(createResult), 8, 16, QLatin1Char('0'));
        }
        return false;
    }

    const int openResult = MV_CC_OpenDevice(handle, MV_ACCESS_Exclusive, 0);
    if (openResult != MV_OK) {
        MV_CC_DestroyHandle(handle);
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("打开 CXP 相机失败，错误码=0x%1")
                                .arg(static_cast<quint32>(openResult), 8, 16, QLatin1Char('0'));
        }
        return false;
    }

    const unsigned int triggerModeValue = (m_triggerMode == 2) ? MV_TRIGGER_MODE_ON
                                  : (m_triggerMode == 1) ? MV_TRIGGER_MODE_ON
                                                         : MV_TRIGGER_MODE_OFF;
    int ret = MV_CC_SetEnumValue(handle, "TriggerMode", triggerModeValue);
    if (ret != MV_OK) {
        qWarning() << QStringLiteral("[%1] 设置 TriggerMode=%2 失败 0x%3")
                          .arg(m_roleName)
                          .arg(triggerModeValue)
                          .arg(static_cast<quint32>(ret), 8, 16, QLatin1Char('0'));
    }

    if (m_triggerMode == 1) {
        ret = MV_CC_SetEnumValue(handle, "TriggerSource", MV_TRIGGER_SOURCE_SOFTWARE);
        if (ret != MV_OK) {
            qWarning() << QStringLiteral("[%1] 设置 TriggerSource=SOFTWARE 失败 0x%2")
                              .arg(m_roleName)
                              .arg(static_cast<quint32>(ret), 8, 16, QLatin1Char('0'));
        }
    } else if (m_triggerMode == 2) {
        ret = MV_CC_SetEnumValue(handle, "TriggerSource", MV_TRIGGER_SOURCE_LINE0);
        if (ret != MV_OK) {
            qWarning() << QStringLiteral("[%1] 设置 TriggerSource=LINE0 失败 0x%2")
                              .arg(m_roleName)
                              .arg(static_cast<quint32>(ret), 8, 16, QLatin1Char('0'));
        }
    }

    ret = MV_CC_SetEnumValue(handle, "PixelFormat", 0x01080001);
    if (ret != MV_OK) {
        qWarning() << QStringLiteral("[%1] 设置 Mono8 失败（可能已是其他格式）0x%2")
                          .arg(m_roleName)
                          .arg(static_cast<quint32>(ret), 8, 16, QLatin1Char('0'));
    }

    ret = MV_CC_SetFloatValue(handle, "ExposureTime", m_exposureTimeUs);
    if (ret != MV_OK) {
        qWarning() << QStringLiteral("[%1] 设置 ExposureTime 失败").arg(m_roleName);
    }

    ret = MV_CC_SetFloatValue(handle, "Gain", m_gain);
    if (ret != MV_OK) {
        qWarning() << QStringLiteral("[%1] 设置 Gain 失败").arg(m_roleName);
    }

    ret = MV_CC_StartGrabbing(handle);
    if (ret != MV_OK && ret != MV_E_CALLORDER) {
        qWarning() << QStringLiteral("[%1] StartGrabbing 失败 0x%2")
                          .arg(m_roleName)
                          .arg(static_cast<quint32>(ret), 8, 16, QLatin1Char('0'));
    } else {
        QThread::msleep(100);
    }

    {
        QMutexLocker locker(&m_impl->mutex);
        m_impl->handle = handle;
        m_impl->deviceInfo = *matchedDevice;
        m_impl->serialNumber = matchedSerial;
        m_impl->modelName = matchedModel;
        m_impl->userDefinedName = matchedUser;
        m_impl->deviceId = matchedDeviceId;
        m_impl->interfaceId = matchedInterfaceId;
        m_impl->connected = true;
        m_impl->grabbing = (ret == MV_OK || ret == MV_E_CALLORDER);
    }

    if (errorMessage != nullptr) {
        errorMessage->clear();
    }
    qInfo().noquote() << QStringLiteral("[%1] CXP 已连接 sn=%2 model=%3 iface=%4")
                             .arg(m_roleName, matchedSerial, matchedModel, matchedInterfaceId);
    return true;
}

void HikCxpCameraService::closeDevice()
{
    if (m_impl == nullptr) {
        return;
    }
    QMutexLocker sdkLocker(&g_cxpSdkMutex);
    QMutexLocker locker(&m_impl->mutex);
    if (m_impl->handle != nullptr) {
        MV_CC_StopGrabbing(m_impl->handle);
        MV_CC_CloseDevice(m_impl->handle);
        MV_CC_DestroyHandle(m_impl->handle);
        m_impl->handle = nullptr;
    }
    m_impl->connected = false;
    m_impl->grabbing = false;
}

void HikCxpCameraService::startAsyncConnect()
{
    if (!m_started || m_connectInFlight.exchange(true)) {
        return;
    }

    const QString cameraKey = resolveCameraKey({});
    QString errorMessage;
    const bool ok = openMatchedDevice(cameraKey, &errorMessage);
    m_connectInFlight = false;
    if (!m_started) {
        return;
    }

    if (ok) {
        emit stateChanged(
            m_roleName,
            QStringLiteral("connected"),
            QStringLiteral("CXP 相机已连接：%1 (%2)")
                .arg(m_impl->serialNumber, m_impl->modelName));
    } else {
        emit stateChanged(
            m_roleName,
            QStringLiteral("ready"),
            QStringLiteral("CXP 服务已启动，尚未连接：%1").arg(errorMessage));
        emit fatalError(
            VisionErrorCode::DeviceNotFound,
            QStringLiteral("后台连接 CXP 相机失败：%1").arg(cameraKey));
    }
}

}  // namespace vision
}  // namespace scan_tracking
