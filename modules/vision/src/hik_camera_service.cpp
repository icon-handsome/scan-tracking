#include "scan_tracking/vision/hik_camera_service.h"

#include <QtCore/QDateTime>
#include <QtCore/QElapsedTimer>
#include <QtCore/QMetaObject>
#include <QtCore/QMutex>
#include <QtCore/QMutexLocker>
#include <QtCore/QThread>

#include <algorithm>
#include <cstring>
#include <memory>
#include <thread>
#include <vector>
#include<qdebug.h>
#include "MvCameraControl.h"

namespace scan_tracking {
namespace vision {

namespace {

QMutex g_sdkMutex;
int g_sdkRefCount = 0;

QString trimSdkString(const unsigned char* raw, std::size_t maxLength)
{
    if (raw == nullptr || maxLength == 0) {
        return {};
    }
    std::size_t length = 0;
    while (length < maxLength && raw[length] != '\0') {
        ++length;
    }
    return QString::fromLocal8Bit(reinterpret_cast<const char*>(raw), static_cast<int>(length)).trimmed();
}

QString ipToString(unsigned int ip)
{
    return QStringLiteral("%1.%2.%3.%4")
        .arg((ip >> 24) & 0xFFu)
        .arg((ip >> 16) & 0xFFu)
        .arg((ip >> 8) & 0xFFu)
        .arg(ip & 0xFFu);
}

bool containsKey(const QString& text, const QString& key)
{
    return !text.isEmpty() && !key.isEmpty() && text.contains(key, Qt::CaseInsensitive);
}

}  // namespace

class HikCameraService::Impl {
public:
    QMutex mutex;
    void* handle = nullptr;
    MV_CC_DEVICE_INFO deviceInfo{};
    QString modelName;
    QString serialNumber;
    QString ipAddress;
    QString userDefinedName;
    QString lastPixelFormat;
    int lastFrameWidth = 0;
    int lastFrameHeight = 0;
    quint64 lastFrameId = 0;
    bool sdkReady = false;
    bool connected = false;
};

void HikCameraService::registerMetaTypes()
{
    static bool registered = false;
    if (registered) {
        return;
    }
    qRegisterMetaType<scan_tracking::vision::VisionErrorCode>("scan_tracking::vision::VisionErrorCode");
    qRegisterMetaType<scan_tracking::vision::PoseMatrix4x4>("scan_tracking::vision::PoseMatrix4x4");
    qRegisterMetaType<scan_tracking::vision::HikMonoFrame>("scan_tracking::vision::HikMonoFrame");
    qRegisterMetaType<scan_tracking::vision::HikPoseCaptureRequest>("scan_tracking::vision::HikPoseCaptureRequest");
    qRegisterMetaType<scan_tracking::vision::HikPoseCaptureResult>("scan_tracking::vision::HikPoseCaptureResult");
    registered = true;
}

HikCameraService::HikCameraService(const QString& roleName, QObject* parent)
    : QObject(parent)
    , m_roleName(roleName)
    , m_impl(new Impl())
{
    registerMetaTypes();
}

HikCameraService::~HikCameraService()
{
    stop();
    delete m_impl;
    m_impl = nullptr;
}

void HikCameraService::start(
    const scan_tracking::common::VisionCameraEndpointConfig& endpointConfig,
    int defaultCaptureTimeoutMs)
{
    m_endpointConfig = endpointConfig;
    m_defaultCaptureTimeoutMs = defaultCaptureTimeoutMs > 0 ? defaultCaptureTimeoutMs : 1000;

    {
        QMutexLocker locker(&g_sdkMutex);
        if (!m_impl->sdkReady) {
            const int result = MV_CC_Initialize();
            if (MV_OK != result) {
                emit fatalError(VisionErrorCode::SdkInitFailed,
                                QStringLiteral("MVS SDK 初始化失败，错误码=0x%1")
                                    .arg(static_cast<quint32>(result), 8, 16, QLatin1Char('0')));
                return;
            }
            ++g_sdkRefCount;
            m_impl->sdkReady = true;
        }
    }

    m_started = true;
    emit stateChanged(m_roleName, QStringLiteral("ready"), QStringLiteral("海康相机服务已启动，后台正在尝试连接设备。"));
    startAsyncConnect();
}

void HikCameraService::stop()
{
    m_started = false;
    closeDevice();

    if (m_impl != nullptr && m_impl->sdkReady) {
        QMutexLocker locker(&g_sdkMutex);
        if (g_sdkRefCount > 0) {
            --g_sdkRefCount;
        }
        if (g_sdkRefCount == 0) {
            MV_CC_Finalize();
        }
        m_impl->sdkReady = false;
    }

    emit stateChanged(m_roleName, QStringLiteral("stopped"), QStringLiteral("海康相机服务已停止。"));
}

bool HikCameraService::isConnected() const
{
    if (m_impl == nullptr) {
        return false;
    }
    QMutexLocker locker(&m_impl->mutex);
    return m_impl->connected;
}

QString HikCameraService::resolveCameraKey(const QString& preferredCameraKey) const
{
    if (!preferredCameraKey.trimmed().isEmpty()) return preferredCameraKey.trimmed();
    if (!m_endpointConfig.cameraKey.trimmed().isEmpty()) return m_endpointConfig.cameraKey.trimmed();
    if (!m_endpointConfig.ipAddress.trimmed().isEmpty()) return m_endpointConfig.ipAddress.trimmed();
    if (!m_endpointConfig.serialNumber.trimmed().isEmpty()) return m_endpointConfig.serialNumber.trimmed();
    return m_endpointConfig.logicalName.trimmed();
}

bool HikCameraService::captureMonoFrame(int timeoutMs, const QString& cameraKey, QString* errorMessage)
{
    QMutexLocker locker(&m_impl->mutex);
    if (m_impl->handle == nullptr || !m_impl->connected) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("海康相机尚未连接，无法抓取黑白图：%1").arg(cameraKey);
        }
        return false;
    }

    // 增加超时时间到 5 秒
    const unsigned int waitMs = static_cast<unsigned int>(timeoutMs > 0 ? timeoutMs : m_defaultCaptureTimeoutMs);
    const unsigned int actualWaitMs = waitMs < 5000 ? 5000 : waitMs;
    
    qInfo() << "[采图] 开始采图，超时=" << actualWaitMs << "ms";
    
    // 尝试使用 MV_CC_GetImageBuffer 代替 GetOneFrameTimeout
    // 这个 API 更适合连续采集模式
    
    // 相机已经在连接时启动了采集
    const int startResult = MV_CC_StartGrabbing(m_impl->handle);
    if (startResult != MV_OK && startResult != MV_E_CALLORDER) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("MV_CC_StartGrabbing 失败，错误码=0x%1")
                .arg(static_cast<quint32>(startResult), 8, 16, QLatin1Char('0'));
        }
        qWarning() << "[采图] StartGrabbing 失败，错误码=0x" << QString::number(startResult, 16);
        return false;
    }
    if (startResult == MV_E_CALLORDER) {
        qInfo() << "[采图] 相机已在采集中";
    } else {
        qInfo() << "[采图] StartGrabbing 成功";
    }

    qInfo() << "[采图] 等待图像数据...";
    
    // 尝试使用 GetImageBuffer（推荐用于连续采集）
    MV_FRAME_OUT pFrameInfo = {0};
    int getBufferResult = MV_CC_GetImageBuffer(m_impl->handle, &pFrameInfo, actualWaitMs);
    
    if (getBufferResult == MV_OK && pFrameInfo.pBufAddr != nullptr) {
        qInfo() << "[采图] GetImageBuffer 成功获取图像";
        
        const int width = static_cast<int>(pFrameInfo.stFrameInfo.nWidth);
        const int height = static_cast<int>(pFrameInfo.stFrameInfo.nHeight);
        const int frameLen = static_cast<int>(pFrameInfo.stFrameInfo.nFrameLen);
        
        qInfo() << "[采图] 图像信息: width=" << width << "height=" << height << "len=" << frameLen;
        
        if (width > 0 && height > 0 && frameLen > 0 && pFrameInfo.pBufAddr != nullptr) {
            auto pixels = std::make_shared<std::vector<std::uint8_t>>();
            pixels->assign(
                static_cast<unsigned char*>(pFrameInfo.pBufAddr),
                static_cast<unsigned char*>(pFrameInfo.pBufAddr) + frameLen);
            
            HikMonoFrame frame;
            frame.pixels = std::move(pixels);
            frame.width = width;
            frame.height = height;
            frame.stride = width;
            frame.frameId = pFrameInfo.stFrameInfo.nFrameNum;
            frame.timestampMs = QDateTime::currentMSecsSinceEpoch();
            frame.sourceCameraKey = cameraKey;
            
            // 释放图像缓冲区
            MV_CC_FreeImageBuffer(m_impl->handle, &pFrameInfo);
            
            if (frame.isValid()) {
                m_impl->lastFrameWidth = width;
                m_impl->lastFrameHeight = height;
                m_impl->lastFrameId = pFrameInfo.stFrameInfo.nFrameNum;
                m_impl->lastPixelFormat = QStringLiteral("Mono8");
                if (errorMessage) {
                    errorMessage->clear();
                }
                qInfo() << "[采图] 采图完成，帧ID=" << pFrameInfo.stFrameInfo.nFrameNum;
                return true;
            }
        }
        
        MV_CC_FreeImageBuffer(m_impl->handle, &pFrameInfo);
    }
    
    qWarning() << "[采图] GetImageBuffer 失败，尝试 GetOneFrameTimeout，错误码=0x" << QString::number(getBufferResult, 16);
    
    // 如果 GetImageBuffer 失败，回退到 GetOneFrameTimeout
    std::vector<unsigned char> buffer(16 * 1024 * 1024);
    MV_FRAME_OUT_INFO_EX frameInfo;
    std::memset(&frameInfo, 0, sizeof(frameInfo));
    
    const int grabResult = MV_CC_GetOneFrameTimeout(
        m_impl->handle,
        buffer.data(),
        static_cast<unsigned int>(buffer.size()),
        &frameInfo,
        actualWaitMs);

    if (grabResult != MV_OK) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("MV_CC_GetOneFrameTimeout 失败，错误码=0x%1，请检查：1.镜头盖是否打开 2.光照是否充足 3.相机是否正常工作")
                .arg(static_cast<quint32>(grabResult), 8, 16, QLatin1Char('0'));
        }
        qWarning() << "[采图] GetOneFrameTimeout 失败，错误码=0x" << QString::number(grabResult, 16);
        qWarning() << "[采图] 请检查：1.镜头盖是否打开 2.光照是否充足 3.使用 MVS Viewer 测试相机";
        return false;
    }
    
    qInfo() << "[采图] 获取图像成功";

    const int width = static_cast<int>(frameInfo.nWidth);
    const int height = static_cast<int>(frameInfo.nHeight);
    const int frameLen = static_cast<int>(frameInfo.nFrameLen);
    
    qInfo() << "[采图] 图像信息: width=" << width << "height=" << height << "len=" << frameLen;
    
    if (width <= 0 || height <= 0 || frameLen <= 0 || frameLen > static_cast<int>(buffer.size())) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("海康图像帧信息无效。width=%1 height=%2 len=%3")
                .arg(width).arg(height).arg(frameLen);
        }
        return false;
    }

    auto pixels = std::make_shared<std::vector<std::uint8_t>>();
    pixels->assign(buffer.begin(), buffer.begin() + frameLen);

    HikMonoFrame frame;
    frame.pixels = std::move(pixels);
    frame.width = width;
    frame.height = height;
    frame.stride = width;
    frame.frameId = frameInfo.nFrameNum;
    frame.timestampMs = QDateTime::currentMSecsSinceEpoch();
    frame.sourceCameraKey = cameraKey;

    if (!frame.isValid()) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("海康采图成功但黑白帧无效。");
        }
        return false;
    }

    m_impl->lastFrameWidth = width;
    m_impl->lastFrameHeight = height;
    m_impl->lastFrameId = frameInfo.nFrameNum;
    m_impl->lastPixelFormat = QStringLiteral("Mono8");
    if (errorMessage) {
        errorMessage->clear();
    }
    
    qInfo() << "[采图] 采图完成，帧ID=" << frameInfo.nFrameNum;
    return true;
}

quint64 HikCameraService::requestPoseCapture(const QString& preferredCameraKey, int timeoutMs)
{
    if (!m_started) {
        emit fatalError(VisionErrorCode::NotStarted, QStringLiteral("海康相机服务尚未启动。"));
        return 0;
    }
    if (m_captureInFlight.exchange(true)) {
        emit fatalError(VisionErrorCode::Busy, QStringLiteral("海康相机采集请求正在执行，请稍后重试。"));
        return 0;
    }

    HikPoseCaptureResult seedResult;
    seedResult.requestId = m_nextRequestId++;
    seedResult.cameraKey = resolveCameraKey(preferredCameraKey);
    seedResult.logicalName = m_endpointConfig.logicalName;
    const int effectiveTimeoutMs = timeoutMs > 0 ? timeoutMs : m_defaultCaptureTimeoutMs;

    std::thread([this, seedResult, preferredCameraKey, effectiveTimeoutMs]() mutable {
        QElapsedTimer timer;
        timer.start();
        QString errorMessage;

        if (!ensureConnected(preferredCameraKey, &errorMessage)) {
            seedResult.errorCode = VisionErrorCode::DeviceOpenFailed;
            seedResult.errorMessage = errorMessage;
            seedResult.elapsedMs = timer.elapsed();
        } else if (!captureMonoFrame(effectiveTimeoutMs, seedResult.cameraKey, &errorMessage)) {
            seedResult.errorCode = VisionErrorCode::CaptureRejected;
            seedResult.errorMessage = errorMessage.isEmpty()
                ? QStringLiteral("海康 Mono8 采图失败。")
                : errorMessage;
            seedResult.elapsedMs = timer.elapsed();
        } else {
            seedResult.errorCode = VisionErrorCode::Success;
            seedResult.errorMessage = QStringLiteral("海康 Mono8 黑白采图完成。");
            seedResult.frame.frameId = seedResult.requestId;
            seedResult.frame.sourceCameraKey = seedResult.cameraKey;
            seedResult.elapsedMs = timer.elapsed();
        }

        QMetaObject::invokeMethod(
            this,
            [this, seedResult]() {
                m_captureInFlight = false;
                emit poseCaptureFinished(seedResult);
                emit stateChanged(
                    m_roleName,
                    QStringLiteral("ready"),
                    seedResult.success()
                        ? QStringLiteral("海康相机完成 Mono8 黑白采图，等待下一次请求。")
                        : QStringLiteral("海康相机黑白采图失败，请检查相机状态。"));
            },
            Qt::QueuedConnection);
    }).detach();

    return seedResult.requestId;
}

bool HikCameraService::ensureConnected(const QString& preferredCameraKey, QString* errorMessage)
{
    {
        QMutexLocker locker(&m_impl->mutex);
        if (m_impl->connected && m_impl->handle != nullptr) {
            return true;
        }
    }
    return openMatchedDevice(resolveCameraKey(preferredCameraKey), errorMessage);
}

bool HikCameraService::openMatchedDevice(const QString& preferredCameraKey, QString* errorMessage)
{
    if (m_impl == nullptr || !m_impl->sdkReady) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("MVS SDK 尚未初始化。");
        }
        return false;
    }

    closeDevice();

    MV_CC_DEVICE_INFO_LIST deviceList;
    std::memset(&deviceList, 0, sizeof(deviceList));
    const int enumResult = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &deviceList);
    if (enumResult != MV_OK) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("枚举海康相机失败，错误码=0x%1")
                .arg(static_cast<quint32>(enumResult), 8, 16, QLatin1Char('0'));
        }
        return false;
    }

    MV_CC_DEVICE_INFO* matchedDevice = nullptr;
    QString matchedIpAddress;
    QString matchedSerialNumber;
    QString matchedModelName;
    QString matchedUserDefinedName;

    for (unsigned int i = 0; i < deviceList.nDeviceNum; ++i) {
        MV_CC_DEVICE_INFO* deviceInfo = deviceList.pDeviceInfo[i];
        if (deviceInfo == nullptr) {
            continue;
        }

        QString ipAddress;
        QString serialNumber;
        QString modelName;
        QString userDefinedName;

        if (deviceInfo->nTLayerType == MV_GIGE_DEVICE) {
            ipAddress = ipToString(deviceInfo->SpecialInfo.stGigEInfo.nCurrentIp);
            serialNumber = trimSdkString(deviceInfo->SpecialInfo.stGigEInfo.chSerialNumber, sizeof(deviceInfo->SpecialInfo.stGigEInfo.chSerialNumber));
            modelName = trimSdkString(deviceInfo->SpecialInfo.stGigEInfo.chModelName, sizeof(deviceInfo->SpecialInfo.stGigEInfo.chModelName));
            userDefinedName = trimSdkString(deviceInfo->SpecialInfo.stGigEInfo.chUserDefinedName, sizeof(deviceInfo->SpecialInfo.stGigEInfo.chUserDefinedName));
        } else if (deviceInfo->nTLayerType == MV_USB_DEVICE) {
            serialNumber = trimSdkString(deviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber, sizeof(deviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber));
            modelName = trimSdkString(deviceInfo->SpecialInfo.stUsb3VInfo.chModelName, sizeof(deviceInfo->SpecialInfo.stUsb3VInfo.chModelName));
            userDefinedName = trimSdkString(deviceInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName, sizeof(deviceInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName));
        }

        const bool matched = preferredCameraKey.isEmpty() ||
            preferredCameraKey.compare(ipAddress, Qt::CaseInsensitive) == 0 ||
            preferredCameraKey.compare(serialNumber, Qt::CaseInsensitive) == 0 ||
            preferredCameraKey.compare(userDefinedName, Qt::CaseInsensitive) == 0 ||
            preferredCameraKey.compare(modelName, Qt::CaseInsensitive) == 0 ||
            containsKey(ipAddress, preferredCameraKey) ||
            containsKey(serialNumber, preferredCameraKey) ||
            containsKey(userDefinedName, preferredCameraKey) ||
            containsKey(modelName, preferredCameraKey);

        if (matched) {
            matchedDevice = deviceInfo;
            matchedIpAddress = ipAddress;
            matchedSerialNumber = serialNumber;
            matchedModelName = modelName;
            matchedUserDefinedName = userDefinedName;
            break;
        }
    }

    if (matchedDevice == nullptr) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("未找到匹配的海康相机：%1").arg(preferredCameraKey);
        }
        return false;
    }

    if (!MV_CC_IsDeviceAccessible(matchedDevice, MV_ACCESS_Exclusive)) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("目标海康相机当前不可独占访问：%1").arg(preferredCameraKey);
        }
        return false;
    }

    void* handle = nullptr;
    const int createResult = MV_CC_CreateHandle(&handle, matchedDevice);
    if (createResult != MV_OK || handle == nullptr) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("创建海康相机句柄失败，错误码=0x%1")
                .arg(static_cast<quint32>(createResult), 8, 16, QLatin1Char('0'));
        }
        return false;
    }

    const int openResult = MV_CC_OpenDevice(handle, MV_ACCESS_Exclusive, 0);
    if (openResult != MV_OK) {
        MV_CC_DestroyHandle(handle);
        if (errorMessage) {
            *errorMessage = QStringLiteral("打开海康相机失败，错误码=0x%1")
                .arg(static_cast<quint32>(openResult), 8, 16, QLatin1Char('0'));
        }
        return false;
    }

    if (matchedDevice->nTLayerType == MV_GIGE_DEVICE) {
        const int packetSize = MV_CC_GetOptimalPacketSize(handle);
        if (packetSize > 0) {
            MV_CC_SetIntValueEx(handle, "GevSCPSPacketSize", packetSize);
        }
    }

    // 配置相机参数
    int ret = 0;
    
    // 1. 设置触发模式为关闭（连续采集）
    ret = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
    if (ret != MV_OK) {
        qWarning() << "设置 TriggerMode 失败，错误码=0x" << QString::number(ret, 16);
    } else {
        qInfo() << "设置 TriggerMode=0 (连续采集) 成功";
    }
    
    // 2. 设置像素格式为 Mono8
    ret = MV_CC_SetEnumValue(handle, "PixelFormat", 0x01080001);
    if (ret != MV_OK) {
        qWarning() << "设置 PixelFormat 失败，错误码=0x" << QString::number(ret, 16);
    } else {
        qInfo() << "设置 PixelFormat=Mono8 成功";
    }
    
    // 3. 设置曝光时间（微秒）- 增加到 50ms
    ret = MV_CC_SetFloatValue(handle, "ExposureTime", 50000.0f);
    if (ret != MV_OK) {
        qWarning() << "设置 ExposureTime 失败，错误码=0x" << QString::number(ret, 16);
    } else {
        qInfo() << "设置 ExposureTime=50000us 成功";
    }
    
    // 4. 设置增益
    ret = MV_CC_SetFloatValue(handle, "Gain", 0.0f);
    if (ret != MV_OK) {
        qWarning() << "设置 Gain 失败，错误码=0x" << QString::number(ret, 16);
    } else {
        qInfo() << "设置 Gain=0dB 成功";
    }
    
    // 5. 读取并验证当前参数
    MVCC_ENUMVALUE triggerModeValue;
    if (MV_CC_GetEnumValue(handle, "TriggerMode", &triggerModeValue) == MV_OK) {
        qInfo() << "当前 TriggerMode =" << triggerModeValue.nCurValue;
    }
    
    MVCC_FLOATVALUE exposureValue;
    if (MV_CC_GetFloatValue(handle, "ExposureTime", &exposureValue) == MV_OK) {
        qInfo() << "当前 ExposureTime =" << exposureValue.fCurValue << "us";
    }
    
    // 6. 检查相机是否支持 AcquisitionMode
    MVCC_ENUMVALUE acqMode;
    if (MV_CC_GetEnumValue(handle, "AcquisitionMode", &acqMode) == MV_OK) {
        qInfo() << "当前 AcquisitionMode =" << acqMode.nCurValue;
        // 设置为连续采集模式
        MV_CC_SetEnumValue(handle, "AcquisitionMode", 2);  // 2=Continuous
        qInfo() << "设置 AcquisitionMode=2 (Continuous)";
    }
    
    // 7. 启动采集（提前启动，让相机开始准备图像）
    ret = MV_CC_StartGrabbing(handle);
    if (ret != MV_OK) {
        qWarning() << "提前启动采集失败，错误码=0x" << QString::number(ret, 16);
    } else {
        qInfo() << "提前启动采集成功，相机开始准备图像";
        
        // 等待一小段时间让相机稳定
        QThread::msleep(100);
        qInfo() << "等待相机稳定完成";
    }

    {
        QMutexLocker locker(&m_impl->mutex);
        m_impl->handle = handle;
        m_impl->deviceInfo = *matchedDevice;
        m_impl->ipAddress = matchedIpAddress;
        m_impl->serialNumber = matchedSerialNumber;
        m_impl->modelName = matchedModelName;
        m_impl->userDefinedName = matchedUserDefinedName;
        m_impl->connected = true;
    }

    if (errorMessage) {
        errorMessage->clear();
    }
    return true;
}

void HikCameraService::closeDevice()
{
    if (m_impl == nullptr) {
        return;
    }
    QMutexLocker locker(&m_impl->mutex);
    if (m_impl->handle != nullptr) {
        MV_CC_CloseDevice(m_impl->handle);
        MV_CC_DestroyHandle(m_impl->handle);
        m_impl->handle = nullptr;
    }
    m_impl->connected = false;
}

void HikCameraService::startAsyncConnect()
{
    if (!m_started || m_connectInFlight.exchange(true)) {
        return;
    }

    std::thread([this]() {
        const QString cameraKey = resolveCameraKey({});
        QString errorMessage;
        const bool ok = openMatchedDevice(cameraKey, &errorMessage);

        QMetaObject::invokeMethod(
            this,
            [this, ok, cameraKey, errorMessage]() {
                m_connectInFlight = false;
                if (!m_started) {
                    return;
                }
                if (ok) {
                    emit stateChanged(m_roleName, QStringLiteral("ready"), QStringLiteral("海康相机已连接：%1 (%2)").arg(m_impl->serialNumber, m_impl->ipAddress));
                } else {
                    emit stateChanged(m_roleName, QStringLiteral("ready"), QStringLiteral("海康相机服务已启动，但尚未连接设备：%1").arg(errorMessage));
                    emit fatalError(VisionErrorCode::DeviceNotFound, QStringLiteral("后台连接海康相机失败：%1").arg(cameraKey));
                }
            },
            Qt::QueuedConnection);
    }).detach();
}

}  // namespace vision
}  // namespace scan_tracking
