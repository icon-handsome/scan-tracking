#pragma once

// 海康相机服务骨架。
// 当前版本已经接入 MVS SDK 的设备枚举与打开流程，
// 并将连接与采集请求放到后台线程执行，避免阻塞主线程和 PLC 轮询链路。

#include <QtCore/QObject>

#include <atomic>

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/vision/vision_types.h"

namespace scan_tracking {
namespace vision {

class HikCameraService : public QObject {
    Q_OBJECT

public:
    explicit HikCameraService(const QString& roleName, QObject* parent = nullptr);
    ~HikCameraService() override;

    void start(
        const scan_tracking::common::VisionCameraEndpointConfig& endpointConfig,
        int defaultCaptureTimeoutMs);
    void stop();

    bool isStarted() const { return m_started; }
    bool isConnected() const;
    QString roleName() const { return m_roleName; }
    const scan_tracking::common::VisionCameraEndpointConfig& endpointConfig() const
    {
        return m_endpointConfig;
    }

    quint64 requestPoseCapture(const QString& preferredCameraKey = {}, int timeoutMs = 0);

signals:
    void poseCaptureFinished(scan_tracking::vision::HikPoseCaptureResult result);
    void stateChanged(QString roleName, QString stateText, QString description);
    void fatalError(scan_tracking::vision::VisionErrorCode code, QString message);

private:
    class Impl;

    static void registerMetaTypes();
    QString resolveCameraKey(const QString& preferredCameraKey) const;
    bool ensureConnected(const QString& preferredCameraKey, QString* errorMessage);
    bool captureMonoFrame(int timeoutMs, const QString& cameraKey, QString* errorMessage);
    bool openMatchedDevice(const QString& preferredCameraKey, QString* errorMessage);
    void closeDevice();
    void startAsyncConnect();

    QString m_roleName;
    scan_tracking::common::VisionCameraEndpointConfig m_endpointConfig;
    int m_defaultCaptureTimeoutMs = 1000;
    quint64 m_nextRequestId = 1;
    bool m_started = false;
    std::atomic_bool m_connectInFlight = false;
    std::atomic_bool m_captureInFlight = false;
    Impl* m_impl = nullptr;
};

}  // namespace vision
}  // namespace scan_tracking
