#pragma once

// 海康 CoaXPress（PCIe 采集卡）相机服务。
//
// 通过 MVS SDK 按序列号匹配 CXP 设备，在独立 std::thread 中执行连接与采图，
// 避免阻塞主线程及 PLC Modbus 轮询。供 VisionPipelineService 作为双目左 A / 右 B 使用。
// 输出 Mono8 灰度帧，经 HikPoseCaptureResult 异步回传。

#include <QtCore/QObject>

#include <atomic>

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/vision/vision_types.h"

namespace scan_tracking {
namespace vision {

class HikCxpCameraService : public QObject {
    Q_OBJECT

public:
    /* @param roleName 逻辑角色名（如 "CXP-A" / "CXP-B"），用于日志与 stateChanged */
    explicit HikCxpCameraService(const QString& roleName, QObject* parent = nullptr);
    ~HikCxpCameraService() override;

    /* 启动服务：保存端点配置，后台异步连接设备并写入曝光/增益 */
    void start(
        const scan_tracking::common::VisionCameraEndpointConfig& endpointConfig,
        int defaultCaptureTimeoutMs,
        float exposureTimeUs = 50000.0f,
        float gain = 0.0f,
        quint32 triggerMode = 1);

    /* 停止服务：关闭设备并拒绝新采集请求 */
    void stop();

    bool isStarted() const;
    bool isConnected() const;
    QString roleName() const;

    /* 请求一次 Mono8 采图
     * @param preferredCameraKey 优先匹配的设备键（序列号等），空则用配置默认值
     * @param timeoutMs          采图超时，0 表示使用 start 时的默认值
     * @return 请求 ID；忙或未启动时返回 0，结果经 monoCaptureFinished 异步返回
     */
    quint64 requestMonoCapture(const QString& preferredCameraKey = {}, int timeoutMs = 0);

    /// 与 HikCameraService 对齐，供 VisionPipeline 组合采集调用。
    quint64 requestPoseCapture(const QString& preferredCameraKey = {}, int timeoutMs = 0)
    {
        return requestMonoCapture(preferredCameraKey, timeoutMs);
    }

signals:
    void monoCaptureFinished(scan_tracking::vision::HikPoseCaptureResult result);
    void poseCaptureFinished(scan_tracking::vision::HikPoseCaptureResult result);
    void stateChanged(QString roleName, QString stateText, QString description);
    void fatalError(scan_tracking::vision::VisionErrorCode code, QString message);

private:
    class Impl;  ///< PIMPL：封装 MVS CXP 设备句柄与采图缓冲，隔离 SDK 头文件

    static void registerMetaTypes();
    QString resolveCameraKey(const QString& preferredCameraKey) const;
    bool ensureConnected(const QString& preferredCameraKey, QString* errorMessage);
    bool captureMonoFrame(
        int timeoutMs,
        const QString& cameraKey,
        QString* errorMessage,
        HikMonoFrame* outFrame = nullptr);
    bool openMatchedDevice(const QString& preferredCameraKey, QString* errorMessage);
    void closeDevice();
    void startAsyncConnect();

    QString m_roleName;
    scan_tracking::common::VisionCameraEndpointConfig m_endpointConfig;
    int m_defaultCaptureTimeoutMs = 5000;
    float m_exposureTimeUs = 50000.0f;
    float m_gain = 0.0f;
    quint32 m_triggerMode = 1;
    quint64 m_nextRequestId = 1;
    bool m_started = false;
    std::atomic_bool m_connectInFlight{false};
    std::atomic_bool m_captureInFlight{false};
    Impl* m_impl = nullptr;
};

}  // namespace vision
}  // namespace scan_tracking
