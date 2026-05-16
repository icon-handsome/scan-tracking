#pragma once

// Worker 只负责在独立线程中操作 SDK。
// Facade 会把上层请求投递到这里，避免主线程直接触碰阻塞式接口。

#include <QtCore/QObject>
#include <QtCore/QString>

#include "scan_tracking/mech_eye/mech_eye_types.h"

namespace mmind {
namespace eye {
struct CameraInfo;
class Frame3D;
class Frame2DAnd3D;
}  // namespace eye
}  // namespace mmind

namespace scan_tracking {
namespace mech_eye {

class MechEyeWorker : public QObject {
    Q_OBJECT

public:
    /* 构造函数
     * @param parent Qt 父对象指针
     */
    explicit MechEyeWorker(QObject* parent = nullptr);

    /* 析构函数，负责在对象销毁前断开相机并释放内部实现 */
    ~MechEyeWorker() override;

public slots:
    /* 启动 worker，并尝试连接默认相机
     * @param defaultCameraKey 默认相机标识
     */
    void startWorker(const QString& defaultCameraKey);

    /* 停止 worker，并安全释放 SDK 资源 */
    void stopWorker();

    /* 刷新相机状态，必要时重新连接 */
    void refreshStatus();

    /* 执行一次采集请求
     * @param request 采集请求参数
     */
    void performCapture(const scan_tracking::mech_eye::CaptureRequest& request);

signals:
    /* 采集完成信号
     * @param result 采集结果
     */
    void captureFinished(scan_tracking::mech_eye::CaptureResult result);

    /* 状态变化信号
     * @param newState 新状态
     * @param description 状态说明
     */
    void stateChanged(scan_tracking::mech_eye::CameraRuntimeState newState, QString description);

    /* 致命错误信号
     * @param code 错误码
     * @param message 错误描述
     */
    void fatalError(scan_tracking::mech_eye::CaptureErrorCode code, QString message);

private:
    class Impl;

    /* 设置运行状态并向外发射通知
     * @param newState 新状态
     * @param description 状态说明
     */
    void setRuntimeState(CameraRuntimeState newState, const QString& description = {});

    /* 将 SDK 错误码映射为项目内部错误码
     * @param sdkErrorCode SDK 返回值
     * @return 项目内部错误码
     */
    CaptureErrorCode mapSdkError(int sdkErrorCode) const;

    /* 将相机信息转换为快照结构
     * @param info SDK 相机信息
     * @param connected 是否认为当前已连接
     * @return 相机快照
     */
    CameraInfoSnapshot makeSnapshot(const mmind::eye::CameraInfo& info, bool connected) const;

    /* 确保指定相机已连接
     * @param cameraKey 相机标识
     * @param timeoutMs 连接超时时间
     * @param errorMessage 错误信息输出参数
     * @return 是否连接成功
     */
    bool ensureConnected(const QString& cameraKey, int timeoutMs, QString* errorMessage);

    /* 主动连接指定相机
     * @param cameraKey 相机标识
     * @param timeoutMs 连接超时时间
     * @param errorMessage 错误信息输出参数
     * @return 是否连接成功
     */
    bool connectCamera(const QString& cameraKey, int timeoutMs, QString* errorMessage);

    /* 断开当前相机连接
     * @param errorMessage 错误信息输出参数
     * @return 是否断开成功
     */
    bool disconnectCamera(QString* errorMessage = nullptr);

    /* 构造失败结果
     * @param request 原始请求
     * @param errorCode 错误码
     * @param errorMessage 错误描述
     * @param elapsedMs 已耗时
     * @return 失败结果对象
     */
    CaptureResult makeFailureResult(
        const CaptureRequest& request,
        CaptureErrorCode errorCode,
        const QString& errorMessage,
        qint64 elapsedMs) const;

    /* 将 3D 点云帧转换为内部点云结构 */
    PointCloudFrame buildPointCloud3D(const mmind::eye::Frame3D& frame) const;

    /* 将 2D+3D 点云帧转换为内部点云结构 */
    PointCloudFrame buildPointCloud2DAnd3D(const mmind::eye::Frame2DAnd3D& frame) const;

    /* 连接成功后打印相机基础参数（曝光、增益、分辨率等） */
    void printCameraParameters();

    /* 使用 SDK 内置方法保存 3D 点云到文件 */
    void saveFrameToFile(const mmind::eye::Frame3D& frame, quint64 requestId);

    /* 使用 SDK 内置方法保存纹理点云到文件 */
    void saveFrame2DAnd3DToFile(const mmind::eye::Frame2DAnd3D& frame, quint64 requestId);

    QString m_defaultCameraKey;
    CameraRuntimeState m_state = CameraRuntimeState::Idle;
    CameraInfoSnapshot m_cameraInfo;
    bool m_busy = false;
    bool m_connected = false;
    Impl* m_impl = nullptr;
};

}  // namespace mech_eye
}  // namespace scan_tracking