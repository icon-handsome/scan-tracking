#pragma once

/**
 * @file mech_eye_types.h
 * @brief Mech-Eye 3D 相机模块的公共数据类型
 *
 * 将跨线程传递的数据类型集中定义于此，避免 MechEyeService 与 MechEyeWorker 相互包含。
 * 类型同时供 flow_control、tracking、vision 等上层模块复用。
 * 所有结构体均通过 Q_DECLARE_METATYPE 注册，以支持 Qt 信号槽的 QueuedConnection 投递。
 */

#include <memory>
#include <vector>

#include <QtCore/QMetaType>
#include <QtCore/QString>
#include <QtCore/QtGlobal>

namespace scan_tracking {
namespace mech_eye {

/** @brief SDK 采集模式，对应 Mech-Eye capture3D / capture2DAnd3D / capture2D 接口 */
enum class CaptureMode {
    Capture3DOnly = 0,   ///< 仅 3D 点云（分段扫描主流程默认模式）
    Capture2DAnd3D = 1,  ///< 同步 2D 灰度纹理 + 3D 点云（需纹理对齐时使用）
    Capture2DOnly = 2,   ///< 仅 2D 灰度图（调试用）
};

/** @brief 采集与连接过程中的统一错误码，供上层 StateMachine / HMI 统一处理 */
enum class CaptureErrorCode {
    Success = 0,
    NotStarted = 1,       ///< 服务尚未 start()
    NotConnected = 2,     ///< 相机未连接或已掉线
    Busy = 3,             ///< 单相机单并发：上一次采集尚未结束
    DiscoverFailed = 4,   ///< discoverCameras 未发现设备
    ConnectFailed = 5,    ///< connect 失败
    CaptureFailed = 6,    ///< SDK 采图失败或结果为空
    DisconnectFailed = 7,
    Timeout = 8,          ///< 发现/连接/采集超时
    InvalidRequest = 9,   ///< 参数非法（cameraKey 空、模式不支持等）
    UnknownError = 10,    ///< SDK 异常或未映射错误
};

/** @brief 相机运行状态机，由 MechEyeWorker 驱动并经 MechEyeService 转发 */
enum class CameraRuntimeState {
    Idle = 0,
    Discovering = 1,   ///< 正在局域网搜索 Mech-Eye 设备
    Connecting = 2,    ///< 正在建立 TCP 连接
    Ready = 3,         ///< 已连接，可接受采集请求
    Capturing = 4,     ///< 采集中
    Disconnecting = 5,
    Error = 6,         ///< 连接/采集失败，需 refresh 或重启服务
    Stopped = 7,       ///< stop() 后终态
};

/** @brief 相机信息快照，用于跨线程传递和 HMI 状态展示 */
struct CameraInfoSnapshot {
    QString model;  // 相机型号
    QString serialNumber;   // 序列号
    QString ipAddress;  // IP 地址
    QString firmwareVersion;    // 固件版本
    bool connected = false; // 是否已连接
};

/**
 * @brief 与 3D 点云像素对齐的 2D 灰度图
 * @note Capture2DAnd3D 模式下由 SDK Frame2D 转换；Capture2DOnly 时仅含此字段
 */
struct GrayTextureFrame {
    std::shared_ptr<std::vector<uint8_t>> pixels;
    int width = 0;
    int height = 0;

    bool isValid() const
    {
        return pixels && !pixels->empty() && width > 0 && height > 0 &&
               static_cast<int>(pixels->size()) >= width * height;
    }
};

/**
 * @brief 点云帧数据
 *
 * 使用 shared_ptr 持有大数组，跨线程 QueuedConnection 传递时仅拷贝指针。
 * pointsXYZ 为行优先排列：index i 对应 (x,y,z) = [i*3], [i*3+1], [i*3+2]。
 */
struct PointCloudFrame {
    std::shared_ptr<std::vector<float>> pointsXYZ;  // 每三个 float 表示一个点的 x, y, z 坐标
    std::shared_ptr<std::vector<float>> normalsXYZ;  // 每三个 float 表示一个点的法向量 x, y, z 分量
    int width = 0;  // 点云的宽度
    int height = 0; // 点云的高度
    int pointCount = 0; // 点云中的点数量
    quint64 frameId = 0;  // 唯一标识符
    qint64 timestampMs = 0; // 采集时间戳，单位毫秒

    /* 判断点云是否有效 */
    bool isValid() const
    {
        return pointsXYZ && !pointsXYZ->empty() && pointCount > 0;
    }

    /* 判断是否带有法向量数据 */
    bool hasNormals() const
    {
        return normalsXYZ && static_cast<int>(normalsXYZ->size()) == pointCount * 3;
    }

    /* 获取法向量数量 */
    int normalCount() const
    {
        return normalsXYZ ? static_cast<int>(normalsXYZ->size() / 3) : 0;
    }
};

/**
 * @brief 采集请求参数，由 MechEyeService::requestCapture 构造并投递至 Worker
 */
struct CaptureRequest {
    quint64 requestId = 0;
    QString cameraKey;  ///< 型号/序列号/IP/设备名，空则使用 config.ini 默认相机
    CaptureMode mode = CaptureMode::Capture3DOnly;
    int timeoutMs = 30000;
    /** @brief 同次额外采一帧 NoiseRemoval=Off，与主流程 Normal 对比滤波效果 */
    bool comparisonCaptureEnabled = false;
};

/** @brief 采集结果，通过 captureFinished 信号回传主线程 */
struct CaptureResult {
    quint64 requestId = 0;  // 唯一标识符，与 CaptureRequest 中的 requestId 一致
    QString cameraKey;  // 相机唯一标识
    CaptureMode mode = CaptureMode::Capture3DOnly;  // 采集模式
    CaptureErrorCode errorCode = CaptureErrorCode::Success; // 错误码，Success 表示采集成功
    QString errorMessage;   // 错误描述，便于日志记录和调试
    CameraInfoSnapshot cameraInfo;  // 采集时的相机信息快照
    PointCloudFrame pointCloud; // 采集到的点云数据
    PointCloudFrame comparisonPointCloud; ///< comparisonCaptureEnabled 时的 Noise=Off 点云
    GrayTextureFrame texture2D;           ///< Capture2DAnd3D / Capture2DOnly 时的灰度图
    qint64 elapsedMs = 0;   // 采集耗时，单位毫秒
    qint64 comparisonElapsedMs = 0; // 对比点云采集耗时，单位毫秒

    /* 判断采集是否成功 */
    bool success() const
    {
        return errorCode == CaptureErrorCode::Success;
    }

    bool hasComparisonPointCloud() const
    {
        return comparisonPointCloud.isValid();
    }
};

}  // namespace mech_eye
}  // namespace scan_tracking

Q_DECLARE_METATYPE(scan_tracking::mech_eye::CaptureMode)    
Q_DECLARE_METATYPE(scan_tracking::mech_eye::CaptureErrorCode)
Q_DECLARE_METATYPE(scan_tracking::mech_eye::CameraRuntimeState)
Q_DECLARE_METATYPE(scan_tracking::mech_eye::CameraInfoSnapshot)
Q_DECLARE_METATYPE(scan_tracking::mech_eye::GrayTextureFrame)
Q_DECLARE_METATYPE(scan_tracking::mech_eye::PointCloudFrame)
Q_DECLARE_METATYPE(scan_tracking::mech_eye::CaptureRequest)
Q_DECLARE_METATYPE(scan_tracking::mech_eye::CaptureResult)
