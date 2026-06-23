#pragma once

// 视觉域共享类型定义。
//
// 供 VisionPipelineService、StateMachine、TrackingService 及 HMI 序列化复用。
// 核心数据结构为 MultiCameraCaptureBundle：一次分段组合采集的完整结果容器
//（Mech-Eye 点云 + 海康 CXP 双目 Mono + LB/LBN 位姿矩阵）。

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include <QtCore/QMetaType>
#include <QtCore/QString>
#include <QtCore/QtGlobal>

#include "scan_tracking/mech_eye/mech_eye_types.h"

namespace scan_tracking {
namespace vision {

/// 视觉子系统统一错误码（海康 / 流水线层）
enum class VisionErrorCode {
    Success = 0,           ///< 成功
    NotStarted = 1,        ///< 服务未 start
    Busy = 2,              ///< 上一帧采集尚未完成
    InvalidConfig = 3,     ///< 配置缺失或非法
    CaptureRejected = 4,   ///< 请求被拒绝（如未连接）
    NotImplemented = 5,    ///< 功能未实现
    DeviceNotFound = 6,    ///< 未找到匹配设备
    DeviceOpenFailed = 7,  ///< 打开设备失败
    SdkInitFailed = 8,     ///< MVS / 采集卡 SDK 初始化失败
    UnknownError = 9,      ///< 其它未知错误
};

/// VisionPipelineService 运行状态
enum class VisionPipelineState {
    Idle = 0,       ///< 构造后未 start
    Ready = 1,      ///< 已加载配置，可接受采集请求
    Capturing = 2,  ///< 组合采集中（梅卡 + CXP 双目）
    Error = 3,      ///< 发生不可恢复错误
    Stopped = 4,    ///< 已 stop
};

/// 4×4 位姿变换矩阵（行优先存储，与 LB/LBN 算法及 PLC 寄存器约定一致）
struct PoseMatrix4x4 {
    std::array<float, 16> values = {
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f,
    };
    quint64 frameId = 0;
    qint64 timestampMs = 0;
    QString sourceCameraKey;
    bool valid = false;

    bool isValid() const { return valid; }
};

struct HikMonoFrame {
    std::shared_ptr<std::vector<std::uint8_t>> pixels;
    int width = 0;
    int height = 0;
    int stride = 0;
    quint64 frameId = 0;
    qint64 timestampMs = 0;
    QString sourceCameraKey;
    QString pixelFormat = QStringLiteral("Mono8");

    bool isValid() const
    {
        return pixels != nullptr && !pixels->empty() && width > 0 && height > 0;
    }
};

struct HikPoseCaptureRequest {
    quint64 requestId = 0;
    QString cameraKey;
    QString logicalName;
    int timeoutMs = 1000;
};

struct HikPoseCaptureResult {
    quint64 requestId = 0;
    QString cameraKey;
    QString logicalName;
    VisionErrorCode errorCode = VisionErrorCode::Success;
    QString errorMessage;
    HikMonoFrame frame;
    PoseMatrix4x4 poseMatrix;
    qint64 elapsedMs = 0;

    bool success() const
    {
        return errorCode == VisionErrorCode::Success && frame.isValid();
    }

    bool hasPose() const
    {
        return poseMatrix.isValid();
    }
};

// 海康相机可读写的关键参数快照
struct HikCameraParams {
    // ---- 曝光 ----
    float exposureTimeUs = 0.0f;      // 曝光时间（微秒），对应 GenICam 节点 "ExposureTime"
    float exposureTimeMinUs = 0.0f;
    float exposureTimeMaxUs = 0.0f;
    bool  autoExposureEnabled = false; // ExposureAuto != Off

    // ---- 增益 ----
    float gainDb = 0.0f;              // 模拟增益（dB），对应 "Gain"
    float gainMinDb = 0.0f;
    float gainMaxDb = 0.0f;
    bool  autoGainEnabled = false;    // GainAuto != Off

    // ---- 帧率 ----
    float frameRateFps = 0.0f;        // 当前帧率，对应 "AcquisitionFrameRate"
    bool  frameRateEnabled = false;   // AcquisitionFrameRateEnable

    // ---- 触发 ----
    quint32 triggerMode = 0;          // 0=Off(连续), 1=On(触发)，对应 "TriggerMode"

    // ---- 图像尺寸 ----
    qint64 width = 0;                 // 对应 "Width"
    qint64 height = 0;                // 对应 "Height"

    // ---- 像素格式 ----
    quint32 pixelFormat = 0;          // 枚举值，对应 "PixelFormat"
    QString pixelFormatStr;           // 枚举符号名，如 "Mono8"

    // ---- 读取状态 ----
    bool valid = false;               // true 表示本次读取成功
    QString errorMessage;             // 读取失败时的错误描述
};

// 海康智能相机拍照类型
enum class CaptureType {
    SurfaceDefect = 0,      // 表面普通缺陷识别
    WeldDefect = 1,         // 焊缝缺陷识别
    NumberRecognition = 2,  // 编号识别
};

/// LB（封头段）位姿检测结果，由 runLbPoseDetection 填充
struct LbPoseResult {
    bool invoked = false;       ///< 是否已调用 LB 算法（未调用时 success 无意义）
    bool success = false;       ///< 算法是否成功输出有效位姿
    QString message;            ///< 失败原因或摘要
    int leftImageWidth = 0;     ///< 左目输入图宽
    int leftImageHeight = 0;    ///< 左目输入图高
    int rightImageWidth = 0;    ///< 右目输入图宽
    int rightImageHeight = 0;   ///< 右目输入图高
    int framePointCount = 0;    ///< 重建得到的标记点数量
    PoseMatrix4x4 poseMatrix;   ///< 4×4 Rt 位姿矩阵
    QString diagnosticText;     ///< LB 诊断明细（标记点/矩阵/时间戳），写入 meta.txt 与日志
};

/// LBN（转盘段）位姿检测结果，由 runLbnPoseDetection 填充
struct LbnPoseResult {
    bool invoked = false;         ///< 是否已调用 LBN 算法
    bool success = false;         ///< 算法是否成功
    QString message;              ///< 失败原因或摘要
    int textureWidth = 0;         ///< Mech-Eye 2D 纹理宽
    int textureHeight = 0;        ///< Mech-Eye 2D 纹理高
    int pointCloudWidth = 0;      ///< 组织化点云宽
    int pointCloudHeight = 0;     ///< 组织化点云高
    int matchedPointCount = 0;    ///< 模板匹配成功点数
    PoseMatrix4x4 poseMatrix;     ///< 4×4 Rt 位姿矩阵
};

/// 组合采集请求参数（由 VisionPipelineService 在 requestCaptureBundle 时构造）
struct MultiCameraCaptureRequest {
    quint64 requestId = 0;
    quint32 taskId = 0;
    int segmentIndex = 0;
    bool needMechEye2D = false;
    scan_tracking::mech_eye::CaptureMode mechCaptureMode =
        scan_tracking::mech_eye::CaptureMode::Capture3DOnly;
    bool mechComparisonCaptureEnabled = false;
    QString mechEyeCameraKey;
    int mechEyeTimeoutMs = 5000;
    QString hikCameraAKey;
    QString hikCameraBKey;
    int hikTimeoutMs = 1000;
};

/// 一次分段组合采集的完整结果，经 Qt 信号跨线程传递给 StateMachine
struct MultiCameraCaptureBundle {
    MultiCameraCaptureRequest request;                    ///< 原始请求（段号、taskId 等）
    scan_tracking::mech_eye::CaptureResult mechEyeResult; ///< Mech-Eye 3D/2D 采集结果
    HikPoseCaptureResult hikCameraAResult;              ///< CXP 左目 Mono 帧
    HikPoseCaptureResult hikCameraBResult;              ///< CXP 右目 Mono 帧
    LbPoseResult lbPoseResult;                          ///< 封头段 LB 位姿（转盘段时 invoked=false）
    LbnPoseResult lbnPoseResult;                        ///< 转盘段 LBN 位姿（封头段时 invoked=false）

    /// 梅卡 + 双目 + 已调用位姿算法均成功（LBN 未调用时不计入）
    bool success() const
    {
        return mechEyeResult.success() &&
               hikCameraAResult.success() &&
               hikCameraBResult.success() &&
               lbPoseResult.success;
    }

    QString summary() const
    {
        const auto flag = [](bool ok) {
            return ok ? QStringLiteral("成功") : QStringLiteral("失败");
        };
        const QString lbnFlag = lbnPoseResult.invoked
            ? (lbnPoseResult.success ? QStringLiteral("成功") : QStringLiteral("失败"))
            : QStringLiteral("跳过");
        return QStringLiteral(
                   "组合采集 requestId=%1 taskId=%2 段号=%3 梅卡=%4 海康A=%5 海康B=%6 LB=%7 LBN=%8")
            .arg(request.requestId)
            .arg(request.taskId)
            .arg(request.segmentIndex)
            .arg(flag(mechEyeResult.success()))
            .arg(flag(hikCameraAResult.success()))
            .arg(flag(hikCameraBResult.success()))
            .arg(flag(lbPoseResult.success))
            .arg(lbnFlag);
    }
};

}  // namespace vision
}  // namespace scan_tracking

Q_DECLARE_METATYPE(scan_tracking::vision::VisionErrorCode)
Q_DECLARE_METATYPE(scan_tracking::vision::VisionPipelineState)
Q_DECLARE_METATYPE(scan_tracking::vision::PoseMatrix4x4)
Q_DECLARE_METATYPE(scan_tracking::vision::HikMonoFrame)
Q_DECLARE_METATYPE(scan_tracking::vision::HikPoseCaptureRequest)
Q_DECLARE_METATYPE(scan_tracking::vision::HikPoseCaptureResult)
Q_DECLARE_METATYPE(scan_tracking::vision::LbPoseResult)
Q_DECLARE_METATYPE(scan_tracking::vision::LbnPoseResult)
Q_DECLARE_METATYPE(scan_tracking::vision::MultiCameraCaptureRequest)
Q_DECLARE_METATYPE(scan_tracking::vision::MultiCameraCaptureBundle)
Q_DECLARE_METATYPE(scan_tracking::vision::CaptureType)
