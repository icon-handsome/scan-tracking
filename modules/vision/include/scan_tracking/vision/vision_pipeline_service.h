#pragma once

// VisionPipelineService 是多相机视觉流水线的编排门面类。
//
// 负责将 Mech-Eye（3D 点云 / 2D 纹理）与海康 CXP 双目（左 A / 右 B）组合成一次
// "组合采集"（MultiCameraCaptureBundle），并在采集完成后按段类型触发位姿检测：
//   - 转盘段（needMechEye2D=true）：仅跑 LBN 位姿（基于梅卡 2D+3D）
//   - 封头段（needMechEye2D=false）：仅跑 LB 位姿（基于 CXP 双目）
//
// 采集时序：先梅卡 → 延迟 kMechToHikCaptureDelayMs → 再并行启动 CXP 双目，
// 三路全部完成后在后台线程执行位姿算法，结果通过 bundleCaptureFinished 回传。
// 所有槽函数与信号均走 Qt::QueuedConnection，不阻塞主线程。

#include <QtCore/QObject>

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/mech_eye/mech_eye_service.h"
#include "scan_tracking/vision/vision_types.h"

namespace scan_tracking {
namespace vision {

class HikCxpCameraService;

class VisionPipelineService : public QObject {
    Q_OBJECT

public:
    /* 构造函数：注入三路相机服务指针，并建立异步结果回调连接
     * @param mechEyeService   Mech-Eye 3D/2D 采集服务
     * @param hikCameraAService 海康 CXP 左目（相机 A）服务
     * @param hikCameraBService 海康 CXP 右目（相机 B）服务
     * @param parent           Qt 父对象
     */
    VisionPipelineService(
        scan_tracking::mech_eye::MechEyeService* mechEyeService,
        HikCxpCameraService* hikCameraAService,
        HikCxpCameraService* hikCameraBService,
        QObject* parent = nullptr);
    ~VisionPipelineService() override = default;

    /* 启动视觉流水线：加载 VisionConfig 及 LB/LBN 位姿配置，进入 Ready 状态 */
    void start(const scan_tracking::common::VisionConfig& config);

    /* 停止视觉流水线：丢弃进行中的采集上下文，进入 Stopped 状态 */
    void stop();

    /* 流水线是否已通过 start() 激活 */
    bool isStarted() const { return m_started; }

    /* 当前流水线状态（Idle / Ready / Capturing / Error / Stopped） */
    VisionPipelineState state() const { return m_state; }

    /* 发起一次多相机组合采集
     *
     * 根据 mechCaptureMode 决定是否同时采集梅卡 2D 纹理（Capture2DAnd3D），
     * 进而决定后续是否执行 LBN 位姿检测。同一时刻仅允许一个采集请求在途。
     *
     * @param segmentIndex     当前扫描段号，写入 bundle 供上层关联
     * @param taskId           上层任务 ID
     * @param mechCaptureMode  梅卡采集模式（默认仅 3D）
     * @return 本次请求 ID；失败时返回 0 并 emit fatalError
     */
    quint64 requestCaptureBundle(
        int segmentIndex,
        quint32 taskId,
        scan_tracking::mech_eye::CaptureMode mechCaptureMode,
        bool mechEdgeArtifactRemovalEnabled,
        bool mechComparisonCaptureEnabled);

    /// 从 config.ini [Vision] 读取边缘伪点去除与对比采集开关
    quint64 requestCaptureBundle(
        int segmentIndex,
        quint32 taskId,
        scan_tracking::mech_eye::CaptureMode mechCaptureMode);

signals:
    /* 组合采集（含位姿检测结果）完成；即使部分失败也会发出，调用方需检查各子结果 */
    void bundleCaptureFinished(scan_tracking::vision::MultiCameraCaptureBundle bundle);

    /* 流水线状态变更通知 */
    void stateChanged(scan_tracking::vision::VisionPipelineState state, QString description);

    /* 不可恢复的前置错误（未启动、忙、配置缺失、采集被拒绝等） */
    void fatalError(scan_tracking::vision::VisionErrorCode code, QString message);

private slots:
    /* Mech-Eye 采集完成回调：记录结果并启动 CXP 延迟定时器 */
    void onMechEyeCaptureFinished(scan_tracking::mech_eye::CaptureResult result);

    /* 海康 CXP 单目采集完成回调：按 logicalName 区分 A/B，两路齐则触发 finishBundleIfReady */
    void onHikPoseCaptureFinished(scan_tracking::vision::HikPoseCaptureResult result);

private:
    /* 单次组合采集的进行中上下文，用于汇聚三路异步结果 */
    struct PendingCaptureContext {
        bool active = false;       // 是否有采集正在进行
        bool mechDone = false;     // 梅卡是否已完成
        bool hikADone = false;     // CXP 左目是否已完成
        bool hikBDone = false;     // CXP 右目是否已完成
        quint64 mechRequestId = 0; // 梅卡侧 requestId，用于过滤过期回调
        quint64 hikARequestId = 0; // CXP 左目 requestId
        quint64 hikBRequestId = 0; // CXP 右目 requestId
        scan_tracking::vision::MultiCameraCaptureBundle bundle;
    };

    static void registerMetaTypes();
    void setState(VisionPipelineState state, const QString& description);

    /* 梅卡完成且延迟到期后，并行发起 CXP 双目 poseCapture */
    void startPendingHikCapture();

    /* 三路采集均就绪时，在后台线程执行 LB/LBN 位姿检测并 emit bundleCaptureFinished */
    void finishBundleIfReady();

    scan_tracking::mech_eye::MechEyeService* m_mechEyeService = nullptr;
    HikCxpCameraService* m_hikCameraAService = nullptr;
    HikCxpCameraService* m_hikCameraBService = nullptr;
    scan_tracking::common::VisionConfig m_config;
    scan_tracking::common::LbPoseConfig m_lbPoseConfig;   // 封头段 CXP 双目 LB 位姿参数
    scan_tracking::common::LbnPoseConfig m_lbnPoseConfig; // 转盘段梅卡 LBN 位姿参数
    PendingCaptureContext m_pending;
    quint64 m_nextRequestId = 1;
    bool m_started = false;
    bool m_processing = false; // 位姿后处理线程是否正在运行
    VisionPipelineState m_state = VisionPipelineState::Idle;
};

}  // namespace vision
}  // namespace scan_tracking
