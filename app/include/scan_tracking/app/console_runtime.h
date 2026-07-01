#pragma once

// ConsoleRuntime 是控制台版扫描跟踪应用的生命周期管理器。
//
// 职责：
//   - 按依赖顺序创建并启动各业务模块（Modbus → Mech-Eye → CXP 双目 → 视觉流水线 → 状态机 → HMI）
//   - 注册 Ctrl+C 等控制台信号，驱动 Qt 事件循环
//   - 退出时按依赖逆序安全停止，避免异步回调落到已析构对象
//   - 可选：读取 config.ini [Debug] 自动触发组合采集延时测试（LatencyTest）
//
// 启动阶段可通过环境变量 SCAN_TRACKING_STARTUP_STAGE（0~5）截断模块加载，便于分步联调。

#include <QtCore/QCoreApplication>
#include <memory>
#include <thread>

#include <QtCore/QMetaObject>

class QTimer;

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/livox_mid360/livox_mid360_service.h"
#include "scan_tracking/orbbec_gemini/orbbec_gemini_service.h"
#include "scan_tracking/tfmini_plus/tfmini_plus_service.h"
#include "scan_tracking/vision/vision_types.h"

namespace scan_tracking {
namespace modbus { class ModbusService; }
namespace mech_eye { class MechEyeService; }
namespace tracking { class TrackingService; }
namespace flow_control { class StateMachine; }
namespace vision {
class HikCxpCameraService;
class VisionPipelineService;
class HikCameraCController;
}
namespace hmi_server { class HmiTcpServer; }
}
#include "scan_tracking/hmi_server/hmi_tcp_server.h"

namespace scan_tracking::app {

class ConsoleRuntime final {
public:
    explicit ConsoleRuntime(QCoreApplication& application);
    ~ConsoleRuntime();

    /* 注册信号处理、初始化模块并进入 application_.exec() 事件循环 */
    int run();

private:
    void printStartupStatus();
    void printShutdownStatus();

    /* 按 startupStage 分级创建模块、连接信号、启动服务 */
    void initModules();

    /* 在事件循环启动后初始化海康 C（避免与 MechEye SDK 同步初始化冲突） */
    void initFlowModules(int startupStage, scan_tracking::common::VisionConfig visionConfig);

    /* 视觉流水线 / 跟踪 / 状态机：再延迟一轮事件循环，避开 SDK 加载后的堆不稳定窗口 */
    void initVisionFlowModules(int startupStage, scan_tracking::common::VisionConfig visionConfig);

    /* [Debug] 自动延时测试：读取 config.ini 并调度定时器 */
    void scheduleAutoLatencyBundleTest();

    /* 向 VisionPipeline 发起一次 Mech 2D + CXP 双目组合采集 */
    void triggerAutoLatencyBundleTest();

    /* 组合采集完成回调：打印各相机耗时汇总并落盘 PNG/BMP */
    void onAutoLatencyBundleFinished(const scan_tracking::vision::MultiCameraCaptureBundle& bundle);

    void stopAutoLatencyBundleTest();

    void stopOfflineReplayTimers();

    /* [Hole] 离线回放：启动后从 session 目录加载点云并跑 Hole 检测 */
    void scheduleOfflineHoleReplay();
    void triggerOfflineHoleReplay();

    /* [InternalSurface] 离线回放：启动后从 TXT/PLY 加载点云并跑内表面检测 */
    void scheduleOfflineInternalSurfaceReplay();
    void triggerOfflineInternalSurfaceReplay();

    /* [Bevel] 离线回放：启动后从 PCD 目录加载点云并跑坡口测量 */
    void scheduleOfflineBevelReplay();
    void triggerOfflineBevelReplay();

    /* [Thickness] 离线回放：启动后从目录加载 inner/outer 点云并跑厚度测量 */
    void scheduleOfflineThicknessReplay();
    void triggerOfflineThicknessReplay();

    // ---- 自动延时测试状态 ----
    qint64 m_autoLatencyTriggerMs = 0;   // 本次触发时刻（epoch ms），用于计算墙钟耗时
    bool m_autoLatencyTestPending = false; // 是否有采集尚未完成
    bool m_autoLatencyPeriodic = false;    // true=周期模式，false=单次
    int m_autoLatencyIntervalMs = 20000; // 触发间隔（ms）
    quint32 m_latencyBundleSeq = 0;        // 轮次序号，taskId = 90001 + round
    QTimer* m_autoLatencyTimer = nullptr;
    QTimer* m_offlineHoleReplayTimer = nullptr;
    QTimer* m_offlineInternalSurfaceReplayTimer = nullptr;
    QTimer* m_offlineBevelReplayTimer = nullptr;
    QTimer* m_offlineThicknessReplayTimer = nullptr;
    bool m_offlineInternalSurfaceReplayInFlight = false;
    bool m_shutdownCompleted = false;

    // ---- 业务模块（unique_ptr 持有，析构顺序与声明顺序相反） ----
    QCoreApplication& application_;
    std::unique_ptr<scan_tracking::modbus::ModbusService> modbusService_;
    std::unique_ptr<scan_tracking::mech_eye::MechEyeService> mechEyeService_;
    std::unique_ptr<scan_tracking::orbbec_gemini::OrbbecGeminiService> orbbecGeminiService_;
    std::unique_ptr<scan_tracking::livox_mid360::LivoxMid360Service> livoxMid360Service_;
    std::unique_ptr<scan_tracking::tfmini_plus::TfminiPlusService> tfminiPlusService_;
    std::unique_ptr<scan_tracking::vision::HikCxpCameraService> hikCxpCameraAService_;
    std::unique_ptr<scan_tracking::vision::HikCxpCameraService> hikCxpCameraBService_;
    std::unique_ptr<scan_tracking::vision::VisionPipelineService> visionPipelineService_;
    std::unique_ptr<scan_tracking::vision::HikCameraCController> hikCameraCController_;
    std::unique_ptr<scan_tracking::tracking::TrackingService> trackingService_;
    std::unique_ptr<scan_tracking::flow_control::StateMachine> stateMachine_;
    std::unique_ptr<scan_tracking::hmi_server::HmiTcpServer> hmiTcpServer_;
};

}
