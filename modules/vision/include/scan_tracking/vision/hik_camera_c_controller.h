#pragma once

// 海康相机 C 控制器
// 独立管理第三台海康相机（与跟踪相机A/B不同型号、不同用途）
// 负责相机的初始化、连接状态监控和基础采图功能

#include <QtCore/QObject>
#include <QtCore/QTimer>

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/vision/vision_types.h"

namespace scan_tracking {
namespace vision {

class HikCameraService;

enum class HikCameraCState {
    Idle = 0,
    Initializing = 1,
    Ready = 2,
    Capturing = 3,
    Error = 4,
    Stopped = 5,
};

class HikCameraCController : public QObject {
    Q_OBJECT

public:
    explicit HikCameraCController(
        HikCameraService* hikCameraCService,
        QObject* parent = nullptr);
    ~HikCameraCController() override = default;

    void start(const scan_tracking::common::VisionConfig& config);
    void stop();

    bool isStarted() const { return m_started; }
    HikCameraCState state() const { return m_state; }

    // 测试采图接口
    quint64 requestTestCapture();
    
    // 启用/禁用自动测试采图（连接成功后自动触发一次）
    void setAutoTestCaptureEnabled(bool enabled) { m_autoTestEnabled = enabled; }

signals:
    void stateChanged(scan_tracking::vision::HikCameraCState state, QString description);
    void testCaptureFinished(scan_tracking::vision::HikPoseCaptureResult result);
    void fatalError(scan_tracking::vision::VisionErrorCode code, QString message);

private slots:
    void onCameraCStateChanged(QString roleName, QString stateText, QString description);
    void onCameraCaptureFailed(scan_tracking::vision::VisionErrorCode code, QString message);
    void onCameraCaptureFinished(scan_tracking::vision::HikPoseCaptureResult result);
    void onAutoTestCaptureTimer();

private:
    static void registerMetaTypes();
    void setState(HikCameraCState state, const QString& description);

    HikCameraService* m_hikCameraCService = nullptr;
    scan_tracking::common::VisionConfig m_config;
    QTimer* m_autoTestTimer = nullptr;
    quint64 m_nextRequestId = 1;
    bool m_started = false;
    bool m_autoTestEnabled = true;
    bool m_autoTestTriggered = false;
    HikCameraCState m_state = HikCameraCState::Idle;
};

}  // namespace vision
}  // namespace scan_tracking
