#pragma once

// 海康相机 C 控制器
// 独立管理第三台海康相机（读码相机）
// 只负责连接状态监控，不包含图像采集功能

#include <QtCore/QObject>

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/vision/vision_types.h"

namespace scan_tracking {
namespace vision {

class HikCameraService;

enum class HikCameraCState {
    Idle = 0,
    Initializing = 1,
    Ready = 2,
    Error = 3,
    Stopped = 4,
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

signals:
    void stateChanged(scan_tracking::vision::HikCameraCState state, QString description);
    void fatalError(scan_tracking::vision::VisionErrorCode code, QString message);

private slots:
    void onCameraCStateChanged(QString roleName, QString stateText, QString description);
    void onCameraError(scan_tracking::vision::VisionErrorCode code, QString message);

private:
    static void registerMetaTypes();
    void setState(HikCameraCState state, const QString& description);

    HikCameraService* m_hikCameraCService = nullptr;
    scan_tracking::common::VisionConfig m_config;
    bool m_started = false;
    HikCameraCState m_state = HikCameraCState::Idle;
};

}  // namespace vision
}  // namespace scan_tracking
