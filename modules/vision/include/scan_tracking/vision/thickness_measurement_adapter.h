#pragma once

// 厚度测量算法适配层。
//
// 从分段缓存取 inner/outer 两帧点云，执行 direct_raw 厚度测量，
// 输出厚度与兼容字段，供 TrackingService / StateMachine 写寄存器与 HMI 上报。

#include <QtCore/QString>

#include "scan_tracking/mech_eye/mech_eye_types.h"

namespace scan_tracking::vision::thickness {

struct ThicknessInspectionResult {
    bool invoked = false;
    bool ok = false;
    QString message;
    double thicknessMm = 0.0;
    /// 保留兼容字段，direct 路径下恒为 0
    double icpFitnessScore = 0.0;
    double innerIcpFitnessScore = 0.0;
    double outerIcpFitnessScore = 0.0;
    QString thicknessMethod;
};

QString resolveThicknessConfigPath(int inspectionPathId = 0);

/// 运行时配置：offlineReplayEnabled=true 时优先 offlineReplayAlgorithmConfigPath
QString resolveRuntimeThicknessConfigPath(int inspectionPathId = 0);

ThicknessInspectionResult runThicknessMeasurement(
    const scan_tracking::mech_eye::PointCloudFrame& innerCloud,
    const scan_tracking::mech_eye::PointCloudFrame& outerCloud,
    int inspectionPathId = 0);

}  // namespace scan_tracking::vision::thickness
