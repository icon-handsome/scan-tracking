#pragma once

// 厚度测量算法适配层（third_party/Thicknessmeasurement）。
//
// 从分段缓存取 inner/outer 两帧点云，调用 MeasureThicknessFromClouds，
// 输出厚度与 ICP 得分，供 TrackingService / StateMachine 写寄存器与 HMI 上报。

#include <QtCore/QString>

#include "scan_tracking/mech_eye/mech_eye_types.h"

namespace scan_tracking::vision::thickness {

struct ThicknessInspectionResult {
    bool invoked = false;
    bool ok = false;
    QString message;
    double thicknessMm = 0.0;
    /// max(inner, outer) ICP fitness，供寄存器/HMI 向后兼容
    double icpFitnessScore = 0.0;
    double innerIcpFitnessScore = 0.0;
    double outerIcpFitnessScore = 0.0;
    QString thicknessMethod;
};

QString resolveThicknessConfigPath(int inspectionPathId = 0);

ThicknessInspectionResult runThicknessMeasurement(
    const scan_tracking::mech_eye::PointCloudFrame& innerCloud,
    const scan_tracking::mech_eye::PointCloudFrame& outerCloud,
    int inspectionPathId = 0);

}  // namespace scan_tracking::vision::thickness
