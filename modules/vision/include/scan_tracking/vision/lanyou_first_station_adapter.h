#pragma once

#include <pcl/common/common.h>

#include <QtCore/QString>

#include "scan_tracking/mech_eye/mech_eye_types.h"
#include "utils/Params.h"

namespace scan_tracking::vision::lanyou {

struct FirstStationFrameSet {
    scan_tracking::mech_eye::PointCloudFrame outerSurfaceFrame; // 外表面点云
    scan_tracking::mech_eye::PointCloudFrame innerSurfaceFrame; // 内表面点云
    scan_tracking::mech_eye::PointCloudFrame innerHoleFrame; // 内孔点云
};
//检测参数
struct FirstStationDetectionResult {
    bool invoked = false;   // 是否调用了检测
    bool firstOutInvoked = false;   // 是否调用了外表面检测
    bool firstOutSuccess = false;   // 外表面检测是否成功
    bool firstInlinerInvoked = false;   // 是否调用了内表面检测
    bool firstInlinerSuccess = false;   // 内表面检测是否成功
    int outerPointCount = 0;    // 外表面点云数量
    int innerPointCount = 0;    // 内表面点云数量
    int holePointCount = 0;     // 内孔点云数量
    FirstPoseDetectionParams params;    // 检测参数
    QString message;    // 检测结果信息
};

FirstStationDetectionResult runFirstStationDetection(
    const FirstStationFrameSet& frames);

}  // namespace scan_tracking::vision::lanyou
