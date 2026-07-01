#pragma once

// HeadMeasure 柱面/开孔测量算法适配层。
//
// 调用 MeasurePipeline::runWithScanCloud / runWithScanClouds（与 demo 的 preprocess +
// mergeFrames 一致），输出内径、圆度、直边、开孔等尺寸。

#include <QtCore/QList>
#include <QtCore/QString>

#include "HeadMeasure/Types.h"
#include "scan_tracking/mech_eye/mech_eye_types.h"

namespace scan_tracking::vision::hole {

struct HoleInspectionResult {
    bool invoked = false;
    bool ok = false;
    QString message;
    hm::MeasureResult measureResult;
    double icpRmsMm = 0.0;
    double cylinderRmsMm = 0.0;
};

QString resolveHoleConfigPath(int inspectionPathId = 0);

HoleInspectionResult runHoleMeasurement(
    const scan_tracking::mech_eye::PointCloudFrame& cloud,
    int inspectionPathId = 0);

/// 正式检测：多扫描分段逐帧 preprocess 后合并（与 demo mergeFrames 一致）
HoleInspectionResult runHoleMeasurementFromSegmentFrames(
    const QList<scan_tracking::mech_eye::PointCloudFrame>& segmentClouds,
    int inspectionPathId = 0,
    int sourcePointCount = 0);

}  // namespace scan_tracking::vision::hole
