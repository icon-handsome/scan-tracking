#pragma once

#include <QtCore/QString>

#include "scan_tracking/mech_eye/mech_eye_types.h"

namespace scan_tracking::vision::internal_surface {

struct InternalSurfaceInspectionResult {
    bool invoked = false;
    bool ok = false;
    double headDepthMm = 0.0;
    double headVolumeM3 = 0.0;
    double volumeLiter = 0.0;
    int filteredPointCount = 0;
    int downsampledPointCount = 0;
    int meshVertexCount = 0;
    int meshFaceCount = 0;
    QString message;
};

QString resolveInternalSurfaceConfigPath();

InternalSurfaceInspectionResult runInternalSurfaceMeasurement(
    const scan_tracking::mech_eye::PointCloudFrame& cloud);

/// 离线联调：直接从扫描点云文件（.txt/.ply）执行内表面测量，避免大点云载入内存
InternalSurfaceInspectionResult runInternalSurfaceMeasurementFromScanFile(
    const QString& scanCloudPath);

}  // namespace scan_tracking::vision::internal_surface
