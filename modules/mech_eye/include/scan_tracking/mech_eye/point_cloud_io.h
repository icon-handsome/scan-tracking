#pragma once

#include <QtCore/QString>

#include "scan_tracking/mech_eye/mech_eye_types.h"

namespace scan_tracking::mech_eye {

/// 默认采集缓存根目录：<applicationDir>/ScanTracking_CaptureCache
QString defaultScanCacheDirectory();

/// 生成分段 PLY 绝对路径（mech_3d 子目录）：segment_{N}_task{T}_{timestamp}.ply
/// @param configuredRoot config.ini scanCacheDirectory，空则默认根目录
/// @param timestamp 与同段海康图共用；空则自动生成
QString buildSegmentPlyPath(
    const QString& configuredRoot,
    int segmentIndex,
    quint32 taskId,
    const QString& timestamp = QString());

/// 比较采集 PLY：<root>/mech_3d/compare/{edge_on|edge_off}/segment_..._cmp.ply
QString buildComparisonPlyPath(
    const QString& configuredRoot,
    int segmentIndex,
    quint32 taskId,
    bool edgeArtifactRemovalEnabled,
    const QString& timestamp = QString());

/// 将 PointCloudFrame 保存为 ASCII PLY（x,y,z,nx,ny,nz）
bool savePointCloudFrameToPly(const PointCloudFrame& frame, const QString& absolutePath);

/// 从 ASCII PLY 加载点云；支持 x,y,z 与 x,y,z,nx,ny,nz
bool loadPointCloudFrameFromPly(const QString& absolutePath, PointCloudFrame* outFrame);

/// 释放 PointCloudFrame 中的大数组，保留 pointCount/width/height 等元数据
void releasePointCloudFrameBuffers(PointCloudFrame* frame);

/// 生成分段 Mech 2D PNG 绝对路径（mech_2d 子目录）
QString buildSegmentMech2DPngPath(
    const QString& configuredRoot,
    int segmentIndex,
    quint32 taskId,
    const QString& timestamp = QString());

/// 将 GrayTextureFrame 保存为 8 位灰度 PNG
bool saveGrayTextureFrameToPng(const GrayTextureFrame& frame, const QString& absolutePath);

}  // namespace scan_tracking::mech_eye
