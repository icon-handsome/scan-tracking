#pragma once

#include <QtCore/QString>

namespace scan_tracking::common {

/// 默认采集缓存根目录：<applicationDir>/ScanTracking_CaptureCache
QString defaultCaptureCacheRoot();

/// 配置为空时使用默认根目录，否则使用配置路径（绝对化）
QString resolveCaptureCacheRoot(const QString& configuredRoot);

/// 确保目录存在；失败返回空字符串
QString ensureDirectoryExists(const QString& directoryPath);

/// Mech-Eye 3D 点云：<root>/mech_3d
QString captureCacheMech3DDir(const QString& root);

/// Mech-Eye 2D 灰度图：<root>/mech_2d
QString captureCacheMech2DDir(const QString& root);

/// 海康 Mono 根目录：<root>/hik_mono
QString captureCacheHikMonoDir(const QString& root);

/// 海康 A/B 分目录：<root>/hik_mono/camera_a 或 camera_b（cameraTag 为 hikA / hikB）
QString captureCacheHikMonoCameraDir(const QString& root, const QString& cameraTag);

/// 同一次分段落盘共用的时间戳：yyyyMMdd_HHmmss_zzz
QString buildCaptureTimestamp();

}  // namespace scan_tracking::common
