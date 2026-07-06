#pragma once

/**
 * @file point_cloud_io.h
 * @brief Mech-Eye 点云与 2D 纹理的磁盘 I/O 工具
 *
 * 负责分段采集结果的落盘与回放：
 * - PLY：binary_little_endian xyz（保存时过滤 NaN；加载时兼容 ASCII 与带法向格式）
 * - PCD：PCL PointXYZ（Po_Kou 坡口离线测试数据）
 * - PNG：8 位灰度 Mech 2D 纹理
 *
 * 路径规则与 common/capture_cache_paths 一致，目录结构示例：
 * @code
 *   <scanCacheDirectory>/
 *     mech_3d/segment_{N}_task{T}_{timestamp}.ply
 *     mech_3d/compare/noise_on|noise_off/segment_{N}_task{T}_{timestamp}_cmp.ply
 *     mech_2d/segment_{N}_task{T}_{timestamp}.png
 * @endcode
 */

#include <QtCore/QList>
#include <QtCore/QString>

#include <vector>

#include "scan_tracking/mech_eye/mech_eye_types.h"

namespace scan_tracking::mech_eye {

/** @brief 默认采集缓存根目录：<applicationDir>/ScanTracking_CaptureCache */
QString defaultScanCacheDirectory();

/**
 * @brief 生成分段 PLY 绝对路径（mech_3d 子目录）
 * @param configuredRoot config.ini [Paths] scanCacheDirectory，空则使用默认根目录
 * @param segmentIndex 分段序号（从 1 起）
 * @param taskId 当前任务 ID
 * @param timestamp 时间戳字符串；空则自动生成，需与海康 2D 图共用同一 timestamp
 * @return 绝对路径；目录创建失败时返回空字符串
 */
QString buildSegmentPlyPath(
    const QString& configuredRoot,
    int segmentIndex,
    quint32 taskId,
    const QString& timestamp = QString());

/**
 * @brief 生成对比采集 PLY 路径
 * @param noiseRemovalNormal true → noise_on（主流程 Normal），false → noise_off（对比帧 Off）
 */
QString buildComparisonPlyPath(
    const QString& configuredRoot,
    int segmentIndex,
    quint32 taskId,
    bool noiseRemovalNormal,
    const QString& timestamp = QString());

/**
 * @brief 将 PointCloudFrame 保存为 binary little-endian PLY
 * @note 仅写入 x,y,z 三通道；NaN/Inf 点会被跳过；不写入 normals
 */
bool savePointCloudFrameToPly(const PointCloudFrame& frame, const QString& absolutePath);

/**
 * @brief 将 TXT 点云（每行 x y z）流式转换为 binary PLY，避免整帧驻留内存
 */
bool convertTxtPointCloudToPly(const QString& txtPath, const QString& plyPath);

/**
 * @brief 将 PointCloudFrame 保存为 PCL binary PCD（与内表面 demo LoadCloud 输入一致）
 * @note 仅写入有限 x,y,z；NaN/Inf 点会被跳过
 */
bool savePointCloudFrameToPcd(const PointCloudFrame& frame, const QString& absolutePath);

/**
 * @brief 按路径后缀保存二进制点云（.pcd → PCL binary；.ply → binary_little_endian xyz；缺省后缀按 pcd）
 */
bool savePointCloudFrameToBinaryFile(const PointCloudFrame& frame, const QString& absolutePath);

/**
 * @brief 按序合并多段 PointCloudFrame 并保存为二进制 PCD 或 PLY（由 absolutePath 后缀决定）
 */
bool mergePointCloudFramesToBinaryFile(
    const QList<PointCloudFrame>& frames,
    const QString& absolutePath,
    int* outPointCount = nullptr);

/**
 * @brief 将 TXT 点云（每行 x y z）转换为 binary PCD
 */
bool convertTxtPointCloudToPcd(const QString& txtPath, const QString& pcdPath);

/**
 * @brief 按序合并多段 PointCloudFrame 并保存为 binary PCD
 */
bool mergePointCloudFramesToPcd(
    const QList<PointCloudFrame>& frames,
    const QString& absolutePath,
    int* outPointCount = nullptr);

/**
 * @brief 从 PLY 文件加载点云
 * @note 支持 binary_little_endian 与 legacy ASCII；可识别 x,y,z 或 x,y,z,nx,ny,nz 属性
 */
bool loadPointCloudFrameFromPly(const QString& absolutePath, PointCloudFrame* outFrame);

/** @brief 从 PCD 文件加载点云（PCL PointXYZ；跳过 NaN/Inf 点） */
bool loadPointCloudFrameFromPcd(const QString& absolutePath, PointCloudFrame* outFrame);

/**
 * @brief 从 PLY 流式提取有限 xyz 浮点缓冲（不整包读 body，不建 PointCloudFrame）
 * @param maxPointCount 大于 0 时按 header 顶点数 stride 采样，上限约 maxPointCount
 */
bool loadPointCloudXyzFromPly(
    const QString& absolutePath,
    std::vector<float>* outXyz,
    int maxPointCount = 0);

/**
 * @brief 从 PCD 提取有限 xyz 浮点缓冲（PCL 仅在持锁期间使用，不跨模块传递点云）
 * @param maxPointCount 大于 0 时对有效点 stride 采样
 */
bool loadPointCloudXyzFromPcd(
    const QString& absolutePath,
    std::vector<float>* outXyz,
    int maxPointCount = 0);

/**
 * @brief 从 TXT 文件加载点云（每行 x y z，空格分隔）
 * @note 跳过空行与无效点；不读取法向
 */
bool loadPointCloudFrameFromTxt(const QString& absolutePath, PointCloudFrame* outFrame);

/**
 * @brief 释放 PointCloudFrame 中的大数组，保留 pointCount/width/height 等元数据
 * @note 用于分段 refinement 完成后主动回收内存，避免多段点云同时驻留
 */
void releasePointCloudFrameBuffers(PointCloudFrame* frame);

/** @brief 生成分段 Mech 2D PNG 绝对路径（mech_2d 子目录） */
QString buildSegmentMech2DPngPath(
    const QString& configuredRoot,
    int segmentIndex,
    quint32 taskId,
    const QString& timestamp = QString());

/** @brief 将 GrayTextureFrame 保存为 8 位灰度 PNG */
bool saveGrayTextureFrameToPng(const GrayTextureFrame& frame, const QString& absolutePath);

}  // namespace scan_tracking::mech_eye
