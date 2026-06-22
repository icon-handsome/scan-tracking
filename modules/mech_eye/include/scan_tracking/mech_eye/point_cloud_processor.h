#pragma once

// 点云后处理与坐标变换工具。
//
// 供 StateMachine 分段 refinement 后台线程使用：
//   - processPointCloudFrame：PCL 滤波/下采样等可配置后处理
//   - transformPointCloudFrame：按 T0（或 T0'×T 兼容形式）将分段点云变换到统一坐标系
//   - multiplyRowMajor4x4：位姿矩阵链式乘法，与 LBN/LB 及 scan_paths_config 一致
//
// Windows 下 PCL/Eigen 非线程安全，所有入口须持有 pointCloudAlgorithmMutex()。

#include <QtCore/QString>

#include <array>
#include <mutex>

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/mech_eye/mech_eye_types.h"

namespace scan_tracking::mech_eye {

/// 进程内 PCL/Eigen 全局串行化锁（Windows 下多线程并发会触发 aligned_free 崩溃）
std::mutex& pointCloudAlgorithmMutex();

/// 单次后处理统计（输入/输出点数及摘要信息）
struct PointCloudProcessReport {
    int inputPointCount = 0;   ///< 输入点数
    int outputPointCount = 0;  ///< 输出点数
    QString message;           ///< 处理摘要或跳过原因
};

/* 对 Mech-Eye 点云执行可配置后处理
 *
 * @param input   原始或上一段输出点云
 * @param config  [PointCloudProcessing] 开关与各滤波参数
 * @param output  输出点云（enabled=false 时为深拷贝直通）
 * @param report  可选统计信息
 * @return 是否成功（失败时 output 可能未写入）
 */
bool processPointCloudFrame(
    const PointCloudFrame& input,
    const common::PointCloudProcessingConfig& config,
    PointCloudFrame* output,
    PointCloudProcessReport* report = nullptr);

/// 行优先 4×4 矩阵乘法：out = left × right（与 StateMachine / LBN 链一致）
std::array<float, 16> multiplyRowMajor4x4(
    const std::array<float, 16>& left,
    const std::array<float, 16>& right);

/* 将点云变换到统一坐标系：p' = p × (calibration × stereo)。
 * 当前 IPC 约定：LB 成功时 calibration=Rt_global、stereo=I；否则 calibration=T0'、stereo=I。
 */
bool transformPointCloudFrame(
    const PointCloudFrame& input,
    const std::array<float, 16>& calibrationMatrixT0Prime,
    const std::array<float, 16>& stereoTrackingMatrixT,
    PointCloudFrame* output,
    QString* message = nullptr);

}  // namespace scan_tracking::mech_eye
