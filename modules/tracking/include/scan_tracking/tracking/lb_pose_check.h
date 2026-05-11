/**
 * @file lb_pose_check.h
 * @brief LB位姿检测头文件
 *
 * 提供传统的LB位姿检测算法接口，用于左右相机图像的三维重建和模板匹配，
 * 计算目标物体的位姿并输出4x4变换矩阵。
 */

#pragma once

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/tracking/tracking_service.h"

namespace scan_tracking::tracking {

/// 执行传统的LB位姿检测算法
// @param config LB位姿检测配置参数
// @return 位姿检测结果结构体
PoseCheckResult runLegacyLbPoseCheck(const scan_tracking::common::LbPoseConfig& config);

}  // namespace scan_tracking::tracking
