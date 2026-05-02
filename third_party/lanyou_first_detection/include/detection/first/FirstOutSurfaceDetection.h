#ifndef FIRST_OUTSURFCE_DETECTION_H
#define FIRST_OUTSURFCE_DETECTION_H

#include <string>
#include <pcl/common/common.h>
#include "model/PointsModel.h"
#include "utils/Params.h"

class FirstOutSurfaceDetection {
public:
/*****************************************
 * @brief                   构造函数：初始化第一检测位参数结构体
 *****************************************/
    FirstOutSurfaceDetection();

/*****************************************
 * @brief                   第一检测位外表面检测主流程
 * @param cloud             输入点云指针（外表面点云）
 * @return bool             检测是否成功
 *****************************************/
    bool Detect(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

/*****************************************
 * @brief                   获取检测结果参数（只读）
 * @return const FirstPoseDetectionParams&  检测结果参数的常量引用
 *****************************************/
    const FirstPoseDetectionParams& GetParams() const;

/*****************************************
 * @brief                   获取检测结果参数（可写）
 * @return FirstPoseDetectionParams&     检测结果参数的引用
 *****************************************/
    FirstPoseDetectionParams& GetParams();

private:
    struct FirstOutConfig {
        // 预处理 + 粗拟合参数
        float downsample_leaf = 2.0f;
        float coarse_normal_radius = 3.0f;
        float coarse_fit_distance_threshold = 2.0f;
        float radius_limit_min = 580.0f;
        float radius_limit_max = 620.0f;

        // 轴向区间裁剪参数（去掉两端，各保留中间）
        float axis_end_trim_ratio = 0.15f; // 去掉上下两端各 15%

        // 切片拟合圆参数
        float slice_step = 5.0f;          // mm
        float slice_thickness = 1.0f;     // mm
        int slice_min_points = 80;        // 单切片最少点数
        int circle_min_inlier_points = 50;// 拟合圆内点最少点数
        float circle_inlier_sigma_scale = 2.5f;
        float circle_inlier_thr_min = 0.5f;
        float circle_inlier_thr_max = 3.0f;

        // 圆心拟合轴线参数
        float centerline_inlier_sigma_scale = 3.0f;
        float centerline_inlier_thr_min = 0.5f;
        float centerline_inlier_thr_max = 5.0f;
        int axis_refine_max_iter = 4;
        float axis_refine_converge_rad = 1e-4f;

        // 最终圆柱内点筛选参数
        float radial_inlier_sigma_scale = 3.0f;
        float radial_inlier_thr_min = 0.5f;
        float radial_inlier_thr_max = 3.0f;

        // 轴向切片直线拟合与统计参数
        float line_slice_angle_step_deg = 5.0f;
        float line_slice_distance_tolerance = 0.1f;
        float line_fit_distance_tolerance = 0.2f;
        float vertical_line_parallel_max_angle_deg = 12.0f; // 直边与圆柱轴最大夹角
        float edge_stat_filter_ratio = 0.2f;                // 高度/斜度/坡口角两端过滤比例
    };

    bool LoadConfig(const std::string& config_file_path);

    // 第一检测位监测结构体。
    FirstPoseDetectionParams& firstpose_params_;
    // 每个检测类独立持有点云模型，避免外部共享实例。
    PointsModel points_model_;
    // first_out 参数配置。
    FirstOutConfig config_;
};

#endif