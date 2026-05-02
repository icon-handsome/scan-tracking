#ifndef SECOND_OUTSURFACE_DETECTION_H
#define SECOND_OUTSURFACE_DETECTION_H

#include <string>
#include <pcl/common/common.h>
#include "model/PointsModel.h"
#include "utils/Params.h"


class SecondOutSurfaceDetection {
public:
    // 构造函数：初始化第二检测位参数结构体。
    SecondOutSurfaceDetection();

    // 兼容旧接口：单点云输入。
    bool Detect(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    // 兼容新接口：双点云输入（当前仅使用圆柱点云）。
    bool Detect(pcl::PointCloud<pcl::PointXYZ>::Ptr& cylinder_cloud,
                pcl::PointCloud<pcl::PointXYZ>::Ptr& seam_cloud);

    // 获取检测结果参数（只读/可写）。
    const SecondPoseDetectionParams& GetParams() const;
    SecondPoseDetectionParams& GetParams();

private:
    struct SecondOutConfig {
        float cylinder_voxel_leaf = 2.0f;
        bool enable_cylinder_smoothing = true;
        int cylinder_smooth_max_points = 80000;
        float cylinder_smooth_radius = 1.5f;
        int cylinder_smooth_polynomial_order = 2;
        bool cylinder_smooth_polynomial_fit = true;
        float seam_voxel_leaf = 0.5f;
        float normal_radius = 3.0f;
        float fit_distance_threshold = 0.25f;
        float radius_limit_min = 595.0f;
        float radius_limit_max = 605.0f;

        int st_theta_bins = 720;
        float st_candidate_band_half_width = 20.0f;
        float st_candidate_min_abs_h = 0.6f;
        int st_min_bin_points = 20;
        float st_centerline_s_step = 5.0f;
        float st_centerline_s_window = 2.5f;
        int st_min_centerline_points = 20;
        float st_section_step = 5.0f;
        float st_section_half_thickness = 1.5f;
        float st_section_half_width_t = 50.0f;
        int st_min_points_per_section = 20;

        float st_profile_bin_du = 1.0f;
        float st_toe_search_half_width_t = 25.0f;
        float st_toe_min_gap_t = 6.0f;
        float st_parent_d1 = 8.0f;
        float st_parent_d2 = 25.0f;
        float st_huber_delta = 0.8f;
        int st_min_parent_points = 10;
        int st_min_valid_sections = 8;
    };

    bool LoadConfig(const std::string& config_file_path);

    // 圆柱拟合核心流程。
    bool DetectCylinder(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cylinder_cloud);

    // 第二检测位监测结构体。
    SecondPoseDetectionParams secondpose_params_;
    // 第二检测位圆柱拟合结果缓存。
    CylinderModel cylinder_model_;
    // 每个检测类独立持有点云模型，避免外部共享实例。
    PointsModel points_model_;
    // second_out 参数配置。
    SecondOutConfig config_;
};

#endif
