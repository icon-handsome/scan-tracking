#ifndef FIRST_INLINER_SURFACE_DETECTION_H
#define FIRST_INLINER_SURFACE_DETECTION_H

#include <string>
#include <pcl/common/common.h>
#include "model/PointsModel.h"
#include "utils/Params.h"

class FirstInlinerSurfaceDetection {
public:
/*****************************************
 * @brief                   构造函数：初始化第一检测位参数结构体
 *****************************************/
    FirstInlinerSurfaceDetection();

/*****************************************
 * @brief                   第一检测位内表面检测主流程
 * @param cloud             输入点云指针（内表面点云）
 * @param hole_cloud        输出开孔点云指针
 * @return bool             检测是否成功
 *****************************************/
    bool Detect(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& hole_cloud);

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
    struct FirstInlinerConfig {
        // 切片拟圆参数（读取 first_config.cfg 对应键）
        float slice_step = 5.0f;
        float slice_thickness = 0.6f;
        int slice_min_points = 80;
        int circle_min_inlier_points = 50;
        float circle_inlier_sigma_scale = 2.5f;
        float circle_inlier_thr_min = 0.5f;
        float circle_inlier_thr_max = 2.0f;

        float downsample_leaf = 2.0f;    // 原始点云均匀下采样体素边长(mm)
        float hole_cylinder_fit_distance_threshold = 0.5f; // 开孔圆柱拟合距离阈值(mm)

        // 直径统计稳健参数（可选，不配置时走默认）
        float diameter_mad_sigma_scale = 3.0f;
        float diameter_mad_thr_min = 0.5f;

        float hole_radius_limit_min = 8.0f;  // 封头开孔半径下限(mm)
        float hole_radius_limit_max = 20.0f; // 封头开口半径上限(mm)
    };

    bool LoadConfig(const std::string& config_file_path);

    // 第一检测位监测结构体。
    FirstPoseDetectionParams& firstpose_params_;
    // 每个检测类独立持有点云模型，避免外部共享实例。
    PointsModel points_model_;
    // first_inliner 参数配置。
    FirstInlinerConfig config_;
};

#endif