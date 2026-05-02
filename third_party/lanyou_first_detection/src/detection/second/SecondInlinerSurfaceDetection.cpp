#include "detection/second/SecondInlinerSurfaceDetection.h"

#include <cmath>
#include <vector>
#include <pcl/io/ply_io.h>
#include "utils/Utils.h"
#include "utils/tic_toc.h"
#include "log_manager/LogMacros.h"

// 构造函数：初始化第二检测位参数缓存。
SecondInlinerSurfaceDetection::SecondInlinerSurfaceDetection() : secondpose_params_(), points_model_() {}

bool SecondInlinerSurfaceDetection::Detect(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    // 1) 输入点云有效性检查。
    if (!cloud || cloud->empty()) {
        LOG_ERROR("Second inliner cloud is empty!");
        return false;
    }

    // 2) 启动计时器。
    TicToc timer;
    timer.tic();

    // 3) 统计点云包围盒用于日志与调试。
    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    LOG_INFO("min_pt: " << min_pt.transpose());
    LOG_INFO("max_pt: " << max_pt.transpose());

    // 4) 均匀下采样，降低计算复杂度。
    pcl::PointCloud<pcl::PointXYZ>::Ptr ds_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    Utils::UniformDownSample(cloud, ds_cloud, 2.5f);
    if (!ds_cloud || ds_cloud->empty()) {
        LOG_ERROR("Downsampled second inliner cloud is empty!");
        return false;
    }

    // 5) 计算法向点云。
    pcl::PointCloud<pcl::PointNormal>::Ptr normal_cloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    points_model_.ComputePointCloudNormal(ds_cloud, normals, normal_cloud, 3.0f);

    // 6) 在先验范围内拟合圆柱。
    CylinderModel cylinder_model;
    std::vector<float> radius_limits = {590.0f, 610.0f};

    pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (!points_model_.FitCylinder(normal_cloud, cylinder_cloud, cylinder_model, 2.5f, radius_limits)) {
        LOG_ERROR("FitCylinder failed in second inliner detection.");
        return false;
    }
    // 7) 保存拟合圆柱点云用于离线核验。
    pcl::io::savePLYFileASCII("../data/2_inliner_surface_cylinder_point_cloud.ply", *cylinder_cloud);

    // 8) 回写第二检测位内表面参数。
    secondpose_params_.inner_diameter = cylinder_model.radius * 2.0f;
    secondpose_params_.pipe_length = cylinder_model.height;
    secondpose_params_.inner_circle_perimeter_tol = secondpose_params_.inner_diameter * static_cast<float>(M_PI);
    secondpose_params_.volume_tol = static_cast<float>(
        M_PI * std::pow(secondpose_params_.inner_diameter * 0.5f, 2.0f) * secondpose_params_.pipe_length);

    LOG_INFO("第二检测位内表面检测时间: " << timer.toc() << " ms");

    return true;
}

// 只读访问参数。
const SecondPoseDetectionParams& SecondInlinerSurfaceDetection::GetParams() const
{
    return secondpose_params_;
}

// 可写访问参数。
SecondPoseDetectionParams& SecondInlinerSurfaceDetection::GetParams()
{
    return secondpose_params_;
}
