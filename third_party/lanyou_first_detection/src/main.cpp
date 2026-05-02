#include <iostream>
#include <sstream>
#include <string>
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <chrono>

#include "tools/test.h"
#include "utils/tic_toc.h"
#include "utils/Utils.h"
#include "detection/first/FirstInlinerSurfaceDetection.h"
#include "detection/first/FirstOutSurfaceDetection.h"
#include "detection/second/SecondInlinerSurfaceDetection.h"
#include "detection/second/SecondOutSurfaceDetection.h"

#include "detection/third/ThirdOutSurfaceDetection.h"
#include "log_manager/LogManager.h"
#include "log_manager/LogMacros.h"
using namespace std;

namespace {

template <typename Derived>
std::string vecToString(const Eigen::MatrixBase<Derived>& vec)
{
    std::ostringstream oss;
    oss << vec.transpose();
    return oss.str();
}

void printFirstPoseParams(const FirstPoseDetectionParams& params, const std::string& title)
{
    LOG_INFO("========== " << title << " ==========");
    LOG_INFO("bbox_min_pt=" << vecToString(params.bbox_min_pt));
    LOG_INFO("bbox_max_pt=" << vecToString(params.bbox_max_pt));
    LOG_INFO("cylinder_axis=" << vecToString(params.cylinder_axis));
    LOG_INFO("cylinder_center=" << vecToString(params.cylinder_center));
    LOG_INFO("inner_diameter=" << params.inner_diameter);
    LOG_INFO("inner_diameter_tol=" << params.inner_diameter_tol);
    LOG_INFO("inner_circle_perimeter_tol=" << params.inner_circle_perimeter_tol);
    LOG_INFO("head_depth_tol=" << params.head_depth_tol);
    LOG_INFO("hole_diameter_tol=" << params.hole_diameter_tol);
    LOG_INFO("head_angle_tol=" << params.head_angle_tol);
    LOG_INFO("blunt_height_tol=" << params.blunt_height_tol);
    LOG_INFO("straight_slope_tol=" << params.straight_slope_tol);
    LOG_INFO("straight_height_tol=" << params.straight_height_tol);
    LOG_INFO("inliner_error_log=" << params.inliner_error_log);
    LOG_INFO("outliner_error_log=" << params.outliner_error_log);
}

}  // namespace



int main(int argc, char** argv)
{
    // 日志保存
    // log_manager::LogManager::Instance().InitFromConfig("../config/config.cfg");

    if(argc != 4)
    {
        LOG_ERROR("输入参数错误，需要输入圆柱和焊缝的点云文件名");
        return -1;
    }
    string cylinder_filename = argv[1];
    string inliner_filename = argv[2];
    string inliner_hole_filename = argv[3];
    TicToc timer;
    timer.tic(); // 开始计时
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inliner_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inliner_hole_cloud(new pcl::PointCloud<pcl::PointXYZ>);



    if (pcl::io::loadPLYFile(cylinder_filename, *cloud) == -1)
    {
        LOG_ERROR("Failed to load " << cylinder_filename);
        return -1;
    }
    if (pcl::io::loadPLYFile(inliner_filename, *inliner_cloud) == -1)
    {
        LOG_ERROR("Failed to load " << inliner_filename);
        return -1;
    }
    if (pcl::io::loadPLYFile(inliner_hole_filename, *inliner_hole_cloud) == -1)
    {
        LOG_ERROR("Failed to load " << inliner_hole_filename);
        return -1;
    }
    


    LOG_INFO("Loaded " << cloud->points.size() << " data points from " << cylinder_filename);
    LOG_INFO("读取点云时间: " << timer.toc() << " 毫秒");
    // LOG_INFO("Loaded " << inliner_cloud->points.size() << " data points from " << inliner_filename);
    
    FirstInlinerSurfaceDetection first_inliner_detector;
    FirstOutSurfaceDetection first_out_detector;
    SecondInlinerSurfaceDetection second_inliner_detector;
    SecondOutSurfaceDetection second_out_detector;
    ThirdOutSurfaceDetection third_out_detector;

    const bool first_out_ok = first_out_detector.Detect(cloud); // 第一检测位外表面点云检测结果
    const bool first_inliner_ok = first_inliner_detector.Detect(inliner_cloud, inliner_hole_cloud); // 第一检测位内表面点云检测结果

    LOG_INFO("FirstOutDetect=" << (first_out_ok ? "success" : "failed"));
    LOG_INFO("FirstInlinerDetect=" << (first_inliner_ok ? "success" : "failed"));
    printFirstPoseParams(first_out_detector.GetParams(), "FirstOutSurfaceDetection Params");
    printFirstPoseParams(first_inliner_detector.GetParams(), "FirstInlinerSurfaceDetection Params");

    FirstPoseDetectionParams merged_params = first_out_detector.GetParams();
    const auto& inliner_params = first_inliner_detector.GetParams();
    merged_params.inner_diameter = inliner_params.inner_diameter;
    merged_params.inner_diameter_tol = inliner_params.inner_diameter_tol;
    merged_params.inner_circle_perimeter_tol = inliner_params.inner_circle_perimeter_tol;
    merged_params.head_depth_tol = inliner_params.head_depth_tol;
    merged_params.hole_diameter_tol = inliner_params.hole_diameter_tol;
    merged_params.head_angle_tol = inliner_params.head_angle_tol;
    merged_params.blunt_height_tol = inliner_params.blunt_height_tol;
    merged_params.straight_slope_tol = inliner_params.straight_slope_tol;
    merged_params.straight_height_tol = inliner_params.straight_height_tol;
    merged_params.inliner_error_log = inliner_params.inliner_error_log;
    if (merged_params.outliner_error_log.empty()) {
        merged_params.outliner_error_log = first_out_detector.GetParams().outliner_error_log;
    }
    printFirstPoseParams(merged_params, "Merged FirstPoseDetectionParams");

    // bool is_success = second_out_detector.Detect(cloud, seam_cloud); // 第二检测位外表面点云检测结果
    // if (is_success) {
    //     const auto& secondpose_params_ = second_out_detector.GetParams();
    // }

    // third_out_detector.Detect(cloud);




    // 输出运行时间
    LOG_INFO("运行时间: " << timer.toc() << " 毫秒");


    // generateCylinder();
    // addPointCloud();

    return (first_out_ok && first_inliner_ok) ? 0 : 1;
}
