#include "detection/first/FirstOutSurfaceDetection.h"

#include <vector>
#include <string>
#include <fstream>
#include <exception>
#include <algorithm>
#include <limits>
#include <cmath>
#include <pcl/io/ply_io.h>
#include <pcl/filters/crop_box.h>
#include "utils/Utils.h"
#include "utils/tic_toc.h"
#include "log_manager/LogMacros.h"

// 构造函数：初始化第一检测位参数缓存。
FirstOutSurfaceDetection::FirstOutSurfaceDetection() : firstpose_params_(GlobalFirstPoseParams()), points_model_(), config_()
{
    if (!LoadConfig("../config/first_config.cfg") &&
        !LoadConfig("config/first_config.cfg")) {
        LOG_WARN("[LY-1004-config-missing] FirstOutSurfaceDetection 使用默认参数（配置加载失败）");
    }
}

namespace {
std::string Trim(const std::string& s)
{
    const size_t start = s.find_first_not_of(" \t\r\n");
    const size_t end = s.find_last_not_of(" \t\r\n");
    if (start == std::string::npos || end == std::string::npos) return "";
    return s.substr(start, end - start + 1);
}

bool ParseLineKV(const std::string& line, std::string& key, std::string& value)
{
    std::string raw = line;
    const size_t comment_pos = raw.find('#');
    if (comment_pos != std::string::npos) raw = raw.substr(0, comment_pos);
    raw = Trim(raw);
    if (raw.empty()) return false;

    const size_t colon_pos = raw.find(':');
    if (colon_pos == std::string::npos) return false;
    key = Trim(raw.substr(0, colon_pos));
    value = Trim(raw.substr(colon_pos + 1));
    if (value.size() >= 2 && value.front() == '"' && value.back() == '"') {
        value = value.substr(1, value.size() - 2);
    }
    value = Trim(value);
    return !key.empty() && !value.empty();
}
} // namespace

bool FirstOutSurfaceDetection::LoadConfig(const std::string& config_file_path)
{
    std::ifstream in(config_file_path);
    if (!in.is_open()) {
        LOG_WARN("[LY-1003-file-not-found] FirstOut config open failed: " << config_file_path);
        return false;
    }

    std::string line;
    while (std::getline(in, line))
    {
        std::string key, value;
        if (!ParseLineKV(line, key, value)) continue;

        try {
            if (key == "downsample_leaf") config_.downsample_leaf = std::stof(value);
            else if (key == "coarse_normal_radius") config_.coarse_normal_radius = std::stof(value);
            else if (key == "coarse_fit_distance_threshold") config_.coarse_fit_distance_threshold = std::stof(value);
            else if (key == "radius_limit_min") config_.radius_limit_min = std::stof(value);
            else if (key == "radius_limit_max") config_.radius_limit_max = std::stof(value);
            else if (key == "axis_end_trim_ratio") config_.axis_end_trim_ratio = std::stof(value);
            else if (key == "slice_step") config_.slice_step = std::stof(value);
            else if (key == "slice_thickness") config_.slice_thickness = std::stof(value);
            else if (key == "slice_min_points") config_.slice_min_points = std::stoi(value);
            else if (key == "circle_min_inlier_points") config_.circle_min_inlier_points = std::stoi(value);
            else if (key == "circle_inlier_sigma_scale") config_.circle_inlier_sigma_scale = std::stof(value);
            else if (key == "circle_inlier_thr_min") config_.circle_inlier_thr_min = std::stof(value);
            else if (key == "circle_inlier_thr_max") config_.circle_inlier_thr_max = std::stof(value);
            else if (key == "centerline_inlier_sigma_scale") config_.centerline_inlier_sigma_scale = std::stof(value);
            else if (key == "centerline_inlier_thr_min") config_.centerline_inlier_thr_min = std::stof(value);
            else if (key == "centerline_inlier_thr_max") config_.centerline_inlier_thr_max = std::stof(value);
            else if (key == "axis_refine_max_iter") config_.axis_refine_max_iter = std::stoi(value);
            else if (key == "axis_refine_converge_rad") config_.axis_refine_converge_rad = std::stof(value);
            else if (key == "radial_inlier_sigma_scale") config_.radial_inlier_sigma_scale = std::stof(value);
            else if (key == "radial_inlier_thr_min") config_.radial_inlier_thr_min = std::stof(value);
            else if (key == "radial_inlier_thr_max") config_.radial_inlier_thr_max = std::stof(value);
            else if (key == "line_slice_angle_step_deg") config_.line_slice_angle_step_deg = std::stof(value);
            else if (key == "line_slice_distance_tolerance") config_.line_slice_distance_tolerance = std::stof(value);
            else if (key == "line_fit_distance_tolerance") config_.line_fit_distance_tolerance = std::stof(value);
            else if (key == "vertical_line_parallel_max_angle_deg") config_.vertical_line_parallel_max_angle_deg = std::stof(value);
            else if (key == "edge_stat_filter_ratio") config_.edge_stat_filter_ratio = std::stof(value);
        } catch (const std::exception&) {
            LOG_WARN("[LY-1005-parameter-error] FirstOut config parse failed for key: " << key << ", value: " << value);
        }
    }

    // 防御性修正：避免非法参数导致拟合退化。
    config_.downsample_leaf = std::max(0.1f, config_.downsample_leaf);
    config_.coarse_normal_radius = std::max(0.1f, config_.coarse_normal_radius);
    config_.coarse_fit_distance_threshold = std::max(0.01f, config_.coarse_fit_distance_threshold);
    config_.radius_limit_max = std::max(config_.radius_limit_min + 1e-3f, config_.radius_limit_max);
    config_.axis_end_trim_ratio = std::clamp(config_.axis_end_trim_ratio, 0.0f, 0.49f);
    config_.slice_step = std::max(0.1f, config_.slice_step);
    config_.slice_thickness = std::max(0.1f, config_.slice_thickness);
    config_.slice_min_points = std::max(3, config_.slice_min_points);
    config_.circle_min_inlier_points = std::max(3, config_.circle_min_inlier_points);
    config_.circle_inlier_sigma_scale = std::max(0.1f, config_.circle_inlier_sigma_scale);
    config_.circle_inlier_thr_min = std::max(1e-3f, config_.circle_inlier_thr_min);
    config_.circle_inlier_thr_max = std::max(config_.circle_inlier_thr_min, config_.circle_inlier_thr_max);
    config_.centerline_inlier_sigma_scale = std::max(0.1f, config_.centerline_inlier_sigma_scale);
    config_.centerline_inlier_thr_min = std::max(1e-3f, config_.centerline_inlier_thr_min);
    config_.centerline_inlier_thr_max = std::max(config_.centerline_inlier_thr_min, config_.centerline_inlier_thr_max);
    config_.axis_refine_max_iter = std::max(1, config_.axis_refine_max_iter);
    config_.axis_refine_converge_rad = std::max(1e-8f, config_.axis_refine_converge_rad);
    config_.radial_inlier_sigma_scale = std::max(0.1f, config_.radial_inlier_sigma_scale);
    config_.radial_inlier_thr_min = std::max(1e-3f, config_.radial_inlier_thr_min);
    config_.radial_inlier_thr_max = std::max(config_.radial_inlier_thr_min, config_.radial_inlier_thr_max);
    config_.line_slice_angle_step_deg = std::max(0.1f, config_.line_slice_angle_step_deg);
    config_.line_slice_distance_tolerance = std::max(1e-3f, config_.line_slice_distance_tolerance);
    config_.line_fit_distance_tolerance = std::max(1e-3f, config_.line_fit_distance_tolerance);
    config_.vertical_line_parallel_max_angle_deg = std::clamp(config_.vertical_line_parallel_max_angle_deg, 0.1f, 89.9f);
    config_.edge_stat_filter_ratio = std::clamp(config_.edge_stat_filter_ratio, 0.0f, 0.49f);
    

    LOG_INFO("[LY-0000-success] FirstOut config loaded from " << config_file_path
             << ", downsample_leaf=" << config_.downsample_leaf
             << ", coarse_normal_radius=" << config_.coarse_normal_radius
             << ", coarse_dist=" << config_.coarse_fit_distance_threshold
             << ", radius_limits=[" << config_.radius_limit_min << ", " << config_.radius_limit_max << "]"
             << ", slice_step=" << config_.slice_step
             << ", slice_thickness=" << config_.slice_thickness
             << ", line_slice_angle_step_deg=" << config_.line_slice_angle_step_deg
             << ", vertical_line_parallel_max_angle_deg=" << config_.vertical_line_parallel_max_angle_deg);
    return true;
}

bool FirstOutSurfaceDetection::Detect(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    // 1) 输入点云有效性检查。
    if (!cloud || cloud->empty()) {
        LOG_ERROR("[LY-1001-input-empty] First out-surface cloud is empty!");
        return false;
    }

    // 2) 启动计时器。
    TicToc timer;
    timer.tic();

    // 3) 统计点云包围盒，确定外表面高度截取区间。
    // Eigen::Vector4f min_pt, max_pt;
    // pcl::getMinMax3D(*cloud, min_pt, max_pt);
    // LOG_INFO("min_pt: " << min_pt.transpose());
    // LOG_INFO("max_pt: " << max_pt.transpose());

    // // 4) 高程双边滤波：截取第一工位外轮廓区域。
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // Utils::ElevationBilateralFilter(cloud, filtered_cloud, std::vector<float>({min_pt.z(), min_pt.z() + 80}));

    // 5) 均匀下采样：降低法向估计与粗拟合成本。
    //    参数：downsample_leaf（体素边长，mm）
    Utils::UniformDownSample(cloud, filtered_cloud, config_.downsample_leaf);

    // 6) 计算法向点云，供粗拟合圆柱使用。
    //    参数：coarse_normal_radius（法向搜索半径，mm）
    pcl::PointCloud<pcl::PointNormal>::Ptr normal_cloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    points_model_.ComputePointCloudNormal(filtered_cloud, normals, normal_cloud, config_.coarse_normal_radius);

    // 7) 依据法向分离水平/垂直区域点云。
    // pcl::PointCloud<pcl::PointXYZ>::Ptr hrizontal_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr vertical_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // Utils::FilteredPointsByNormal(normal_cloud, hrizontal_cloud, vertical_cloud);

    // // 8) 保存分类点云，便于调参与质量分析。
    // pcl::io::savePLYFileASCII("../data/hrizontal_point_cloud.ply", *hrizontal_cloud);
    // pcl::io::savePLYFileASCII("../data/vertical_point_cloud.ply", *vertical_cloud);
    // LOG_INFO("hrizontal_cloud: " << hrizontal_cloud->points.size());
    // LOG_INFO("vertical_cloud: " << vertical_cloud->points.size());

    // 9) 第一次粗拟合圆柱，作为后续精拟合的几何先验。
    CylinderModel cylinder_model;
    // 半径先验约束：用于粗拟合 RANSAC 搜索空间收敛。
    std::vector<float> radius_limits = {config_.radius_limit_min, config_.radius_limit_max};

    pcl::PointCloud<pcl::PointXYZ>::Ptr coarse_cylinder_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    CylinderModel coarse_cylinder_model;
    // 参数：coarse_fit_distance_threshold（RANSAC 点到模型距离阈值，mm）
    if (!points_model_.FitCylinder(normal_cloud, coarse_cylinder_cloud, coarse_cylinder_model, config_.coarse_fit_distance_threshold, radius_limits)) {
        LOG_ERROR("[LY-1007-algorithm-failure] Coarse FitCylinder failed in first out-surface detection.");
        return false;
    }

    if (coarse_cylinder_cloud->empty()) {
        LOG_ERROR("[LY-1006-result-invalid] Coarse cylinder inliers are empty.");
        return false;
    }
    pcl::io::savePLYFileASCII("../data/1_outsurface_cylinder_inliers_coarse.ply", *coarse_cylinder_cloud);

    // 10) 用粗拟合圆柱内点计算包围盒，并在原始输入点云中裁剪该范围点云。
    // Eigen::Vector4f bbox_min_pt, bbox_max_pt;
    pcl::getMinMax3D(*coarse_cylinder_cloud, firstpose_params_.bbox_min_pt, firstpose_params_.bbox_max_pt);
    pcl::CropBox<pcl::PointXYZ> crop_box;
    crop_box.setInputCloud(cloud);
    crop_box.setMin(firstpose_params_.bbox_min_pt);
    crop_box.setMax(firstpose_params_.bbox_max_pt);
    pcl::PointCloud<pcl::PointXYZ>::Ptr bbox_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    crop_box.filter(*bbox_cloud);
    if (bbox_cloud->size() < 1000) {
        LOG_ERROR("[LY-1002-point-cloud-invalid] Cropped bbox cloud is too small: " << bbox_cloud->size());
        return false;
    }

    // 11) 在包围盒点云内按轴向去掉两端，只保留中间区间点：
    //     保留区间 = [trim, 1-trim]，其中 trim=axis_end_trim_ratio（默认 0.15）。
    const Eigen::Vector3f axis = coarse_cylinder_model.axis.normalized();
    const Eigen::Vector3f center = coarse_cylinder_model.center;

    float min_proj = std::numeric_limits<float>::max();
    float max_proj = std::numeric_limits<float>::lowest();
    std::vector<float> projections;
    projections.reserve(bbox_cloud->size());

    for (const auto& p : bbox_cloud->points) {
        const Eigen::Vector3f pt(p.x, p.y, p.z);
        const float proj = (pt - center).dot(axis);
        projections.push_back(proj);
        min_proj = std::min(min_proj, proj);
        max_proj = std::max(max_proj, proj);
    }

    const float proj_range = max_proj - min_proj;
    if (!std::isfinite(proj_range) || proj_range <= 1e-6f) {
        LOG_ERROR("[LY-1002-point-cloud-invalid] Invalid projection range in bbox cloud: " << proj_range);
        return false;
    }

    const float band_ratio = config_.axis_end_trim_ratio;
    const float band_len = proj_range * band_ratio;
    const float middle_min = min_proj + band_len;
    const float middle_max = max_proj - band_len;

    pcl::PointCloud<pcl::PointXYZ>::Ptr middle_band_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    middle_band_cloud->reserve(bbox_cloud->size());
    for (std::size_t i = 0; i < bbox_cloud->size(); ++i) {
        const float proj = projections[i];
        if (proj >= middle_min && proj <= middle_max) {
            middle_band_cloud->push_back(bbox_cloud->points[i]);
        }
    }

    if (middle_band_cloud->size() < 1000) {
        LOG_WARN("Middle-band cloud too small (" << middle_band_cloud->size()
                 << "), fallback to bbox cloud for fine fitting.");
        *middle_band_cloud = *bbox_cloud;
    }

    // 12) 使用中间区间点云做“切片拟合圆 -> 圆心拟合轴线”迭代精化。
    pcl::PointCloud<pcl::PointXYZ>::Ptr fine_input_cloud = middle_band_cloud;
    pcl::io::savePLYFileASCII("../data/1_outsurface_cylinder_fine_input_cloud.ply", *fine_input_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    bool refined_ok = false;
    auto median_of = [](std::vector<float> values) -> float {
        if (values.empty()) return 0.0f;
        const size_t n = values.size() / 2;
        std::nth_element(values.begin(), values.begin() + n, values.end());
        float med = values[n];
        if (values.size() % 2 == 0) {
            std::nth_element(values.begin(), values.begin() + n - 1, values.end());
            med = 0.5f * (med + values[n - 1]);
        }
        return med;
    };
    auto mad_of = [&](const std::vector<float>& values, float med) -> float {
        if (values.empty()) return 0.0f;
        std::vector<float> abs_dev;
        abs_dev.reserve(values.size());
        for (const float v : values) abs_dev.push_back(std::abs(v - med));
        return median_of(std::move(abs_dev));
    };

    Eigen::Vector3f refined_axis = coarse_cylinder_model.axis.normalized();
    Eigen::Vector3f refined_center = coarse_cylinder_model.center;
    double refined_radius = coarse_cylinder_model.radius;
    refined_ok = Utils::RefineCylinderAxisAndRadius(fine_input_cloud, refined_radius, refined_axis, refined_center);

    if (refined_ok) {
        std::vector<float> radial_distances;
        radial_distances.reserve(fine_input_cloud->size());
        for (const auto& p : fine_input_cloud->points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
            const Eigen::Vector3f pt(p.x, p.y, p.z);
            const Eigen::Vector3f d = pt - refined_center;
            const float t = d.dot(refined_axis);
            radial_distances.push_back((d - t * refined_axis).norm());
        }

        if (radial_distances.size() >= 1000) {
            const float radius_med = static_cast<float>(refined_radius);
            std::vector<float> radial_residuals;
            radial_residuals.reserve(radial_distances.size());
            for (const float v : radial_distances) radial_residuals.push_back(std::abs(v - radius_med));

            const float residual_med = median_of(radial_residuals);
            const float residual_mad = mad_of(radial_residuals, residual_med);
            const float residual_sigma = std::max(1e-4f, 1.4826f * residual_mad);
            const float radial_tol = std::clamp(config_.radial_inlier_sigma_scale * residual_sigma,
                                                config_.radial_inlier_thr_min,
                                                config_.radial_inlier_thr_max);

            std::vector<float> inlier_t;
            inlier_t.reserve(fine_input_cloud->size());
            cylinder_cloud->reserve(fine_input_cloud->size());
            for (const auto& p : fine_input_cloud->points) {
                if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
                const Eigen::Vector3f pt(p.x, p.y, p.z);
                const Eigen::Vector3f d = pt - refined_center;
                const float t = d.dot(refined_axis);
                const float radial = (d - t * refined_axis).norm();
                if (std::abs(radial - radius_med) <= radial_tol) {
                    cylinder_cloud->push_back(p);
                    inlier_t.push_back(t);
                }
            }

            if (cylinder_cloud->size() >= 1000 && !inlier_t.empty()) {
                const auto mm_t = std::minmax_element(inlier_t.begin(), inlier_t.end());
                const float min_t = *mm_t.first;
                const float max_t = *mm_t.second;
                const float center_t = 0.5f * (min_t + max_t);
                const Eigen::Vector3f final_center = refined_center + center_t * refined_axis;

                cylinder_model.center = final_center;
                cylinder_model.axis = refined_axis;
                cylinder_model.radius = std::clamp(refined_radius,
                                                   static_cast<double>(radius_limits[0]),
                                                   static_cast<double>(radius_limits[1]));
                
                cylinder_model.height = static_cast<double>(max_t - min_t);
            } else {
                refined_ok = false;
            }
        } else {
            refined_ok = false;
        }
    }

    if (!refined_ok) {
        LOG_WARN("Slice circle-center axis refinement failed, fallback to coarse cylinder result.");
        cylinder_model = coarse_cylinder_model;
        *cylinder_cloud = *coarse_cylinder_cloud;
    } else {
        LOG_INFO("Axis refinement done via Utils::RefineCylinderAxisAndRadius"
                 << ", refined_radius=" << cylinder_model.radius
                 << ", refined_height=" << cylinder_model.height);
        LOG_INFO("Refined cylinder model: center=("
                 << cylinder_model.center.x() << ", "
                 << cylinder_model.center.y() << ", "
                 << cylinder_model.center.z() << "), axis=("
                 << cylinder_model.axis.x() << ", "
                 << cylinder_model.axis.y() << ", "
                 << cylinder_model.axis.z() << "), radius="
                 << cylinder_model.radius << ", height=" << cylinder_model.height);
    }

    // 缓存轴线与中心到第一检测位结构体，供 FirstInliner 直接使用。
    firstpose_params_.cylinder_axis = cylinder_model.axis.normalized();
    firstpose_params_.cylinder_center = cylinder_model.center;

    LOG_INFO("FirstOut cylinder two-stage fitting: ds=" << filtered_cloud->size()
             << ", bbox=" << bbox_cloud->size()
             << ", middle_band=" << middle_band_cloud->size()
             << ", fine_input=" << fine_input_cloud->size()
             << ", final_inliers=" << cylinder_cloud->size());

    // 14) 保存两次拟合圆柱内点云（粗拟合 + 精拟合）。
    pcl::io::savePLYFileASCII("../data/1_outsurface_cylinder_inliers_fine.ply", *cylinder_cloud);
    pcl::io::savePLYFileASCII("../data/1_outsurface_cylinder_point_cloud.ply", *cylinder_cloud);

    // 15) 基于“下采样+法向”的点云，沿精拟合轴线切平面拟合直线，提取坡口边与直边。
    //     这里的切片参数与直线拟合阈值由 first_config.cfg 控制。
    std::vector<std::vector<pcl::ModelCoefficients>> line_coeffs;
    std::vector<std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>> line_clouds;
    points_model_.FitLinesOnCylindricalSlices(normal_cloud,
                                              cylinder_model,
                                              line_coeffs,
                                              line_clouds,
                                              config_.line_slice_angle_step_deg,
                                              config_.line_slice_distance_tolerance,
                                              config_.line_fit_distance_tolerance);

    // 16) 先计算所有切片的直边高度，再按高度区间筛选切片。
    std::vector<float> all_vertical_edge_height;
    points_model_.ComputeVerticalEdgeHeight(line_clouds, line_coeffs, all_vertical_edge_height);

    constexpr float kMinStraightHeightMm = 40.0f;
    constexpr float kMaxStraightHeightMm = 60.0f;
    std::vector<std::vector<pcl::ModelCoefficients>> filtered_line_coeffs;
    std::vector<std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>> filtered_line_clouds;
    std::vector<float> vertical_edge_height;
    filtered_line_coeffs.reserve(line_coeffs.size());
    filtered_line_clouds.reserve(line_clouds.size());
    vertical_edge_height.reserve(all_vertical_edge_height.size());

    const size_t n = std::min({line_coeffs.size(), line_clouds.size(), all_vertical_edge_height.size()});
    for (size_t i = 0; i < n; ++i) {
        const float h = all_vertical_edge_height[i];
        if (!std::isfinite(h) || h == INVALID_VALUE) continue;
        if (h < kMinStraightHeightMm || h > kMaxStraightHeightMm) continue;
        filtered_line_coeffs.push_back(line_coeffs[i]);
        filtered_line_clouds.push_back(line_clouds[i]);
        vertical_edge_height.push_back(h);
    }
    LOG_INFO("Height range filter [40,60]mm: total_slices=" << n
             << ", kept=" << filtered_line_coeffs.size());

    // 17) 使用筛选后的切片计算封头坡口角度。
    points_model_.GetHeadAngle(filtered_line_coeffs, firstpose_params_.head_angle_tol, config_.edge_stat_filter_ratio);

    // 18) 使用筛选后的切片计算直边斜度（内倾为负，外倾为正）。
    std::vector<float> vertical_edge_tilt;
    points_model_.ClassifyVerticalEdgeTilt(vertical_edge_height, filtered_line_coeffs, cylinder_model, vertical_edge_tilt);

    // 19) 使用筛选后的切片统计高度/斜度（中位数）。
    points_model_.GetMedianEdgeHeightAndTilt(vertical_edge_height,
                                             vertical_edge_tilt,
                                             firstpose_params_.straight_height_tol,
                                             firstpose_params_.straight_slope_tol,
                                             config_.edge_stat_filter_ratio);

    // 20) 输出结果日志。
    LOG_INFO("head_angle: " << firstpose_params_.head_angle_tol);
    for (size_t i = 0; i < vertical_edge_tilt.size(); ++i) {
        LOG_INFO("直边" << std::to_string(i) << "长度：" << vertical_edge_height[i]
                 << "mm 斜度：" << vertical_edge_tilt[i] << "mm");
    }
    LOG_INFO("直边高度中位数: " << firstpose_params_.straight_height_tol << " mm");
    LOG_INFO("直边斜度中位数: " << firstpose_params_.straight_slope_tol << " mm");
    LOG_INFO("第一检测位外轮廓点云检测时间: " << timer.toc() << " ms");

    return true;
}

// 只读访问参数。
const FirstPoseDetectionParams& FirstOutSurfaceDetection::GetParams() const
{
    return firstpose_params_;
}

// 可写访问参数。
FirstPoseDetectionParams& FirstOutSurfaceDetection::GetParams()
{
    return firstpose_params_;
}
