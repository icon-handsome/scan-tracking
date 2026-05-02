#include "detection/first/FirstInlinerSurfaceDetection.h"

#include <map>
#include <vector>
#include <string>
#include <fstream>
#include <exception>
#include <cmath>
#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <pcl/io/ply_io.h>
#include <pcl/filters/crop_box.h>
#include "utils/Utils.h"
#include "utils/tic_toc.h"
#include "log_manager/LogMacros.h"

namespace {
std::vector<std::string> BuildConfigCandidates(const std::string& config_file_name)
{
    namespace fs = std::filesystem;

    std::vector<std::string> candidates;
    if (const char* config_dir = std::getenv("SCAN_TRACKING_LANYOU_CONFIG_DIR")) {
        const fs::path env_path(config_dir);
        if (!env_path.empty()) {
            candidates.push_back((env_path / config_file_name).string());
        }
    }

    const fs::path source_path(__FILE__);
    const fs::path repo_config_path =
        source_path.parent_path().parent_path().parent_path().parent_path() / "config" / config_file_name;
    candidates.push_back(repo_config_path.string());
    candidates.push_back("../config/" + config_file_name);
    candidates.push_back("config/" + config_file_name);
    return candidates;
}

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

float MedianOf(std::vector<float> values)
{
    if (values.empty()) return 0.0f;
    const size_t n = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + n, values.end());
    float med = values[n];
    if (values.size() % 2 == 0) {
        std::nth_element(values.begin(), values.begin() + n - 1, values.end());
        med = 0.5f * (med + values[n - 1]);
    }
    return med;
}

float MadOf(const std::vector<float>& values, float med)
{
    if (values.empty()) return 0.0f;
    std::vector<float> abs_dev;
    abs_dev.reserve(values.size());
    for (const float v : values) abs_dev.push_back(std::abs(v - med));
    return MedianOf(std::move(abs_dev));
}

void BuildOrthonormalBasis(const Eigen::Vector3f& axis, Eigen::Vector3f& u, Eigen::Vector3f& v)
{
    Eigen::Vector3f helper = (std::abs(axis.z()) < 0.9f) ? Eigen::Vector3f::UnitZ() : Eigen::Vector3f::UnitX();
    u = axis.cross(helper);
    if (u.norm() < 1e-6f) u = axis.cross(Eigen::Vector3f::UnitY());
    u.normalize();
    v = axis.cross(u).normalized();
}
} // namespace

FirstInlinerSurfaceDetection::FirstInlinerSurfaceDetection() : firstpose_params_(GlobalFirstPoseParams()), points_model_(), config_()
{
    for (const auto& config_path : BuildConfigCandidates("first_config.cfg")) {
        if (!std::filesystem::exists(config_path)) {
            continue;
        }
        if (LoadConfig(config_path)) {
            return;
        }
    }

    LOG_WARN("FirstInlinerSurfaceDetection fallback to default config");
}

bool FirstInlinerSurfaceDetection::LoadConfig(const std::string& config_file_path)
{
    std::ifstream in(config_file_path);
    if (!in.is_open()) {
        LOG_WARN("FirstInliner config open failed: " << config_file_path);
        return false;
    }
    std::string line;
    while (std::getline(in, line))
    {
        std::string key, value;
        if (!ParseLineKV(line, key, value)) continue;
        try {
            if (key == "slice_step") config_.slice_step = std::stof(value);
            else if (key == "slice_thickness") config_.slice_thickness = std::stof(value);
            else if (key == "slice_min_points") config_.slice_min_points = std::stoi(value);
            else if (key == "circle_min_inlier_points") config_.circle_min_inlier_points = std::stoi(value);
            else if (key == "circle_inlier_sigma_scale") config_.circle_inlier_sigma_scale = std::stof(value);
            else if (key == "circle_inlier_thr_min") config_.circle_inlier_thr_min = std::stof(value);
            else if (key == "circle_inlier_thr_max") config_.circle_inlier_thr_max = std::stof(value);
            else if (key == "diameter_mad_sigma_scale") config_.diameter_mad_sigma_scale = std::stof(value);
            else if (key == "diameter_mad_thr_min") config_.diameter_mad_thr_min = std::stof(value);
            else if (key == "hole_radius_limit_min") config_.hole_radius_limit_min = std::stof(value);
            else if (key == "hole_radius_limit_max") config_.hole_radius_limit_max = std::stof(value);
            else if (key == "downsample_leaf") config_.downsample_leaf = std::stof(value);
            else if (key == "hole_cylinder_fit_distance_threshold") config_.hole_cylinder_fit_distance_threshold = std::stof(value);
        } catch (const std::exception&) {
            LOG_WARN("FirstInliner config parse failed for key: " << key << ", value: " << value);
        }
    }
    config_.slice_step = std::max(0.1f, config_.slice_step);
    config_.slice_thickness = std::max(0.1f, config_.slice_thickness);
    config_.slice_min_points = std::max(3, config_.slice_min_points);
    config_.circle_min_inlier_points = std::max(3, config_.circle_min_inlier_points);
    config_.circle_inlier_sigma_scale = std::max(0.1f, config_.circle_inlier_sigma_scale);
    config_.circle_inlier_thr_min = std::max(1e-3f, config_.circle_inlier_thr_min);
    config_.circle_inlier_thr_max = std::max(config_.circle_inlier_thr_min, config_.circle_inlier_thr_max);
    config_.diameter_mad_sigma_scale = std::max(0.1f, config_.diameter_mad_sigma_scale);
    config_.diameter_mad_thr_min = std::max(1e-3f, config_.diameter_mad_thr_min);

    LOG_INFO("FirstInliner config loaded from " << config_file_path
             << ", slice_step=" << config_.slice_step
             << ", slice_thickness=" << config_.slice_thickness
             << ", slice_min_points=" << config_.slice_min_points
             << ", circle_min_inlier_points=" << config_.circle_min_inlier_points
             << ", downsample_leaf=" << config_.downsample_leaf);
    return true;
}

bool FirstInlinerSurfaceDetection::Detect(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& hole_cloud)
{
    if (!cloud || cloud->empty() || !hole_cloud || hole_cloud->empty()) {
        LOG_ERROR("First inliner cloud or hole cloud is empty!");
        firstpose_params_.inliner_error_log = "inner surface cloud or hole cloud is empty";
        return false;
    }
    Eigen::Vector3f axis = firstpose_params_.cylinder_axis;
    if (!axis.allFinite() || axis.norm() <= 1e-6f) {
        LOG_ERROR("First inliner cylinder_axis is invalid, axis=" << axis.transpose());
        firstpose_params_.inliner_error_log = "invalid axis from first out detection";
        return false;
    }
    axis.normalize();
    TicToc timer;
    timer.tic();
    LOG_INFO("FirstInliner开始: inner_points=" << cloud->size()
             << ", hole_points=" << hole_cloud->size()
             << ", axis=" << axis.transpose()
             << ", center=" << firstpose_params_.cylinder_center.transpose()
             << ", bbox_min=" << firstpose_params_.bbox_min_pt.head<3>().transpose()
             << ", bbox_max=" << firstpose_params_.bbox_max_pt.head<3>().transpose());
    pcl::CropBox<pcl::PointXYZ> crop_box;
    crop_box.setInputCloud(cloud);
    crop_box.setMin(firstpose_params_.bbox_min_pt);
    crop_box.setMax(firstpose_params_.bbox_max_pt);
    pcl::PointCloud<pcl::PointXYZ>::Ptr bbox_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    crop_box.filter(*bbox_cloud);
    LOG_INFO("FirstInliner裁剪后点数=" << bbox_cloud->size());
    if (bbox_cloud->size() < 1000) {
        LOG_ERROR("Cropped bbox cloud is too small in first inliner detection: " << bbox_cloud->size());
        firstpose_params_.inliner_error_log = "too few points after inner bbox crop";
        return false;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    Utils::StatisticalOutlierRemoval(bbox_cloud, filtered_cloud);
    LOG_INFO("FirstInliner统计滤波后点数=" << filtered_cloud->size());
    if (filtered_cloud->size() < static_cast<size_t>(config_.slice_min_points)) {
        LOG_ERROR("Filtered inliner cloud is too small: " << filtered_cloud->size());
        firstpose_params_.inliner_error_log = "too few points after inner statistical filter";
        return false;
    }
    Eigen::Vector3f axis_origin = firstpose_params_.cylinder_center;
    if (!axis_origin.allFinite()) {
        LOG_ERROR("First inliner cylinder_center is invalid, center=" << axis_origin.transpose());
        return false;
    }
    std::vector<float> projections;
    projections.reserve(filtered_cloud->size());
    float min_t = std::numeric_limits<float>::max();
    float max_t = std::numeric_limits<float>::lowest();
    for (const auto& p : filtered_cloud->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        const float t = (Eigen::Vector3f(p.x, p.y, p.z) - axis_origin).dot(axis);
        projections.push_back(t);
        min_t = std::min(min_t, t);
        max_t = std::max(max_t, t);
    }

    const float axis_range = max_t - min_t;
    LOG_INFO("FirstInliner轴向范围: min_t=" << min_t
             << ", max_t=" << max_t
             << ", axis_range=" << axis_range);
    if (!std::isfinite(axis_range) || axis_range <= config_.slice_step) {
        LOG_ERROR("Invalid axial projection range in first inliner detection: " << axis_range);
        firstpose_params_.inliner_error_log = "invalid axis projection range";
        return false;
    }
    Eigen::Vector3f u, v;
    BuildOrthonormalBasis(axis, u, v);
    const float half_thickness = config_.slice_thickness * 0.5f;
    std::vector<float> diameters;
    diameters.reserve(static_cast<size_t>(std::ceil(axis_range / config_.slice_step)) + 1);

    size_t total_slices = 0;
    size_t valid_slices = 0;
    size_t skipped_by_min_points = 0;
    size_t skipped_by_invalid_circle = 0;
    size_t skipped_by_circle_inliers = 0;
    for (float t_center = min_t; t_center <= max_t; t_center += config_.slice_step) {
        ++total_slices;
        std::vector<Eigen::Vector2d> pts2d;
        pts2d.reserve(256);
        for (const auto& p : filtered_cloud->points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
            const Eigen::Vector3f pt(p.x, p.y, p.z);
            const Eigen::Vector3f d = pt - axis_origin;
            const float t = d.dot(axis);
            if (std::abs(t - t_center) > half_thickness) continue;
            const Eigen::Vector3f radial = d - t * axis;
            pts2d.emplace_back(static_cast<double>(radial.dot(u)),
                               static_cast<double>(radial.dot(v)));
        }

        if (static_cast<int>(pts2d.size()) < config_.slice_min_points) {
            ++skipped_by_min_points;
            continue;
        }
        const Eigen::Vector3d c0 = Utils::fitCircle2D(pts2d);
        if (!std::isfinite(c0[2]) || c0[2] <= 1e-6) {
            ++skipped_by_invalid_circle;
            continue;
        }
        std::vector<float> residuals;
        residuals.reserve(pts2d.size());
        for (const auto& q : pts2d) {
            const float dx = static_cast<float>(q.x() - c0[0]);
            const float dy = static_cast<float>(q.y() - c0[1]);
            residuals.push_back(std::abs(std::sqrt(dx * dx + dy * dy) - static_cast<float>(c0[2])));
        }
        const float med_res = MedianOf(residuals);
        const float mad_res = MadOf(residuals, med_res);
        const float sigma_res = std::max(1e-4f, 1.4826f * mad_res);
        const float inlier_thr = std::clamp(config_.circle_inlier_sigma_scale * sigma_res,
                                            config_.circle_inlier_thr_min,
                                            config_.circle_inlier_thr_max);
        std::vector<Eigen::Vector2d> inlier_pts2d;
        inlier_pts2d.reserve(pts2d.size());
        for (size_t i = 0; i < pts2d.size(); ++i) {
            if (residuals[i] <= inlier_thr) inlier_pts2d.push_back(pts2d[i]);
        }
        if (static_cast<int>(inlier_pts2d.size()) < config_.circle_min_inlier_points) {
            ++skipped_by_circle_inliers;
            continue;
        }
        const Eigen::Vector3d c1 = Utils::fitCircle2D(inlier_pts2d);
        if (!std::isfinite(c1[2]) || c1[2] <= 1e-6) continue;

        diameters.push_back(2.0f * static_cast<float>(c1[2]));
        ++valid_slices;
    }

    LOG_INFO("FirstInliner切片统计: total=" << total_slices
             << ", valid=" << valid_slices
             << ", skipped_by_min_points=" << skipped_by_min_points
             << ", skipped_by_invalid_circle=" << skipped_by_invalid_circle
             << ", skipped_by_circle_inliers=" << skipped_by_circle_inliers);
    if (diameters.size() < 3) {
        LOG_ERROR("Not enough valid circle slices in first inliner detection, valid_slices=" << diameters.size()
                  << ", total_slices=" << total_slices
                  << ", skipped_by_min_points=" << skipped_by_min_points
                  << ", skipped_by_invalid_circle=" << skipped_by_invalid_circle
                  << ", skipped_by_circle_inliers=" << skipped_by_circle_inliers);
        firstpose_params_.inliner_error_log = "too few valid slices for inner detection";
        return false;
    }
    const float raw_median_d = MedianOf(diameters);
    const float raw_mad_d = MadOf(diameters, raw_median_d);
    const float raw_sigma_d = std::max(1e-4f, 1.4826f * raw_mad_d);
    const float d_keep_thr = std::max(config_.diameter_mad_thr_min, config_.diameter_mad_sigma_scale * raw_sigma_d);

    std::vector<float> kept_diameters;
    kept_diameters.reserve(diameters.size());
    for (const float d : diameters) {
        if (std::abs(d - raw_median_d) <= d_keep_thr) kept_diameters.push_back(d);
    }
    if (kept_diameters.size() < 3) {
        LOG_WARN("MAD filtered diameter slices too few, fallback to all diameters.");
        kept_diameters = diameters;
    }
    const auto mm_d = std::minmax_element(kept_diameters.begin(), kept_diameters.end());
    const float min_d = *mm_d.first;
    const float max_d = *mm_d.second;

    firstpose_params_.inner_diameter = MedianOf(kept_diameters);
    firstpose_params_.inner_diameter_tol = max_d - min_d;
    firstpose_params_.inner_circle_perimeter_tol = firstpose_params_.inner_diameter * static_cast<float>(M_PI);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ds(new pcl::PointCloud<pcl::PointXYZ>);
    Utils::VoxelGridDownSample(cloud, cloud_ds, config_.downsample_leaf);
    points_model_.GetProjectedHeightOnAxis(cloud_ds,
                                           axis_origin,
                                           axis,
                                           firstpose_params_.head_depth_tol);
    pcl::PointCloud<pcl::PointXYZ>::Ptr hole_cloud_ds(new pcl::PointCloud<pcl::PointXYZ>);
    Utils::VoxelGridDownSample(hole_cloud, hole_cloud_ds, config_.downsample_leaf);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr normal_cloud(new pcl::PointCloud<pcl::PointNormal>);
    points_model_.ComputePointCloudNormal(hole_cloud_ds, normals, normal_cloud, 3.0f);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    CylinderModel hole_cylinder_model;

    const std::vector<float> radius_limits = {config_.hole_radius_limit_min, config_.hole_radius_limit_max};
  
    if(!points_model_.FitCylinder(normal_cloud, cylinder_cloud, hole_cylinder_model, config_.hole_cylinder_fit_distance_threshold, radius_limits)) {
        LOG_ERROR("Hole cylinder fit failed");
        firstpose_params_.inliner_error_log = "hole cylinder fit failed";
        return false;
    } else {
    firstpose_params_.hole_diameter_tol = hole_cylinder_model.radius * 2.0f;
        Eigen::Vector3f hole_axis = hole_cylinder_model.axis.normalized();

        const float axis_dot = std::clamp(hole_axis.dot(axis), -1.0f, 1.0f);
        float Hole2Main_AxisAngle = std::acos(axis_dot) * (180.0f / static_cast<float>(M_PI));
        if (Hole2Main_AxisAngle < 90.0f) {
            firstpose_params_.head_angle_tol = Hole2Main_AxisAngle;
        } else {
            firstpose_params_.head_angle_tol = 180.0f - Hole2Main_AxisAngle;
        }

    }
    LOG_INFO("first-station inner detection result:");
    LOG_INFO("axis from first out: " << axis.transpose());
    LOG_INFO("slice stats: total=" << total_slices << ", valid=" << valid_slices
             << ", kept=" << kept_diameters.size());
    LOG_INFO("slice min diameter: " << min_d);
    LOG_INFO("slice max diameter: " << max_d);
    LOG_INFO("inner diameter: " << firstpose_params_.inner_diameter);
    LOG_INFO("inner diameter tolerance: " << firstpose_params_.inner_diameter_tol);
    LOG_INFO("inner perimeter: " << firstpose_params_.inner_circle_perimeter_tol);
    LOG_INFO("head depth: " << firstpose_params_.head_depth_tol);
    LOG_INFO("hole diameter: " << firstpose_params_.hole_diameter_tol);
    LOG_INFO("head angle: " << firstpose_params_.head_angle_tol);
    LOG_INFO("first-station inner detection time: " << timer.toc() << " ms");

    return true;
}

const FirstPoseDetectionParams& FirstInlinerSurfaceDetection::GetParams() const
{
    return firstpose_params_;
}

FirstPoseDetectionParams& FirstInlinerSurfaceDetection::GetParams()
{
    return firstpose_params_;
}
