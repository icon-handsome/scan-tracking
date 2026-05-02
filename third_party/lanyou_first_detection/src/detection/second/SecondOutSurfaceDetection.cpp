#include "detection/second/SecondOutSurfaceDetection.h"

#include <cmath>
#include <vector>
#include <list>
#include <fstream>
#include <algorithm>
#include <limits>
#include <exception>
#include <numeric>
#include <pcl/search/kdtree.h>
#include <pcl/common/centroid.h>
#include <Eigen/Eigenvalues>
#include "utils/Utils.h"
#include "log_manager/LogMacros.h"
#include "pcl/io/ply_io.h"



// 构造函数：初始化第二检测位参数缓存。
SecondOutSurfaceDetection::SecondOutSurfaceDetection() : secondpose_params_(), cylinder_model_(), points_model_(), config_()
{
    if (!LoadConfig("../config/second_config.cfg")) {
        LOG_WARN("SecondOutSurfaceDetection 使用默认参数（配置加载失败）");
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

struct CylSample {
    int idx = -1;
    float s = 0.0f;
    float theta = 0.0f;
    float h = 0.0f;
};

struct CenterlineNode {
    float s = 0.0f;
    float theta = 0.0f;
};

inline float WrapToPi(float a)
{
    while (a > static_cast<float>(M_PI)) a -= 2.0f * static_cast<float>(M_PI);
    while (a <= -static_cast<float>(M_PI)) a += 2.0f * static_cast<float>(M_PI);
    return a;
}

inline float AngleDiffAbs(float a, float b)
{
    return std::abs(WrapToPi(a - b));
}

inline float AlignAngleNear(float angle, float ref)
{
    float a = angle;
    while (a - ref > static_cast<float>(M_PI)) a -= 2.0f * static_cast<float>(M_PI);
    while (a - ref < -static_cast<float>(M_PI)) a += 2.0f * static_cast<float>(M_PI);
    return a;
}

inline void BuildAxisBasis(const Eigen::Vector3f& axis, Eigen::Vector3f& u, Eigen::Vector3f& v)
{
    Eigen::Vector3f d = axis.normalized();
    Eigen::Vector3f helper = (std::abs(d.z()) < 0.9f) ? Eigen::Vector3f::UnitZ() : Eigen::Vector3f::UnitX();
    u = d.cross(helper);
    if (u.norm() < 1e-6f) u = d.cross(Eigen::Vector3f::UnitY());
    u.normalize();
    v = d.cross(u).normalized();
}
} // namespace

bool SecondOutSurfaceDetection::LoadConfig(const std::string& config_file_path)
{
    std::ifstream in(config_file_path);
    if (!in.is_open()) {
        LOG_WARN("SecondOut config open failed: " << config_file_path);
        return false;
    }

    std::string line;
    while (std::getline(in, line))
    {
        std::string key, value;
        if (!ParseLineKV(line, key, value)) continue;

        try {
            if (key == "cylinder_voxel_leaf") config_.cylinder_voxel_leaf = std::stof(value);
            else if (key == "enable_cylinder_smoothing") config_.enable_cylinder_smoothing = (std::stoi(value) != 0);
            else if (key == "cylinder_smooth_max_points") config_.cylinder_smooth_max_points = std::stoi(value);
            else if (key == "cylinder_smooth_radius") config_.cylinder_smooth_radius = std::stof(value);
            else if (key == "cylinder_smooth_polynomial_order") config_.cylinder_smooth_polynomial_order = std::stoi(value);
            else if (key == "cylinder_smooth_polynomial_fit") config_.cylinder_smooth_polynomial_fit = (std::stoi(value) != 0);
            else if (key == "seam_voxel_leaf") config_.seam_voxel_leaf = std::stof(value);
            else if (key == "normal_radius") config_.normal_radius = std::stof(value);
            else if (key == "fit_distance_threshold") config_.fit_distance_threshold = std::stof(value);
            else if (key == "radius_limit_min") config_.radius_limit_min = std::stof(value);
            else if (key == "radius_limit_max") config_.radius_limit_max = std::stof(value);
            else if (key == "st_theta_bins") config_.st_theta_bins = std::stoi(value);
            else if (key == "st_candidate_band_half_width") config_.st_candidate_band_half_width = std::stof(value);
            else if (key == "st_candidate_min_abs_h") config_.st_candidate_min_abs_h = std::stof(value);
            else if (key == "st_min_bin_points") config_.st_min_bin_points = std::stoi(value);
            else if (key == "st_centerline_s_step") config_.st_centerline_s_step = std::stof(value);
            else if (key == "st_centerline_s_window") config_.st_centerline_s_window = std::stof(value);
            else if (key == "st_min_centerline_points") config_.st_min_centerline_points = std::stoi(value);
            else if (key == "st_section_step") config_.st_section_step = std::stof(value);
            else if (key == "st_section_half_thickness") config_.st_section_half_thickness = std::stof(value);
            else if (key == "st_section_half_width_t") config_.st_section_half_width_t = std::stof(value);
            else if (key == "st_min_points_per_section") config_.st_min_points_per_section = std::stoi(value);
            else if (key == "st_profile_bin_du") config_.st_profile_bin_du = std::stof(value);
            else if (key == "st_toe_search_half_width_t") config_.st_toe_search_half_width_t = std::stof(value);
            else if (key == "st_toe_min_gap_t") config_.st_toe_min_gap_t = std::stof(value);
            else if (key == "st_parent_d1") config_.st_parent_d1 = std::stof(value);
            else if (key == "st_parent_d2") config_.st_parent_d2 = std::stof(value);
            else if (key == "st_huber_delta") config_.st_huber_delta = std::stof(value);
            else if (key == "st_min_parent_points") config_.st_min_parent_points = std::stoi(value);
            else if (key == "st_min_valid_sections") config_.st_min_valid_sections = std::stoi(value);
        } catch (const std::exception&) {
            LOG_WARN("SecondOut config parse failed for key: " << key << ", value: " << value);
        }
    }

    // 防御性修正
    config_.cylinder_smooth_max_points = std::max(1000, config_.cylinder_smooth_max_points);
    config_.cylinder_smooth_radius = std::max(0.1f, config_.cylinder_smooth_radius);
    config_.cylinder_smooth_polynomial_order = std::max(1, config_.cylinder_smooth_polynomial_order);
    config_.radius_limit_max = std::max(config_.radius_limit_min + 1e-3f, config_.radius_limit_max);
    config_.st_theta_bins = std::max(72, config_.st_theta_bins);
    config_.st_candidate_band_half_width = std::max(1.0f, config_.st_candidate_band_half_width);
    config_.st_candidate_min_abs_h = std::max(0.01f, config_.st_candidate_min_abs_h);
    config_.st_min_bin_points = std::max(5, config_.st_min_bin_points);
    config_.st_centerline_s_step = std::max(1.0f, config_.st_centerline_s_step);
    config_.st_centerline_s_window = std::max(0.5f, config_.st_centerline_s_window);
    config_.st_min_centerline_points = std::max(5, config_.st_min_centerline_points);
    config_.st_section_step = std::max(1.0f, config_.st_section_step);
    config_.st_section_half_thickness = std::max(0.2f, config_.st_section_half_thickness);
    config_.st_section_half_width_t = std::max(5.0f, config_.st_section_half_width_t);
    config_.st_min_points_per_section = std::max(5, config_.st_min_points_per_section);
    config_.st_profile_bin_du = std::max(0.1f, config_.st_profile_bin_du);
    config_.st_toe_search_half_width_t = std::max(5.0f, config_.st_toe_search_half_width_t);
    config_.st_toe_min_gap_t = std::max(1.0f, config_.st_toe_min_gap_t);
    config_.st_parent_d1 = std::max(1.0f, config_.st_parent_d1);
    config_.st_parent_d2 = std::max(config_.st_parent_d1 + 1.0f, config_.st_parent_d2);
    config_.st_huber_delta = std::max(0.05f, config_.st_huber_delta);
    config_.st_min_parent_points = std::max(5, config_.st_min_parent_points);
    config_.st_min_valid_sections = std::max(3, config_.st_min_valid_sections);

    LOG_INFO("SecondOut config loaded from " << config_file_path
             << ", radius_limits=[" << config_.radius_limit_min << ", " << config_.radius_limit_max << "]"
             << ", smooth=" << (config_.enable_cylinder_smoothing ? "on" : "off")
             << ", smooth_radius=" << config_.cylinder_smooth_radius
             << ", smooth_max_points=" << config_.cylinder_smooth_max_points);
    return true;
}

bool SecondOutSurfaceDetection::Detect(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr seam_cloud(new pcl::PointCloud<pcl::PointXYZ>(*cloud));
    return Detect(cloud, seam_cloud);
}

bool SecondOutSurfaceDetection::Detect(pcl::PointCloud<pcl::PointXYZ>::Ptr& cylinder_cloud,
                                       pcl::PointCloud<pcl::PointXYZ>::Ptr& seam_cloud)
{
    if (!DetectCylinder(cylinder_cloud)) {
        LOG_ERROR("Failed to detect cylinder in double-cloud interface.");
        return false;
    }
    if (!seam_cloud || seam_cloud->empty()) {
        LOG_WARN("Seam cloud is empty! Cannot classify seams.");
        return true;
    }
    Utils::VoxelGridDownSample(seam_cloud, seam_cloud, config_.seam_voxel_leaf);

    const Eigen::Vector3f c = cylinder_model_.center;
    const Eigen::Vector3f d = cylinder_model_.axis.normalized();
    const float R = static_cast<float>(cylinder_model_.radius);
    Eigen::Vector3f u, v;
    BuildAxisBasis(d, u, v);

    auto medianOf = [](std::vector<float> vals) -> float {
        if (vals.empty()) return 0.0f;
        const size_t n = vals.size();
        std::nth_element(vals.begin(), vals.begin() + n / 2, vals.end());
        float med = vals[n / 2];
        if ((n % 2) == 0) {
            std::nth_element(vals.begin(), vals.begin() + (n / 2 - 1), vals.end());
            med = 0.5f * (med + vals[n / 2 - 1]);
        }
        return med;
    };

    auto robustLineFit = [&](const std::vector<std::pair<float, float>>& pts, float& a, float& b) -> bool {
        if (pts.size() < static_cast<size_t>(config_.st_min_parent_points)) return false;
        a = 0.0f; b = 0.0f;
        for (int it = 0; it < 8; ++it) {
            std::vector<float> residuals;
            residuals.reserve(pts.size());
            for (const auto& p : pts) residuals.push_back(std::abs(p.second - (a * p.first + b)));
            const float mad = medianOf(residuals);
            const float sigma = std::max(1e-4f, 1.4826f * mad);
            const float delta = std::max(config_.st_huber_delta, 1.345f * sigma);

            double Sxx = 0.0, Sx = 0.0, Sw = 0.0, Sxy = 0.0, Sy = 0.0;
            for (const auto& p : pts) {
                const float r = std::abs(p.second - (a * p.first + b));
                const float w = (r <= delta || r < 1e-8f) ? 1.0f : (delta / r);
                Sxx += w * p.first * p.first;
                Sx += w * p.first;
                Sw += w;
                Sxy += w * p.first * p.second;
                Sy += w * p.second;
            }
            const double det = Sxx * Sw - Sx * Sx;
            if (std::abs(det) < 1e-9) return false;
            const float new_a = static_cast<float>((Sxy * Sw - Sx * Sy) / det);
            const float new_b = static_cast<float>((Sxx * Sy - Sx * Sxy) / det);
            if (std::abs(new_a - a) < 1e-5f && std::abs(new_b - b) < 1e-5f) { a = new_a; b = new_b; break; }
            a = new_a; b = new_b;
        }
        return std::isfinite(a) && std::isfinite(b);
    };

    // 1) 构建 H(s,t) 样本
    std::vector<CylSample> samples;
    samples.reserve(seam_cloud->points.size());
    float min_s = std::numeric_limits<float>::max();
    float max_s = -std::numeric_limits<float>::max();
    for (size_t i = 0; i < seam_cloud->points.size(); ++i) {
        const auto& p = seam_cloud->points[i];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        const Eigen::Vector3f P(p.x, p.y, p.z);
        const Eigen::Vector3f q = P - c;
        const float s = q.dot(d);
        const Eigen::Vector3f radial = q - s * d;
        const float rho = radial.norm();
        if (!std::isfinite(rho) || rho < 1e-6f) continue;
        const float theta = std::atan2(radial.dot(v), radial.dot(u));
        samples.push_back(CylSample{static_cast<int>(i), s, theta, rho - R});
        min_s = std::min(min_s, s);
        max_s = std::max(max_s, s);
    }
    if (samples.size() < static_cast<size_t>(config_.st_theta_bins)) {
        LOG_ERROR("有效样本不足，无法执行H(s,t)测量: " << samples.size());
        return false;
    }

    // 2) 候选焊缝带（theta峰值）
    std::vector<float> score_sum(config_.st_theta_bins, 0.0f);
    std::vector<int> score_cnt(config_.st_theta_bins, 0);
    for (const auto& smp : samples) {
        const float t01 = (smp.theta + static_cast<float>(M_PI)) / (2.0f * static_cast<float>(M_PI));
        int bin = std::clamp(static_cast<int>(t01 * config_.st_theta_bins), 0, config_.st_theta_bins - 1);
        score_sum[bin] += std::abs(smp.h);
        score_cnt[bin] += 1;
    }
    std::vector<float> smooth_score(config_.st_theta_bins, 0.0f);
    for (int i = 0; i < config_.st_theta_bins; ++i) {
        float acc = 0.0f;
        for (int k = -2; k <= 2; ++k) {
            int j = (i + k + config_.st_theta_bins) % config_.st_theta_bins;
            const float mean = (score_cnt[j] >= config_.st_min_bin_points) ? (score_sum[j] / score_cnt[j]) : 0.0f;
            acc += mean;
        }
        smooth_score[i] = acc / 5.0f;
    }
    const auto peak_it = std::max_element(smooth_score.begin(), smooth_score.end());
    const int peak_bin = static_cast<int>(std::distance(smooth_score.begin(), peak_it));
    const float peak_score = *peak_it;
    if (peak_score < config_.st_candidate_min_abs_h) {
        LOG_ERROR("焊缝候选带强度不足，peak_score=" << peak_score);
        return false;
    }
    const float theta_center = ((static_cast<float>(peak_bin) + 0.5f) / static_cast<float>(config_.st_theta_bins))
                             * (2.0f * static_cast<float>(M_PI)) - static_cast<float>(M_PI);
    const float half_angle = config_.st_candidate_band_half_width / std::max(1.0f, R);

    std::vector<int> candidate_ids;
    candidate_ids.reserve(samples.size() / 3);
    pcl::PointCloud<pcl::PointXYZ>::Ptr candidate_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < samples.size(); ++i) {
        if (AngleDiffAbs(samples[i].theta, theta_center) <= half_angle) {
            candidate_ids.push_back(static_cast<int>(i));
            candidate_cloud->push_back(seam_cloud->points[samples[i].idx]);
        }
    }
    pcl::io::savePLYFile("../data/second/SeamCandidateBand.ply", *candidate_cloud);

    // 3) 中心线 theta(s)
    std::vector<CenterlineNode> centerline;
    float prev_theta = theta_center;
    for (float s0 = min_s; s0 <= max_s; s0 += config_.st_centerline_s_step) {
        float wcos = 0.0f, wsin = 0.0f, wsum = 0.0f;
        int cnt = 0;
        for (const int cid : candidate_ids) {
            const auto& smp = samples[cid];
            if (std::abs(smp.s - s0) > config_.st_centerline_s_window) continue;
            const float w0 = std::max(1e-3f, std::abs(smp.h));
            wcos += w0 * std::cos(smp.theta);
            wsin += w0 * std::sin(smp.theta);
            wsum += w0;
            cnt++;
        }
        if (cnt < config_.st_min_centerline_points || wsum <= 1e-6f) continue;
        float theta_s = std::atan2(wsin / wsum, wcos / wsum);
        theta_s = AlignAngleNear(theta_s, prev_theta);
        prev_theta = theta_s;
        centerline.push_back(CenterlineNode{s0, theta_s});
    }
    if (centerline.empty()) {
        LOG_ERROR("中心线提取失败，节点为空");
        return false;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr centerline_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& node : centerline) {
        const Eigen::Vector3f radial_dir = std::cos(node.theta) * u + std::sin(node.theta) * v;
        const Eigen::Vector3f Pc = c + node.s * d + R * radial_dir;
        centerline_cloud->push_back(pcl::PointXYZ(Pc.x(), Pc.y(), Pc.z()));
    }
    pcl::io::savePLYFile("../data/second/SeamCenterline.ply", *centerline_cloud);

    // 4) 截面计算：余高 + 错边
    std::vector<float> section_reinf;
    std::vector<float> section_misalign;
    pcl::PointCloud<pcl::PointXYZ>::Ptr section_support_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    float last_section_s = -std::numeric_limits<float>::max();
    for (const auto& node : centerline) {
        if (node.s - last_section_s < config_.st_section_step) continue;

        std::vector<std::pair<float, float>> profile; // (u, h)
        profile.reserve(256);
        for (const auto& smp : samples) {
            if (std::abs(smp.s - node.s) > config_.st_section_half_thickness) continue;
            const float du = R * WrapToPi(smp.theta - node.theta);
            if (std::abs(du) > config_.st_section_half_width_t) continue;
            profile.emplace_back(du, smp.h);
            section_support_cloud->push_back(seam_cloud->points[smp.idx]);
        }
        if (static_cast<int>(profile.size()) < config_.st_min_points_per_section) continue;

        // bin profile and detect toe by gradient peak
        const float u_min = -config_.st_toe_search_half_width_t;
        const float u_max = config_.st_toe_search_half_width_t;
        const int nb = std::max(8, static_cast<int>(std::ceil((u_max - u_min) / config_.st_profile_bin_du)));
        std::vector<std::vector<float>> bins(static_cast<size_t>(nb));
        for (const auto& p : profile) {
            if (p.first < u_min || p.first > u_max) continue;
            int bi = static_cast<int>((p.first - u_min) / (u_max - u_min) * nb);
            bi = std::clamp(bi, 0, nb - 1);
            bins[bi].push_back(p.second);
        }
        std::vector<float> hbin(static_cast<size_t>(nb), 0.0f);
        for (int i = 0; i < nb; ++i) {
            if (bins[i].empty()) continue;
            hbin[i] = medianOf(bins[i]);
        }
        std::vector<float> grad(static_cast<size_t>(nb), 0.0f);
        for (int i = 1; i + 1 < nb; ++i) grad[i] = (hbin[i + 1] - hbin[i - 1]) * 0.5f;

        const auto bin_u = [&](int bi) { return u_min + (bi + 0.5f) * (u_max - u_min) / nb; };
        float best_l = -1.0f, best_r = -1.0f;
        int idx_l = -1, idx_r = -1;
        for (int i = 1; i + 1 < nb; ++i) {
            const float ub = bin_u(i);
            if (ub < -config_.st_toe_min_gap_t && std::abs(grad[i]) > best_l) { best_l = std::abs(grad[i]); idx_l = i; }
            if (ub >  config_.st_toe_min_gap_t && std::abs(grad[i]) > best_r) { best_r = std::abs(grad[i]); idx_r = i; }
        }
        if (idx_l < 0 || idx_r < 0) continue;
        const float uL = bin_u(idx_l);
        const float uR = bin_u(idx_r);
        if (uR <= uL + 2.0f) continue;

        std::vector<std::pair<float, float>> left_parent, right_parent;
        left_parent.reserve(profile.size() / 4);
        right_parent.reserve(profile.size() / 4);
        for (const auto& p : profile) {
            if (p.first >= uL - config_.st_parent_d2 && p.first <= uL - config_.st_parent_d1) left_parent.push_back(p);
            if (p.first >= uR + config_.st_parent_d1 && p.first <= uR + config_.st_parent_d2) right_parent.push_back(p);
        }
        float aL = 0.0f, bL = 0.0f, aR = 0.0f, bR = 0.0f;
        if (!robustLineFit(left_parent, aL, bL) || !robustLineFit(right_parent, aR, bR)) continue;

        const float u0 = 0.0f;
        const float misalign = std::abs((aL * u0 + bL) - (aR * u0 + bR));
        float reinf_max = -std::numeric_limits<float>::max();
        float reinf_min = std::numeric_limits<float>::max();
        for (const auto& p : profile) {
            if (p.first < uL || p.first > uR) continue;
            const float href = 0.5f * ((aL * p.first + bL) + (aR * p.first + bR));
            const float dh = p.second - href;
            reinf_max = std::max(reinf_max, dh);
            reinf_min = std::min(reinf_min, dh);
        }
        if (!std::isfinite(reinf_max) || !std::isfinite(reinf_min)) continue;
        const float reinf = (std::abs(reinf_max) >= std::abs(reinf_min)) ? reinf_max : reinf_min;
        section_misalign.push_back(misalign);
        section_reinf.push_back(reinf);
        last_section_s = node.s;
    }
    pcl::io::savePLYFile("../data/second/SeamSectionsSupport.ply", *section_support_cloud);
    LOG_INFO("有效截面数:"
                  << " section_reinf=" << section_reinf.size()
                  << ", section_misalign=" << section_misalign.size());
    if (static_cast<int>(section_reinf.size()) < config_.st_min_valid_sections ||
        static_cast<int>(section_misalign.size()) < config_.st_min_valid_sections) {
        LOG_ERROR("有效截面数量不足，无法稳定输出余高/错边"
                  << " section_reinf=" << section_reinf.size()
                  << ", section_misalign=" << section_misalign.size());
        return false;
    }

    const float reinforcement = medianOf(section_reinf);
    const float misalignment = medianOf(section_misalign);
    const auto mm_reinf = std::minmax_element(section_reinf.begin(), section_reinf.end());
    const auto mm_mis = std::minmax_element(section_misalign.begin(), section_misalign.end());
    secondpose_params_.A_welding_left_height_tol = reinforcement;
    secondpose_params_.A_welding_error_edge_tol = misalignment;
    secondpose_params_.B1_welding_left_height_tol = reinforcement;
    secondpose_params_.B1_welding_error_edge_tol = misalignment;

    LOG_INFO("[S-T] sections=" << section_reinf.size()
             << ", reinforcement(median)=" << reinforcement
             << ", reinforcement[min,max]=[" << *mm_reinf.first << "," << *mm_reinf.second << "]"
             << ", misalignment(median)=" << misalignment
             << ", misalignment[min,max]=[" << *mm_mis.first << "," << *mm_mis.second << "]");
    return true;
}

/*
 * @brief 圆柱拟合核心流程。
 * @param cylinder_cloud 输入圆柱点云。
 * @return true 成功。
 * @return false 失败。
*/
bool SecondOutSurfaceDetection::DetectCylinder(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cylinder_cloud)
{
    // 1) 输入点云有效性检查。
    if (!cylinder_cloud || cylinder_cloud->empty()) {
        LOG_ERROR("Cylinder cloud is empty!");
        return false;
    }

    // 2) 体素下采样，降低计算量并抑制噪声。
    pcl::PointCloud<pcl::PointXYZ>::Ptr ds_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    Utils::VoxelGridDownSample(cylinder_cloud, ds_cloud, config_.cylinder_voxel_leaf);
    if (!ds_cloud || ds_cloud->empty()) {
        LOG_ERROR("Downsampled cylinder cloud is empty!");
        return false;
    }

    // 3) 下采样后可选平滑，提升法向与圆柱拟合稳定性。
    pcl::PointCloud<pcl::PointXYZ>::Ptr fit_input_cloud = ds_cloud;
    // if (config_.enable_cylinder_smoothing && static_cast<int>(ds_cloud->size()) <= config_.cylinder_smooth_max_points)
    if (config_.enable_cylinder_smoothing)  {
        pcl::PointCloud<pcl::PointXYZ>::Ptr smooth_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        const bool smooth_ok = points_model_.SmoothPointCloudMLS(ds_cloud,
                                                                 smooth_cloud,
                                                                 config_.cylinder_smooth_radius,
                                                                 config_.cylinder_smooth_polynomial_order,
                                                                 config_.cylinder_smooth_polynomial_fit);
        if (smooth_ok && smooth_cloud && !smooth_cloud->empty()) {
            fit_input_cloud = smooth_cloud;
            LOG_INFO("Cylinder smoothing enabled: in=" << ds_cloud->size() << ", out=" << smooth_cloud->size());
        } else {
            LOG_WARN("Cylinder smoothing failed, fallback to downsampled cloud.");
        }
    } else if (config_.enable_cylinder_smoothing) {
        LOG_INFO("Skip smoothing due to large cloud size: " << ds_cloud->size()
                 << " > " << config_.cylinder_smooth_max_points);
    }

    // 4) 计算法向点云，供圆柱RANSAC拟合使用。
    pcl::PointCloud<pcl::PointNormal>::Ptr normal_cloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    points_model_.ComputePointCloudNormal(fit_input_cloud, normals, normal_cloud, config_.normal_radius);
    if (!normal_cloud || normal_cloud->empty()) {
        LOG_ERROR("Normal cloud is empty in cylinder detection!");
        return false;
    }

    // 5) 在先验半径范围内拟合圆柱模型。
    std::vector<float> radius_limits = {config_.radius_limit_min, config_.radius_limit_max};
    pcl::PointCloud<pcl::PointXYZ>::Ptr fitted_cylinder_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (!points_model_.FitCylinder(normal_cloud, fitted_cylinder_cloud, cylinder_model_, config_.fit_distance_threshold, radius_limits)) {
        LOG_ERROR("FitCylinder failed in second out-surface cylinder detection.");
        return false;
    }

    return true;
}

// 只读访问参数。
const SecondPoseDetectionParams& SecondOutSurfaceDetection::GetParams() const
{
    return secondpose_params_;
}

// 可写访问参数。
SecondPoseDetectionParams& SecondOutSurfaceDetection::GetParams()
{
    return secondpose_params_;
}
