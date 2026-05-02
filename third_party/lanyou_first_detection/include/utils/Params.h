#ifndef PARAMS_H
#define PARAMS_H

#include <string>
#include <vector>

#include <Eigen/Core>

struct Point2D {
    double depth = 0.0;
    double length = 0.0;

    Point2D() = default;
    Point2D(double depthValue, double lengthValue)
        : depth(depthValue), length(lengthValue)
    {
    }
};

struct FirstPoseDetectionParams {
    Eigen::Vector4f bbox_min_pt = Eigen::Vector4f::Zero();
    Eigen::Vector4f bbox_max_pt = Eigen::Vector4f::Zero();
    Eigen::Vector3f cylinder_axis = Eigen::Vector3f::Zero();
    Eigen::Vector3f cylinder_center = Eigen::Vector3f::Zero();

    float inner_diameter = 0.0f;
    float inner_diameter_tol = 0.0f;
    float inner_circle_perimeter_tol = 0.0f;
    float head_depth_tol = 0.0f;
    float hole_diameter_tol = 0.0f;

    float head_angle_tol = 0.0f;
    float blunt_height_tol = 0.0f;
    float straight_slope_tol = 0.0f;
    float straight_height_tol = 0.0f;

    std::string inliner_error_log;
    std::string outliner_error_log;
};

inline FirstPoseDetectionParams& GlobalFirstPoseParams()
{
    static FirstPoseDetectionParams params;
    return params;
}

inline void ResetGlobalFirstPoseParams()
{
    GlobalFirstPoseParams() = FirstPoseDetectionParams{};
}

struct SecondPoseDetectionParams {
    float inner_diameter = 0.0f;
    float pipe_length = 0.0f;
    float inner_circle_perimeter_tol = 0.0f;
    float volume_tol = 0.0f;

    float A_welding_error_edge_tol = 0.0f;
    float A_welding_seam_edge_angle_tol = 0.0f;
    float A_welding_left_height_tol = 0.0f;
    int A_welding_undercut_tol = 0;

    float B1_welding_error_edge_tol = 0.0f;
    float B1_welding_seam_edge_angle_tol = 0.0f;
    float B1_welding_left_height_tol = 0.0f;
    int B1_welding_undercut_tol = 0;

    float B2_welding_error_edge_tol = 0.0f;
    float B2_welding_seam_edge_angle_tol = 0.0f;
    float B2_welding_left_height_tol = 0.0f;
    int B2_welding_undercut_tol = 0;
};

struct ThirdPoseDetectionParams {
    float container_length = 0.0f;
    std::vector<float> steel_ring_distance_tol;
    float upper_part_size_tol = 0.0f;
    float lower_part_size_tol = 0.0f;
    float lifting_lug_angle_tol = 0.0f;
};

#endif
