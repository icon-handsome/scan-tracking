#pragma once

#include <opencv2/opencv.hpp>
#include <string>

class AppConfig
{
public:
	struct Paths
	{
		std::string left_images;
		std::string right_images;
		std::string template_points;
	};

	struct GeoHash
	{
		float angle_tolerance_deg;
		float length_tolerance;
		float min_percent;
		float max_distance;
		float min_distance;
		cv::Mat scan_to_marker_RT;
	};

	struct Recon
	{
		cv::Mat I1;
		cv::Mat D1;
		cv::Mat E1;
		cv::Mat I2;
		cv::Mat D2;
		cv::Mat E2;
		double epipolar_threshold;
		float min_z_range;
		float max_z_range;
		double max_reproj_err;
		double max_ratio;
	};

	struct Detector
	{
		int pyramid_levels;
		int min_area;
		int max_area;
		int ROI_w;
		int ROI_h;
		double perimeter_radius_px;
		double min_circularity;
		int intensity_threshold;
		int debscan_min_pts;
	};

	struct Limits
	{
		int mark_point_size_max;
		float debscan_filter_dist_max;
		int vote_pnt_size_max;
		int vote_filter_pnt_size_min;
		int recon_neighbor_count_min;
		float pose_ransac_inlier_dist;
		int pose_ransac_iterations;
	};

	static AppConfig& Instance();

	bool Load(const wchar_t* file_name = L"track_config.ini");
	const std::wstring& loaded_path() const;

	Paths paths;
	GeoHash geo_hash;
	Recon recon;
	Detector detector;
	Limits limits;

private:
	AppConfig();
	std::wstring loaded_path_;
};
