#pragma once

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <map>
#include <string>
#include <vector>

namespace bevel
{
using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

struct BevelMeasurementResult
{
    bool ok = false;
    double angleDeg = 0.0;      // Bevel angle, degree
	double length = 0.0;        // Blunt edge length
    double icpFitness = 0.0;
    Eigen::Matrix4f scanToTemplate = Eigen::Matrix4f::Identity();
    std::string message;
};

struct BevelConfig
{
    bool uniformDownsample = true;
    double uniformLeafSize = 1.0;
    bool saveDownsampledCloud = true;
    std::string downsampledCloudPath = "data/debug/downsampled_cloud.pcd";

    bool poseCorrection = true;
    Eigen::Vector3f poseTranslation = Eigen::Vector3f::Zero();
    Eigen::Vector3f poseRotationDeg = Eigen::Vector3f::Zero();

    bool cropBox = true;
    Eigen::Vector4f cropMin = Eigen::Vector4f(-800.0f, -800.0f, -200.0f, 1.0f);
    Eigen::Vector4f cropMax = Eigen::Vector4f(800.0f, 800.0f, 200.0f, 1.0f);

    bool outlierRemoval = true;
    int sorMeanK = 30;
    double sorStddevMulThresh = 1.0;  // Statistical outlier stddev multiplier
    int icpMaxIterations = 80;
    double icpMaxCorrespondenceDistance = 5.0;
    double icpTransformationEpsilon = 1e-8;
    double icpEuclideanFitnessEpsilon = 1e-6;
    bool icpTrimEnable = true;
    double icpTrimOverlapRatio = 0.7;
    int icpTrimMinCorrespondences = 50;
    bool saveAlignedCloud = true;
    std::string alignedCloudPath = "data/debug/aligned_scan.pcd";

    std::string templatePath = "data/templates/type_0_template.pcd";
    std::string measurementMethod = "plane_fit";

};

struct TemplateFeature
{
    std::string name;
    Eigen::Vector3f point;
};

BevelConfig loadConfig(const std::string& configPath);

BevelMeasurementResult solveBevelFromRawCloud(const CloudT::ConstPtr& rawCloud,
                                              const std::string& configPath = "config.txt");

BevelMeasurementResult solveBevelFromRawCloud(const CloudT::ConstPtr& rawCloud,
                                              const std::string& configPath,
                                              const std::string& templateDir);

bool loadTextPointCloud(const std::string& path, CloudT::Ptr cloud);

} // namespace bevel
