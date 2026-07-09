// 厚度测量核心算法，当前仅保留 direct raw 点云路径。
#include "ThicknessMeasurement.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <vector>

#include <Eigen/Dense>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace
{
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> CloudT;

// Windows 下 PCL SOR/KdTree 在大点云或跨 DLL 堆上易触发堆损坏
constexpr std::size_t kMaxOutlierRemovalPoints = 80000;
constexpr std::size_t kMaxPointsBeforePclFilters = 500000;

Point3d FromEigen(const Eigen::Vector3d& value)
{
    Point3d point;
    point.x = value.x();
    point.y = value.y();
    point.z = value.z();
    return point;
}

std::string LowerExtension(const std::string& path)
{
    const std::string::size_type dot = path.find_last_of('.');
    if (dot == std::string::npos)
    {
        return "";
    }

    std::string ext = path.substr(dot + 1);
    for (std::string::size_type i = 0; i < ext.size(); ++i)
    {
        if (ext[i] >= 'A' && ext[i] <= 'Z')
        {
            ext[i] = static_cast<char>(ext[i] - 'A' + 'a');
        }
    }
    return ext;
}

CloudT::Ptr reownCloudPoints(const CloudT::ConstPtr& input)
{
    CloudT::Ptr output(new CloudT);
    if (input == nullptr || input->empty())
    {
        output->width = 0;
        output->height = 1;
        output->is_dense = true;
        return output;
    }

    output->points.reserve(input->points.size());
    for (const PointT& point : input->points)
    {
        output->points.push_back(point);
    }
    output->width = input->width;
    output->height = input->height;
    output->is_dense = input->is_dense;
    return output;
}

void adoptPclOwnedCloud(CloudT::Ptr& cloud)
{
    if (!cloud)
    {
        return;
    }
    static std::vector<CloudT::Ptr>* leaks = new std::vector<CloudT::Ptr>();
    leaks->push_back(cloud);
    cloud.reset();
}

CloudT::Ptr detachPclFilterOutput(CloudT::Ptr pclOutput)
{
    CloudT::Ptr owned = reownCloudPoints(pclOutput);
    adoptPclOwnedCloud(pclOutput);
    return owned;
}

CloudT::Ptr strideDownsampleIfNeeded(const CloudT::ConstPtr& input, std::size_t maxPoints)
{
    CloudT::Ptr owned = reownCloudPoints(input);
    if (owned->size() <= maxPoints)
    {
        return owned;
    }

    const std::size_t stride =
        (owned->size() + maxPoints - 1) / maxPoints;
    CloudT::Ptr reduced(new CloudT);
    reduced->points.reserve((owned->size() + stride - 1) / stride);
    for (std::size_t index = 0; index < owned->size(); index += stride)
    {
        reduced->points.push_back(owned->points[index]);
    }
    reduced->width = static_cast<std::uint32_t>(reduced->points.size());
    reduced->height = 1;
    reduced->is_dense = owned->is_dense;
    return reduced;
}

bool LoadCloud(const std::string& path, CloudT::Ptr cloud, std::string* error)
{
    if (cloud == NULL)
    {
        if (error != NULL)
        {
            *error = "cloud output pointer is null";
        }
        return false;
    }

    const std::string ext = LowerExtension(path);
    int status = -1;
    if (ext == "pcd")
    {
        status = pcl::io::loadPCDFile<PointT>(path, *cloud);
    }
    else if (ext == "ply")
    {
        status = pcl::io::loadPLYFile<PointT>(path, *cloud);
    }
    else
    {
        if (error != NULL)
        {
            *error = "unsupported point cloud format, only .pcd and .ply are supported: " + path;
        }
        return false;
    }

    if (status < 0 || cloud->empty())
    {
        if (error != NULL)
        {
            *error = "failed to load point cloud or cloud is empty: " + path;
        }
        return false;
    }

    CloudT::Ptr loaded = cloud;
    cloud = reownCloudPoints(loaded);
    adoptPclOwnedCloud(loaded);
    return true;
}

CloudT::Ptr PreprocessCloud(const CloudT::ConstPtr& input, const PreprocessConfig& config)
{
    CloudT::Ptr current = strideDownsampleIfNeeded(input, kMaxPointsBeforePclFilters);

    if (config.enableVoxelDownsample && !current->empty())
    {
        pcl::VoxelGrid<PointT> voxel;
        CloudT::Ptr filteredPcl = pcl::make_shared<CloudT>();
        const float leaf = static_cast<float>(config.leafSize);
        voxel.setInputCloud(current);
        voxel.setLeafSize(leaf, leaf, leaf);
        voxel.filter(*filteredPcl);
        current = detachPclFilterOutput(filteredPcl);
    }

    if (config.enableOutlierRemoval
        && !current->empty()
        && current->size() > static_cast<std::size_t>(config.meanK)
        && current->size() <= kMaxOutlierRemovalPoints)
    {
        pcl::StatisticalOutlierRemoval<PointT> sor;
        CloudT::Ptr filteredPcl = pcl::make_shared<CloudT>();
        sor.setInputCloud(current);
        sor.setMeanK(config.meanK);
        sor.setStddevMulThresh(config.stddevMulThresh);
        sor.filter(*filteredPcl);
        current = detachPclFilterOutput(filteredPcl);
    }

    return current;
}

bool FindNearestPoint(
    const CloudT::ConstPtr& cloud,
    const Eigen::Vector3d& query,
    Eigen::Vector3d* nearest,
    std::string* error)
{
    if (nearest == NULL)
    {
        if (error != NULL)
        {
            *error = "nearest output pointer is null";
        }
        return false;
    }

    if (cloud == NULL || cloud->empty())
    {
        if (error != NULL)
        {
            *error = "point cloud is null or empty";
        }
        return false;
    }

    const double qx = query.x();
    const double qy = query.y();
    const double qz = query.z();
    std::size_t bestIndex = 0;
    double bestDistSq = std::numeric_limits<double>::max();
    for (std::size_t index = 0; index < cloud->size(); ++index)
    {
        const PointT& point = cloud->points[index];
        const double dx = static_cast<double>(point.x) - qx;
        const double dy = static_cast<double>(point.y) - qy;
        const double dz = static_cast<double>(point.z) - qz;
        const double distSq = dx * dx + dy * dy + dz * dz;
        if (distSq < bestDistSq)
        {
            bestDistSq = distSq;
            bestIndex = index;
        }
    }

    const PointT& found = cloud->points[bestIndex];
    *nearest = Eigen::Vector3d(found.x, found.y, found.z);
    return true;
}
}  // namespace

bool MeasureThicknessOnScanClouds(
    const ThicknessConfig& config,
    const CloudT::ConstPtr& innerScanCloud,
    const CloudT::ConstPtr& outerScanCloud,
    ThicknessResult* result,
    std::string* error)
{
    if (result == NULL)
    {
        if (error != NULL)
        {
            *error = "result output pointer is null";
        }
        return false;
    }

    if (innerScanCloud == NULL || outerScanCloud == NULL
        || innerScanCloud->empty() || outerScanCloud->empty())
    {
        if (error != NULL)
        {
            *error = "scan point cloud is null or empty";
        }
        return false;
    }

    if (config.templateFeaturePoints.size() < 2)
    {
        if (error != NULL)
        {
            *error = "thickness config requires exactly two template feature points";
        }
        return false;
    }

    CloudT::Ptr innerScanFiltered = PreprocessCloud(innerScanCloud, config.preprocess);
    CloudT::Ptr outerScanFiltered = PreprocessCloud(outerScanCloud, config.preprocess);
    if (innerScanFiltered->empty() || outerScanFiltered->empty())
    {
        if (error != NULL)
        {
            *error = "preprocessed point cloud is empty";
        }
        return false;
    }

    const Eigen::Vector3d outerFeature = ToEigen(config.templateFeaturePoints[0]);
    const Eigen::Vector3d innerFeature = ToEigen(config.templateFeaturePoints[1]);
    result->templateFeaturePoints[0] = FromEigen(outerFeature);
    result->templateFeaturePoints[1] = FromEigen(innerFeature);

    Eigen::Vector3d outerNearest;
    Eigen::Vector3d innerNearest;
    if (!FindNearestPoint(outerScanFiltered, outerFeature, &outerNearest, error))
    {
        return false;
    }
    if (!FindNearestPoint(innerScanFiltered, innerFeature, &innerNearest, error))
    {
        return false;
    }

    result->innerIcpFitnessScore = 0.0;
    result->outerIcpFitnessScore = 0.0;
    result->thicknessMethod = "direct_raw_nearest_between_surfaces";
    result->nearestScanPoints[0] = FromEigen(outerNearest);
    result->nearestScanPoints[1] = FromEigen(innerNearest);
    result->projectedPoints[0] = FromEigen(outerNearest);
    result->projectedPoints[1] = FromEigen(innerNearest);
    result->thickness = (outerNearest - innerNearest).norm();
    return true;
}

bool MeasureThickness(const ThicknessConfig& config, ThicknessResult* result, std::string* error)
{
    CloudT::Ptr innerTemplateCloud(new CloudT);
    CloudT::Ptr outerTemplateCloud(new CloudT);
    CloudT::Ptr innerScanCloud(new CloudT);
    CloudT::Ptr outerScanCloud(new CloudT);
    if (!LoadCloud(config.pointCloud.innerTemplateCloudPath, innerTemplateCloud, error)
        || !LoadCloud(config.pointCloud.outerTemplateCloudPath, outerTemplateCloud, error)
        || !LoadCloud(config.pointCloud.innerScanCloudPath, innerScanCloud, error)
        || !LoadCloud(config.pointCloud.outerScanCloudPath, outerScanCloud, error))
    {
        return false;
    }

    return MeasureThicknessOnScanClouds(config, innerScanCloud, outerScanCloud, result, error);
}

bool MeasureThicknessFromScanClouds(
    const ThicknessConfig& config,
    const ThicknessPointCloudConstPtr& innerScanCloud,
    const ThicknessPointCloudConstPtr& outerScanCloud,
    ThicknessResult* result,
    std::string* error)
{
    CloudT::Ptr innerTemplateCloud(new CloudT);
    CloudT::Ptr outerTemplateCloud(new CloudT);
    if (!LoadCloud(config.pointCloud.innerTemplateCloudPath, innerTemplateCloud, error)
        || !LoadCloud(config.pointCloud.outerTemplateCloudPath, outerTemplateCloud, error))
    {
        return false;
    }

    const CloudT::ConstPtr innerOwned = reownCloudPoints(innerScanCloud);
    const CloudT::ConstPtr outerOwned = reownCloudPoints(outerScanCloud);
    return MeasureThicknessOnScanClouds(config, innerOwned, outerOwned, result, error);
}

bool SaveResult(const std::string& path, const ThicknessResult& result, std::string* error)
{
    std::ofstream out(path.c_str());
    if (!out)
    {
        if (error != NULL)
        {
            *error = "failed to open result file: " + path;
        }
        return false;
    }

    out.precision(12);
    out << "{\n";
    out << "  \"inner_icp_fitness_score\": " << result.innerIcpFitnessScore << ",\n";
    out << "  \"outer_icp_fitness_score\": " << result.outerIcpFitnessScore << ",\n";
    out << "  \"thickness_method\": \"" << result.thicknessMethod << "\",\n";
    out << "  \"thickness\": " << result.thickness << ",\n";
    out << "  \"template_feature_points\": [\n";
    for (int i = 0; i < 2; ++i)
    {
        out << "    {\"x\": " << result.templateFeaturePoints[i].x
            << ", \"y\": " << result.templateFeaturePoints[i].y
            << ", \"z\": " << result.templateFeaturePoints[i].z << "}";
        out << (i == 0 ? "," : "") << "\n";
    }
    out << "  ],\n";
    out << "  \"nearest_scan_points\": [\n";
    for (int i = 0; i < 2; ++i)
    {
        out << "    {\"x\": " << result.nearestScanPoints[i].x
            << ", \"y\": " << result.nearestScanPoints[i].y
            << ", \"z\": " << result.nearestScanPoints[i].z << "}";
        out << (i == 0 ? "," : "") << "\n";
    }
    out << "  ],\n";
    out << "  \"projected_points\": [\n";
    for (int i = 0; i < 2; ++i)
    {
        out << "    {\"x\": " << result.projectedPoints[i].x
            << ", \"y\": " << result.projectedPoints[i].y
            << ", \"z\": " << result.projectedPoints[i].z << "}";
        out << (i == 0 ? "," : "") << "\n";
    }
    out << "  ]\n";
    out << "}\n";
    return true;
}
