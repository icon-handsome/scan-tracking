// 厚度测量核心算法，同步自「厚度测量 V1.2」。
// IPC 集成保留 MeasureThicknessFromClouds 内存接口；V1.2 调试输出与临时写盘已剔除。
#include "ThicknessMeasurement.h"

#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <vector>

#include <Eigen/Dense>

#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>


namespace
{
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> CloudT;

void ApplyPcdViewpointIfNeeded(const std::string& path, CloudT::Ptr cloud);

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

    ApplyPcdViewpointIfNeeded(path, cloud);
    return true;
}

bool ReadPcdViewpoint(const std::string& path, Eigen::Vector3f* origin, Eigen::Quaternionf* orientation)
{
    std::ifstream input(path.c_str(), std::ios::binary);
    if (!input)
    {
        return false;
    }

    std::string line;
    while (std::getline(input, line))
    {
        if (line.find("VIEWPOINT") == 0)
        {
            std::istringstream stream(line);
            std::string tag;
            float tx = 0.0f;
            float ty = 0.0f;
            float tz = 0.0f;
            float qw = 1.0f;
            float qx = 0.0f;
            float qy = 0.0f;
            float qz = 0.0f;
            stream >> tag >> tx >> ty >> tz >> qw >> qx >> qy >> qz;
            if (!stream)
            {
                return false;
            }

            *origin = Eigen::Vector3f(tx, ty, tz);
            *orientation = Eigen::Quaternionf(qw, qx, qy, qz);
            orientation->normalize();
            return true;
        }

        if (line.find("DATA") == 0)
        {
            break;
        }
    }

    return false;
}

bool IsIdentityViewpoint(const Eigen::Vector3f& origin, const Eigen::Quaternionf& orientation)
{
    return origin.norm() < 1e-6f &&
        std::abs(orientation.w() - 1.0f) < 1e-6f &&
        std::abs(orientation.x()) < 1e-6f &&
        std::abs(orientation.y()) < 1e-6f &&
        std::abs(orientation.z()) < 1e-6f;
}

void ApplyPcdViewpointIfNeeded(const std::string& path, CloudT::Ptr cloud)
{
    if (LowerExtension(path) != "pcd")
    {
        return;
    }

    Eigen::Vector3f origin;
    Eigen::Quaternionf orientation;
    if (!ReadPcdViewpoint(path, &origin, &orientation))
    {
        return;
    }

    if (IsIdentityViewpoint(origin, orientation))
    {
        return;
    }

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = orientation.toRotationMatrix();
    transform.block<3, 1>(0, 3) = origin;
    pcl::transformPointCloud(*cloud, *cloud, transform);
}

CloudT::Ptr PreprocessCloud(const CloudT::ConstPtr& input, const PreprocessConfig& config)
{
    CloudT::Ptr current(new CloudT);
    pcl::copyPointCloud(*input, *current);

    if (config.enableOutlierRemoval && current->size() > static_cast<std::size_t>(config.meanK))
    {
        pcl::StatisticalOutlierRemoval<PointT> sor;
        CloudT::Ptr filtered(new CloudT);
        sor.setInputCloud(current);
        sor.setMeanK(config.meanK);
        sor.setStddevMulThresh(config.stddevMulThresh);
        sor.filter(*filtered);
        current = filtered;
    }

    if (config.enableVoxelDownsample)
    {
        pcl::VoxelGrid<PointT> voxel;
        CloudT::Ptr filtered(new CloudT);
        const float leaf = static_cast<float>(config.leafSize);
        voxel.setInputCloud(current);
        voxel.setLeafSize(leaf, leaf, leaf);
        voxel.filter(*filtered);
        current = filtered;
    }

    return current;
}

bool AlignScanToTemplate(const CloudT::Ptr& scanCloud,
                         const CloudT::Ptr& templateCloud,
                         const IcpConfig& config,
                         CloudT::Ptr alignedCloud,
                         double* fitnessScore,
                         std::string* error)
{
    if (scanCloud->empty() || templateCloud->empty())
    {
        if (error != NULL)
        {
            *error = "ICP input cloud is empty";
        }
        return false;
    }

    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(scanCloud);
    icp.setInputTarget(templateCloud);
    icp.setMaximumIterations(config.maxIterations);
    icp.setMaxCorrespondenceDistance(config.maxCorrespondenceDistance);
    icp.setTransformationEpsilon(config.transformationEpsilon);
    icp.setEuclideanFitnessEpsilon(config.euclideanFitnessEpsilon);
    if (0)
    {
        pcl::registration::CorrespondenceRejectorTrimmed::Ptr trimmed(
            new pcl::registration::CorrespondenceRejectorTrimmed);
        trimmed->setOverlapRatio(static_cast<float>(0.98));
        trimmed->setMinCorrespondences(200.0);
        icp.addCorrespondenceRejector(trimmed);
    }

    icp.align(*alignedCloud);
    if (!icp.hasConverged())
    {
        if (error != NULL)
        {
            *error = "ICP did not converge";
        }
        return false;
    }

    *fitnessScore = icp.getFitnessScore();
    return true;
}

bool FindNearestPoint(
    pcl::KdTreeFLANN<PointT>& tree,
    const Eigen::Vector3d& query,
    Eigen::Vector3d* nearest,
    std::string* error)
{
    PointT point;
    point.x = static_cast<float>(query.x());
    point.y = static_cast<float>(query.y());
    point.z = static_cast<float>(query.z());

    std::vector<int> indices(1);
    std::vector<float> distances(1);
    if (tree.nearestKSearch(point, 1, indices, distances) <= 0)
    {
        if (error != NULL)
        {
            *error = "nearest point search failed";
        }
        return false;
    }

    CloudT::ConstPtr cloud = tree.getInputCloud();
    const PointT& found = cloud->points[indices[0]];
    *nearest = Eigen::Vector3d(found.x, found.y, found.z);
    return true;
}

Eigen::Vector3d ProjectToPlane(const Eigen::Vector3d& point,
                                const Eigen::Vector3d& planePoint,
                                const Eigen::Vector3d& planeNormal)
{
    const double distance = (point - planePoint).dot(planeNormal);
    return point - distance * planeNormal;
}

bool ComputeThicknessOnTangentPlane(const ThicknessConfig& config,
                                    const Eigen::Vector3d nearest[2],
                                    Eigen::Vector3d projected[2],
                                    double* thickness,
                                    std::string* error)
{
    const Eigen::Vector3d axisPoint = ToEigen(config.templateCylinder.axisPoint);
    const Eigen::Vector3d axisDirection = ToEigen(config.templateCylinder.axisDirection).normalized();

    Eigen::Vector3d radial = nearest[0] - axisPoint;
    radial = radial - radial.dot(axisDirection) * axisDirection;
    if (radial.norm() <= 1e-12)
    {
        if (error != NULL)
        {
            *error = "first nearest point lies on or too close to cylinder axis; tangent plane is undefined";
        }
        return false;
    }

    const Eigen::Vector3d planeNormal = axisDirection.cross(radial).normalized();
    const Eigen::Vector3d projectedAxisPoint =
        axisPoint + (nearest[0] - axisPoint).dot(axisDirection) * axisDirection;

    projected[0] = ProjectToPlane(nearest[0], projectedAxisPoint, planeNormal);
    projected[1] = ProjectToPlane(nearest[1], projectedAxisPoint, planeNormal);
    *thickness = (projected[0] - projected[1]).norm();
    return true;
}

bool MeasureByNearestBetweenSurfaces(const ThicknessConfig& config,
                                     const CloudT::Ptr& innerScanInTemplate,
                                     const CloudT::Ptr& outerScanInTemplate,
                                     ThicknessResult* result,
                                     std::string* error)
{
    pcl::KdTreeFLANN<PointT> outerTree;
    pcl::KdTreeFLANN<PointT> innerTree;
    outerTree.setInputCloud(outerScanInTemplate);
    innerTree.setInputCloud(innerScanInTemplate);

    const Eigen::Vector3d feature = ToEigen(config.templateFeaturePoints[0]);
    result->templateFeaturePoints[0] = FromEigen(feature);
    result->templateFeaturePoints[1] = FromEigen(feature);

    Eigen::Vector3d outerNearest;
    Eigen::Vector3d innerNearest;
    if (!FindNearestPoint(outerTree, feature, &outerNearest, error))
    {
        return false;
    }
    if (!FindNearestPoint(innerTree, outerNearest, &innerNearest, error))
    {
        return false;
    }

    result->nearestScanPoints[0] = FromEigen(outerNearest);
    result->nearestScanPoints[1] = FromEigen(innerNearest);
    result->projectedPoints[0] = FromEigen(outerNearest);
    result->projectedPoints[1] = FromEigen(innerNearest);
    result->thickness = (outerNearest - innerNearest).norm();
    return true;
}

bool MeasureByTangentPlaneProjection(const ThicknessConfig& config,
                                     const CloudT::Ptr& innerScanInTemplate,
                                     const CloudT::Ptr& outerScanInTemplate,
                                     ThicknessResult* result,
                                     std::string* error)
{
    pcl::KdTreeFLANN<PointT> outerTree;
    pcl::KdTreeFLANN<PointT> innerTree;
    outerTree.setInputCloud(outerScanInTemplate);
    innerTree.setInputCloud(innerScanInTemplate);

    Eigen::Vector3d nearest[2];
    const Eigen::Vector3d outerFeature = ToEigen(config.templateFeaturePoints[0]);
    const Eigen::Vector3d innerFeature = ToEigen(config.templateFeaturePoints[1]);
    result->templateFeaturePoints[0] = FromEigen(outerFeature);
    result->templateFeaturePoints[1] = FromEigen(innerFeature);

    if (!FindNearestPoint(outerTree, outerFeature, &nearest[0], error))
    {
        return false;
    }
    if (!FindNearestPoint(innerTree, innerFeature, &nearest[1], error))
    {
        return false;
    }
    result->nearestScanPoints[0] = FromEigen(nearest[0]);
    result->nearestScanPoints[1] = FromEigen(nearest[1]);

    Eigen::Vector3d projected[2];
    if (!ComputeThicknessOnTangentPlane(config, nearest, projected, &result->thickness, error))
    {
        return false;
    }

    result->projectedPoints[0] = FromEigen(projected[0]);
    result->projectedPoints[1] = FromEigen(projected[1]);
    return true;
}

bool RunThicknessMeasurement(const ThicknessConfig& config,
                             const CloudT::Ptr& innerTemplateCloud,
                             const CloudT::Ptr& outerTemplateCloud,
                             const CloudT::Ptr& innerScanCloud,
                             const CloudT::Ptr& outerScanCloud,
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

    if (innerTemplateCloud == NULL || innerTemplateCloud->empty())
    {
        if (error != NULL)
        {
            *error = "inner template point cloud is null or empty";
        }
        return false;
    }
    if (outerTemplateCloud == NULL || outerTemplateCloud->empty())
    {
        if (error != NULL)
        {
            *error = "outer template point cloud is null or empty";
        }
        return false;
    }
    if (innerScanCloud == NULL || innerScanCloud->empty())
    {
        if (error != NULL)
        {
            *error = "inner scan point cloud is null or empty";
        }
        return false;
    }
    if (outerScanCloud == NULL || outerScanCloud->empty())
    {
        if (error != NULL)
        {
            *error = "outer scan point cloud is null or empty";
        }
        return false;
    }

    if (config.templateFeaturePoints.size() < 2)
    {
        if (error != NULL)
        {
            *error = "thickness config requires at least two template feature points";
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

    CloudT::Ptr innerScanInTemplate(new CloudT);
    CloudT::Ptr outerScanInTemplate(new CloudT);
    if (!AlignScanToTemplate(innerScanFiltered,
                             innerTemplateCloud,
                             config.icp,
                             innerScanInTemplate,
                             &result->innerIcpFitnessScore,
                             error))
    {
        if (error != NULL)
        {
            *error = "inner surface " + *error;
        }
        return false;
    }
    if (!AlignScanToTemplate(outerScanFiltered,
                             outerTemplateCloud,
                             config.icp,
                             outerScanInTemplate,
                             &result->outerIcpFitnessScore,
                             error))
    {
        if (error != NULL)
        {
            *error = "outer surface " + *error;
        }
        return false;
    }

    result->thicknessMethod = ThicknessMethodToString(config.thicknessMethod);
    if (config.thicknessMethod == ThicknessMethodTangentPlaneProjection)
    {
        return MeasureByTangentPlaneProjection(config,
                                               innerScanInTemplate,
                                               outerScanInTemplate,
                                               result,
                                               error);
    }

    return MeasureByNearestBetweenSurfaces(config,
                                           innerScanInTemplate,
                                           outerScanInTemplate,
                                           result,
                                           error);
}
}

bool MeasureThicknessFromClouds(
    const ThicknessConfig& config,
    const ThicknessPointCloudConstPtr& innerTemplateCloud,
    const ThicknessPointCloudConstPtr& outerTemplateCloud,
    const ThicknessPointCloudConstPtr& innerScanCloud,
    const ThicknessPointCloudConstPtr& outerScanCloud,
    ThicknessResult* result,
    std::string* error)
{
    CloudT::Ptr innerTemplate(new CloudT);
    CloudT::Ptr outerTemplate(new CloudT);
    CloudT::Ptr innerScan(new CloudT);
    CloudT::Ptr outerScan(new CloudT);

    if (innerTemplateCloud != NULL)
    {
        pcl::copyPointCloud(*innerTemplateCloud, *innerTemplate);
    }
    if (outerTemplateCloud != NULL)
    {
        pcl::copyPointCloud(*outerTemplateCloud, *outerTemplate);
    }
    if (innerScanCloud != NULL)
    {
        pcl::copyPointCloud(*innerScanCloud, *innerScan);
    }
    if (outerScanCloud != NULL)
    {
        pcl::copyPointCloud(*outerScanCloud, *outerScan);
    }

    return RunThicknessMeasurement(
        config,
        innerTemplate,
        outerTemplate,
        innerScan,
        outerScan,
        result,
        error);
}

bool MeasureThickness(const ThicknessConfig& config, ThicknessResult* result, std::string* error)
{
    CloudT::Ptr innerTemplateCloud(new CloudT);
    CloudT::Ptr outerTemplateCloud(new CloudT);
    CloudT::Ptr innerScanCloud(new CloudT);
    CloudT::Ptr outerScanCloud(new CloudT);
    if (!LoadCloud(config.pointCloud.innerTemplateCloudPath, innerTemplateCloud, error))
    {
        return false;
    }
    if (!LoadCloud(config.pointCloud.outerTemplateCloudPath, outerTemplateCloud, error))
    {
        return false;
    }
    if (!LoadCloud(config.pointCloud.innerScanCloudPath, innerScanCloud, error))
    {
        return false;
    }
    if (!LoadCloud(config.pointCloud.outerScanCloudPath, outerScanCloud, error))
    {
        return false;
    }

    return RunThicknessMeasurement(
        config,
        innerTemplateCloud,
        outerTemplateCloud,
        innerScanCloud,
        outerScanCloud,
        result,
        error);
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
