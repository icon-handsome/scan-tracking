#define WIN32_LEAN_AND_MEAN
#define NOMINMAX

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include <windows.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "detection/first/FirstInlinerSurfaceDetection.h"
#include "detection/first/FirstOutSurfaceDetection.h"
#include "utils/Params.h"

namespace {

bool getEnvVar(const char* name, std::string& value)
{
    if (!name || !*name) {
        return false;
    }

    char* buffer = nullptr;
    std::size_t length = 0;
    const errno_t err = _dupenv_s(&buffer, &length, name);
    if (err != 0 || !buffer) {
        return false;
    }

    value.assign(buffer);
    free(buffer);
    return true;
}

using Cloud = pcl::PointCloud<pcl::PointXYZ>;

bool loadCloud(const std::string& path, Cloud::Ptr cloud)
{
    const int status = pcl::io::loadPLYFile(path, *cloud);
    if (status < 0) {
        std::cerr << "Failed to load PLY: " << path << std::endl;
        return false;
    }

    Cloud filteredCloud;
    filteredCloud.reserve(cloud->size());
    for (const auto& point : cloud->points) {
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
            std::abs(point.x) < 5000.0f && std::abs(point.y) < 5000.0f && std::abs(point.z) < 5000.0f) {
            filteredCloud.push_back(point);
        }
    }
    filteredCloud.width = static_cast<std::uint32_t>(filteredCloud.size());
    filteredCloud.height = 1;
    filteredCloud.is_dense = true;
    const auto removed = cloud->size() - filteredCloud.size();
    cloud->swap(filteredCloud);

    std::cout << "Loaded " << cloud->size() << " finite points from " << path
              << ", removed_invalid=" << removed << std::endl;
    return true;
}

void downsampleForOfflineRun(Cloud::Ptr cloud, float leafSize, const std::string& label)
{
    if (!cloud || cloud->empty() || leafSize <= 0.0f) {
        return;
    }

    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
    voxelGrid.setInputCloud(cloud);
    voxelGrid.setLeafSize(leafSize, leafSize, leafSize);

    Cloud filtered;
    voxelGrid.filter(filtered);
    std::cout << "Pre-downsample " << label << ": " << cloud->size()
              << " -> " << filtered.size() << ", leaf=" << leafSize << std::endl;
    cloud->swap(filtered);
}

void strideLimitForOfflineRun(Cloud::Ptr cloud, std::size_t maxPoints, const std::string& label)
{
    if (!cloud || cloud->size() <= maxPoints || maxPoints == 0) {
        return;
    }

    Cloud limited;
    limited.reserve(maxPoints);
    const double step = static_cast<double>(cloud->size()) / static_cast<double>(maxPoints);
    for (std::size_t i = 0; i < maxPoints; ++i) {
        const auto index = static_cast<std::size_t>(i * step);
        limited.push_back((*cloud)[std::min(index, cloud->size() - 1)]);
    }
    limited.width = static_cast<std::uint32_t>(limited.size());
    limited.height = 1;
    limited.is_dense = true;

    std::cout << "Stride-limit " << label << ": " << cloud->size()
              << " -> " << limited.size() << std::endl;
    cloud->swap(limited);
}

void printCloudBounds(const Cloud::Ptr& cloud, const std::string& label)
{
    if (!cloud || cloud->empty()) {
        return;
    }

    pcl::PointXYZ minPoint;
    pcl::PointXYZ maxPoint;
    pcl::getMinMax3D(*cloud, minPoint, maxPoint);
    std::cout << "Bounds " << label
              << ": min=(" << minPoint.x << ", " << minPoint.y << ", " << minPoint.z << ")"
              << ", max=(" << maxPoint.x << ", " << maxPoint.y << ", " << maxPoint.z << ")"
              << ", span=(" << (maxPoint.x - minPoint.x) << ", "
              << (maxPoint.y - minPoint.y) << ", "
              << (maxPoint.z - minPoint.z) << ")" << std::endl;
}

void printParams(const FirstPoseDetectionParams& params)
{
    std::cout << "cylinder_center=" << params.cylinder_center.transpose() << std::endl;
    std::cout << "cylinder_axis=" << params.cylinder_axis.transpose() << std::endl;
    std::cout << "head_angle_tol=" << params.head_angle_tol << std::endl;
    std::cout << "straight_height_tol=" << params.straight_height_tol << std::endl;
    std::cout << "straight_slope_tol=" << params.straight_slope_tol << std::endl;
    std::cout << "inner_diameter=" << params.inner_diameter << std::endl;
    std::cout << "inner_diameter_tol=" << params.inner_diameter_tol << std::endl;
    std::cout << "inner_circle_perimeter_tol=" << params.inner_circle_perimeter_tol << std::endl;
    std::cout << "head_depth_tol=" << params.head_depth_tol << std::endl;
    std::cout << "hole_diameter_tol=" << params.hole_diameter_tol << std::endl;
    std::cout << "outliner_error_log=" << params.outliner_error_log << std::endl;
    std::cout << "inliner_error_log=" << params.inliner_error_log << std::endl;
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc != 4) {
        std::cerr << "Usage: scan_tracking_lanyou_first_station_ply_runner <outer.ply> <inner.ply> <hole.ply>" << std::endl;
        return 2;
    }

    const std::string outerPath = argv[1];
    const std::string innerPath = argv[2];
    const std::string holePath = argv[3];

    const std::string configDir = "D:/work/scan-tracking/third_party/lanyou_first_detection/config";
    _putenv_s("SCAN_TRACKING_LANYOU_CONFIG_DIR", configDir.c_str());
    std::string envValue;
    if (getEnvVar("SCAN_TRACKING_LANYOU_CONFIG_DIR", envValue)) {
        std::cout << "SCAN_TRACKING_LANYOU_CONFIG_DIR=" << envValue << std::endl;
    }

    Cloud::Ptr outerCloud(new Cloud);
    Cloud::Ptr innerCloud(new Cloud);
    Cloud::Ptr holeCloud(new Cloud);

    if (!loadCloud(outerPath, outerCloud) || !loadCloud(innerPath, innerCloud) || !loadCloud(holePath, holeCloud)) {
        return 3;
    }

    downsampleForOfflineRun(outerCloud, 10.0f, "outer");
    downsampleForOfflineRun(innerCloud, 10.0f, "inner");
    downsampleForOfflineRun(holeCloud, 2.0f, "hole");
    strideLimitForOfflineRun(outerCloud, 20000, "outer");
    strideLimitForOfflineRun(innerCloud, 50000, "inner");
    strideLimitForOfflineRun(holeCloud, 5000, "hole");
    printCloudBounds(outerCloud, "outer");
    printCloudBounds(innerCloud, "inner");
    printCloudBounds(holeCloud, "hole");

    ResetGlobalFirstPoseParams();

    FirstOutSurfaceDetection firstOut;
    const bool firstOutOk = firstOut.Detect(outerCloud);
    std::cout << "FirstOutSurfaceDetection=" << (firstOutOk ? "success" : "failed") << std::endl;

    bool firstInlinerOk = false;
    if (firstOutOk) {
        FirstInlinerSurfaceDetection firstInliner;
        firstInlinerOk = firstInliner.Detect(innerCloud, holeCloud);
    }
    std::cout << "FirstInlinerSurfaceDetection=" << (firstInlinerOk ? "success" : "skipped_or_failed") << std::endl;

    const auto& params = GlobalFirstPoseParams();
    printParams(params);

    return firstOutOk && firstInlinerOk ? 0 : 1;
}
