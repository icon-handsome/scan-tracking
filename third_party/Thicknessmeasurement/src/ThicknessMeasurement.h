#pragma once

#include <memory>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "Config.h"

struct ThicknessResult
{
    double innerIcpFitnessScore = 0.0;
    double outerIcpFitnessScore = 0.0;
    double thickness = 0.0;
    std::string thicknessMethod;
    Point3d templateFeaturePoints[2] = {};
    Point3d nearestScanPoints[2] = {};
    Point3d projectedPoints[2] = {};
};

using ThicknessPointCloud = pcl::PointCloud<pcl::PointXYZ>;
using ThicknessPointCloudConstPtr = ThicknessPointCloud::ConstPtr;

bool MeasureThickness(const ThicknessConfig& config, ThicknessResult* result, std::string* error);

/// IPC 运行时入口：扫描点云由内存传入，模板仍从 config 路径加载，算法与 demo 一致。
bool MeasureThicknessFromScanClouds(
    const ThicknessConfig& config,
    const ThicknessPointCloudConstPtr& innerScanCloud,
    const ThicknessPointCloudConstPtr& outerScanCloud,
    ThicknessResult* result,
    std::string* error);

bool SaveResult(const std::string& path, const ThicknessResult& result, std::string* error);
