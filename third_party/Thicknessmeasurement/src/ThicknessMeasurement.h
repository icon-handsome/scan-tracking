#pragma once

#include <memory>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "Config.h"

struct ThicknessResult
{
    double innerIcpFitnessScore;
    double outerIcpFitnessScore;
    double thickness;
    std::string thicknessMethod;
    Point3d templateFeaturePoints[2];
    Point3d nearestScanPoints[2];
    Point3d projectedPoints[2];
};

using ThicknessPointCloud = pcl::PointCloud<pcl::PointXYZ>;
using ThicknessPointCloudConstPtr = ThicknessPointCloud::ConstPtr;

bool MeasureThicknessFromClouds(
    const ThicknessConfig& config,
    const ThicknessPointCloudConstPtr& innerTemplateCloud,
    const ThicknessPointCloudConstPtr& outerTemplateCloud,
    const ThicknessPointCloudConstPtr& innerScanCloud,
    const ThicknessPointCloudConstPtr& outerScanCloud,
    ThicknessResult* result,
    std::string* error);

bool MeasureThickness(const ThicknessConfig& config, ThicknessResult* result, std::string* error);
bool SaveResult(const std::string& path, const ThicknessResult& result, std::string* error);
