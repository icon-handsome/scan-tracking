#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

struct Point3d
{
    double x;
    double y;
    double z;
};

struct PointCloudConfig
{
    std::string innerTemplateCloudPath;
    std::string outerTemplateCloudPath;
    std::string innerScanCloudPath;
    std::string outerScanCloudPath;
};

struct PreprocessConfig
{
    bool enableOutlierRemoval;
    int meanK;
    double stddevMulThresh;
    bool enableVoxelDownsample;
    double leafSize;
};

struct IcpConfig
{
    int maxIterations;
    double maxCorrespondenceDistance;
    double transformationEpsilon;
    double euclideanFitnessEpsilon;
};

struct CylinderConfig
{
    Point3d axisPoint;
    Point3d axisDirection;
};

struct OutputConfig
{
    std::string resultPath;
};

enum ThicknessMethod
{
    ThicknessMethodNearestBetweenSurfaces,
    ThicknessMethodTangentPlaneProjection
};

struct ThicknessConfig
{
    PointCloudConfig pointCloud;
    PreprocessConfig preprocess;
    IcpConfig icp;
    ThicknessMethod thicknessMethod;
    CylinderConfig templateCylinder;
    std::vector<Point3d> templateFeaturePoints;
    OutputConfig output;
};

bool LoadConfig(const std::string& path, ThicknessConfig* config, std::string* error);
Eigen::Vector3d ToEigen(const Point3d& point);
std::string ThicknessMethodToString(ThicknessMethod method);
