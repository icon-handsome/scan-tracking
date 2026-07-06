#pragma once

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>
#include <vector>

namespace hm
{

using PointT = pcl::PointXYZ;
using Cloud = pcl::PointCloud<PointT>;
using CloudPtr = Cloud::Ptr;
using CloudConstPtr = Cloud::ConstPtr;

struct CropBox 
{
    Eigen::Vector3f min{-100000.0f, -100000.0f, -100000.0f};
    Eigen::Vector3f max{100000.0f, 100000.0f, 100000.0f};
};

struct OpeningFeature 
{
    std::string name;
    double searchRadiusMm{80.0};
    double expectedDiameterMm{40.0};
    double diameterToleranceMm{15.0};
    Eigen::Vector3d projectionDirection{0.0, 0.0, 1.0};
    int projectionImageWidth{600};
    int projectionImageHeight{600};
    CropBox projectionCrop;
    std::vector<Eigen::Vector3d> cylinderFeaturePoints;
    double icpMaxCorrespondenceDistanceMm{10.0};
    int icpMaxIterations{80};
    double icpTransformationEpsilon{1e-8};
    double icpFitnessEpsilon{1e-6};
};

struct StraightEndpointPair
{
    std::string name;
    Eigen::Vector3d pointB{0.0, 0.0, 0.0};
};

struct MeasureConfig 
{
    std::vector<std::string> inputFrames;
    std::string templateCloud;
    std::vector<OpeningFeature> templateOpenings;
    std::vector<StraightEndpointPair> straightEndpointPairs;

    Eigen::Matrix4f poseCorrection = Eigen::Matrix4f::Identity();
    std::vector<CropBox> cropBoxes;

    int statisticalMeanK{30};
    double statisticalStddevMul{1.0};
    double voxelLeafMm{1.0};

    double cylinderDistanceThresholdMm{2.0};
    int cylinderMaxIterations{2000};

    double icpMaxCorrespondenceDistanceMm{10.0};
    int icpMaxIterations{80};
    double icpTransformationEpsilon{1e-8};
    double icpFitnessEpsilon{1e-6};

    double sliceSpacingMm{5.0};
    int sliceCount{5};
    int sliceMinPoints{500};
    double sliceThicknessMm{2.0};
    double circleMaxFitErrorMm{3.0};

    std::vector<Eigen::Vector3d> templateTopPlaneFeaturePoints;
    Eigen::Vector3d templateTopPlaneNormal{0.0, 0.0, -1.0};
    double topFeatureSearchRadiusMm{30.0};
    double straightSideOffsetBelowTopMm{10.0};
    double straightSideCylinderCropHeightMm{35.0};
    double straightSideCropHeightMm{80.0};
    double straightEndpointSearchRadiusMm{20.0};
    double straightEndpointSliceThicknessMm{2.0};
    double straightLineDistanceThresholdMm{2.0};
    double straightAToBMaxDistanceMm{80.0};
    std::string straightBMethod = "nearest";
    double straightBLineFitLengthMm{35.0};
    double straightBResidualMinThresholdMm{0.8};
    int straightBResidualWindowCount{3};
    double straightBevel30OffsetMm{0.0};
    double straightBevel45OffsetMm{0.0};
    int straightBevelType{30};

};

struct FitReport 
{
    std::string name;
    double rmsMm{0.0};
    double maxAbsMm{0.0};
    int inlierCount{0};
};

struct CylinderModel 
{
    Eigen::Vector3d point{0.0, 0.0, 0.0};
    Eigen::Vector3d axis{0.0, 0.0, 1.0};
    double radiusMm{0.0};
    FitReport fit;
};

struct CircleSection 
{
    double zMm{0.0};
    Eigen::Vector2d center{0.0, 0.0};
    double radiusMm{0.0};
    FitReport fit;
};

struct OpeningResult
{
    std::string name;
    double centerToInnerWallDistanceMm{0.0};  // 开孔距离
    double axisToHeadAxisAngleDeg{0.0};       // 开孔接头角度
    FitReport fit;
};

struct MeasureResult 
{
    double innerDiameterMm{0.0};               // 接口输出：封头内径
    double innerCircumferenceMm{0.0};          // 接口输出：封头内径
    double roundnessToleranceMm{0.0};          // 接口输出：封头圆度公差
    double straightSideSlopeDeg{0.0};          // 接口输出：封头直边斜度
    double straightSideHeightMm{0.0};          // 接口输出：封头直边高度
    FitReport cylinderFit;
    FitReport topPlaneFit;
    FitReport icpFit;
    std::vector<CircleSection> sections;
    OpeningResult opening;                     // 接口输出：开孔测量（单个开孔：开孔距离、开孔接头角度）
};

}  // namespace hm
