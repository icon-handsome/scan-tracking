// ??/???????????????????? V1.1??
// IPC ???? runWithScanCloud / runPipelineWithPreprocessedScan ?????
#include "HeadMeasure/MeasurePipeline.h"

#include "HeadMeasure/Geometry.h"

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace hm {
namespace {
const double kPi = 3.141592653589793238462643383279502884;
Eigen::Vector4d g_topPlane(0.0, 0.0, 1.0, 0.0);
bool g_topPlaneValid = false;
void cylinderBasis(const Eigen::Vector3d& axis, Eigen::Vector3d& u, Eigen::Vector3d& v);

double clampValue(double v, double lo, double hi)
{
    return std::max(lo, std::min(hi, v));
}

bool hasSuffix(const std::string& text, const std::string& suffix) {
    if (text.size() < suffix.size()) {
        return false;
    }
    return std::equal(suffix.rbegin(), suffix.rend(), text.rbegin());
}

CloudPtr loadCloud(const std::string& path) 
{
    CloudPtr cloud(new Cloud);
    int rc = -1;
    if (hasSuffix(path, ".pcd") || hasSuffix(path, ".PCD"))
	{
        rc = pcl::io::loadPCDFile<PointT>(path, *cloud);
    }
	else if (hasSuffix(path, ".ply") || hasSuffix(path, ".PLY")) 
	{
        rc = pcl::io::loadPLYFile<PointT>(path, *cloud);
    }
	else
	{
        throw std::runtime_error("unsupported point cloud format: " + path);
    }

    if (rc < 0 || cloud->empty())
	{
        throw std::runtime_error("failed to load point cloud: " + path);
    }
    return cloud;
}

CloudPtr cropCloud(const CloudConstPtr& input, const CropBox& box) 
{
    pcl::CropBox<PointT> crop;
    crop.setInputCloud(input);
    crop.setMin(Eigen::Vector4f(box.min.x(), box.min.y(), box.min.z(), 1.0f));
    crop.setMax(Eigen::Vector4f(box.max.x(), box.max.y(), box.max.z(), 1.0f));
    CloudPtr out(new Cloud);
    crop.filter(*out);
    return out;
}

bool isPointInCropBox(const PointT& point, const CropBox& box)
{
    return point.x >= box.min.x() && point.x <= box.max.x() &&
           point.y >= box.min.y() && point.y <= box.max.y() &&
           point.z >= box.min.z() && point.z <= box.max.z();
}

CloudPtr cropCloudAny(const CloudConstPtr& input, const std::vector<CropBox>& boxes)
{
    if (boxes.empty())
    {
        return CloudPtr(new Cloud(*input));
    }

    CloudPtr out(new Cloud);
    out->reserve(input->size());
    for (std::size_t i = 0; i < input->size(); ++i)
    {
        const PointT& point = input->points[i];
        for (std::size_t b = 0; b < boxes.size(); ++b)
        {
            if (isPointInCropBox(point, boxes[b]))
            {
                out->push_back(point);
                break;
            }
        }
    }
    out->width = static_cast<uint32_t>(out->size());
    out->height = 1;
    out->is_dense = input->is_dense;
    return out;
}

void printCloudInfo(const std::string& name, const CloudConstPtr& cloud) {
    if (!cloud || cloud->empty()) {
        std::cout << "cloud_info name=" << name << " points=0" << std::endl;
        return;
    }
    PointT minPt;
    PointT maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    std::cout << "cloud_info name=" << name
              << " points=" << cloud->size()
              << " min=(" << minPt.x << "," << minPt.y << "," << minPt.z << ")"
              << " max=(" << maxPt.x << "," << maxPt.y << "," << maxPt.z << ")" << std::endl;
}

CloudPtr preprocess(const CloudConstPtr& input, const MeasureConfig& cfg)
{
	// ?�??
    CloudPtr transformed(new Cloud);
    pcl::transformPointCloud(*input, *transformed, cfg.poseCorrection);

	// ?????????????????????? SOR/ICP ?? OOM ????
    CloudPtr down(new Cloud);
    if (cfg.voxelLeafMm > 0.0) 
	{
        pcl::VoxelGrid<PointT> voxel;
        voxel.setInputCloud(transformed);
        const float leaf = static_cast<float>(cfg.voxelLeafMm);
        voxel.setLeafSize(leaf, leaf, leaf);
        voxel.filter(*down);
    } 
	else
	{
        down = transformed;
    }

	// ?�?�?����?�
    CloudPtr filtered(new Cloud);
	if (cfg.statisticalMeanK > 0 && down->size() > static_cast<std::size_t>(cfg.statisticalMeanK))
	{
        pcl::StatisticalOutlierRemoval<PointT> sor;
		sor.setInputCloud(down);
        sor.setMeanK(cfg.statisticalMeanK);
        sor.setStddevMulThresh(cfg.statisticalStddevMul);
        sor.filter(*filtered);
    }
	else
	{
		filtered = down;
    }

    return filtered;
}

CloudPtr mergePreprocessedScans(
    const std::vector<CloudConstPtr>& rawScans,
    const MeasureConfig& cfg)
{
    CloudPtr merged(new Cloud);
    for (std::size_t i = 0; i < rawScans.size(); ++i)
    {
        const CloudConstPtr& frame = rawScans[i];
        if (!frame || frame->empty())
        {
            continue;
        }

        PointT min_pt;
        PointT max_pt;
        pcl::getMinMax3D(*frame, min_pt, max_pt);
        std::cout << "frame_bounds index=" << i
                  << " X: [" << min_pt.x << ", " << max_pt.x << "], "
                  << "Y: [" << min_pt.y << ", " << max_pt.y << "], "
                  << "Z: [" << min_pt.z << ", " << max_pt.z << "]" << std::endl;

        CloudPtr current = preprocess(frame, cfg);
        *merged += *current;
        std::cout << "frame_preprocessed index=" << i
                  << " points_after_preprocess=" << current->size() << std::endl;
    }
    if (merged->empty())
    {
        throw std::runtime_error("merged cloud is empty");
    }
    return merged;
}

CloudPtr mergeFrames(const std::vector<std::string>& paths, const MeasureConfig& cfg) 
{
    std::vector<CloudConstPtr> rawScans;
    rawScans.reserve(paths.size());
    for (std::size_t i = 0; i < paths.size(); ++i)
    {
        rawScans.push_back(loadCloud(paths[i]));
        std::cout << "frame_loaded path=" << paths[i]
                  << " points=" << rawScans.back()->size() << std::endl;
    }
    return mergePreprocessedScans(rawScans, cfg);
}


int nearestPoint(const CloudConstPtr& cloud, const Eigen::Vector3d& query, double maxDistMm, PointT &best_pnt)
{
    double bestD2 = maxDistMm * maxDistMm;
    int found = 1;
    for (std::size_t i = 0; i < cloud->size(); ++i)
	{
        const PointT& p = (*cloud)[i];
        const Eigen::Vector3d q(p.x, p.y, p.z);
        const double d2 = (q - query).squaredNorm();
        if (d2 < bestD2) 
		{
            bestD2   = d2;
			best_pnt = p;
            found    = 0;
        }
    }
     
	return found;
}

FitReport fitPlanePca(const CloudConstPtr& cloud, Eigen::Vector4d& plane)
{
    FitReport report;
    report.name = "top_plane";
    report.inlierCount = static_cast<int>(cloud->size());
    if (cloud->size() < 3) 
	{
        report.rmsMm = std::numeric_limits<double>::quiet_NaN();
        report.maxAbsMm = std::numeric_limits<double>::quiet_NaN();
        return report;
    }

    Eigen::Vector3d mean(0.0, 0.0, 0.0);
    for (std::size_t i = 0; i < cloud->size(); ++i)
	{
        mean += Eigen::Vector3d((*cloud)[i].x, (*cloud)[i].y, (*cloud)[i].z);
    }
    mean /= static_cast<double>(cloud->size());

    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (std::size_t i = 0; i < cloud->size(); ++i)
	{
        const Eigen::Vector3d d = Eigen::Vector3d((*cloud)[i].x, (*cloud)[i].y, (*cloud)[i].z) - mean;
        cov += d * d.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    Eigen::Vector3d normal = solver.eigenvectors().col(0).normalized();
    if (normal.z() < 0.0)
	{
        normal = -normal;
    }
    plane = Eigen::Vector4d(normal.x(), normal.y(), normal.z(), -normal.dot(mean));

    double sum2 = 0.0;
    double maxAbs = 0.0;
    for (std::size_t i = 0; i < cloud->size(); ++i)
	{
        const Eigen::Vector3d p((*cloud)[i].x, (*cloud)[i].y, (*cloud)[i].z);
        const double residual = plane.head<3>().dot(p) + plane.w();
        sum2 += residual * residual;
        maxAbs = std::max(maxAbs, std::abs(residual));
    }
    report.rmsMm = std::sqrt(sum2 / static_cast<double>(cloud->size()));
    report.maxAbsMm = maxAbs;
    return report;
}

CloudPtr subsampleByStrideForIcp(const CloudConstPtr& input, int targetPoints)
{
    CloudPtr output(new Cloud);
    if (!input || input->empty() || targetPoints <= 0) {
        return output;
    }
    const int pointCount = static_cast<int>(input->size());
    if (pointCount <= targetPoints) {
        *output = *input;
        output->width = static_cast<std::uint32_t>(output->size());
        output->height = 1;
        return output;
    }
    const int stride = std::max(1, (pointCount + targetPoints - 1) / targetPoints);
    output->points.reserve(static_cast<std::size_t>(targetPoints));
    for (int index = 0; index < pointCount; index += stride) {
        output->points.push_back(input->points[static_cast<std::size_t>(index)]);
    }
    output->width = static_cast<std::uint32_t>(output->points.size());
    output->height = 1;
    return output;
}

CloudPtr downsampleForIcp(const CloudConstPtr& input, int targetPoints)
{
    if (!input || input->empty()) {
        return CloudPtr(new Cloud);
    }
    if (static_cast<int>(input->size()) <= targetPoints) {
        return CloudPtr(new Cloud(*input));
    }

    CloudPtr current = subsampleByStrideForIcp(input, targetPoints);
    if (static_cast<int>(current->size()) > targetPoints) {
        current = subsampleByStrideForIcp(current, targetPoints);
    }
    if (static_cast<int>(input->size()) > targetPoints) {
        std::cout << "icp_stride_downsample points=" << input->size() << "->" << current->size()
                  << std::endl;
    }
    return current;
}

bool isGlobalIcpUsable(const FitReport& icpFit, const MeasureConfig& cfg)
{
    if (!std::isfinite(icpFit.rmsMm)) {
        return false;
    }
    const double maxRms = cfg.icpMaxCorrespondenceDistanceMm > 0.0
        ? cfg.icpMaxCorrespondenceDistanceMm
        : 100.0;
    return icpFit.rmsMm <= maxRms;
}

// �����scanInTemplate---�����?�
FitReport alignScanToTemplate(const CloudConstPtr& scan,
                              const CloudConstPtr& templ,
                              const MeasureConfig& cfg,
                              CloudPtr& scanInTemplate)
{
    FitReport report;
    report.name = "global_template_icp";
    scanInTemplate.reset(new Cloud);
    if (templ->empty() || scan->empty())
	{
        report.rmsMm = std::numeric_limits<double>::quiet_NaN();
        report.maxAbsMm = std::numeric_limits<double>::quiet_NaN();
        return report;
    }

    constexpr int kGlobalIcpMaxPoints = 50000;
    CloudPtr scanDs = downsampleForIcp(scan, kGlobalIcpMaxPoints);
    CloudPtr templDs = downsampleForIcp(templ, kGlobalIcpMaxPoints);
    std::cout << "global_icp_downsample scan=" << scan->size() << "->" << scanDs->size()
              << " template=" << templ->size() << "->" << templDs->size() << std::endl;

    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(scanDs);
    icp.setInputTarget(templDs);
    icp.setMaxCorrespondenceDistance(cfg.icpMaxCorrespondenceDistanceMm);
    icp.setMaximumIterations(cfg.icpMaxIterations);
    icp.setTransformationEpsilon(cfg.icpTransformationEpsilon);
    icp.setEuclideanFitnessEpsilon(cfg.icpFitnessEpsilon);
    Cloud alignedDs;
    icp.align(alignedDs);

    const Eigen::Matrix4f transform = icp.getFinalTransformation();
    pcl::transformPointCloud(*scan, *scanInTemplate, transform);

    report.inlierCount = static_cast<int>(scanInTemplate->size());
    report.rmsMm       = std::sqrt(std::max(0.0, icp.getFitnessScore()));
    report.maxAbsMm    = cfg.icpMaxCorrespondenceDistanceMm;
    std::cout << "icp_status name=" << report.name
              << " converged=" << (icp.hasConverged() ? 1 : 0) << '\n';
    return report;
}

CloudPtr buildTopPlaneCloud(const CloudConstPtr& scan, const MeasureConfig& cfg, FitReport& fit)
{
    CloudPtr featureCloud(new Cloud);
    if (!cfg.templateTopPlaneFeaturePoints.empty())
    {
        for (std::size_t i = 0; i < cfg.templateTopPlaneFeaturePoints.size(); ++i)
        {
			std::cout << i << std::endl;
			PointT feature;
			int status = nearestPoint(scan, cfg.templateTopPlaneFeaturePoints[i], cfg.topFeatureSearchRadiusMm, feature);
			if (status != 0)
			{
				continue;
			}
            featureCloud->push_back(feature);
			std::cout << "top_feature_point: " << cfg.templateTopPlaneFeaturePoints[i].x() << " " 
				                               << cfg.templateTopPlaneFeaturePoints[i].y() << " " 
											   << cfg.templateTopPlaneFeaturePoints[i].z() << std::endl;
			std::cout << "corres_point: " << feature.x << " " 
				                          << feature.y << " " 
										  << feature.z << std::endl<<std::endl;
			
        }
    }

	std::cout << "fitting plane: " << std::endl;
    Eigen::Vector4d plane;
    fit = fitPlanePca(featureCloud, plane);
    g_topPlane = plane;
    g_topPlaneValid = (featureCloud->size() >= 3);
    if (g_topPlaneValid)
    {
        std::cout << "top_plane_model nx=" << plane.x()
                  << " ny=" << plane.y()
                  << " nz=" << plane.z()
                  << " d=" << plane.w() << std::endl;
    }
    else
    {
        std::cout << "top plane feature count < 3, plane is invalid" << std::endl;
    }
    return featureCloud;
}

CloudPtr cropByPlaneDistance(const CloudConstPtr& scan, const Eigen::Vector4d& plane, double minDistance, double maxDistance, double directionSign)
{
    CloudPtr out(new Cloud);
    const Eigen::Vector3d n = plane.head<3>().normalized();
    for (std::size_t i = 0; i < scan->size(); ++i)
    {
        const PointT& src = (*scan)[i];
        const Eigen::Vector3d p(src.x, src.y, src.z);
        const double signedDistance = n.dot(p) + plane.w();
        const double cropDistance = directionSign * signedDistance;
        if (cropDistance >= minDistance && cropDistance <= maxDistance)
        {
            out->push_back(src);
        }
    }
    out->width = static_cast<uint32_t>(out->size());
    out->height = 1;
    out->is_dense = scan->is_dense;
    return out;
}
CloudPtr buildStraightSideCloud(const CloudConstPtr& scan, const MeasureConfig& cfg, const FitReport&)
{
    if (!g_topPlaneValid)
    {
        std::cout << "top plane invalid, straight side cloud is empty" << std::endl;
        return CloudPtr(new Cloud);
    }

    const Eigen::Vector3d fittedNormal = g_topPlane.head<3>().normalized();
    const Eigen::Vector3d templateNormal = cfg.templateTopPlaneNormal.normalized();
    const double dot = fittedNormal.dot(templateNormal);
    const bool angleOver90 = (dot < 0.0);
    const double directionSign = angleOver90 ? -1.0 : 1.0;

    const double minDistance = cfg.straightSideOffsetBelowTopMm;
    const double maxDistance = cfg.straightSideOffsetBelowTopMm + cfg.straightSideCylinderCropHeightMm;
    CloudPtr straight = cropByPlaneDistance(scan, g_topPlane, minDistance, maxDistance, directionSign);
    std::cout << "straight_side_plane_crop min_mm=" << minDistance
              << " max_mm=" << maxDistance
              << " dot_template_normal=" << dot
              << " direction=" << (angleOver90 ? "negative_fitted_normal" : "positive_fitted_normal")
              << " points=" << straight->size() << std::endl;

    //// ����PCD������?�??�����������??��?������?�?�
    //const std::string straightPath = "C:/Users/lenovo/Desktop/straight_side_cropped.pcd";
    //if (straight && !straight->empty())
    //{
    //    const int saveRc = pcl::io::savePCDFileBinary(straightPath, *straight);
    //    if (saveRc == 0)
    //    {
    //        std::cout << "?�?�������?��?" << straightPath << " ������" << straight->size() << std::endl;
    //    }
    //    else
    //    {
    //        std::cout << "?�?�����?���?�?�" << straightPath << " �����?" << saveRc << std::endl;
    //    }
    //}
    //else
    //{
    //    std::cout << "?�?������?�?�?����" << std::endl;
    //}
    //
    return straight;
}

CloudPtr buildStraightSideFeatureCloud(const CloudConstPtr& scan, const MeasureConfig& cfg)
{
    if (!g_topPlaneValid)
    {
        std::cout << "top plane invalid, straight side feature cloud is empty" << std::endl;
        return CloudPtr(new Cloud);
    }

    const Eigen::Vector3d fittedNormal = g_topPlane.head<3>().normalized();
    const Eigen::Vector3d templateNormal = cfg.templateTopPlaneNormal.normalized();
    const double dot = fittedNormal.dot(templateNormal);
    const bool angleOver90 = (dot < 0.0);
    const double directionSign = angleOver90 ? -1.0 : 1.0;

    const double minDistance = 0.0;
    const double maxDistance = cfg.straightSideOffsetBelowTopMm + cfg.straightSideCropHeightMm;
    CloudPtr featureCloud = cropByPlaneDistance(scan, g_topPlane, minDistance, maxDistance, directionSign);
    std::cout << "straight_side_feature_crop min_mm=" << minDistance
              << " max_mm=" << maxDistance
              << " dot_template_normal=" << dot
              << " direction=" << (angleOver90 ? "negative_fitted_normal" : "positive_fitted_normal")
              << " points=" << featureCloud->size() << std::endl;

    /* ����PCD������?�??�����������??��?������?�?�
    const std::string featurePath = "C:/Users/lenovo/Desktop/straight_side_feature_cropped.pcd";
    if (featureCloud && !featureCloud->empty())
    {
        const int saveRc = pcl::io::savePCDFileBinary(featurePath, *featureCloud);
        if (saveRc == 0)
        {
            std::cout << "?�����������?��?" << featurePath << " ������" << featureCloud->size() << std::endl;
        }
        else
        {
            std::cout << "?���������?���?�?�" << featurePath << " �����?" << saveRc << std::endl;
        }
    }
    else
    {
        std::cout << "?����������?�?�?����" << std::endl;
    }
    */
    return featureCloud;
}

CylinderModel fitCylinderByPcaAxis(const CloudConstPtr& cloud, const MeasureConfig& cfg)
{
    CylinderModel model;
    model.fit.name = "straight_side_cylinder";
    model.fit.inlierCount = cloud ? static_cast<int>(cloud->size()) : 0;
    std::cout << "fitCylinderByPcaAxis enter points=" << model.fit.inlierCount << std::endl;
    if (!cloud || cloud->size() < 10)
    {
        model.fit.rmsMm = std::numeric_limits<double>::quiet_NaN();
        model.fit.maxAbsMm = std::numeric_limits<double>::quiet_NaN();
        return model;
    }

    Eigen::Vector3d origin(0.0, 0.0, 0.0);
    for (std::size_t i = 0; i < cloud->size(); ++i)
    {
        origin += Eigen::Vector3d((*cloud)[i].x, (*cloud)[i].y, (*cloud)[i].z);
    }
    origin /= static_cast<double>(cloud->size());

    Eigen::Vector3d axis = g_topPlaneValid ? g_topPlane.head<3>().normalized() : cfg.templateTopPlaneNormal.normalized();
    if (axis.norm() < 1e-9)
    {
        axis = Eigen::Vector3d(0.0, 0.0, 1.0);
    }

    std::vector<int> inliers;
    inliers.reserve(cloud->size());
    for (std::size_t i = 0; i < cloud->size(); ++i)
    {
        inliers.push_back(static_cast<int>(i));
    }

    const int maxIter = 2;         // ����?����?����?�?�?�?��?�?������?�������?�������?�?
    double radius = 0.0;
    Eigen::Vector3d axisPoint = origin;

    for (int iter = 0; iter < maxIter && inliers.size() >= 10; ++iter)
    {
        Eigen::Vector3d u;
        Eigen::Vector3d v;
        cylinderBasis(axis, u, v);

        std::vector<Eigen::Vector2d> sectionPts;
        sectionPts.reserve(inliers.size());
        for (std::size_t k = 0; k < inliers.size(); ++k)
        {
            const PointT& pt = (*cloud)[inliers[k]];
            const Eigen::Vector3d p(pt.x, pt.y, pt.z);
            const Eigen::Vector3d d = p - axisPoint;
            sectionPts.push_back(Eigen::Vector2d(d.dot(u), d.dot(v)));
        }

        CircleFit2D circle = fitCircleLeastSquares(sectionPts, "straight_side_cylinder_section");
        radius = circle.radiusMm;
        axisPoint = axisPoint + u * circle.center.x() + v * circle.center.y();

        std::vector<double> absResiduals;
        absResiduals.reserve(inliers.size());
        std::vector<double> signedResiduals;
        signedResiduals.reserve(inliers.size());
        double sum2 = 0.0;
        double maxAbs = 0.0;
        for (std::size_t k = 0; k < inliers.size(); ++k)
        {
            const PointT& pt = (*cloud)[inliers[k]];
            const Eigen::Vector3d p(pt.x, pt.y, pt.z);
            const double radial = (p - closestPointOnLine(p, axisPoint, axis)).norm();
            const double residual = radial - radius;
            signedResiduals.push_back(residual);
            absResiduals.push_back(std::abs(residual));
            sum2 += residual * residual;
            maxAbs = std::max(maxAbs, std::abs(residual));
        }

        std::vector<double> sortedAbs = absResiduals;
        std::sort(sortedAbs.begin(), sortedAbs.end());
        const double medianAbs = sortedAbs[sortedAbs.size() / 2];
        const double configuredThreshold = cfg.cylinderDistanceThresholdMm > 0.0 ? cfg.cylinderDistanceThresholdMm : 2.0;
        double robustThreshold = 3.0 * 1.4826 * medianAbs;
        if (robustThreshold <= 0.0 || !std::isfinite(robustThreshold))
        {
            robustThreshold = configuredThreshold;
        }
        robustThreshold = std::max(configuredThreshold, std::min(robustThreshold, 2.5 * configuredThreshold));

        std::vector<int> nextInliers;
        nextInliers.reserve(inliers.size());
        for (std::size_t k = 0; k < inliers.size(); ++k)
        {
            if (std::abs(signedResiduals[k]) <= robustThreshold)
            {
                nextInliers.push_back(inliers[k]);
            }
        }
        std::cout << "cylinder_fit_iter=" << iter
                  << " radius_mm=" << radius
                  << " threshold_mm=" << robustThreshold
                  << " inliers_before=" << inliers.size()
                  << " inliers_after=" << nextInliers.size() << std::endl;
        if (nextInliers.size() >= 10)
        {
            inliers.swap(nextInliers);
        }

        std::vector<Eigen::Vector3d> sliceCenters;
        double minH = std::numeric_limits<double>::max();
        double maxH = -std::numeric_limits<double>::max();
        std::vector<double> heights;
        heights.reserve(inliers.size());
        for (std::size_t k = 0; k < inliers.size(); ++k)
        {
            const PointT& pt = (*cloud)[inliers[k]];
            const Eigen::Vector3d p(pt.x, pt.y, pt.z);
            const double h = (p - axisPoint).dot(axis);
            heights.push_back(h);
            minH = std::min(minH, h);
            maxH = std::max(maxH, h);
        }

        const int sliceCount = 8;
        const double span = maxH - minH;
        if (span > 1e-6)
        {
            for (int s = 0; s < sliceCount; ++s)
            {
                const double h0 = minH + span * static_cast<double>(s) / static_cast<double>(sliceCount);
                const double h1 = minH + span * static_cast<double>(s + 1) / static_cast<double>(sliceCount);
                std::vector<Eigen::Vector2d> pts2;
                double hSum = 0.0;
                for (std::size_t k = 0; k < inliers.size(); ++k)
                {
                    if (heights[k] >= h0 && heights[k] < h1)
                    {
                        const PointT& pt = (*cloud)[inliers[k]];
                        const Eigen::Vector3d p(pt.x, pt.y, pt.z);
                        const Eigen::Vector3d d = p - axisPoint;
                        pts2.push_back(Eigen::Vector2d(d.dot(u), d.dot(v)));
                        hSum += heights[k];
                    }
                }
                if (pts2.size() >= 8)
                {
                    CircleFit2D c = fitCircleLeastSquares(pts2, "straight_side_axis_slice_center");
                    const double hMean = hSum / static_cast<double>(pts2.size());
                    sliceCenters.push_back(axisPoint + axis * hMean + u * c.center.x() + v * c.center.y());
                }
            }
        }

        if (iter + 1 < maxIter && sliceCenters.size() >= 2)
        {
            Eigen::Vector3d centerMean(0.0, 0.0, 0.0);
            for (std::size_t i = 0; i < sliceCenters.size(); ++i)
            {
                centerMean += sliceCenters[i];
            }
            centerMean /= static_cast<double>(sliceCenters.size());
            Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
            for (std::size_t i = 0; i < sliceCenters.size(); ++i)
            {
                const Eigen::Vector3d d = sliceCenters[i] - centerMean;
                cov += d * d.transpose();
            }
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
            Eigen::Vector3d refinedAxis = solver.eigenvectors().col(2).normalized();
            if (refinedAxis.dot(axis) < 0.0)
            {
                refinedAxis = -refinedAxis;
            }
            axis = refinedAxis;
            axisPoint = centerMean;
        }

        model.fit.rmsMm = std::sqrt(sum2 / static_cast<double>(signedResiduals.size()));
        model.fit.maxAbsMm = maxAbs;
        model.fit.inlierCount = static_cast<int>(inliers.size());
    }

    double sum2Final = 0.0;
    double maxAbsFinal = 0.0;
    std::vector<double> radialDistances;
    radialDistances.reserve(inliers.size());
    for (std::size_t k = 0; k < inliers.size(); ++k)
    {
        const PointT& pt = (*cloud)[inliers[k]];
        const Eigen::Vector3d p(pt.x, pt.y, pt.z);
        const double radial = (p - closestPointOnLine(p, axisPoint, axis)).norm();
        radialDistances.push_back(radial);
    }
    for (std::size_t i = 0; i < radialDistances.size(); ++i)
    {
        const double residual = radialDistances[i] - radius;
        sum2Final += residual * residual;
        maxAbsFinal = std::max(maxAbsFinal, std::abs(residual));
    }

    model.point = axisPoint;
    model.axis = axis;
    model.radiusMm = radius;
    model.fit.inlierCount = static_cast<int>(inliers.size());
    model.fit.rmsMm = radialDistances.empty() ? std::numeric_limits<double>::quiet_NaN() : std::sqrt(sum2Final / static_cast<double>(radialDistances.size()));
    model.fit.maxAbsMm = maxAbsFinal;
    std::cout << "fitCylinderByPcaAxis leave inliers=" << model.fit.inlierCount
              << " radius_mm=" << model.radiusMm
              << " rms_mm=" << model.fit.rmsMm << std::endl;
    return model;
}
CylinderModel fitOpeningCylinderIterative(const CloudConstPtr& cloud,
                                          const Eigen::Vector3d& axisInitial,
                                          const std::string& name)
{
    CylinderModel model;
    model.fit.name = name;
    model.fit.inlierCount = cloud ? static_cast<int>(cloud->size()) : 0;
    if (!cloud || cloud->size() < 6)
    {
        model.fit.rmsMm = std::numeric_limits<double>::quiet_NaN();
        model.fit.maxAbsMm = std::numeric_limits<double>::quiet_NaN();
        return model;
    }

    Eigen::Vector3d axis = axisInitial.normalized();
    if (axis.norm() < 1e-9)
    {
        axis = Eigen::Vector3d(0.0, 0.0, 1.0);
    }
    Eigen::Vector3d axisPoint(0.0, 0.0, 0.0);
    for (std::size_t i = 0; i < cloud->size(); ++i)
    {
        axisPoint += Eigen::Vector3d((*cloud)[i].x, (*cloud)[i].y, (*cloud)[i].z);
    }
    axisPoint /= static_cast<double>(cloud->size());

    double radius = 0.0;
    for (int iter = 0; iter < 30; ++iter)
    {
        Eigen::Vector3d u;
        Eigen::Vector3d v;
        cylinderBasis(axis, u, v);
        std::vector<Eigen::Vector2d> sectionPts;
        sectionPts.reserve(cloud->size());
        std::vector<double> heights;
        heights.reserve(cloud->size());
        double minH = std::numeric_limits<double>::max();
        double maxH = -std::numeric_limits<double>::max();
        for (std::size_t i = 0; i < cloud->size(); ++i)
        {
            const Eigen::Vector3d p((*cloud)[i].x, (*cloud)[i].y, (*cloud)[i].z);
            const Eigen::Vector3d d = p - axisPoint;
            sectionPts.push_back(Eigen::Vector2d(d.dot(u), d.dot(v)));
            const double h = d.dot(axis);
            heights.push_back(h);
            minH = std::min(minH, h);
            maxH = std::max(maxH, h);
        }

        CircleFit2D circle = fitCircleLeastSquares(sectionPts, name + "_section");
        radius = circle.radiusMm;
        axisPoint = axisPoint + u * circle.center.x() + v * circle.center.y();

        std::vector<Eigen::Vector3d> centers;
        const double span = maxH - minH;
        if (span > 10.0)
        {
            const int bins = 3;
            for (int b = 0; b < bins; ++b)
            {
                const double h0 = minH + span * static_cast<double>(b) / bins;
                const double h1 = minH + span * static_cast<double>(b + 1) / bins;
                std::vector<Eigen::Vector2d> binPts;
                double hSum = 0.0;
                for (std::size_t i = 0; i < cloud->size(); ++i)
                {
                    if (heights[i] >= h0 && heights[i] <= h1)
                    {
                        binPts.push_back(sectionPts[i]);
                        hSum += heights[i];
                    }
                }
                if (binPts.size() >= 6)
                {
                    CircleFit2D c = fitCircleLeastSquares(binPts, name + "_slice");
                    const double hm = hSum / static_cast<double>(binPts.size());
                    centers.push_back(axisPoint + axis * hm + u * c.center.x() + v * c.center.y());
                }
            }
        }
        if (centers.size() >= 2)
        {
            Eigen::Vector3d cmean(0.0, 0.0, 0.0);
            for (std::size_t i = 0; i < centers.size(); ++i)
            {
                cmean += centers[i];
            }
            cmean /= static_cast<double>(centers.size());
            Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
            for (std::size_t i = 0; i < centers.size(); ++i)
            {
                const Eigen::Vector3d d = centers[i] - cmean;
                cov += d * d.transpose();
            }
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
            Eigen::Vector3d refined = solver.eigenvectors().col(2).normalized();
            if (refined.dot(axis) < 0.0)
            {
                refined = -refined;
            }
            axis = refined;
            axisPoint = cmean;
        }
    }

    double sum2 = 0.0;
    double maxAbs = 0.0;
    for (std::size_t i = 0; i < cloud->size(); ++i)
    {
        const Eigen::Vector3d p((*cloud)[i].x, (*cloud)[i].y, (*cloud)[i].z);
        const double radial = (p - closestPointOnLine(p, axisPoint, axis)).norm();
        const double residual = radial - radius;
        sum2 += residual * residual;
        maxAbs = std::max(maxAbs, std::abs(residual));
    }

    model.point = axisPoint;
    model.axis = axis;
    model.radiusMm = radius;
    model.fit.inlierCount = static_cast<int>(cloud->size());
    model.fit.rmsMm = std::sqrt(sum2 / static_cast<double>(cloud->size()));
    model.fit.maxAbsMm = maxAbs;
    std::cout << "fit name=" << model.fit.name
		      << " radius=" <<  model.radiusMm
              << " opening_axis_initial=(" << axisInitial.x() << "," << axisInitial.y() << "," << axisInitial.z() << ")"
              << " rms_mm=" << model.fit.rmsMm
              << " max_abs_mm=" << model.fit.maxAbsMm
              << " inliers=" << model.fit.inlierCount << std::endl;
    return model;
}

// ��?
std::vector<CircleSection> sliceByCylinderAxis(const CloudConstPtr& cloud,
                                               const CylinderModel& cylinder,
                                               const MeasureConfig& cfg)
{
    std::vector<CircleSection> sections;
    if (!cloud || cloud->empty()) {
        return sections;
    }

    Eigen::Vector3d u;
    Eigen::Vector3d v;
    cylinderBasis(cylinder.axis, u, v);

    std::vector<double> h;
    h.reserve(cloud->size());
    double minH = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < cloud->size(); ++i) 
	{
        const Eigen::Vector3d p((*cloud)[i].x, (*cloud)[i].y, (*cloud)[i].z);
        const double value = (p - cylinder.point).dot(cylinder.axis);
        h.push_back(value);
        minH = std::min(minH, value);
    }
	minH += 10.0;                         // �?�����?������?�?��10.0mm
    for (int i = 0; i < cfg.sliceCount; ++i)
	{
        const double centerH = minH + static_cast<double>(i) * cfg.sliceSpacingMm;
        const Eigen::Vector3d planeCenter = cylinder.point + cylinder.axis * centerH;

        std::vector<Eigen::Vector2d> pts;
        pts.reserve(cloud->size() / 20);
        CloudPtr sliceCloud(new Cloud);
        for (std::size_t j = 0; j < cloud->size(); ++j) 
		{
            if (std::abs(h[j] - centerH) <= cfg.sliceThicknessMm * 0.5) 
			{
                const Eigen::Vector3d p((*cloud)[j].x, (*cloud)[j].y, (*cloud)[j].z);
                const Eigen::Vector3d d = p - planeCenter;
                pts.push_back(Eigen::Vector2d(d.dot(u), d.dot(v)));
                sliceCloud->push_back((*cloud)[j]);
            }
        }
        if (pts.size() < static_cast<std::size_t>(cfg.sliceMinPoints))
		{
            continue;
        }
        sliceCloud->width = static_cast<uint32_t>(sliceCloud->size());
        sliceCloud->height = 1;
        sliceCloud->is_dense = cloud->is_dense;
    /* ����PCD������?�??�����������??��?������?�?�
        std::ostringstream slicePath;
        slicePath << "C:/Users/lenovo/Desktop/axis_slice_" << i << ".pcd";
        const int sliceSaveRc = pcl::io::savePCDFileBinary(slicePath.str(), *sliceCloud);
        if (sliceSaveRc == 0) 
		{
            std::cout << "��?�����?��?" << slicePath.str() << " ������" << sliceCloud->size() << std::endl;
        } 
		else 
		{
            std::cout << "��?���?���?�?�" << slicePath.str() << " �����?" << sliceSaveRc << std::endl;
        }

    */
        Eigen::Vector2d center(0.0, 0.0);
        double radius = cylinder.radiusMm;
        std::vector<int> inliers;
        inliers.reserve(pts.size());
        for (std::size_t k = 0; k < pts.size(); ++k) 
		{
            inliers.push_back(static_cast<int>(k));
        }

        double threshold = cfg.circleMaxFitErrorMm > 0.0 ? cfg.circleMaxFitErrorMm : 1.5;
        for (int iter = 0; iter < 20 && inliers.size() >= 8; ++iter)
		{
            std::vector<double> radii;
            radii.reserve(inliers.size());
            for (std::size_t k = 0; k < inliers.size(); ++k) 
			{
                radii.push_back((pts[inliers[k]] - center).norm());
            }
            std::vector<double> sortedR = radii;
            std::sort(sortedR.begin(), sortedR.end());
            radius = sortedR[sortedR.size() / 2];

            std::vector<double> absResiduals;
            absResiduals.reserve(inliers.size());
            for (std::size_t k = 0; k < inliers.size(); ++k) 
			{
                absResiduals.push_back(std::abs(radii[k] - radius));
            }
            std::sort(absResiduals.begin(), absResiduals.end());
            const double mad = absResiduals[absResiduals.size() / 2];
            threshold = std::max(cfg.circleMaxFitErrorMm, 3.0 * 1.4826 * mad);
            if (threshold <= 0.0 || !std::isfinite(threshold)) 
			{
                threshold = cfg.circleMaxFitErrorMm > 0.0 ? cfg.circleMaxFitErrorMm : 1.5;
            }

            std::vector<int> nextInliers;
            nextInliers.reserve(inliers.size());
            for (std::size_t k = 0; k < inliers.size(); ++k) 
			{
                if (std::abs(radii[k] - radius) <= threshold)
				{
                    nextInliers.push_back(inliers[k]);
                }
            }
            if (nextInliers.size() >= 8) 
			{
                inliers.swap(nextInliers);
            }

            Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
            Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
            for (std::size_t k = 0; k < inliers.size(); ++k) 
			{
                const Eigen::Vector2d q = pts[inliers[k]] - center;
                const double dist = q.norm();
                if (dist < 1e-9) 
				{
                    continue;
                }
                const double residual = dist - radius;
                Eigen::Vector3d jac(-q.x() / dist, -q.y() / dist, -1.0);
                normal += jac * jac.transpose();
                rhs += -jac * residual;
            }

            Eigen::Vector3d delta = normal.ldlt().solve(rhs);
            if (!std::isfinite(delta.x()) || !std::isfinite(delta.y()) || !std::isfinite(delta.z()))
			{
                break;
            }
            center.x() += delta.x();
            center.y() += delta.y();
            radius += delta.z();
            if (radius < 0.0) 
			{
                radius = -radius;
            }
            if (delta.norm() < 1e-5) 
			{
                break;
            }
        }

        double sum2 = 0.0;
        double maxAbs = 0.0;
        int finalInliers = 0;
        for (std::size_t k = 0; k < pts.size(); ++k) 
		{
            const double residual = (pts[k] - center).norm() - radius;
            if (std::abs(residual) <= threshold) 
			{
                sum2 += residual * residual;
                maxAbs = std::max(maxAbs, std::abs(residual));
                ++finalInliers;
            }
        }
        if (finalInliers < 8) 
		{
            continue;
        }

        std::ostringstream name;
        name << "axis_slice_" << i;
        CircleSection s;
        s.zMm = centerH;
        s.center = center;
        s.radiusMm = radius;
        s.fit.name = name.str();
        s.fit.inlierCount = finalInliers;
        s.fit.rmsMm = std::sqrt(sum2 / static_cast<double>(finalInliers));
        s.fit.maxAbsMm = maxAbs;
        sections.push_back(s);

        std::cout << "slice_circle name=" << s.fit.name
                  << " center_init_axis_intersection=(0,0)"
                  << " center_fit=(" << center.x() << "," << center.y() << ")"
                  << " diameter_mm=" << 2.0 * s.radiusMm
                  << " raw_points=" << pts.size()
                  << " inliers=" << finalInliers
                  << " threshold_mm=" << threshold
                  << " rms_mm=" << s.fit.rmsMm << std::endl;
    }
    return sections;
}
void cylinderBasis(const Eigen::Vector3d& axis, Eigen::Vector3d& u, Eigen::Vector3d& v) 
{
    u = axis.cross(Eigen::Vector3d::UnitX());
    if (u.norm() < 1e-6)
	{
        u = axis.cross(Eigen::Vector3d::UnitY());
    }
    u.normalize();
    v = axis.cross(u).normalized();
}

FitReport localIcp(const CloudConstPtr& templ, const CloudConstPtr& scan, const MeasureConfig& cfg, Eigen::Matrix4f& transform) {
    FitReport report;
    report.name = "opening_local_icp";
    transform = Eigen::Matrix4f::Identity();
    if (templ->empty() || scan->empty()) {
        report.rmsMm = std::numeric_limits<double>::quiet_NaN();
        report.maxAbsMm = std::numeric_limits<double>::quiet_NaN();
        return report;
    }
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(templ);
    icp.setInputTarget(scan);
    icp.setMaxCorrespondenceDistance(cfg.icpMaxCorrespondenceDistanceMm);
    icp.setMaximumIterations(cfg.icpMaxIterations);
    icp.setTransformationEpsilon(cfg.icpTransformationEpsilon);
    icp.setEuclideanFitnessEpsilon(cfg.icpFitnessEpsilon);
    Cloud aligned;
    icp.align(aligned);
    transform = icp.getFinalTransformation();
    report.inlierCount = static_cast<int>(aligned.size());
    report.rmsMm = std::sqrt(std::max(0.0, icp.getFitnessScore()));
    report.maxAbsMm = cfg.icpMaxCorrespondenceDistanceMm;
    std::cout << "icp_status name=" << report.name
              << " converged=" << (icp.hasConverged() ? 1 : 0) << '\n';
    return report;
}

void projectionBasis(const Eigen::Vector3d& direction, Eigen::Vector3d& u, Eigen::Vector3d& v)
{
    Eigen::Vector3d n = direction.normalized();
    u = n.cross(Eigen::Vector3d::UnitX());
    if (u.norm() < 1e-6)
    {
        u = n.cross(Eigen::Vector3d::UnitY());
    }
    u.normalize();
    v = n.cross(u).normalized();
}

bool detectOpeningByProjectionImage(const CloudConstPtr& cloud,
                                    const OpeningFeature& feature,
                                    Eigen::Vector3d& center3d,
                                    Eigen::Vector2d& center2d,
                                    double& diameterMm)
{
    CloudPtr local = cropCloud(cloud, feature.projectionCrop);
    if (!local || local->size() < 20)
    {
        return false;
    }

	// ??����������
    Eigen::Vector3d u;
    Eigen::Vector3d v;
    projectionBasis(feature.projectionDirection, u, v);
    Eigen::Vector3d origin(0.0, 0.0, 0.0);
    for (std::size_t i = 0; i < local->size(); ++i)
    {
        origin += Eigen::Vector3d((*local)[i].x, (*local)[i].y, (*local)[i].z);
    }
    origin /= static_cast<double>(local->size());

    std::vector<Eigen::Vector2d> pts;
    pts.reserve(local->size());
    double minX = std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double maxX = -std::numeric_limits<double>::max();
    double maxY = -std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < local->size(); ++i)
    {
        const Eigen::Vector3d p((*local)[i].x, (*local)[i].y, (*local)[i].z);
        const Eigen::Vector3d d = p - origin;
        const Eigen::Vector2d q(d.dot(u), d.dot(v));
        pts.push_back(q);
        minX = std::min(minX, q.x());
        minY = std::min(minY, q.y());
        maxX = std::max(maxX, q.x());
        maxY = std::max(maxY, q.y());
    }

	// ??��?��
    const int width = std::max(50, feature.projectionImageWidth);
    const int height = std::max(50, feature.projectionImageHeight);
    const double sx = (maxX - minX) / static_cast<double>(std::max(1, width - 1));
    const double sy = (maxY - minY) / static_cast<double>(std::max(1, height - 1));
    const double pixelMm = std::max(1e-6, 0.5 * (sx + sy));
    std::vector<unsigned char> occ(width * height, 0);       // ?�?�?�?����?��?�������?
    for (std::size_t i = 0; i < pts.size(); ++i)
    {
        const int x = static_cast<int>((pts[i].x() - minX) / sx + 0.5);
        const int y = static_cast<int>((pts[i].y() - minY) / sy + 0.5);
        for (int dy = -1; dy <= 1; ++dy)
        {
            for (int dx = -1; dx <= 1; ++dx)
            {
                const int xx = x + dx;
                const int yy = y + dy;
                if (xx >= 0 && xx < width && yy >= 0 && yy < height)
                {
                    occ[yy * width + xx] = 1;
                }
            }
        }
    }

	// ��?�����?��?��?�??�(??�������)��?��
    std::vector<unsigned char> visited(width * height, 0);
    double bestScore = std::numeric_limits<double>::max();
    bool found = false;
    Eigen::Vector2d bestCenter(0.0, 0.0);
    double bestDiameter = 0.0;
    int bestBoundaryCount = 0;
    const int dirs[4][2] = {{1,0},{-1,0},{0,1},{0,-1}};  // 4�������������������?�
    for (int y0 = 1; y0 < height - 1; ++y0)
    {
        for (int x0 = 1; x0 < width - 1; ++x0)
        {
            const int seed = y0 * width + x0;           // ���?�?��?���???�����?���?�?���?�?������?���?��������������??����?����?����?��
            if (occ[seed] || visited[seed])
            {
                continue;
            }
            std::vector<int> queue;
            queue.push_back(seed);
            visited[seed] = 1;
            bool touchesBorder = false;
            int area = 0;
            int boundaryCount = 0;
            double sumX = 0.0;
            double sumY = 0.0;
            double boundarySumX = 0.0;
            double boundarySumY = 0.0;
            for (std::size_t qi = 0; qi < queue.size(); ++qi)
            {
                const int idx = queue[qi];
                const int x = idx % width;
                const int y = idx / width;
                ++area;
                sumX += x;
                sumY += y;
                if (x == 0 || y == 0 || x == width - 1 || y == height - 1)
                {
                    touchesBorder = true;
                }
                bool boundaryPixel = false;
                for (int di = 0; di < 4; ++di)
                {
                    const int nx = x + dirs[di][0];
                    const int ny = y + dirs[di][1];
                    if (nx < 0 || nx >= width || ny < 0 || ny >= height)
                    {
                        continue;
                    }
                    const int ni = ny * width + nx;
                    if (occ[ni])
                    {
                        boundaryPixel = true;
                    }
                    else if (!visited[ni])
                    {
                        visited[ni] = 1;
                        queue.push_back(ni);
                    }
                }
                if (boundaryPixel)
                {
                    ++boundaryCount;
                    boundarySumX += x;
                    boundarySumY += y;
                }
            }
            if (touchesBorder || area < 10 || boundaryCount < 4)  // �����?�����??��������������?�?���
            {
                continue;
            }

			// 1. ����?�����? Area = pi * r^2 ����?�� d = 2 * sqrt(Area / pi)
			// 2. ���� pixelMm���������?������??��?�����?��������?�??��?�������?�?
            const double d = 2.0 * std::sqrt(static_cast<double>(area) / kPi) * pixelMm;
            if (std::abs(d - feature.expectedDiameterMm) > feature.diameterToleranceMm)
            {
                continue;
            }
            const double score = std::abs(d - feature.expectedDiameterMm);
            if (score < bestScore)
            {
                bestScore = score;
                bestDiameter = d;
                bestCenter = Eigen::Vector2d(minX + (boundarySumX / boundaryCount) * sx, minY + (boundarySumY / boundaryCount) * sy);
                bestBoundaryCount = boundaryCount;
                found = true;
            }
        }
    }

    if (!found)
    {
        std::cout << "opening_projection name=" << feature.name << " status=not_found points=" << local->size() << std::endl;
        return false;
    }

    center2d = bestCenter;
    diameterMm = bestDiameter;

    // 2D�����?���?�?���???���?������������?����??�
    // ������???����3D����??�?�?�?�2D������projection_direction?����?���?�
    std::vector<Eigen::Vector3d> boundary3d;
    const double boundaryRadius = bestDiameter * 0.5;
    const double boundaryBand = std::max(2.0, pixelMm * 3.0);
    for (std::size_t i = 0; i < local->size(); ++i)
    {
        const double dist2d = (pts[i] - bestCenter).norm();
        if (std::abs(dist2d - boundaryRadius) <= boundaryBand)
        {
            boundary3d.push_back(Eigen::Vector3d((*local)[i].x, (*local)[i].y, (*local)[i].z));
        }
    }

    Eigen::Vector3d planePoint = origin;
    Eigen::Vector3d planeNormal = feature.projectionDirection.normalized();
    if (boundary3d.size() >= 3)
    {
        planePoint = Eigen::Vector3d(0.0, 0.0, 0.0);
        for (std::size_t i = 0; i < boundary3d.size(); ++i)
        {
            planePoint += boundary3d[i];
        }
        planePoint /= static_cast<double>(boundary3d.size());
        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        for (std::size_t i = 0; i < boundary3d.size(); ++i)
        {
            const Eigen::Vector3d d = boundary3d[i] - planePoint;
            cov += d * d.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
        planeNormal = solver.eigenvectors().col(0).normalized();
        if (planeNormal.dot(feature.projectionDirection.normalized()) < 0.0)
        {
            planeNormal = -planeNormal;
        }
    }

    const Eigen::Vector3d projectedCenterOnProjectionPlane = origin + u * bestCenter.x() + v * bestCenter.y();
    const Eigen::Vector3d rayDir = feature.projectionDirection.normalized();
    const double denom = planeNormal.dot(rayDir);
    if (std::abs(denom) > 1e-9)
    {
        const double t = planeNormal.dot(planePoint - projectedCenterOnProjectionPlane) / denom;
        center3d = projectedCenterOnProjectionPlane + rayDir * t;
    }
    else
    {
        center3d = projectedCenterOnProjectionPlane;
    }
    std::cout << "opening_projection name=" << feature.name
              << " status=ok diameter_mm=" << diameterMm
              << " center2d=(" << center2d.x() << "," << center2d.y() << ")"
              << " center3d=(" << center3d.x() << "," << center3d.y() << "," << center3d.z() << ")"
              << " boundary_pixels=" << bestBoundaryCount
              << " boundary3d_points=" << boundary3d.size() << std::endl;
    return true;
}

Eigen::Matrix4f rotationAroundAxis(const Eigen::Vector3d& point, const Eigen::Vector3d& axis, double angle)
{
    Eigen::Matrix3d r = Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
    Eigen::Vector3d t = point - r * point;
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    m.block<3,3>(0,0) = r.cast<float>();
    m.block<3,1>(0,3) = t.cast<float>();
    return m;
}



// ���?��
OpeningResult solveOpeningsByProjectionAndLocalIcp(const MeasureConfig& cfg,
                                                   const CloudConstPtr& templ,
                                                   const CloudConstPtr& scan,
                                                   const CylinderModel& headCylinder)
{
    OpeningResult result;
    if (templ->empty() || cfg.templateOpenings.empty())
    {
        return result;
    }

    for (std::size_t i = 0; i < cfg.templateOpenings.size(); ++i)
    {
        const OpeningFeature& feature = cfg.templateOpenings[i];
        Eigen::Vector3d templateCenter3d;
        Eigen::Vector3d scanCenter3d;
        Eigen::Vector2d templateCenter2d;
        Eigen::Vector2d scanCenter2d;
        double templateDiameter = 0.0;
        double scanDiameter = 0.0;
  //      const bool templateFound = detectOpeningByProjectionImage(templ, feature, templateCenter3d, templateCenter2d, templateDiameter);
  //      const bool scanFound = detectOpeningByProjectionImage(scan, feature, scanCenter3d, scanCenter2d, scanDiameter);
  //      if (!templateFound || !scanFound)
  //      {
  //          std::cout << "opening name=" << feature.name << " status=skip reason=projection_center_not_found" << std::endl;
  //          continue;
  //      }

  //      // ��?������?�?������??�??����?�������??���?���?�
  //      // templateRadial / scanRadial ��?��������������?�?���??�����
  //      // ������������������???�����?������?����?�?������?��?�???�
  //      Eigen::Vector3d templateRadial = templateCenter3d - closestPointOnLine(templateCenter3d, headCylinder.point, headCylinder.axis);
  //      Eigen::Vector3d scanRadial = scanCenter3d - closestPointOnLine(scanCenter3d, headCylinder.point, headCylinder.axis);
  //      if (templateRadial.norm() < 1e-6 || scanRadial.norm() < 1e-6)
  //      {
  //          continue;
  //      }
  //      templateRadial.normalize();
  //      scanRadial.normalize();
  //      const double crossSign = headCylinder.axis.normalized().dot(scanRadial.cross(templateRadial));
  //      const double dot = clampValue(scanRadial.dot(templateRadial), -1.0, 1.0);
  //      const double delta = (crossSign >= 0.0 ? 1.0 : -1.0) * std::acos(dot);
  //      Eigen::Matrix4f roughRotation = rotationAroundAxis(headCylinder.point, headCylinder.axis, delta);
  //      CloudPtr rotatedScan(new Cloud);
  //      pcl::transformPointCloud(*scan, *rotatedScan, roughRotation); 
  //      Eigen::Vector4f scanCenterHp(static_cast<float>(scanCenter3d.x()),
  //                                   static_cast<float>(scanCenter3d.y()),
  //                                   static_cast<float>(scanCenter3d.z()),
  //                                   1.0f);
  //      Eigen::Vector4f rotatedScanCenterHp = roughRotation * scanCenterHp;
  //      Eigen::Vector3d rotatedScanCenter(rotatedScanCenterHp.x(), rotatedScanCenterHp.y(), rotatedScanCenterHp.z());
		//// ����??��?���

		CloudPtr localTemplate = cropCloud(templ, feature.projectionCrop); 
        //CloudPtr localScan = cropCloud(rotatedScan, feature.projectionCrop);
		CloudPtr localScan = cropCloud(scan, feature.projectionCrop);
        Eigen::Matrix4f localTf;
        FitReport fit = localIcp(localTemplate, localScan, cfg, localTf);
		std::cout << "opening_localIcp= " << fit.name
                          << " rmse=" << fit.rmsMm
                          << " maxrmse=" << fit.maxAbsMm
						  << " inner count =" << fit.inlierCount << std::endl;
		// ����icp��?���


		// �?�?���������?������?�������������
        CloudPtr featureCloud(new Cloud);
        for (std::size_t k = 0; k < feature.cylinderFeaturePoints.size(); ++k)
        {
            try
            {
                Eigen::Vector4f hp(static_cast<float>(feature.cylinderFeaturePoints[k].x()),
                                   static_cast<float>(feature.cylinderFeaturePoints[k].y()),
                                   static_cast<float>(feature.cylinderFeaturePoints[k].z()),
                                   1.0f);
                Eigen::Vector4f mapped = localTf * hp;
                Eigen::Vector3d mappedFeature(mapped.x(), mapped.y(), mapped.z());
				PointT np;
				int status = nearestPoint(localScan, mappedFeature, feature.searchRadiusMm, np);
				if (status != 0)
				{
					continue;
				}
                featureCloud->push_back(np);
                std::cout << "opening_feature_correspondence name=" << feature.name
                          << " index=" << k
                          << " mapped=(" << mappedFeature.x() << "," << mappedFeature.y() << "," << mappedFeature.z() << ")"
                          << " nearest=(" << np.x << "," << np.y << "," << np.z << ")" << std::endl;
            }
            catch (const std::exception&)
            {
            }
        }
        if (featureCloud->size() < 5)
        {
            std::cout << "opening name=" << feature.name << " status=skip reason=feature_points_not_enough count=" << featureCloud->size() << std::endl;
            continue;
        }

        const Eigen::Vector3d openingAxisHint = feature.projectionDirection.normalized();
        CylinderModel openingCylinder = fitOpeningCylinderIterative(featureCloud, openingAxisHint, "opening_cylinder_" + feature.name);
       
		
		
		CloudPtr openingCylinderSample(new Cloud);
        Eigen::Vector3d cylU;
        Eigen::Vector3d cylV;
        cylinderBasis(openingCylinder.axis, cylU, cylV);
        const double sampleHeightMm = std::max(20.0, feature.expectedDiameterMm);
        for (int si = 0; si < 200; ++si)
        {
            const double angle = 2.0 * kPi * static_cast<double>(si % 40) / 40.0;
            const double h = sampleHeightMm * (static_cast<double>(si / 40) / 4.0 - 0.5);
            const Eigen::Vector3d p = openingCylinder.point
                                    + openingCylinder.axis.normalized() * h
                                    + cylU * (openingCylinder.radiusMm * std::cos(angle))
                                    + cylV * (openingCylinder.radiusMm * std::sin(angle));
            PointT pt;
            pt.x = static_cast<float>(p.x());
            pt.y = static_cast<float>(p.y());
            pt.z = static_cast<float>(p.z());
            openingCylinderSample->push_back(pt);
        }
        openingCylinderSample->width = static_cast<uint32_t>(openingCylinderSample->size());
        openingCylinderSample->height = 1;
        openingCylinderSample->is_dense = true;
    /* ����PCD������?�??�����������??��?������?�?�
        std::ostringstream openingSamplePath;
        openingSamplePath << "C:/Users/lenovo/Desktop/opening_cylinder_sample_" << feature.name << ".pcd";
        const int openingSampleSaveRc = pcl::io::savePCDFileBinary(openingSamplePath.str(), *openingCylinderSample);
        std::cout << "opening_cylinder_sample name=" << feature.name
                  << " saved=" << (openingSampleSaveRc == 0 ? 1 : 0)
                  << " points=" << openingCylinderSample->size()
                  << " path=" << openingSamplePath.str() << std::endl;

    */
		/*Eigen::Vector4f adjustedCenterHp(static_cast<float>(scanCenter3d.x()),
			                             static_cast<float>(scanCenter3d.y()),
			                             static_cast<float>(scanCenter3d.z()),
                                         1.0f);
        Eigen::Vector4f rotatedAdjustedCenterHp = roughRotation * adjustedCenterHp;
        Eigen::Vector3d adjustedCenter(rotatedAdjustedCenterHp.x(), rotatedAdjustedCenterHp.y(), rotatedAdjustedCenterHp.z());
      */
		// C2?��???������?������?�����projection_direction������?��8mm��
        // ��??����?����������?������������?��ICP���?�?�������?��
       // const Eigen::Vector3d openingCenterOnSurface = closestPointOnLine(adjustedCenter, openingCylinder.point, openingCylinder.axis);        
		
		
		std::cout << "opening_cylinder name=" << feature.name
                  << " axis=(" << openingCylinder.axis.x() << "," << openingCylinder.axis.y() << "," << openingCylinder.axis.z() << ")"
                  << " axis_point=(" << openingCylinder.point.x() << "," << openingCylinder.point.y() << "," << openingCylinder.point.z() << ")"
                 // << " surface_center=(" << openingCenterOnSurface.x() << "," << openingCenterOnSurface.y() << "," << openingCenterOnSurface.z() << ")"
                  //<< " adjusted_scan_center=(" << adjustedCenter.x() << "," << adjustedCenter.y() << "," << adjustedCenter.z() << ")"
                  << " diameter_mm=" << 2.0 * openingCylinder.radiusMm << std::endl;

        result = OpeningResult();
        result.name = feature.name;
		// ���?�
        result.axisToHeadAxisAngleDeg = angleDeg(openingCylinder.axis, headCylinder.axis);
        if (result.axisToHeadAxisAngleDeg > 90.0)
        {
            result.axisToHeadAxisAngleDeg = 180.0 - result.axisToHeadAxisAngleDeg;
        }
		// ?�����??��?�������??���
		const double c2Radius = (openingCylinder.point - closestPointOnLine(openingCylinder.point, headCylinder.point, headCylinder.axis)).norm();
        // ���?���
		result.centerToInnerWallDistanceMm = std::abs(headCylinder.radiusMm - c2Radius);
        result.fit = openingCylinder.fit;

		std::cout << "���?���������" 
			      << " ���?��? mm  " << result.centerToInnerWallDistanceMm
				  << " ���???���  " << result.axisToHeadAxisAngleDeg << std::endl;
        return result;
    }

    return result;
}


// ���?��
OpeningResult solveOpeningsByProjectionAndLocalIcp_1(const MeasureConfig& cfg,
                                                                const CloudConstPtr& templ,
                                                                const CloudConstPtr& scan,
                                                                const CylinderModel& headCylinder)
{
    OpeningResult result;
    if (templ->empty() || cfg.templateOpenings.empty())
    {
        return result;
    }

    for (std::size_t i = 0; i < cfg.templateOpenings.size(); ++i)
    {
        const OpeningFeature& feature = cfg.templateOpenings[i];
        Eigen::Vector3d templateCenter3d;
        Eigen::Vector3d scanCenter3d;
        Eigen::Vector2d templateCenter2d;
        Eigen::Vector2d scanCenter2d;
        double templateDiameter = 0.0;
        double scanDiameter = 0.0;
        const bool templateFound = detectOpeningByProjectionImage(templ, feature, templateCenter3d, templateCenter2d, templateDiameter);
        const bool scanFound = detectOpeningByProjectionImage(scan, feature, scanCenter3d, scanCenter2d, scanDiameter);
        if (!templateFound || !scanFound)
        {
            std::cout << "opening name=" << feature.name << " status=skip reason=projection_center_not_found" << std::endl;
            continue;
        }

        // ��?������?�?������??�??����?�������??���?���?�
        // templateRadial / scanRadial ��?��������������?�?���??�����
        // ������������������???�����?������?����?�?������?��?�???�
        Eigen::Vector3d templateRadial = templateCenter3d - closestPointOnLine(templateCenter3d, headCylinder.point, headCylinder.axis);
        Eigen::Vector3d scanRadial = scanCenter3d - closestPointOnLine(scanCenter3d, headCylinder.point, headCylinder.axis);
        if (templateRadial.norm() < 1e-6 || scanRadial.norm() < 1e-6)
        {
            continue;
        }
        templateRadial.normalize();
        scanRadial.normalize();
        const double crossSign = headCylinder.axis.normalized().dot(scanRadial.cross(templateRadial));
        const double dot = clampValue(scanRadial.dot(templateRadial), -1.0, 1.0);
        const double delta = (crossSign >= 0.0 ? 1.0 : -1.0) * std::acos(dot);
        Eigen::Matrix4f roughRotation = rotationAroundAxis(headCylinder.point, headCylinder.axis, delta);
        CloudPtr rotatedScan(new Cloud);
        pcl::transformPointCloud(*scan, *rotatedScan, roughRotation); 
        Eigen::Vector4f scanCenterHp(static_cast<float>(scanCenter3d.x()),
                                     static_cast<float>(scanCenter3d.y()),
                                     static_cast<float>(scanCenter3d.z()),
                                     1.0f);
        Eigen::Vector4f rotatedScanCenterHp = roughRotation * scanCenterHp;
        Eigen::Vector3d rotatedScanCenter(rotatedScanCenterHp.x(), rotatedScanCenterHp.y(), rotatedScanCenterHp.z());
		// ����??��?���

        CloudPtr localTemplate = cropCloud(templ, feature.projectionCrop);
        CloudPtr localScan = cropCloud(rotatedScan, feature.projectionCrop);
        Eigen::Matrix4f localTf;
        FitReport fit = localIcp(localTemplate, localScan, cfg, localTf);
		// ����icp��?���


		// �?�?���������?������?�������������
        CloudPtr featureCloud(new Cloud);
        for (std::size_t k = 0; k < feature.cylinderFeaturePoints.size(); ++k)
        {
            try
            {
                Eigen::Vector4f hp(static_cast<float>(feature.cylinderFeaturePoints[k].x()),
                                   static_cast<float>(feature.cylinderFeaturePoints[k].y()),
                                   static_cast<float>(feature.cylinderFeaturePoints[k].z()),
                                   1.0f);
                Eigen::Vector4f mapped = localTf * hp;
                Eigen::Vector3d mappedFeature(mapped.x(), mapped.y(), mapped.z());
				PointT np;
				int status = nearestPoint(localScan, mappedFeature, feature.searchRadiusMm, np);
				if (status != 0)
				{
					continue;
				}
                featureCloud->push_back(np);
                std::cout << "opening_feature_correspondence name=" << feature.name
                          << " index=" << k
                          << " mapped=(" << mappedFeature.x() << "," << mappedFeature.y() << "," << mappedFeature.z() << ")"
                          << " nearest=(" << np.x << "," << np.y << "," << np.z << ")" << std::endl;
            }
            catch (const std::exception&)
            {
            }
        }
        if (featureCloud->size() < 6)
        {
            std::cout << "opening name=" << feature.name << " status=skip reason=feature_points_not_enough count=" << featureCloud->size() << std::endl;
            continue;
        }

        const Eigen::Vector3d openingAxisHint = feature.projectionDirection.normalized();
        CylinderModel openingCylinder = fitOpeningCylinderIterative(featureCloud, openingAxisHint, "opening_cylinder_" + feature.name);
       
		
		
		CloudPtr openingCylinderSample(new Cloud);
        Eigen::Vector3d cylU;
        Eigen::Vector3d cylV;
        cylinderBasis(openingCylinder.axis, cylU, cylV);
        const double sampleHeightMm = std::max(20.0, feature.expectedDiameterMm);
        for (int si = 0; si < 200; ++si)
        {
            const double angle = 2.0 * kPi * static_cast<double>(si % 40) / 40.0;
            const double h = sampleHeightMm * (static_cast<double>(si / 40) / 4.0 - 0.5);
            const Eigen::Vector3d p = openingCylinder.point
                                    + openingCylinder.axis.normalized() * h
                                    + cylU * (openingCylinder.radiusMm * std::cos(angle))
                                    + cylV * (openingCylinder.radiusMm * std::sin(angle));
            PointT pt;
            pt.x = static_cast<float>(p.x());
            pt.y = static_cast<float>(p.y());
            pt.z = static_cast<float>(p.z());
            openingCylinderSample->push_back(pt);
        }
        openingCylinderSample->width = static_cast<uint32_t>(openingCylinderSample->size());
        openingCylinderSample->height = 1;
        openingCylinderSample->is_dense = true;
    /* ����PCD������?�??�����������??��?������?�?�
        std::ostringstream openingSamplePath;
        openingSamplePath << "C:/Users/lenovo/Desktop/opening_cylinder_sample_" << feature.name << ".pcd";
        const int openingSampleSaveRc = pcl::io::savePCDFileBinary(openingSamplePath.str(), *openingCylinderSample);
        std::cout << "opening_cylinder_sample name=" << feature.name
                  << " saved=" << (openingSampleSaveRc == 0 ? 1 : 0)
                  << " points=" << openingCylinderSample->size()
                  << " path=" << openingSamplePath.str() << std::endl;

    */
		Eigen::Vector4f adjustedCenterHp(static_cast<float>(scanCenter3d.x()),
			                             static_cast<float>(scanCenter3d.y()),
			                             static_cast<float>(scanCenter3d.z()),
                                         1.0f);
        Eigen::Vector4f rotatedAdjustedCenterHp = roughRotation * adjustedCenterHp;
        Eigen::Vector3d adjustedCenter(rotatedAdjustedCenterHp.x(), rotatedAdjustedCenterHp.y(), rotatedAdjustedCenterHp.z());
      
		// C2?��???������?������?�����projection_direction������?��8mm��
        // ��??����?����������?������������?��ICP���?�?�������?��
        const Eigen::Vector3d openingCenterOnSurface = closestPointOnLine(adjustedCenter, openingCylinder.point, openingCylinder.axis);        
		
		
		std::cout << "opening_cylinder name=" << feature.name
                  << " axis=(" << openingCylinder.axis.x() << "," << openingCylinder.axis.y() << "," << openingCylinder.axis.z() << ")"
                  << " axis_point=(" << openingCylinder.point.x() << "," << openingCylinder.point.y() << "," << openingCylinder.point.z() << ")"
                  << " surface_center=(" << openingCenterOnSurface.x() << "," << openingCenterOnSurface.y() << "," << openingCenterOnSurface.z() << ")"
                  << " adjusted_scan_center=(" << adjustedCenter.x() << "," << adjustedCenter.y() << "," << adjustedCenter.z() << ")"
                  << " diameter_mm=" << 2.0 * openingCylinder.radiusMm << std::endl;

        result = OpeningResult();
        result.name = feature.name;
		// ���?�
        result.axisToHeadAxisAngleDeg = angleDeg(openingCylinder.axis, headCylinder.axis);
        if (result.axisToHeadAxisAngleDeg > 90.0)
        {
            result.axisToHeadAxisAngleDeg = 180.0 - result.axisToHeadAxisAngleDeg;
        }
		// ?�����??��?�������??���
        const double c2Radius = (openingCenterOnSurface - closestPointOnLine(openingCenterOnSurface, headCylinder.point, headCylinder.axis)).norm();
        // ���?���
		result.centerToInnerWallDistanceMm = std::abs(headCylinder.radiusMm - c2Radius);
        result.fit = openingCylinder.fit;

		std::cout << "���?���������" 
			      << " ���?��? mm  " << result.centerToInnerWallDistanceMm
				  << " ���???���  " << result.axisToHeadAxisAngleDeg << std::endl;
        return result;
    }

    return result;
}

struct StraightSlicePoint
{
    Eigen::Vector3d p;
    double h;
    double r;
    double t;
    double topDistance;
};

struct Line2DModel
{
    bool valid;
    Eigen::Vector2d point;
    Eigen::Vector2d dir;
    int inlierCount;
    Line2DModel() : valid(false), point(0.0, 0.0), dir(1.0, 0.0), inlierCount(0) {}
};

struct QuadraticModel
{
    bool valid;
    double a;
    double b;
    double c;
    int inlierCount;
    QuadraticModel() : valid(false), a(0.0), b(0.0), c(0.0), inlierCount(0) {}
};

Eigen::Vector3d pointToVec(const PointT& p)
{
    return Eigen::Vector3d(p.x, p.y, p.z);
}

double medianValue(std::vector<double> values)
{
    if (values.empty())
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    std::sort(values.begin(), values.end());
    return values[values.size() / 2];
}

double straightTopDirectionSign(const MeasureConfig& cfg)
{
    if (!g_topPlaneValid)
    {
        return 1.0;
    }
    const Eigen::Vector3d fittedNormal = g_topPlane.head<3>().normalized();
    const Eigen::Vector3d templateNormal = cfg.templateTopPlaneNormal.normalized();
    return fittedNormal.dot(templateNormal) < 0.0 ? -1.0 : 1.0;
}

double straightBevelOffsetMm(const MeasureConfig& cfg)
{
    if (cfg.straightBevelType == 45)
    {
        return cfg.straightBevel45OffsetMm;
    }
    return cfg.straightBevel30OffsetMm;
}

Line2DModel fitLinePca2D(const std::vector<Eigen::Vector2d>& pts)
{
    Line2DModel line;
    if (pts.size() < 5)
    {
        return line;
    }
    Eigen::Vector2d mean(0.0, 0.0);
    for (std::size_t i = 0; i < pts.size(); ++i)
    {
        mean += pts[i];
    }
    mean /= static_cast<double>(pts.size());

    Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
    for (std::size_t i = 0; i < pts.size(); ++i)
    {
        const Eigen::Vector2d d = pts[i] - mean;
        cov += d * d.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov);
    line.valid = true;
    line.point = mean;
    line.dir = solver.eigenvectors().col(1).normalized();
    line.inlierCount = static_cast<int>(pts.size());
    return line;
}

QuadraticModel fitQuadraticLeastSquares(const std::vector<Eigen::Vector2d>& pts)
{
    QuadraticModel q;
    if (pts.size() < 6)
    {
        return q;
    }
    Eigen::Matrix3d ata = Eigen::Matrix3d::Zero();
    Eigen::Vector3d aty(0.0, 0.0, 0.0);
    for (std::size_t i = 0; i < pts.size(); ++i)
    {
        const double x = pts[i].x();
        const double y = pts[i].y();
        const Eigen::Vector3d row(x * x, x, 1.0);
        ata += row * row.transpose();
        aty += row * y;
    }
    if (std::abs(ata.determinant()) < 1e-9)
    {
        return q;
    }
    const Eigen::Vector3d coef = ata.ldlt().solve(aty);
    q.valid = true;
    q.a = coef.x();
    q.b = coef.y();
    q.c = coef.z();
    q.inlierCount = static_cast<int>(pts.size());
    return q;
}

std::vector<StraightSlicePoint> buildStraightBPlaneSlice(const CloudConstPtr& cloud,
                                                          const CylinderModel& cylinder,
                                                          const Eigen::Vector3d& roughB,
                                                          const MeasureConfig& cfg,
                                                          Eigen::Vector3d& radialDir,
                                                          CloudPtr& savedCloud)
{
    std::vector<StraightSlicePoint> pts;
    savedCloud.reset(new Cloud);
    if (!cloud || cloud->empty() || !g_topPlaneValid)
    {
        return pts;
    }

    radialDir = roughB - closestPointOnLine(roughB, cylinder.point, cylinder.axis);
    if (radialDir.norm() < 1e-6)
    {
        Eigen::Vector3d u;
        Eigen::Vector3d v;
        cylinderBasis(cylinder.axis, u, v);
        radialDir = u;
    }
    radialDir.normalize();
    Eigen::Vector3d tangentDir = cylinder.axis.cross(radialDir);
    if (tangentDir.norm() < 1e-6)
    {
        return pts;
    }
    tangentDir.normalize();

    const Eigen::Vector3d topNormal = g_topPlane.head<3>().normalized();
    const double directionSign = straightTopDirectionSign(cfg);
    const double halfThickness = std::max(0.5, cfg.straightEndpointSliceThicknessMm * 0.5);
    for (std::size_t i = 0; i < cloud->size(); ++i)
    {
        const PointT& src = (*cloud)[i];
        const Eigen::Vector3d p(src.x, src.y, src.z);
        const Eigen::Vector3d d = p - cylinder.point;
        const double t = d.dot(tangentDir);
        if (std::abs(t) <= halfThickness)
        {
            StraightSlicePoint sp;
            sp.p = p;
            sp.h = d.dot(cylinder.axis);
            sp.r = d.dot(radialDir);
            sp.t = t;
            sp.topDistance = directionSign * (topNormal.dot(p) + g_topPlane.w());
            pts.push_back(sp);
            savedCloud->push_back(src);
        }
    }
    savedCloud->width = static_cast<uint32_t>(savedCloud->size());
    savedCloud->height = 1;
    savedCloud->is_dense = cloud->is_dense;
    return pts;
}

std::vector<StraightSlicePoint> cropStraightSliceBelowOffsetPlane(const std::vector<StraightSlicePoint>& slice,
                                                                  double bevelOffsetMm)
{
    std::vector<StraightSlicePoint> cropped;
    cropped.reserve(slice.size());
    for (std::size_t i = 0; i < slice.size(); ++i)
    {
        if (slice[i].topDistance >= bevelOffsetMm)
        {
            cropped.push_back(slice[i]);
        }
    }
    return cropped;
}

bool findStraightAFromCroppedSlice(const std::vector<StraightSlicePoint>& croppedSlice,
                                   const CylinderModel& cylinder,
                                   const MeasureConfig& cfg,
                                   const Eigen::Vector3d& roughB,
                                   StraightSlicePoint& outA)
{
    if (croppedSlice.empty())
    {
        return false;
    }

    const double radialThreshold = std::max(cfg.straightLineDistanceThresholdMm, cfg.cylinderDistanceThresholdMm);
    const double maxABDistance = cfg.straightAToBMaxDistanceMm;
    double bestTopDistance = std::numeric_limits<double>::max();
    double bestRadialError = std::numeric_limits<double>::max();
    double bestBDistance = std::numeric_limits<double>::max();
    int radialCandidates = 0;
    int distanceCandidates = 0;
    bool found = false;
    for (std::size_t i = 0; i < croppedSlice.size(); ++i)
    {
        const double radialError = std::abs(std::abs(croppedSlice[i].r) - cylinder.radiusMm);
        if (radialError > radialThreshold)
        {
            continue;
        }
        ++radialCandidates;

        const double bDistance = (croppedSlice[i].p - roughB).norm();
        if (maxABDistance > 0.0 && bDistance > maxABDistance)
        {
            continue;
        }
        ++distanceCandidates;

        if (croppedSlice[i].topDistance < bestTopDistance ||
            (std::abs(croppedSlice[i].topDistance - bestTopDistance) < 1e-6 && bDistance < bestBDistance) ||
            (std::abs(croppedSlice[i].topDistance - bestTopDistance) < 1e-6 && std::abs(bDistance - bestBDistance) < 1e-6 && radialError < bestRadialError))
        {
            bestTopDistance = croppedSlice[i].topDistance;
            bestRadialError = radialError;
            bestBDistance = bDistance;
            outA = croppedSlice[i];
            found = true;
        }
    }

    std::cout << "straight_A_candidate_filter total=" << croppedSlice.size()
              << " radial_threshold_mm=" << radialThreshold
              << " radial_candidates=" << radialCandidates
              << " a_to_b_max_distance_mm=" << maxABDistance
              << " distance_candidates=" << distanceCandidates
              << " found=" << (found ? 1 : 0)
              << " selected_radial_error_mm=" << (found ? bestRadialError : std::numeric_limits<double>::quiet_NaN())
              << " selected_a_to_b_distance_mm=" << (found ? bestBDistance : std::numeric_limits<double>::quiet_NaN()) << std::endl;
    return found;
}

bool solveStraightBByResidualWindow(const std::vector<StraightSlicePoint>& slice,
                                    const StraightSlicePoint& pointA,
                                    const Eigen::Vector3d& roughB,
                                    const MeasureConfig& cfg,
                                    Eigen::Vector3d& pointB,
                                    double& residualMm,
                                    int& lineCount,
                                    int& windowCount)
{
    lineCount = 0;
    windowCount = 0;
    residualMm = std::numeric_limits<double>::quiet_NaN();
    if (slice.size() < 12)
    {
        return false;
    }

    const double maxBDistance = cfg.straightAToBMaxDistanceMm;
    std::vector<StraightSlicePoint> sameSide;
    sameSide.reserve(slice.size());
    for (std::size_t i = 0; i < slice.size(); ++i)
    {
        const double bDistance = (slice[i].p - roughB).norm();
        if (maxBDistance > 0.0 && bDistance > maxBDistance)
        {
            continue;
        }
        sameSide.push_back(slice[i]);
    }
    if (sameSide.size() < 12)
    {
        std::cout << "straight_B_residual_filter same_side_points=" << sameSide.size()
                  << " a_to_b_max_distance_mm=" << maxBDistance << std::endl;
        return false;
    }

    const Eigen::Vector2d a2(pointA.h, pointA.r);
    Eigen::Vector2d roughB2(0.0, 0.0);
    double roughBest = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < sameSide.size(); ++i)
    {
        const double d2 = (sameSide[i].p - roughB).squaredNorm();
        if (d2 < roughBest)
        {
            roughBest = d2;
            roughB2 = Eigen::Vector2d(sameSide[i].h, sameSide[i].r);
        }
    }

    // ?������B�����??��������������B�??�����?��?��?����
    Eigen::Vector2d searchDir = roughB2 - a2;
    if (searchDir.norm() < 5.0)
    {
        return false;
    }
    searchDir.normalize();

    struct OrderedStraightPoint
    {
        StraightSlicePoint sp;
        double s;
    };
    std::vector<OrderedStraightPoint> ordered;
    ordered.reserve(sameSide.size());
    for (std::size_t i = 0; i < sameSide.size(); ++i)
    {
        const Eigen::Vector2d q(sameSide[i].h, sameSide[i].r);
        const double s = (q - a2).dot(searchDir);
        if (s >= 0.0)
        {
            OrderedStraightPoint op;
            op.sp = sameSide[i];
            op.s = s;
            ordered.push_back(op);
        }
    }
    std::sort(ordered.begin(), ordered.end(), [](const OrderedStraightPoint& a, const OrderedStraightPoint& b) { return a.s < b.s; });
    if (ordered.size() < 12)
    {
        return false;
    }

    // A���?����?�?��?�??�?�?�?�������?�??�?�����?������??�?���
    std::vector<Eigen::Vector2d> linePts;
    const double lineFitLength = std::max(10.0, cfg.straightBLineFitLengthMm);
    for (std::size_t i = 0; i < ordered.size(); ++i)
    {
        if (ordered[i].s <= lineFitLength)
        {
            linePts.push_back(Eigen::Vector2d(ordered[i].sp.h, ordered[i].sp.r));
        }
    }
    lineCount = static_cast<int>(linePts.size());
    if (linePts.size() < 8)
    {
        std::cout << "straight_B_residual_filter same_side_points=" << sameSide.size()
                  << " line_points=" << linePts.size()
                  << " line_fit_length_mm=" << lineFitLength << std::endl;
        return false;
    }

    Line2DModel line = fitLinePca2D(linePts);
    if (!line.valid)
    {
        return false;
    }
    if ((roughB2 - a2).dot(line.dir) < 0.0)
    {
        line.dir = -line.dir;
    }
    const Eigen::Vector2d normal(-line.dir.y(), line.dir.x());

    std::vector<double> baseAbsResiduals;
    baseAbsResiduals.reserve(linePts.size());
    for (std::size_t i = 0; i < linePts.size(); ++i)
    {
        const double residual = (linePts[i] - line.point).dot(normal);
        baseAbsResiduals.push_back(std::abs(residual));
    }
    std::sort(baseAbsResiduals.begin(), baseAbsResiduals.end());
    const double baseMad = baseAbsResiduals.empty() ? 0.0 : baseAbsResiduals[baseAbsResiduals.size() / 2]; // ?����?��???�??����?������?��
    const double residualThreshold = std::max(cfg.straightBResidualMinThresholdMm, 1.5 * 1.4826 * baseMad); // 1.4826*MAD����?��?��sigma��1.5?��?�����?�?��
    const int stableCount = std::max(1, cfg.straightBResidualWindowCount);

    std::vector<double> residuals;
    residuals.reserve(ordered.size());
    for (std::size_t i = 0; i < ordered.size(); ++i)
    {
        const Eigen::Vector2d q(ordered[i].sp.h, ordered[i].sp.r);
        residuals.push_back((q - line.point).dot(normal));
    }

    // ��?����??�?������������stableCount����??����?�?���?���
    // ��?���??�?�����?���?�B??�???�����?��?�??
    int foundStart = -1;
    const double roughBS = (roughB2 - a2).dot(searchDir);
    const double searchStartS = lineFitLength;
    for (std::size_t i = 0; i + static_cast<std::size_t>(stableCount) <= ordered.size(); ++i)
    {
        if (ordered[i].s < searchStartS)
        {
            continue;
        }
        bool stableDeviation = true;
        double signSum = 0.0;
        for (int k = 0; k < stableCount; ++k)
        {
            const double r = residuals[i + k];
            if (std::abs(r) <= residualThreshold)
            {
                stableDeviation = false;
                break;
            }
            signSum += r >= 0.0 ? 1.0 : -1.0;
        }
        if (stableDeviation && std::abs(signSum) >= static_cast<double>(stableCount))
        {
            foundStart = static_cast<int>(i);
            break;
        }
    }

    if (foundStart <= 0)
    {
        std::cout << "straight_B_residual_filter same_side_points=" << sameSide.size()
                  << " line_points=" << lineCount
                  << " base_mad_mm=" << baseMad
                  << " residual_threshold_mm=" << residualThreshold
                  << " stable_count=" << stableCount
                  << " roughB_s_mm=" << roughBS
                  << " search_start_s_mm=" << searchStartS
                  << " status=no_stable_deviation" << std::endl;
        return false;
    }

    const int bIndex = foundStart - 1;
    pointB = ordered[bIndex].sp.p;
    residualMm = std::abs(residuals[bIndex]);
    windowCount = stableCount;
    std::cout << "straight_B_residual same_side_points=" << sameSide.size()
              << " line_points=" << lineCount
              << " line_fit_length_mm=" << lineFitLength
              << " base_mad_mm=" << baseMad
              << " residual_threshold_mm=" << residualThreshold
              << " stable_count=" << stableCount
              << " roughB_s_mm=" << roughBS
              << " search_start_s_mm=" << searchStartS
              << " deviation_start_s_mm=" << ordered[foundStart].s
              << " selected_B_s_mm=" << ordered[bIndex].s
              << " selected_B_residual_mm=" << residuals[bIndex] << std::endl;
    return true;
}

bool solveStraightBByCnnPlaceholder(const std::vector<StraightSlicePoint>&,
                                    const StraightSlicePoint&,
                                    const Eigen::Vector3d&,
                                    const MeasureConfig&,
                                    Eigen::Vector3d&)
{
    std::cout << "straight_B_cnn status=not_implemented" << std::endl;
    return false;
}
bool estimateStraightSideFromBTemplates(const CloudConstPtr& straightSideFeature,
                                        const CylinderModel& cylinder,
                                        const MeasureConfig& cfg,
                                        double& deviationMm,
                                        double& heightMm)
{
    deviationMm = std::numeric_limits<double>::quiet_NaN();
    heightMm = std::numeric_limits<double>::quiet_NaN();
    if (!straightSideFeature || straightSideFeature->empty() || cfg.straightEndpointPairs.empty() || !g_topPlaneValid)
    {
        return false;
    }

    const double bevelOffset = straightBevelOffsetMm(cfg);
	std::vector<double> straight_edge_tapers;           // ?��?��
	std::vector<double> straight_edge_lengths;          // ?�??�
    for (std::size_t i = 0; i < cfg.straightEndpointPairs.size(); ++i)
    {
        const StraightEndpointPair& pair = cfg.straightEndpointPairs[i];
        try
        {
			PointT roughBPt;
			int status = nearestPoint(straightSideFeature, pair.pointB, cfg.straightEndpointSearchRadiusMm, roughBPt);
			if (status != 0)
			{
				continue;
			}
            const Eigen::Vector3d roughB = pointToVec(roughBPt);

            Eigen::Vector3d radialDir;
            CloudPtr sliceCloud;
            std::vector<StraightSlicePoint> slice = buildStraightBPlaneSlice(straightSideFeature, cylinder, roughB, cfg, radialDir, sliceCloud);
    /* ����PCD������?�??�����������??��?������?�?�
            std::ostringstream path;
            path << "C:/Users/lenovo/Desktop/straight_B_slice_" << i << ".pcd";
            const int saveRc = pcl::io::savePCDFileBinary(path.str(), *sliceCloud);
            std::cout << "straight_B_slice index=" << i
                      << " saved=" << (saveRc == 0 ? 1 : 0)
                      << " points=" << sliceCloud->size()
                      << " path=" << path.str() << std::endl;

    */
            std::vector<StraightSlicePoint> croppedSlice = cropStraightSliceBelowOffsetPlane(slice, bevelOffset);
    /* ����PCD������?�??�����������??��?������?�?�
            CloudPtr croppedSliceCloud(new Cloud);
            for (std::size_t ci = 0; ci < croppedSlice.size(); ++ci)
            {
                PointT p;
                p.x = static_cast<float>(croppedSlice[ci].p.x());
                p.y = static_cast<float>(croppedSlice[ci].p.y());
                p.z = static_cast<float>(croppedSlice[ci].p.z());
                croppedSliceCloud->push_back(p);
            }
            croppedSliceCloud->width = static_cast<uint32_t>(croppedSliceCloud->size());
            croppedSliceCloud->height = 1;
            croppedSliceCloud->is_dense = sliceCloud->is_dense;
            std::ostringstream cropPath;
            cropPath << "C:/Users/lenovo/Desktop/straight_B_slice_cropped_" << i << ".pcd";
            const int cropSaveRc = pcl::io::savePCDFileBinary(cropPath.str(), *croppedSliceCloud);
            std::cout << "straight_B_slice_cropped index=" << i
                      << " saved=" << (cropSaveRc == 0 ? 1 : 0)
                      << " points=" << croppedSliceCloud->size()
                      << " bevel_offset_mm=" << bevelOffset
                      << " path=" << cropPath.str() << std::endl;

    */
            StraightSlicePoint pointA;
            if (!findStraightAFromCroppedSlice(croppedSlice, cylinder, cfg, roughB, pointA))
            {
                std::cout << "straight_pair name=" << pair.name << " status=skip reason=A_offset_plane_not_found" << std::endl;
                continue;
            }
            std::cout << "straight_A_selected name=" << pair.name
                      << " x=" << pointA.p.x()
                      << " y=" << pointA.p.y()
                      << " z=" << pointA.p.z()
                      << " top_distance_mm=" << pointA.topDistance
                      << " bevel_offset_mm=" << bevelOffset << std::endl;

            Eigen::Vector3d pointB = roughB;
            double tangentResidual = std::numeric_limits<double>::quiet_NaN();
            int lineCount = 0;
            int windowCount = 0;
            if (cfg.straightBMethod == "nearest")
            {
                std::cout << "straight_B_selected name=" << pair.name
                          << " method=nearest_template_B"
                          << " x=" << pointB.x()
                          << " y=" << pointB.y()
                          << " z=" << pointB.z() << std::endl;
            }
            else if (cfg.straightBMethod == "residual_window")
            {
                if (!solveStraightBByResidualWindow(croppedSlice, pointA, roughB, cfg, pointB, tangentResidual, lineCount, windowCount))
                {
                    std::cout << "straight_pair name=" << pair.name << " status=skip reason=B_residual_window_not_found" << std::endl;
                    continue;
                }
            }
            else if (cfg.straightBMethod == "cnn")
            {
                if (!solveStraightBByCnnPlaceholder(croppedSlice, pointA, roughB, cfg, pointB))
                {
                    std::cout << "straight_pair name=" << pair.name << " status=skip reason=B_cnn_not_implemented" << std::endl;
                    continue;
                }
            }
            else
            {
                std::cout << "straight_pair name=" << pair.name << " status=skip reason=unknown_straight_b_method method=" << cfg.straightBMethod << std::endl;
                continue;
            }
            Eigen::Vector3d ab = pointB - pointA.p;
            const double length = ab.norm();
            if (length <= 1e-6)
            {
                continue;
            }
            ab /= length;
            const Eigen::Vector3d axis = cylinder.axis.normalized();
            const double axisCos = clampValue(std::abs(ab.dot(axis)), -1.0, 1.0);
            const double theta = std::acos(axisCos);
            const double taperMagnitude = length * std::sin(theta);
            const double radiusA = (pointA.p - closestPointOnLine(pointA.p, cylinder.point, axis)).norm();
            const double radiusB = (pointB - closestPointOnLine(pointB, cylinder.point, axis)).norm();
            // ?��?�?����?���A��B���???����?����?����?�����?��?��?����?����?����
            const double radialChange = radiusB - radiusA;
            const double deviation = radialChange >= 0.0 ? taperMagnitude : -taperMagnitude;
            straight_edge_lengths.push_back(length);
            straight_edge_tapers.push_back(deviation);
            std::cout << "straight_pair name=" << pair.name
                      << " status=ok"
                      << " bevel_type=" << cfg.straightBevelType
                      << " bevel_offset_mm=" << bevelOffset
                      << " roughB=(" << roughB.x() << "," << roughB.y() << "," << roughB.z() << ")"
                      << " A_offset=(" << pointA.p.x() << "," << pointA.p.y() << "," << pointA.p.z() << ")"
                      << " B_tangent=(" << pointB.x() << "," << pointB.y() << "," << pointB.z() << ")"
                      << " B_method=" << cfg.straightBMethod
                      << " tangent_residual_mm=" << tangentResidual
                      << " radiusA_mm=" << radiusA
                      << " radiusB_mm=" << radiusB
                      << " radial_change_mm=" << radialChange
                      << " length_mm=" << length
                      << " deviation_mm=" << deviation << std::endl;
        }
        catch (const std::exception& e)
        {
            std::cout << "straight_pair name=" << pair.name << " status=skip reason=" << e.what() << std::endl;
        }
    }

	if (straight_edge_lengths.empty() || straight_edge_tapers.empty())
    {
        return false;
    }
	heightMm = medianValue(straight_edge_lengths);     // ?�??���?�?�
	deviationMm = medianValue(straight_edge_tapers);   // ?��?����?�?�
	std::cout << " straight_result valid_pairs=" << straight_edge_lengths.size()
              << " straight_edge_lengths(median) mm=" << heightMm
              << " straight_edge_tapers(median) mm=" << deviationMm << std::endl;
    return true;
}
void printFit(const FitReport& fit) 
{
    std::cout << "fit name=" << fit.name
              << " rms_mm=" << fit.rmsMm
              << " max_abs_mm=" << fit.maxAbsMm
              << " inliers=" << fit.inlierCount << '\n';
}

}  // namespace

MeasurePipeline::MeasurePipeline(MeasureConfig config) : config_(config) {}

MeasureResult MeasurePipeline::runWithScanCloud(const CloudConstPtr& rawScan)
{
    if (!rawScan || rawScan->empty())
    {
        throw std::runtime_error("input scan cloud is empty");
    }

    CloudPtr scan = preprocess(rawScan, config_);
    std::cout << "preprocessed_points=" << scan->size() << '\n';
    return runPipelineWithPreprocessedScan(scan);
}

MeasureResult MeasurePipeline::runWithScanClouds(const std::vector<CloudConstPtr>& rawScans)
{
    if (rawScans.empty())
    {
        throw std::runtime_error("input scan clouds are empty");
    }

    CloudPtr scan = mergePreprocessedScans(rawScans, config_);
    std::cout << "merged_preprocessed_points=" << scan->size() << '\n';
    return runPipelineWithPreprocessedScan(scan);
}

MeasureResult MeasurePipeline::runWithPreprocessedScanCloud(const CloudConstPtr& preprocessedScan)
{
    if (!preprocessedScan || preprocessedScan->empty())
    {
        throw std::runtime_error("input scan cloud is empty");
    }
    std::cout << "preprocessed_points=" << preprocessedScan->size() << '\n';
    return runPipelineWithPreprocessedScan(preprocessedScan);
}

MeasureResult MeasurePipeline::run()
{
    if (config_.inputFrames.empty())
    {
        throw std::runtime_error("config input_frames is empty");
    }

    CloudPtr scan = mergeFrames(config_.inputFrames, config_);
    std::cout << "merged_points=" << scan->size() << '\n';
    return runPipelineWithPreprocessedScan(scan);
}

MeasureResult MeasurePipeline::runPipelineWithPreprocessedScan(const CloudConstPtr& scan)
{
    MeasureResult result;
    CloudPtr templ(new Cloud);
    CloudPtr scanInTemplate(new Cloud);
    if (scan && !scan->empty())
    {
        *scanInTemplate = *scan;
    }

    CloudConstPtr alignmentInput = scan;
    CloudPtr croppedScan;
    if (scan && !config_.cropBoxes.empty())
    {
        croppedScan = cropCloudAny(scan, config_.cropBoxes);
        if (croppedScan && !croppedScan->empty())
        {
            alignmentInput = croppedScan;
            std::cout << "online_scan_cropped points=" << croppedScan->size() << std::endl;
        }
        else
        {
            std::cout << "online_scan_crop_empty keep_full_cloud points=" << scan->size() << std::endl;
        }
    }

	if (1)
	{
		clock_t start_t, end_t;
		start_t = clock();
		if (!config_.templateCloud.empty())
		{
			templ = loadCloud(config_.templateCloud);
			result.icpFit = alignScanToTemplate(alignmentInput, templ, config_, scanInTemplate);
			printFit(result.icpFit);
		}
		end_t = clock();
		std::cout << "global_icp_elapsed_s=" << 0.001 * (end_t - start_t) << std::endl;
	}

    if (!isGlobalIcpUsable(result.icpFit, config_))
    {
        std::cout << "global_icp_failed status=abort rms_mm=" << result.icpFit.rmsMm
                  << " max_mm=" << config_.icpMaxCorrespondenceDistanceMm << std::endl;
        return result;
    }
	 
	// ��?�?�?�����
    FitReport topFit;
    buildTopPlaneCloud(scanInTemplate, config_, topFit);
    result.topPlaneFit = topFit;
    printFit(result.topPlaneFit);

    if (!g_topPlaneValid)
    {
        std::cout << "top_plane_invalid status=abort" << std::endl;
        return result;
    }
	
	// ?��??���
    CloudPtr straightSide = buildStraightSideCloud(scanInTemplate, config_, topFit);
    if (!straightSide)
    {
        std::cout << "straight_side_null status=abort" << std::endl;
        return result;
    }
    std::cout << "straight_side_points=" << straightSide->size() << std::endl;

    if (straightSide->size() < 10)
    {
        std::cout << "straight_side_empty status=abort" << std::endl;
        return result;
    }
	
	// ?�����
    CylinderModel cylinder = fitCylinderByPcaAxis(straightSide, config_);
    result.cylinderFit = cylinder.fit;
    printFit(result.cylinderFit);
    std::cout << "cylinder_model axis=(" << cylinder.axis.x() << "," << cylinder.axis.y() << "," << cylinder.axis.z() << ")"
              << " center_point=(" << cylinder.point.x() << "," << cylinder.point.y() << "," << cylinder.point.z() << ")"
              << " diameter_mm=" << 2.0 * cylinder.radiusMm << std::endl;
    result.innerDiameterMm = 2.0 * cylinder.radiusMm;                    // ��?�?����?�
    result.innerCircumferenceMm = kPi * result.innerDiameterMm;

	// �����������
	if (1)
	{
		CloudPtr straightCylinderSample(new Cloud);
		Eigen::Vector3d straightU;
		Eigen::Vector3d straightV;
		cylinderBasis(cylinder.axis, straightU, straightV);
		const double straightSampleHeightMm = std::max(50.0, config_.straightSideCropHeightMm);
		for (int si = 0; si < 200; ++si)
		{
			const double angle = 2.0 * kPi * static_cast<double>(si % 40) / 40.0;
			const double h = straightSampleHeightMm * (static_cast<double>(si / 40) / 4.0 - 0.5);
			const Eigen::Vector3d p = cylinder.point
				+ cylinder.axis.normalized() * h
				+ straightU * (cylinder.radiusMm * std::cos(angle))
				+ straightV * (cylinder.radiusMm * std::sin(angle));
			PointT pt;
			pt.x = static_cast<float>(p.x());
			pt.y = static_cast<float>(p.y());
			pt.z = static_cast<float>(p.z());
			straightCylinderSample->push_back(pt);
		}
		straightCylinderSample->width = static_cast<uint32_t>(straightCylinderSample->size());
		straightCylinderSample->height = 1;
		straightCylinderSample->is_dense = true;
    /* ����PCD������?�??�����������??��?������?�?�
		const std::string straightCylinderSamplePath = "C:/Users/lenovo/Desktop/straight_cylinder_sample.pcd";
		const int straightCylinderSaveRc = pcl::io::savePCDFileBinary(straightCylinderSamplePath, *straightCylinderSample);
		std::cout << "straight_cylinder_sample saved=" << (straightCylinderSaveRc == 0 ? 1 : 0)
			<< " points=" << straightCylinderSample->size()
			<< " path=" << straightCylinderSamplePath << std::endl;
    */
	} 

    // ��?������?��
    result.sections = sliceByCylinderAxis(straightSide, cylinder, config_);
    std::vector<double> roundnessValues;
    roundnessValues.reserve(result.sections.size());
    Eigen::Vector3d u;
    Eigen::Vector3d v;
    cylinderBasis(cylinder.axis, u, v);

    for (std::size_t sectionIndex = 0; sectionIndex < result.sections.size(); ++sectionIndex)
    {
        const CircleSection& section = result.sections[sectionIndex];
        printFit(section.fit);

        std::vector<Eigen::Vector2d> pts;
        pts.reserve(straightSide->size() / 20);
        for (std::size_t i = 0; i < straightSide->size(); ++i)
        {
            const Eigen::Vector3d p((*straightSide)[i].x, (*straightSide)[i].y, (*straightSide)[i].z);
            const double h = (p - cylinder.point).dot(cylinder.axis);
            if (std::abs(h - section.zMm) <= config_.sliceThicknessMm * 0.5)
            {
                const Eigen::Vector3d d = p - (cylinder.point + cylinder.axis * section.zMm);
                pts.push_back(Eigen::Vector2d(d.dot(u), d.dot(v)));
            }
        }

        if (pts.size() < 4)
        {
            std::cout << "section_roundness index=" << sectionIndex
                      << " status=skip reason=too_few_points points=" << pts.size() << std::endl;
            continue;
        }

        // �����?�������?������?�?��?���??�����򝝝?����?�?�
        std::vector<Eigen::Vector2d> roundnessPts;
        roundnessPts.reserve(pts.size());
        std::vector<double> roundAbsResiduals;
        roundAbsResiduals.reserve(pts.size());
        for (std::size_t ri = 0; ri < pts.size(); ++ri)
        {
            const double residual = (pts[ri] - section.center).norm() - section.radiusMm;
            roundAbsResiduals.push_back(std::abs(residual));
        }
        double roundnessThreshold = config_.circleMaxFitErrorMm > 0.0 ? config_.circleMaxFitErrorMm : 1.5;
        if (!roundAbsResiduals.empty())
        {
            std::sort(roundAbsResiduals.begin(), roundAbsResiduals.end());
            // MAD��?������?�?�������?���?���?�??���?��?����?�������?
            const double mad = roundAbsResiduals[roundAbsResiduals.size() / 2];
            roundnessThreshold = std::max(roundnessThreshold, 3.0 * 1.4826 * mad); // 1.4826��MAD���?�������?�?��???�
        }
        for (std::size_t ri = 0; ri < pts.size(); ++ri)
        {
            const double residual = (pts[ri] - section.center).norm() - section.radiusMm;
            if (std::abs(residual) <= roundnessThreshold)
            {
                roundnessPts.push_back(pts[ri]);
            }
        }

        const double sectionRoundness = evaluateRoundnessMinimumZone(roundnessPts, section.center);
        if (sectionRoundness == sectionRoundness)
        {
            roundnessValues.push_back(sectionRoundness);
        }
        std::cout << "section_roundness index=" << sectionIndex
                  << " z=" << section.zMm
                  << " radius_mm=" << section.radiusMm
                  << " diameter_mm=" << 2.0 * section.radiusMm
                  << " raw=" << pts.size()
                  << " inliers=" << roundnessPts.size()
                  << " threshold_mm=" << roundnessThreshold
                  << " roundness_mm=" << sectionRoundness << std::endl;
    }

    if (!roundnessValues.empty())
    {
        result.roundnessToleranceMm = medianValue(roundnessValues);         // ��??��
        double roundnessSum = 0.0;
        for (std::size_t ri = 0; ri < roundnessValues.size(); ++ri)
        {
            roundnessSum += roundnessValues[ri];
        }
        const double roundnessAverage = roundnessSum / static_cast<double>(roundnessValues.size());
        std::cout << "roundness_summary sections=" << roundnessValues.size()
                  << " median_mm=" << result.roundnessToleranceMm
                  << " average_mm=" << roundnessAverage << std::endl;
    }

	// ?�?��������?��?�?��?�������?��������??�������
	CloudPtr straightSideFeature = buildStraightSideFeatureCloud(scanInTemplate, config_);

	// ?�?��
    if (!estimateStraightSideFromBTemplates(straightSideFeature, cylinder, config_, result.straightSideSlopeDeg, result.straightSideHeightMm))
    {
        std::cout << "straight_result failed" << std::endl;
    }


	// ���?��
    if (cylinder.radiusMm > 0.0 && std::isfinite(cylinder.radiusMm))
    {
        result.opening = solveOpeningsByProjectionAndLocalIcp(config_, templ, scanInTemplate, cylinder);
        if (!result.opening.name.empty() || result.opening.fit.inlierCount > 0)
        {
            printFit(result.opening.fit);
        }
    }
    else
    {
        std::cout << "opening_detection status=skip reason=invalid_cylinder" << std::endl;
    }

    return result;
}

}  // namespace hm

















