#include "BevelMeasurement.h"

#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/icp.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace bevel
{
namespace
{
const double kPi = 3.14159265358979323846;
// Windows 下 PCL StatisticalOutlierRemoval 在 10 万点以上易触发 Eigen aligned_free 崩溃
constexpr std::size_t kMaxOutlierRemovalPoints = 80000;
// crop 后超过此阈值先 VoxelGrid，再考虑 SOR（须先降采样后离群点滤波）
constexpr std::size_t kEarlyDownsampleThreshold = 150000;
constexpr std::size_t kEarlyXyzStrideLimitPoints = 200000;
// PCL VoxelGrid 体素格总数须 < INT_MAX；大场景全幅点云须先 crop 或放大 leaf
constexpr float kMaxVoxelDivisionsPerAxis = 256.0f;
constexpr int64_t kMaxVoxelGridCells = 100000000LL;

/// PCL 滤波/IO 在 Windows DLL 堆上分配点云，若由 exe 侧释放会触发 aligned_free 崩溃。
CloudT::Ptr reownCloudPoints(const CloudT::ConstPtr& input)
{
    CloudT::Ptr output(new CloudT);
    if (input == nullptr || input->empty()) {
        output->width = 0;
        output->height = 1;
        output->is_dense = true;
        return output;
    }

    output->points.reserve(input->points.size());
    for (const PointT& point : input->points) {
        output->points.push_back(point);
    }
    output->width = input->width;
    output->height = input->height;
    output->is_dense = input->is_dense;
    return output;
}

void adoptPclIntermediateCloud(CloudT::Ptr& cloud)
{
    if (!cloud) {
        return;
    }
    static std::vector<CloudT::Ptr> adoptedPclClouds;
    adoptedPclClouds.push_back(cloud);
    cloud.reset();
}

CloudT::Ptr reownFromPclOutput(CloudT::Ptr& pclOutput)
{
    CloudT::Ptr owned = reownCloudPoints(pclOutput);
    adoptPclIntermediateCloud(pclOutput);
    return owned;
}

CloudT::Ptr transformCloudInProcessHeap(const CloudT::ConstPtr& input, const Eigen::Matrix4f& matrix)
{
    CloudT::Ptr output(new CloudT);
    if (input == nullptr || input->empty()) {
        return output;
    }

    output->points.reserve(input->size());
    for (const PointT& point : input->points) {
        const Eigen::Vector4f transformed =
            matrix * Eigen::Vector4f(point.x, point.y, point.z, 1.0f);
        output->points.emplace_back(transformed.x(), transformed.y(), transformed.z());
    }

    output->width = static_cast<std::uint32_t>(output->size());
    output->height = 1;
    output->is_dense = input->is_dense;
    return output;
}

CloudT::Ptr cropCloudInProcessHeap(const CloudT::ConstPtr& input, const BevelConfig& cfg)
{
    CloudT::Ptr output(new CloudT);
    if (input == nullptr || input->empty()) {
        return output;
    }

    output->points.reserve(input->size());
    for (const PointT& point : input->points) {
        if (point.x >= cfg.cropMin.x() && point.x <= cfg.cropMax.x() &&
            point.y >= cfg.cropMin.y() && point.y <= cfg.cropMax.y() &&
            point.z >= cfg.cropMin.z() && point.z <= cfg.cropMax.z()) {
            output->points.push_back(point);
        }
    }

    output->width = static_cast<std::uint32_t>(output->size());
    output->height = 1;
    output->is_dense = true;
    return output;
}

std::string trim(const std::string& s)
{
    const auto begin = std::find_if_not(s.begin(), s.end(), [](unsigned char c) { return std::isspace(c) != 0; });
    const auto end = std::find_if_not(s.rbegin(), s.rend(), [](unsigned char c) { return std::isspace(c) != 0; }).base();
    return begin < end ? std::string(begin, end) : std::string();
}

void replaceAll(std::string& text, const std::string& from, const std::string& to)
{
    size_t pos = 0;
    while ((pos = text.find(from, pos)) != std::string::npos) {
        text.replace(pos, from.size(), to);
        pos += to.size();
    }
}

std::string normalizeConfigText(std::string text)
{
    if (text.size() >= 3 &&
        static_cast<unsigned char>(text[0]) == 0xEF &&
        static_cast<unsigned char>(text[1]) == 0xBB &&
        static_cast<unsigned char>(text[2]) == 0xBF) {
        text.erase(0, 3);
    }

    replaceAll(text, "\xEF\xBC\x9D", "="); // full-width equal sign
    replaceAll(text, "\xEF\xBC\x9A", ":"); // full-width colon
    replaceAll(text, "\xEF\xBC\x8C", " "); // full-width comma
    replaceAll(text, "\xE3\x80\x80", " "); // ideographic space
    replaceAll(text, "\xE2\x88\x92", "-"); // minus sign
    replaceAll(text, "\xE2\x80\x93", "-"); // en dash
    replaceAll(text, "\xE2\x80\x94", "-"); // em dash
    return text;
}

std::string stripInlineComment(const std::string& value)
{
    const size_t comment = value.find('#');
    return trim(comment == std::string::npos ? value : value.substr(0, comment));
}

std::string stripTrailingValuePunctuation(std::string value)
{
    value = trim(value);
    while (!value.empty()) {
        const unsigned char c = static_cast<unsigned char>(value[value.size() - 1]);
        if (value[value.size() - 1] == '.' || value[value.size() - 1] == ',' || value[value.size() - 1] == ';' ||
            c == 0xA1 || c == 0xA3 || c == 0xAC) {
            value.erase(value.size() - 1);
            value = trim(value);
        } else {
            break;
        }
    }
    return value;
}

std::string firstScalarToken(const std::string& value)
{
    std::string v = trim(value);
    if (v.size() >= 2 && ((v[0] == '"' && v[v.size() - 1] == '"') || (v[0] == '\'' && v[v.size() - 1] == '\''))) {
        v = v.substr(1, v.size() - 2);
    }
    size_t end = 0;
    while (end < v.size() && std::isspace(static_cast<unsigned char>(v[end])) == 0) {
        ++end;
    }
    return stripTrailingValuePunctuation(v.substr(0, end));
}

double parseDoubleValue(const std::string& value, const std::string& key, int lineNo)
{
    try {
        size_t used = 0;
        return std::stod(trim(value), &used);
    } catch (const std::exception&) {
        std::ostringstream oss;
        oss << "Invalid numeric value at line " << lineNo << " for " << key << ": " << value;
        throw std::runtime_error(oss.str());
    }
}

int parseIntValue(const std::string& value, const std::string& key, int lineNo)
{
    return static_cast<int>(parseDoubleValue(value, key, lineNo));
}

bool parseBool(const std::string& value)
{
    std::string v = firstScalarToken(value);
    std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return v == "1" || v == "true" || v == "yes" || v == "on" || v == "enable" || v == "enabled";
}
Eigen::Vector3f parseVec3(const std::string& value, const std::string& key = "vec3", int lineNo = 0)
{
    std::stringstream ss(value);
    Eigen::Vector3f out(0.0f, 0.0f, 0.0f);
    ss >> out.x() >> out.y() >> out.z();
    if (!ss) {
        std::ostringstream oss;
        oss << "Invalid vec3 value";
        if (lineNo > 0) {
            oss << " at line " << lineNo;
        }
        oss << " for " << key << ": " << value;
        throw std::runtime_error(oss.str());
    }
    return out;
}

std::string findKnownConfigValue(const std::string& line, const std::vector<std::string>& keys, std::string& keyOut)
{
    size_t bestPos = std::string::npos;
    std::string bestKey;
    for (const std::string& key : keys) {
        const size_t pos = line.find(key);
        if (pos != std::string::npos && (bestPos == std::string::npos || pos < bestPos)) {
            bestPos = pos;
            bestKey = key;
        }
    }
    if (bestPos == std::string::npos) {
        return std::string();
    }

    size_t valueStart = bestPos + bestKey.size();
    while (valueStart < line.size() && std::isspace(static_cast<unsigned char>(line[valueStart])) != 0) {
        ++valueStart;
    }
    if (valueStart < line.size() && (line[valueStart] == '=' || line[valueStart] == ':')) {
        ++valueStart;
    } else {
        return std::string();
    }

    keyOut = bestKey;
    return stripInlineComment(line.substr(valueStart));
}

std::string joinPath(const std::string& dir, const std::string& name)
{
    if (dir.empty()) {
        return name;
    }
    const char last = dir[dir.size() - 1];
    if (last == '/' || last == '\\') {
        return dir + name;
    }
    return dir + "/" + name;
}

void resolveBevelConfigRuntimePaths(BevelConfig& cfg, const std::string& assetRoot)
{
    if (assetRoot.empty()) {
        return;
    }

    cfg.templatePath = joinPath(assetRoot, cfg.templatePath);
    if (cfg.saveDownsampledCloud && !cfg.downsampledCloudPath.empty()) {
        cfg.downsampledCloudPath = joinPath(assetRoot, cfg.downsampledCloudPath);
    }
    if (cfg.saveAlignedCloud && !cfg.alignedCloudPath.empty()) {
        cfg.alignedCloudPath = joinPath(assetRoot, cfg.alignedCloudPath);
    }
}

std::string fileExtensionLower(const std::string& path)
{
    const size_t dot = path.find_last_of('.');
    if (dot == std::string::npos || dot + 1 >= path.size()) {
        return std::string();
    }
    std::string ext = path.substr(dot);
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return ext;
}

bool loadCloudAuto(const std::string& path, CloudT::Ptr cloud)
{
    CloudT::Ptr loaded(new CloudT);
    const std::string ext = fileExtensionLower(path);
    bool ok = false;
    if (ext == ".pcd") {
        ok = pcl::io::loadPCDFile<PointT>(path, *loaded) == 0 && !loaded->empty();
    } else if (ext == ".ply") {
        ok = pcl::io::loadPLYFile<PointT>(path, *loaded) == 0 && !loaded->empty();
    } else {
        ok = loadTextPointCloud(path, loaded);
    }

    if (!ok) {
        return false;
    }

    CloudT::Ptr owned = reownCloudPoints(loaded);
    *cloud = *owned;
    adoptPclIntermediateCloud(loaded);
    return !cloud->empty();
}

std::vector<float> strideSampleXyzBuffer(
    const float* xyz,
    std::size_t pointCount,
    std::size_t maxPointCount)
{
    std::vector<float> sampled;
    if (xyz == nullptr || pointCount == 0 || maxPointCount == 0) {
        return sampled;
    }

    if (pointCount <= maxPointCount) {
        sampled.assign(xyz, xyz + pointCount * 3);
        return sampled;
    }

    const std::size_t stride = (pointCount + maxPointCount - 1) / maxPointCount;
    sampled.reserve(maxPointCount * 3);
    for (std::size_t index = 0; index < pointCount; index += stride) {
        const std::size_t base = index * 3;
        sampled.push_back(xyz[base]);
        sampled.push_back(xyz[base + 1]);
        sampled.push_back(xyz[base + 2]);
    }
    return sampled;
}

Eigen::Matrix4f makePoseTransform(const BevelConfig& cfg)
{
    const Eigen::Vector3f r = cfg.poseRotationDeg * static_cast<float>(kPi / 180.0);
    Eigen::Affine3f tf = Eigen::Affine3f::Identity();
    tf.translation() = cfg.poseTranslation;
    tf.rotate(Eigen::AngleAxisf(r.z(), Eigen::Vector3f::UnitZ()));
    tf.rotate(Eigen::AngleAxisf(r.y(), Eigen::Vector3f::UnitY()));
    tf.rotate(Eigen::AngleAxisf(r.x(), Eigen::Vector3f::UnitX()));
    return tf.matrix();
}

bool needsPoseCorrection(const BevelConfig& cfg)
{
    if (!cfg.poseCorrection) {
        return false;
    }
    if (cfg.poseTranslation.norm() > 1e-6f) {
        return true;
    }
    return cfg.poseRotationDeg.norm() > 1e-6f;
}

CloudT::Ptr filterFinitePoints(const CloudT::ConstPtr& input)
{
    CloudT::Ptr output(new CloudT);
    if (input == nullptr || input->empty()) {
        return output;
    }

    output->points.reserve(input->size());
    for (const PointT& point : input->points) {
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
            output->points.push_back(point);
        }
    }

    output->width = static_cast<std::uint32_t>(output->size());
    output->height = 1;
    output->is_dense = true;
    return output;
}

CloudT::Ptr applyPoseAndCrop(const CloudT::ConstPtr& raw, const BevelConfig& cfg)
{
    CloudT::ConstPtr current = raw;
    if (needsPoseCorrection(cfg)) {
        current = transformCloudInProcessHeap(current, makePoseTransform(cfg));
    }

    if (!cfg.cropBox) {
        return filterFinitePoints(current);
    }

    return cropCloudInProcessHeap(current, cfg);
}

float computeSafeVoxelLeafSize(const CloudT::ConstPtr& cloud, float requestedLeaf)
{
    if (cloud == nullptr || cloud->empty()) {
        return requestedLeaf;
    }

    PointT minPt;
    PointT maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);

    const float spanX = maxPt.x - minPt.x;
    const float spanY = maxPt.y - minPt.y;
    const float spanZ = maxPt.z - minPt.z;

    float minLeaf = std::max(requestedLeaf, 1e-4f);
    if (spanX > 0.0f) {
        minLeaf = std::max(minLeaf, spanX / kMaxVoxelDivisionsPerAxis);
    }
    if (spanY > 0.0f) {
        minLeaf = std::max(minLeaf, spanY / kMaxVoxelDivisionsPerAxis);
    }
    if (spanZ > 0.0f) {
        minLeaf = std::max(minLeaf, spanZ / kMaxVoxelDivisionsPerAxis);
    }

    const float volume = std::max(spanX, 0.0f) * std::max(spanY, 0.0f) * std::max(spanZ, 0.0f);
    if (volume > 0.0f) {
        const float leafForVolume = std::cbrt(
            volume / static_cast<float>(kMaxVoxelGridCells));
        minLeaf = std::max(minLeaf, leafForVolume);
    }

    return minLeaf;
}

CloudT::Ptr preprocess(CloudT::Ptr current, const BevelConfig& cfg)
{
    if (current == nullptr) {
        return CloudT::Ptr(new CloudT);
    }

    if (cfg.outlierRemoval
        && current->size() > static_cast<size_t>(cfg.sorMeanK)
        && current->size() <= kMaxOutlierRemovalPoints) {
        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud(current);
        sor.setMeanK(cfg.sorMeanK);
        sor.setStddevMulThresh(cfg.sorStddevMulThresh);
        CloudT::Ptr filteredPcl(new CloudT);
        sor.filter(*filteredPcl);
        current = reownFromPclOutput(filteredPcl);
    }

    return current;
}

CloudT::Ptr downsampleForIcp(const CloudT::ConstPtr& cloud, const BevelConfig& cfg, bool allowSaveCloud)
{
    if (!cfg.uniformDownsample || cfg.uniformLeafSize <= 0.0 || cloud == nullptr || cloud->empty()) {
        return reownCloudPoints(cloud);
    }

    float leaf = computeSafeVoxelLeafSize(cloud, static_cast<float>(cfg.uniformLeafSize));
    for (int attempt = 0; attempt < 6; ++attempt) {
        pcl::VoxelGrid<PointT> filter;
        filter.setInputCloud(cloud);
        filter.setLeafSize(leaf, leaf, leaf);
        CloudT::Ptr sampledPcl(new CloudT);
        filter.filter(*sampledPcl);
        if (!sampledPcl->empty()) {
            CloudT::Ptr sampled = reownFromPclOutput(sampledPcl);
            if (allowSaveCloud && cfg.saveDownsampledCloud) {
                pcl::io::savePCDFileBinary(cfg.downsampledCloudPath, *sampled);
            }
            return sampled;
        }
        adoptPclIntermediateCloud(sampledPcl);
        leaf *= 2.0f;
    }

    throw std::runtime_error("VoxelGrid downsampling failed: leaf size too small for input cloud");
}

TemplateFeature parseTemplateFeatureLine(const std::string& line)
{
    std::stringstream ss(line);
    TemplateFeature f;
    ss >> f.name >> f.point.x() >> f.point.y() >> f.point.z();
    if (!ss) 
	{
        throw std::runtime_error("Invalid feature line: " + line);
    }
    return f;
}

std::vector<TemplateFeature> loadTemplateFeatures(const std::string& path)
{
    std::ifstream in(path);
    if (!in.is_open())
	{
        throw std::runtime_error("Failed to open feature file: " + path);
    }

    std::vector<TemplateFeature> features;
    std::string line;
    while (std::getline(in, line))
	{
        line = stripInlineComment(line);
        if (line.empty() || line[0] == '[') 
		{
            continue;
        }
        features.push_back(parseTemplateFeatureLine(line));
    }
    return features;
}

std::vector<TemplateFeature> loadTemplateFeaturesFromConfigText(const std::string& text, const std::string& method)
{
    const std::string targetSection = method == "direct_points" ? "features.direct_points" : "features.plane_fit";
    std::vector<TemplateFeature> features;
    bool inTargetSection = false;

    std::istringstream in(text);
    std::string line;
    while (std::getline(in, line))
	{
        line = stripInlineComment(line);
        if (line.empty())
		{
            continue;
        }
        if (line.front() == '[' && line.back() == ']')
		{
            const std::string section = trim(line.substr(1, line.size() - 2));
            inTargetSection = section == targetSection;
            continue;
        }
        if (inTargetSection)
		{
            features.push_back(parseTemplateFeatureLine(line));
        }
    }
    return features;
}
Eigen::Vector3f projectPointToPlane(const Eigen::Vector3f& point, const Eigen::Vector3f& planePoint, const Eigen::Vector3f& planeNormal)
{
    return point - planeNormal * ((point - planePoint).dot(planeNormal));
}

Eigen::Vector3f fitPlaneNormalFromPoints(const std::vector<Eigen::Vector3f>& points)
{
    if (points.size() < 3) {
        throw std::runtime_error("Plane fitting needs at least 3 points");
    }

    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (size_t i = 0; i < points.size(); ++i) {
        centroid += points[i];
    }
    centroid /= static_cast<float>(points.size());

    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
    for (size_t i = 0; i < points.size(); ++i) {
        const Eigen::Vector3f d = points[i] - centroid;
        covariance += d * d.transpose();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
    if (solver.info() != Eigen::Success) {
        throw std::runtime_error("Plane fitting eigen decomposition failed");
    }

    Eigen::Vector3f normal = solver.eigenvectors().col(0);
    if (normal.norm() < 1e-6f) {
        throw std::runtime_error("Plane fitting failed: normal vector is zero");
    }
    normal.normalize();
    return normal;
}

Eigen::Vector3f projectVectorToPlane(const Eigen::Vector3f& vec, const Eigen::Vector3f& planeNormal)
{
    return vec - planeNormal * vec.dot(planeNormal);
}

double acuteAngleFromCos(double cosTheta)
{
    cosTheta = std::max(-1.0, std::min(1.0, cosTheta));
    const double theta = std::acos(cosTheta) * 180.0 / kPi;
    return std::min(theta, 180.0 - theta);
}
BevelMeasurementResult buildMeasurementResult(double angleDeg,
                                              double length,
                                              double icpFitness,
                                              const Eigen::Matrix4f& scanToTemplate,
                                              const BevelConfig& cfg)
{
    BevelMeasurementResult out;
    out.ok = true;
    out.angleDeg = angleDeg;
    out.length = length;
    out.icpFitness = icpFitness;
    out.scanToTemplate = scanToTemplate;
    out.message = "OK";
    return out;
}
BevelMeasurementResult computeDirectPointMeasurement(const std::map<std::string, Eigen::Vector3f>& actual,
                                                     const BevelConfig& cfg,
                                                     double icpFitness,
                                                     const Eigen::Matrix4f& scanToTemplate)
{
    const char* required[] = {"angle_a", "angle_b", "angle_c", "length_a", "length_b"};
    for (int i = 0; i < 5; ++i) {
        if (actual.find(required[i]) == actual.end()) {
            throw std::runtime_error(std::string("Missing direct feature: ") + required[i]);
        }
    }

    const Eigen::Vector3f ba = actual.find("angle_a")->second - actual.find("angle_b")->second;
    const Eigen::Vector3f bc = actual.find("angle_c")->second - actual.find("angle_b")->second;
    if (ba.norm() < 1e-6f || bc.norm() < 1e-6f) {
        throw std::runtime_error("Invalid direct angle points: vector length is zero");
    }
    const double angleDeg = acuteAngleFromCos(ba.normalized().dot(bc.normalized()));
    const double length = static_cast<double>((actual.find("length_a")->second - actual.find("length_b")->second).norm());
    return buildMeasurementResult(angleDeg, length, icpFitness, scanToTemplate, cfg);
}

BevelMeasurementResult computePlaneFitMeasurement(const std::map<std::string, Eigen::Vector3f>& actual,
                                                  const std::map<std::string, Eigen::Vector3f>& templateFeature,
                                                  const BevelConfig& cfg,
                                                  double icpFitness,
                                                  const Eigen::Matrix4f& scanToTemplate)
{
    const char* requiredTemplate[] = {"project_normal", "project_point"};
    for (int i = 0; i < 2; ++i) {
        if (templateFeature.find(requiredTemplate[i]) == templateFeature.end()) {
            throw std::runtime_error(std::string("Missing plane-fit feature: ") + requiredTemplate[i]);
        }
    }

    std::vector<Eigen::Vector3f> plane1;
    std::vector<Eigen::Vector3f> plane2;
    plane1.reserve(6);
    plane2.reserve(6);
    for (int i = 1; i <= 6; ++i) {
        const std::string p1 = "plane1_p" + std::to_string(i);
        const std::string p2 = "plane2_p" + std::to_string(i);
        if (actual.find(p1) == actual.end()) {
            throw std::runtime_error("Missing plane-fit feature: " + p1);
        }
        if (actual.find(p2) == actual.end()) {
            throw std::runtime_error("Missing plane-fit feature: " + p2);
        }
        plane1.push_back(actual.find(p1)->second);
        plane2.push_back(actual.find(p2)->second);
    }
    if (actual.find("length_a") == actual.end() || actual.find("length_b") == actual.end()) {
        throw std::runtime_error("Missing plane-fit length feature: length_a or length_b");
    }

    Eigen::Vector3f n1 = fitPlaneNormalFromPoints(plane1);
    Eigen::Vector3f n2 = fitPlaneNormalFromPoints(plane2);
    Eigen::Vector3f projectNormal = templateFeature.find("project_normal")->second;
    if (projectNormal.norm() < 1e-6f) {
        throw std::runtime_error("Invalid plane-fit project_normal: normal vector is zero");
    }
    projectNormal.normalize();

    Eigen::Vector3f p1 = projectVectorToPlane(n1, projectNormal);
    Eigen::Vector3f p2 = projectVectorToPlane(n2, projectNormal);
    if (p1.norm() < 1e-6f || p2.norm() < 1e-6f) {
        throw std::runtime_error("Invalid projected plane normal: vector length is zero");
    }
    p1.normalize();
    p2.normalize();
    const double angleDeg = acuteAngleFromCos(p1.dot(p2));
    const double length = static_cast<double>((actual.find("length_a")->second - actual.find("length_b")->second).norm());
    return buildMeasurementResult(angleDeg, length, icpFitness, scanToTemplate, cfg);
}

Eigen::Vector3f nearestPoint(pcl::KdTreeFLANN<PointT>& tree,
                             const CloudT::ConstPtr& cloud,
                             const Eigen::Vector3f& query)
{
    std::vector<int> indices(1);
    std::vector<float> sqrDistances(1);
    PointT p(query.x(), query.y(), query.z());
    if (tree.nearestKSearch(p, 1, indices, sqrDistances) <= 0) {
        throw std::runtime_error("Nearest point search failed");
    }
    const PointT& found = cloud->points[indices[0]];
    return Eigen::Vector3f(found.x, found.y, found.z);
}

BevelMeasurementResult solveGeometry(const CloudT::ConstPtr& icpScan,
                                      const CloudT::ConstPtr& measureScan,
                                      const CloudT::ConstPtr& templ,
                                      const std::vector<TemplateFeature>& features,
                                      const BevelConfig& cfg)
{
    const CloudT::ConstPtr icpSourceLocal = reownCloudPoints(icpScan);
    const CloudT::ConstPtr icpTargetLocal = reownCloudPoints(templ);

    Eigen::Matrix4f scanToTemplate = Eigen::Matrix4f::Identity();
    double icpFitness = 0.0;
    CloudT::Ptr aligned;

    {
        pcl::IterativeClosestPoint<PointT, PointT> icp;
        icp.setInputSource(icpSourceLocal);
        icp.setInputTarget(icpTargetLocal);
        icp.setMaximumIterations(cfg.icpMaxIterations);
        icp.setMaxCorrespondenceDistance(cfg.icpMaxCorrespondenceDistance);
        icp.setTransformationEpsilon(cfg.icpTransformationEpsilon);
        icp.setEuclideanFitnessEpsilon(cfg.icpEuclideanFitnessEpsilon);
        if (cfg.icpTrimEnable) {
            pcl::registration::CorrespondenceRejectorTrimmed::Ptr trimmed(
                new pcl::registration::CorrespondenceRejectorTrimmed);
            trimmed->setOverlapRatio(static_cast<float>(cfg.icpTrimOverlapRatio));
            trimmed->setMinCorrespondences(cfg.icpTrimMinCorrespondences);
            icp.addCorrespondenceRejector(trimmed);
        }

        CloudT::Ptr alignedPcl(new CloudT);
        icp.align(*alignedPcl);
        aligned = reownFromPclOutput(alignedPcl);
        scanToTemplate = icp.getFinalTransformation();
        icpFitness = icp.getFitnessScore();
    }

    CloudT::Ptr scanAlignedToTemplate =
        transformCloudInProcessHeap(measureScan, scanToTemplate);
    if (cfg.saveAlignedCloud) {
        pcl::io::savePCDFileBinary(cfg.alignedCloudPath, *scanAlignedToTemplate);
    }

    std::map<std::string, Eigen::Vector3f> actual;
    std::map<std::string, Eigen::Vector3f> templateFeature;
    {
        pcl::KdTreeFLANN<PointT> alignedTree;
        alignedTree.setInputCloud(scanAlignedToTemplate);

        for (const TemplateFeature& feature : features)
        {
            templateFeature[feature.name] = feature.point;
            if (feature.name == "project_normal" || feature.name == "project_point")
            {
                continue;
            }
            actual[feature.name] = nearestPoint(alignedTree, scanAlignedToTemplate, feature.point);
            std::cout << "matched feature point: " << feature.name << " "
                      << actual[feature.name].x() << " "
                      << actual[feature.name].y() << " "
                      << actual[feature.name].z() << std::endl;
        }
    }

    if (cfg.measurementMethod == "direct_points")
    {
        return computeDirectPointMeasurement(actual, cfg, icpFitness, scanToTemplate);
    }
    if (cfg.measurementMethod == "plane_fit")
    {
        return computePlaneFitMeasurement(actual, templateFeature, cfg, icpFitness, scanToTemplate);
    }

    throw std::runtime_error("Invalid measurement.method: use plane_fit or direct_points");
}

} // namespace
std::string readWholeFile(const std::string& path)
{
    std::ifstream in(path, std::ios::binary);
    if (!in.is_open()) {
        throw std::runtime_error("Failed to open config file: " + path);
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return normalizeConfigText(ss.str());
}

size_t findConfigKey(const std::string& text, const std::string& key)
{
    return text.find(key);
}

std::string valueAfterKey(const std::string& text, const std::string& key)
{
    const size_t keyPos = findConfigKey(text, key);
    if (keyPos == std::string::npos) {
        return std::string();
    }

    size_t pos = keyPos + key.size();
    while (pos < text.size() && std::isspace(static_cast<unsigned char>(text[pos])) != 0) {
        ++pos;
    }
    if (pos < text.size() && (text[pos] == '=' || text[pos] == ':')) {
        ++pos;
    }
    while (pos < text.size() && std::isspace(static_cast<unsigned char>(text[pos])) != 0) {
        ++pos;
    }

    size_t end = pos;
    while (end < text.size() && text[end] != '\r' && text[end] != '\n' && text[end] != '#') {
        ++end;
    }
    return trim(text.substr(pos, end - pos));
}

std::string requireConfigValue(const std::string& text, const std::string& key)
{
    const std::string value = valueAfterKey(text, key);
    if (value.empty()) {
        throw std::runtime_error("Missing or empty config item: " + key);
    }
    return value;
}

bool hasConfigKey(const std::string& text, const std::string& key)
{
    return findConfigKey(text, key) != std::string::npos;
}

void validateConfig(const BevelConfig& cfg)
{
    if (cfg.cropMin.x() > cfg.cropMax.x() || cfg.cropMin.y() > cfg.cropMax.y() || cfg.cropMin.z() > cfg.cropMax.z()) {
        throw std::runtime_error("Invalid crop box: preprocess.crop.min must be less than preprocess.crop.max");
    }
    if (cfg.uniformDownsample && cfg.uniformLeafSize <= 0.0) {
        throw std::runtime_error("Invalid uniform leaf size: preprocess.uniform.leaf_size must be > 0");
    }
    if (cfg.icpTrimOverlapRatio < 0.0 || cfg.icpTrimOverlapRatio > 1.0) {
        throw std::runtime_error("Invalid ICP trim ratio: icp.trim.overlap_ratio must be in [0, 1]");
    }
    if (cfg.icpTrimMinCorrespondences < 1) {
        throw std::runtime_error("Invalid ICP trim min correspondences: icp.trim.min_correspondences must be >= 1");
    }
}

std::string firstConfigValueLine(const std::string& text)
{
    std::istringstream in(text);
    std::string line;
    while (std::getline(in, line)) {
        line = stripInlineComment(line);
        if (!line.empty()) {
            return line;
        }
    }
    return std::string();
}
BevelConfig loadConfig(const std::string& configPath)
{
    BevelConfig cfg;
    const std::string text = readWholeFile(configPath);
    cfg.templatePath = firstConfigValueLine(text);
    if (cfg.templatePath.empty()) {
        throw std::runtime_error("Config first line must be template cloud path");
    }

    if (hasConfigKey(text, "preprocess.uniform.enable")) cfg.uniformDownsample = parseBool(requireConfigValue(text, "preprocess.uniform.enable"));
    if (hasConfigKey(text, "preprocess.uniform.leaf_size")) cfg.uniformLeafSize = parseDoubleValue(requireConfigValue(text, "preprocess.uniform.leaf_size"), "preprocess.uniform.leaf_size", 0);
    if (hasConfigKey(text, "preprocess.uniform.save_cloud")) cfg.saveDownsampledCloud = parseBool(requireConfigValue(text, "preprocess.uniform.save_cloud"));
    if (hasConfigKey(text, "preprocess.uniform.cloud_path")) cfg.downsampledCloudPath = requireConfigValue(text, "preprocess.uniform.cloud_path");
    if (hasConfigKey(text, "preprocess.pose.enable")) cfg.poseCorrection = parseBool(requireConfigValue(text, "preprocess.pose.enable"));
    if (hasConfigKey(text, "preprocess.pose.translation")) cfg.poseTranslation = parseVec3(requireConfigValue(text, "preprocess.pose.translation"), "preprocess.pose.translation", 0);
    if (hasConfigKey(text, "preprocess.pose.rotation_deg")) cfg.poseRotationDeg = parseVec3(requireConfigValue(text, "preprocess.pose.rotation_deg"), "preprocess.pose.rotation_deg", 0);
    if (hasConfigKey(text, "preprocess.crop.enable")) cfg.cropBox = parseBool(requireConfigValue(text, "preprocess.crop.enable"));
    if (hasConfigKey(text, "preprocess.crop.min")) cfg.cropMin = parseVec3(requireConfigValue(text, "preprocess.crop.min"), "preprocess.crop.min", 0).homogeneous();
    if (hasConfigKey(text, "preprocess.crop.max")) cfg.cropMax = parseVec3(requireConfigValue(text, "preprocess.crop.max"), "preprocess.crop.max", 0).homogeneous();
    if (hasConfigKey(text, "preprocess.outlier.enable")) cfg.outlierRemoval = parseBool(requireConfigValue(text, "preprocess.outlier.enable"));
    if (hasConfigKey(text, "preprocess.outlier.mean_k")) cfg.sorMeanK = parseIntValue(requireConfigValue(text, "preprocess.outlier.mean_k"), "preprocess.outlier.mean_k", 0);
    if (hasConfigKey(text, "preprocess.outlier.stddev_mul")) cfg.sorStddevMulThresh = parseDoubleValue(requireConfigValue(text, "preprocess.outlier.stddev_mul"), "preprocess.outlier.stddev_mul", 0);

    if (hasConfigKey(text, "icp.max_iterations")) cfg.icpMaxIterations = parseIntValue(requireConfigValue(text, "icp.max_iterations"), "icp.max_iterations", 0);
    if (hasConfigKey(text, "icp.max_correspondence_distance")) cfg.icpMaxCorrespondenceDistance = parseDoubleValue(requireConfigValue(text, "icp.max_correspondence_distance"), "icp.max_correspondence_distance", 0);
    if (hasConfigKey(text, "icp.transformation_epsilon")) cfg.icpTransformationEpsilon = parseDoubleValue(requireConfigValue(text, "icp.transformation_epsilon"), "icp.transformation_epsilon", 0);
    if (hasConfigKey(text, "icp.euclidean_fitness_epsilon")) cfg.icpEuclideanFitnessEpsilon = parseDoubleValue(requireConfigValue(text, "icp.euclidean_fitness_epsilon"), "icp.euclidean_fitness_epsilon", 0);
    if (hasConfigKey(text, "icp.trim.enable")) cfg.icpTrimEnable = parseBool(requireConfigValue(text, "icp.trim.enable"));
    if (hasConfigKey(text, "icp.trim.overlap_ratio")) cfg.icpTrimOverlapRatio = parseDoubleValue(requireConfigValue(text, "icp.trim.overlap_ratio"), "icp.trim.overlap_ratio", 0);
    if (hasConfigKey(text, "icp.trim.min_correspondences")) cfg.icpTrimMinCorrespondences = parseIntValue(requireConfigValue(text, "icp.trim.min_correspondences"), "icp.trim.min_correspondences", 0);
    if (hasConfigKey(text, "icp.save_aligned_cloud")) cfg.saveAlignedCloud = parseBool(requireConfigValue(text, "icp.save_aligned_cloud"));
    if (hasConfigKey(text, "icp.aligned_cloud_path")) cfg.alignedCloudPath = requireConfigValue(text, "icp.aligned_cloud_path");

    if (hasConfigKey(text, "template.path")) cfg.templatePath = requireConfigValue(text, "template.path");
    if (hasConfigKey(text, "measurement.method")) cfg.measurementMethod = firstScalarToken(requireConfigValue(text, "measurement.method"));


    if (cfg.measurementMethod != "plane_fit" && cfg.measurementMethod != "direct_points") {
        throw std::runtime_error("Invalid measurement method: measurement.method must be plane_fit or direct_points");
    }
    validateConfig(cfg);
    return cfg;
}

BevelMeasurementResult solveBevelFromXyzBuffer(const float* xyz,
                                               std::size_t floatCount,
                                               const std::string& configPath,
                                               const std::string& templateDir)
{
    BevelMeasurementResult result;
    if (xyz == nullptr || floatCount < 3 || (floatCount % 3) != 0) {
        result.ok = false;
        result.message = "Invalid xyz buffer: null or length not a multiple of 3";
        return result;
    }

    const std::size_t pointCount = floatCount / 3;
    const float* effectiveXyz = xyz;
    std::vector<float> sampledXyz;
    if (pointCount > kEarlyXyzStrideLimitPoints) {
        sampledXyz = strideSampleXyzBuffer(xyz, pointCount, kEarlyXyzStrideLimitPoints);
        if (sampledXyz.size() < 3 || (sampledXyz.size() % 3) != 0) {
            result.ok = false;
            result.message = "Input xyz buffer has no finite points after early sampling";
            return result;
        }
        effectiveXyz = sampledXyz.data();
        floatCount = sampledXyz.size();
    }

    CloudT::Ptr raw(new CloudT);
    const std::size_t effectivePointCount = floatCount / 3;
    raw->points.reserve(effectivePointCount);
    for (std::size_t index = 0; index < effectivePointCount; ++index) {
        const std::size_t base = index * 3;
        const float x = effectiveXyz[base];
        const float y = effectiveXyz[base + 1];
        const float z = effectiveXyz[base + 2];
        if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
            raw->points.emplace_back(x, y, z);
        }
    }

    raw->width = static_cast<std::uint32_t>(raw->size());
    raw->height = 1;
    raw->is_dense = true;

    if (raw->empty()) {
        result.ok = false;
        result.message = "Input xyz buffer has no finite points";
        return result;
    }

    return solveBevelFromRawCloud(raw, configPath, templateDir);
}

BevelMeasurementResult solveBevelFromPointCloudFile(const std::string& cloudPath,
                                                    const std::string& configPath,
                                                    const std::string& templateDir)
{
    BevelMeasurementResult result;
    CloudT::Ptr raw(new CloudT);
    if (!loadCloudAuto(cloudPath, raw)) {
        result.ok = false;
        result.message = "Failed to load point cloud: " + cloudPath;
        return result;
    }
    return solveBevelFromRawCloud(raw, configPath, templateDir);
}

BevelMeasurementResult solveBevelFromRawCloud(const CloudT::ConstPtr& rawCloud, const std::string& configPath)
{
    return solveBevelFromRawCloud(rawCloud, configPath, std::string());
}

BevelMeasurementResult solveBevelFromRawCloud(const CloudT::ConstPtr& rawCloud,
                                              const std::string& configPath,
                                              const std::string& templateDir)
{
    BevelMeasurementResult result;
    try 
	{
        BevelConfig cfg = loadConfig(configPath);
        resolveBevelConfigRuntimePaths(cfg, templateDir);
        std::cout << "[Bevel] 使用配置文件: " << configPath << std::endl;
        std::cout << "[Bevel] 使用模版文件: " << cfg.templatePath << std::endl;
        CloudT::Ptr finiteCloud = filterFinitePoints(rawCloud);
        if (finiteCloud->empty()) {
            throw std::runtime_error("Input point cloud has no finite XYZ points");
        }

        CloudT::Ptr measureScan = applyPoseAndCrop(finiteCloud, cfg);
        if (measureScan->empty()) {
            throw std::runtime_error(
                "Crop box removed all points; check preprocess.crop.min/max in bevel config");
        }

        if (measureScan->size() > kEarlyDownsampleThreshold
            && cfg.uniformDownsample
            && cfg.uniformLeafSize > 0.0) {
            // 大点云须先体素降采样，否则后续 SOR/VoxelGrid 在 Windows 上易堆损坏
            std::cout << "[Bevel] early voxel downsample: points=" << measureScan->size()
                      << " threshold=" << kEarlyDownsampleThreshold << std::endl;
            measureScan = downsampleForIcp(measureScan, cfg, false);
            std::cout << "[Bevel] after early downsample: points=" << measureScan->size() << std::endl;
        }
        std::cout << "[Bevel] preprocess (SOR max=" << kMaxOutlierRemovalPoints
                  << " enable=" << cfg.outlierRemoval << ") input points=" << measureScan->size()
                  << std::endl;
        measureScan = preprocess(measureScan, cfg);
        if (measureScan->empty())
		{
            throw std::runtime_error("Preprocessed cloud is empty");
        }
        CloudT::Ptr scan = downsampleForIcp(measureScan, cfg, true);
        if (scan->empty())
        {
            throw std::runtime_error("Downsampled cloud is empty");
        }

        CloudT::Ptr templ(new CloudT);
        const std::string templatePath = cfg.templatePath;
        if (!loadCloudAuto(templatePath, templ))
		{
            throw std::runtime_error("Failed to load template cloud: " + templatePath);
        }

        std::vector<TemplateFeature> features = loadTemplateFeaturesFromConfigText(readWholeFile(configPath), cfg.measurementMethod);
        if (features.empty())
		{
            throw std::runtime_error("No inline feature points found for measurement.method: " + cfg.measurementMethod);
        }
		result = solveGeometry(scan, measureScan, templ, features, cfg);
    } 
	catch (const std::exception& e) 
	{
        result.ok = false;
        result.message = e.what();
    }
    return result;
}
bool loadTextPointCloud(const std::string& path, CloudT::Ptr cloud)
{
    std::ifstream in(path);
    if (!in.is_open()) {
        return false;
    }

    cloud->clear();
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    while (in >> x >> y >> z) {
        cloud->push_back(PointT(x, y, z));
    }
    cloud->width = static_cast<unsigned int>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = false;
    return !cloud->empty();
}

} // namespace bevel
