// 配置解析，标定参数同步自「柱面和开孔测量 V1.2」（支持 crop_boxes，兼容 crop_min/max）。
#include "HeadMeasure/Config.h"

#include <Eigen/Dense>

#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <cstdlib>

namespace hm {
namespace {

std::string readAll(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("cannot open config: " + path);
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

std::string stripComments(std::string text) {
    text = std::regex_replace(text, std::regex("//[^\\n\\r]*"), "");
    text = std::regex_replace(text, std::regex("#[^\\n\\r]*"), "");
    return text;
}

double parseDouble(const std::string& text) {
    return std::strtod(text.c_str(), NULL);
}

std::vector<double> numbersForKey(const std::string& text, const std::string& key) {
    const std::regex re("\"" + key + "\"\\s*:\\s*\\[([^\\]]*)\\]");
    std::smatch match;
    if (!std::regex_search(text, match, re)) {
        return {};
    }

    std::vector<double> values;
    const std::string body = match[1].str();
    const std::regex num("[-+]?\\d*\\.?\\d+(?:[eE][-+]?\\d+)?");
    for (std::sregex_iterator it(body.begin(), body.end(), num), end; it != end; ++it) {
        values.push_back(parseDouble((*it)[0].str()));
    }
    return values;
}

std::vector<std::string> stringsForKey(const std::string& text, const std::string& key) {
    const std::regex re("\"" + key + "\"\\s*:\\s*\\[([^\\]]*)\\]");
    std::smatch match;
    if (!std::regex_search(text, match, re)) {
        return {};
    }

    std::vector<std::string> values;
    const std::string body = match[1].str();
    const std::regex str("\"([^\"]*)\"");
    for (std::sregex_iterator it(body.begin(), body.end(), str), end; it != end; ++it) {
        values.push_back((*it)[1].str());
    }
    return values;
}

std::string stringForKey(const std::string& text, const std::string& key, const std::string& fallback = "") {
    const std::regex re("\"" + key + "\"\\s*:\\s*\"([^\"]*)\"");
    std::smatch match;
    return std::regex_search(text, match, re) ? match[1].str() : fallback;
}

double numberForKey(const std::string& text, const std::string& key, double fallback) {
    const std::regex re("\"" + key + "\"\\s*:\\s*([-+]?\\d*\\.?\\d+(?:[eE][-+]?\\d+)?)");
    std::smatch match;
    return std::regex_search(text, match, re) ? parseDouble(match[1].str()) : fallback;
}

int intForKey(const std::string& text, const std::string& key, int fallback) {
    return static_cast<int>(numberForKey(text, key, fallback));
}

std::vector<std::string> objectBlocksForKey(const std::string& text, const std::string& key) {
    const std::string marker = "\"" + key + "\"";
    const auto keyPos = text.find(marker);
    if (keyPos == std::string::npos) {
        return {};
    }
    const auto arrayStart = text.find('[', keyPos);
    if (arrayStart == std::string::npos) {
        return {};
    }

    std::vector<std::string> blocks;
    int arrayDepth = 0;
    int objectDepth = 0;
    std::size_t objectStart = std::string::npos;
    for (std::size_t i = arrayStart; i < text.size(); ++i) {
        if (text[i] == '[') {
            ++arrayDepth;
        } else if (text[i] == ']') {
            --arrayDepth;
            if (arrayDepth == 0) {
                break;
            }
        } else if (text[i] == '{') {
            if (objectDepth == 0) {
                objectStart = i;
            }
            ++objectDepth;
        } else if (text[i] == '}') {
            --objectDepth;
            if (objectDepth == 0 && objectStart != std::string::npos) {
                blocks.push_back(text.substr(objectStart, i - objectStart + 1));
            }
        }
    }
    return blocks;
}

Eigen::Vector3d vector3d(const std::vector<double>& v, const Eigen::Vector3d& fallback) {
    if (v.size() < 3) {
        return fallback;
    }
    return {v[0], v[1], v[2]};
}

CropBox cropBoxFromBlock(const std::string& block) {
    CropBox box;
    std::vector<double> cropMin = numbersForKey(block, "crop_min");
    std::vector<double> cropMax = numbersForKey(block, "crop_max");
    if (cropMin.empty()) {
        cropMin = numbersForKey(block, "min");
    }
    if (cropMax.empty()) {
        cropMax = numbersForKey(block, "max");
    }
    if (cropMin.size() == 3)
    {
        box.min = Eigen::Vector3f(cropMin[0], cropMin[1], cropMin[2]);
    }
    if (cropMax.size() == 3)
    {
        box.max = Eigen::Vector3f(cropMax[0], cropMax[1], cropMax[2]);
    }
    return box;
}


}  // namespace

MeasureConfig loadConfig(const std::string& path)
{
    const std::string text = stripComments(readAll(path));
    MeasureConfig cfg;

    cfg.inputFrames = stringsForKey(text, "input_frames");
    cfg.templateCloud = stringForKey(text, "template_cloud");

    const auto pose = numbersForKey(text, "pose_correction");
    if (pose.size() == 16) {
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                cfg.poseCorrection(r, c) = static_cast<float>(pose[r * 4 + c]);
            }
        }
    }

    const std::vector<std::string> cropBlocks = objectBlocksForKey(text, "crop_boxes");
    for (std::size_t i = 0; i < cropBlocks.size(); ++i)
    {
        cfg.cropBoxes.push_back(cropBoxFromBlock(cropBlocks[i]));
    }
    if (cfg.cropBoxes.empty())
    {
        cfg.cropBoxes.push_back(cropBoxFromBlock(text));
    }
    cfg.statisticalMeanK = intForKey(text, "statistical_mean_k", cfg.statisticalMeanK);
    cfg.statisticalStddevMul = numberForKey(text, "statistical_stddev_mul", cfg.statisticalStddevMul);
    cfg.voxelLeafMm = numberForKey(text, "voxel_leaf_mm", cfg.voxelLeafMm);
    cfg.cylinderDistanceThresholdMm = numberForKey(text, "cylinder_distance_threshold_mm", cfg.cylinderDistanceThresholdMm);
    cfg.cylinderMaxIterations = intForKey(text, "cylinder_max_iterations", cfg.cylinderMaxIterations);
    cfg.icpMaxCorrespondenceDistanceMm = numberForKey(text, "icp_max_correspondence_distance_mm", cfg.icpMaxCorrespondenceDistanceMm);
    cfg.icpMaxIterations = intForKey(text, "icp_max_iterations", cfg.icpMaxIterations);
    cfg.icpTransformationEpsilon = numberForKey(text, "icp_transformation_epsilon", cfg.icpTransformationEpsilon);
    cfg.icpFitnessEpsilon = numberForKey(text, "icp_fitness_epsilon", cfg.icpFitnessEpsilon);
    cfg.sliceSpacingMm = numberForKey(text, "slice_spacing_mm", cfg.sliceSpacingMm);
    cfg.sliceCount = intForKey(text, "slice_count", cfg.sliceCount);
    cfg.sliceMinPoints = intForKey(text, "slice_min_points", cfg.sliceMinPoints);
    cfg.sliceThicknessMm = numberForKey(text, "slice_thickness_mm", cfg.sliceThicknessMm);
    cfg.circleMaxFitErrorMm = numberForKey(text, "circle_max_fit_error_mm", cfg.circleMaxFitErrorMm);
    const auto topFeatureValues = numbersForKey(text, "template_top_plane_feature_points");
    for (std::size_t i = 0; i + 2 < topFeatureValues.size(); i += 3)
	{
        cfg.templateTopPlaneFeaturePoints.push_back(Eigen::Vector3d(topFeatureValues[i], topFeatureValues[i + 1], topFeatureValues[i + 2]));
    }
    cfg.templateTopPlaneNormal = vector3d(numbersForKey(text, "template_top_plane_normal"), cfg.templateTopPlaneNormal).normalized();
    cfg.topFeatureSearchRadiusMm = numberForKey(text, "top_feature_search_radius_mm", cfg.topFeatureSearchRadiusMm);
    cfg.straightSideOffsetBelowTopMm = numberForKey(text, "straight_side_offset_below_top_mm", cfg.straightSideOffsetBelowTopMm);
    cfg.straightSideCylinderCropHeightMm = numberForKey(text, "straight_side_cylinder_crop_height_mm", cfg.straightSideCylinderCropHeightMm);
    cfg.straightSideCropHeightMm = numberForKey(text, "straight_side_crop_height_mm", cfg.straightSideCropHeightMm);
    cfg.straightEndpointSearchRadiusMm = numberForKey(text, "straight_endpoint_search_radius_mm", cfg.straightEndpointSearchRadiusMm);
    cfg.straightEndpointSliceThicknessMm = numberForKey(text, "straight_endpoint_slice_thickness_mm", cfg.straightEndpointSliceThicknessMm);
    cfg.straightLineDistanceThresholdMm = numberForKey(text, "straight_line_distance_threshold_mm", cfg.straightLineDistanceThresholdMm);
    cfg.straightAToBMaxDistanceMm = numberForKey(text, "straight_a_to_b_max_distance_mm", cfg.straightAToBMaxDistanceMm);
    cfg.straightBMethod = stringForKey(text, "straight_b_method", cfg.straightBMethod);
    cfg.straightBLineFitLengthMm = numberForKey(text, "straight_b_line_fit_length_mm", cfg.straightBLineFitLengthMm);
    cfg.straightBResidualMinThresholdMm = numberForKey(text, "straight_b_residual_min_threshold_mm", cfg.straightBResidualMinThresholdMm);
    cfg.straightBResidualWindowCount = intForKey(text, "straight_b_residual_window_count", cfg.straightBResidualWindowCount);
    cfg.straightBevel30OffsetMm = numberForKey(text, "straight_bevel_30_offset_mm", cfg.straightBevel30OffsetMm);
    cfg.straightBevel45OffsetMm = numberForKey(text, "straight_bevel_45_offset_mm", cfg.straightBevel45OffsetMm);
    cfg.straightBevelType = intForKey(text, "straight_bevel_type", cfg.straightBevelType);

    for (const auto& block : objectBlocksForKey(text, "straight_endpoint_pairs"))
    {
        StraightEndpointPair pair;
        pair.name = stringForKey(block, "name", "straight_endpoint_pair");
        pair.pointB = vector3d(numbersForKey(block, "point_b"), pair.pointB);
        cfg.straightEndpointPairs.push_back(pair);
    }

    for (const auto& block : objectBlocksForKey(text, "template_openings")) 
	{
        OpeningFeature f;
        f.name = stringForKey(block, "name", "opening");
        f.searchRadiusMm = numberForKey(block, "search_radius_mm", f.searchRadiusMm);
        f.expectedDiameterMm = numberForKey(block, "expected_diameter_mm", f.expectedDiameterMm);
        f.diameterToleranceMm = numberForKey(block, "diameter_tolerance_mm", f.diameterToleranceMm);
        f.projectionDirection = vector3d(numbersForKey(block, "projection_direction"), f.projectionDirection).normalized();
        f.projectionImageWidth = intForKey(block, "projection_image_width", f.projectionImageWidth);
        f.projectionImageHeight = intForKey(block, "projection_image_height", f.projectionImageHeight);
        const auto projectionMin = numbersForKey(block, "projection_crop_min");
        const auto projectionMax = numbersForKey(block, "projection_crop_max");
        if (projectionMin.size() == 3)
        {
            f.projectionCrop.min = Eigen::Vector3f(projectionMin[0], projectionMin[1], projectionMin[2]);
        }
        if (projectionMax.size() == 3)
        {
            f.projectionCrop.max = Eigen::Vector3f(projectionMax[0], projectionMax[1], projectionMax[2]);
        }
        const auto openingFeatureValues = numbersForKey(block, "cylinder_feature_points");
        for (std::size_t i = 0; i + 2 < openingFeatureValues.size(); i += 3)
        {
            f.cylinderFeaturePoints.push_back(Eigen::Vector3d(openingFeatureValues[i], openingFeatureValues[i + 1], openingFeatureValues[i + 2]));
        }
        std::cout << "opening_config name=" << f.name
                  << " cylinder_feature_points=" << f.cylinderFeaturePoints.size() << std::endl;
        cfg.templateOpenings.push_back(f);
    }

    return cfg;
}



}  // namespace hm
