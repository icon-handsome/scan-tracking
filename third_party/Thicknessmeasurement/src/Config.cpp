#include "Config.h"

#include <fstream>
#include <iomanip>
#include <sstream>

#include <Windows.h>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace
{
void AppendUnicodeEscape(unsigned int codePoint, std::ostringstream* output)
{
    if (codePoint <= 0xFFFF)
    {
        (*output) << "\\u" << std::uppercase << std::hex << std::setw(4) << std::setfill('0') << codePoint;
    }
    else
    {
        codePoint -= 0x10000;
        const unsigned int high = 0xD800 + ((codePoint >> 10) & 0x3FF);
        const unsigned int low = 0xDC00 + (codePoint & 0x3FF);
        (*output) << "\\u" << std::uppercase << std::hex << std::setw(4) << std::setfill('0') << high;
        (*output) << "\\u" << std::uppercase << std::hex << std::setw(4) << std::setfill('0') << low;
    }
    (*output) << std::dec << std::nouppercase;
}

std::wstring DecodeTextByCodePage(const std::string& input, UINT codePage, DWORD flags)
{
    if (input.empty())
    {
        return std::wstring();
    }

    int count = MultiByteToWideChar(
        codePage,
        flags,
        input.data(),
        static_cast<int>(input.size()),
        NULL,
        0);
    if (count <= 0)
    {
        return std::wstring();
    }

    std::wstring output;
    output.resize(count);
    MultiByteToWideChar(
        codePage,
        flags,
        input.data(),
        static_cast<int>(input.size()),
        &output[0],
        count);
    return output;
}

std::wstring DecodeJsonText(const std::string& input)
{
    std::string content = input;

    if (content.size() >= 3 &&
        static_cast<unsigned char>(content[0]) == 0xEF &&
        static_cast<unsigned char>(content[1]) == 0xBB &&
        static_cast<unsigned char>(content[2]) == 0xBF)
    {
        content.erase(0, 3);
    }

    std::wstring decoded = DecodeTextByCodePage(content, CP_UTF8, MB_ERR_INVALID_CHARS);
    if (!decoded.empty())
    {
        return decoded;
    }

    return DecodeTextByCodePage(content, CP_ACP, 0);
}

std::string EscapeWideTextForBoostJson(const std::wstring& input)
{
    std::ostringstream output;
    for (std::wstring::size_type i = 0; i < input.size(); ++i)
    {
        const wchar_t ch = input[i];
        if (ch >= 0 && ch < 0x80)
        {
            output << static_cast<char>(ch);
        }
        else
        {
            AppendUnicodeEscape(static_cast<unsigned int>(ch), &output);
        }
    }
    return output.str();
}

std::string Utf8ToLocalAnsi(const std::string& input)
{
    if (input.empty())
    {
        return input;
    }

    const std::wstring wide = DecodeTextByCodePage(input, CP_UTF8, MB_ERR_INVALID_CHARS);
    if (wide.empty())
    {
        return input;
    }

    const int count = WideCharToMultiByte(
        CP_ACP,
        0,
        wide.data(),
        static_cast<int>(wide.size()),
        NULL,
        0,
        NULL,
        NULL);
    if (count <= 0)
    {
        return input;
    }

    std::string output;
    output.resize(count);
    WideCharToMultiByte(
        CP_ACP,
        0,
        wide.data(),
        static_cast<int>(wide.size()),
        &output[0],
        count,
        NULL,
        NULL);
    return output;
}

bool IsAbsolutePath(const std::string& path)
{
    if (path.size() >= 2 && path[1] == ':')
    {
        return true;
    }
    return !path.empty() && (path[0] == '/' || path[0] == '\\');
}

bool PathExists(const std::string& path)
{
    return GetFileAttributesA(path.c_str()) != INVALID_FILE_ATTRIBUTES;
}

std::string GetDirectoryName(const std::string& path)
{
    const std::string::size_type slash = path.find_last_of("/\\");
    if (slash == std::string::npos)
    {
        return "";
    }
    return path.substr(0, slash);
}

std::string GetExeDirectory()
{
    char path[MAX_PATH] = {0};
    const DWORD length = GetModuleFileNameA(NULL, path, MAX_PATH);
    if (length == 0 || length >= MAX_PATH)
    {
        return "";
    }
    return GetDirectoryName(path);
}

std::string JoinPath(const std::string& dir, const std::string& file)
{
    if (dir.empty())
    {
        return file;
    }

    const char last = dir[dir.size() - 1];
    if (last == '/' || last == '\\')
    {
        return dir + file;
    }

    return dir + "/" + file;
}

std::string ResolvePath(const std::string& path, const std::string& baseDir)
{
    if (path.empty() || IsAbsolutePath(path))
    {
        return path;
    }

    const std::string baseRelativePath = JoinPath(baseDir, path);
    if (PathExists(baseRelativePath))
    {
        return baseRelativePath;
    }

    const std::string projectDir = JoinPath(GetExeDirectory(), "../../..");
    const std::string projectConfigDir = JoinPath(projectDir, "config");
    const std::string projectRelativePath = JoinPath(projectConfigDir, path);
    if (PathExists(projectRelativePath))
    {
        return projectRelativePath;
    }

    return baseRelativePath;
}

std::string ToLower(std::string value)
{
    for (std::string::size_type i = 0; i < value.size(); ++i)
    {
        if (value[i] >= 'A' && value[i] <= 'Z')
        {
            value[i] = static_cast<char>(value[i] - 'A' + 'a');
        }
    }
    return value;
}

bool ParseThicknessMethod(const std::string& value, ThicknessMethod* method)
{
    const std::string normalized = ToLower(value);
    if (normalized == "nearest_between_surfaces")
    {
        *method = ThicknessMethodNearestBetweenSurfaces;
        return true;
    }
    if (normalized == "tangent_plane_projection")
    {
        *method = ThicknessMethodTangentPlaneProjection;
        return true;
    }
    return false;
}

void ReadJsonAllowUtf8Path(const std::string& path, boost::property_tree::ptree* root)
{
    std::ifstream input(path.c_str(), std::ios::binary);
    if (!input)
    {
        boost::property_tree::read_json(path, *root);
        return;
    }

    std::ostringstream buffer;
    buffer << input.rdbuf();

    const std::string escapedJson = EscapeWideTextForBoostJson(DecodeJsonText(buffer.str()));
    std::istringstream stream(escapedJson);
    boost::property_tree::read_json(stream, *root);
}

Point3d ReadPoint(const boost::property_tree::ptree& node)
{
    Point3d point;
    point.x = node.get<double>("x");
    point.y = node.get<double>("y");
    point.z = node.get<double>("z");
    return point;
}

std::string ReadOptionalPath(
    const boost::property_tree::ptree& root,
    const std::string& key,
    const std::string& configDir)
{
    try
    {
        const std::string value = root.get<std::string>(key);
        if (value.empty())
        {
            return "";
        }
        return ResolvePath(Utf8ToLocalAnsi(value), configDir);
    }
    catch (...)
    {
        return "";
    }
}
}

Eigen::Vector3d ToEigen(const Point3d& point)
{
    return Eigen::Vector3d(point.x, point.y, point.z);
}

std::string ThicknessMethodToString(ThicknessMethod method)
{
    switch (method)
    {
    case ThicknessMethodNearestBetweenSurfaces:
        return "nearest_between_surfaces";
    case ThicknessMethodTangentPlaneProjection:
        return "tangent_plane_projection";
    default:
        return "unknown";
    }
}

bool LoadConfig(const std::string& path, ThicknessConfig* config, std::string* error)
{
    if (config == NULL)
    {
        if (error != NULL)
        {
            *error = "config output pointer is null";
        }
        return false;
    }

    try
    {
        boost::property_tree::ptree root;
        ReadJsonAllowUtf8Path(path, &root);
        const std::string configDir = GetDirectoryName(path);

        config->pointCloud.innerTemplateCloudPath =
            ReadOptionalPath(root, "point_cloud.inner_template_cloud_path", configDir);
        config->pointCloud.outerTemplateCloudPath =
            ReadOptionalPath(root, "point_cloud.outer_template_cloud_path", configDir);

        const std::string legacyTemplatePath =
            ReadOptionalPath(root, "point_cloud.template_cloud_path", configDir);
        if (config->pointCloud.innerTemplateCloudPath.empty() && !legacyTemplatePath.empty())
        {
            config->pointCloud.innerTemplateCloudPath = legacyTemplatePath;
        }
        if (config->pointCloud.outerTemplateCloudPath.empty() && !legacyTemplatePath.empty())
        {
            config->pointCloud.outerTemplateCloudPath = legacyTemplatePath;
        }

        config->preprocess.enableOutlierRemoval = root.get<bool>("preprocess.enable_outlier_removal", true);
        config->preprocess.meanK = root.get<int>("preprocess.mean_k", 30);
        config->preprocess.stddevMulThresh = root.get<double>("preprocess.stddev_mul_thresh", 1.0);
        config->preprocess.enableVoxelDownsample = root.get<bool>("preprocess.enable_voxel_downsample", true);
        config->preprocess.leafSize = root.get<double>("preprocess.leaf_size", 0.5);

        config->icp.maxIterations = root.get<int>("icp.max_iterations", 80);
        config->icp.maxCorrespondenceDistance = root.get<double>("icp.max_correspondence_distance", 5.0);
        config->icp.transformationEpsilon = root.get<double>("icp.transformation_epsilon", 1e-8);
        config->icp.euclideanFitnessEpsilon = root.get<double>("icp.euclidean_fitness_epsilon", 1e-6);

        const std::string thicknessMethod =
            root.get<std::string>("measurement.thickness_method", "nearest_between_surfaces");
        if (!ParseThicknessMethod(thicknessMethod, &config->thicknessMethod))
        {
            if (error != NULL)
            {
                *error = "measurement.thickness_method must be nearest_between_surfaces or tangent_plane_projection";
            }
            return false;
        }

        config->templateCylinder.axisPoint = ReadPoint(root.get_child("template_cylinder.axis_point"));
        config->templateCylinder.axisDirection = ReadPoint(root.get_child("template_cylinder.axis_direction"));

        config->templateFeaturePoints.clear();
        const boost::property_tree::ptree& featureNodes = root.get_child("template_feature_points");
        boost::property_tree::ptree::const_iterator it = featureNodes.begin();
        for (; it != featureNodes.end(); ++it)
        {
            config->templateFeaturePoints.push_back(ReadPoint(it->second));
        }

        config->output.resultPath = root.get<std::string>("output.result_path", "thickness_result.txt");
        config->output.resultPath = Utf8ToLocalAnsi(config->output.resultPath);

        if (config->pointCloud.innerTemplateCloudPath.empty()
            || config->pointCloud.outerTemplateCloudPath.empty())
        {
            if (error != NULL)
            {
                *error = "point_cloud requires inner_template_cloud_path and outer_template_cloud_path "
                         "(or legacy template_cloud_path for both)";
            }
            return false;
        }

        if (config->templateFeaturePoints.size() != 2)
        {
            std::ostringstream oss;
            oss << "template_feature_points must contain exactly 2 points, actual: "
                << config->templateFeaturePoints.size();
            if (error != NULL)
            {
                *error = oss.str();
            }
            return false;
        }

        if (config->preprocess.meanK < 1)
        {
            if (error != NULL)
            {
                *error = "preprocess.mean_k must be >= 1";
            }
            return false;
        }

        if (config->preprocess.leafSize <= 0.0)
        {
            if (error != NULL)
            {
                *error = "preprocess.leaf_size must be > 0";
            }
            return false;
        }

        if (ToEigen(config->templateCylinder.axisDirection).norm() <= 1e-12)
        {
            if (error != NULL)
            {
                *error = "template_cylinder.axis_direction must be non-zero";
            }
            return false;
        }
    }
    catch (const std::exception& ex)
    {
        if (error != NULL)
        {
            *error = ex.what();
        }
        return false;
    }

    return true;
}
