#include <iostream>
#include <string>

#include <Windows.h>

#include "Config.h"
#include "ThicknessMeasurement.h"

namespace
{
std::string Utf8ToLocalAnsi(const std::string& input)
{
    if (input.empty())
    {
        return input;
    }

    int wideCount = MultiByteToWideChar(
        CP_UTF8,
        MB_ERR_INVALID_CHARS,
        input.data(),
        static_cast<int>(input.size()),
        NULL,
        0);
    if (wideCount <= 0)
    {
        return input;
    }

    std::wstring wide;
    wide.resize(wideCount);
    MultiByteToWideChar(
        CP_UTF8,
        MB_ERR_INVALID_CHARS,
        input.data(),
        static_cast<int>(input.size()),
        &wide[0],
        wideCount);

    int ansiCount = WideCharToMultiByte(
        CP_ACP,
        0,
        wide.data(),
        static_cast<int>(wide.size()),
        NULL,
        0,
        NULL,
        NULL);
    if (ansiCount <= 0)
    {
        return input;
    }

    std::string output;
    output.resize(ansiCount);
    WideCharToMultiByte(
        CP_ACP,
        0,
        wide.data(),
        static_cast<int>(wide.size()),
        &output[0],
        ansiCount,
        NULL,
        NULL);
    return output;
}
}

int main(int argc, char** argv)
{
    const std::string configPath = Utf8ToLocalAnsi(
        argc > 1 ? argv[1] : "config/thickness_config.json");
    const std::string innerScanCloudPath = Utf8ToLocalAnsi(
        argc > 2 ? argv[2] : "input/inner_surface_sample.pcd");
    const std::string outerScanCloudPath = Utf8ToLocalAnsi(
        argc > 3 ? argv[3] : "input/outer_surface_sample.pcd");

    ThicknessConfig config;
    std::string error;
    if (!LoadConfig(configPath, &config, &error))
    {
        std::cerr << "Load config failed: " << error << std::endl;
        return 1;
    }

    config.pointCloud.innerScanCloudPath = innerScanCloudPath;
    config.pointCloud.outerScanCloudPath = outerScanCloudPath;

    ThicknessResult result;
    if (!MeasureThickness(config, &result, &error))
    {
        std::cerr << "Measure thickness failed: " << error << std::endl;
        return 2;
    }

    if (!SaveResult(config.output.resultPath, result, &error))
    {
        std::cerr << "Save result failed: " << error << std::endl;
        return 3;
    }

    std::cout.precision(12);
    std::cout << "Inner ICP fitness score: " << result.innerIcpFitnessScore << std::endl;
    std::cout << "Outer ICP fitness score: " << result.outerIcpFitnessScore << std::endl;
    std::cout << "Thickness method: " << result.thicknessMethod << std::endl;
    std::cout << "Thickness: " << result.thickness << std::endl;
    std::cout << "Result saved to: " << config.output.resultPath << std::endl;
    return 0;
}
