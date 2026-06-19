#include "BevelMeasurement.h"

#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <cctype>
#include <iostream>
#include <string>
#include <vector>
#include <windows.h>

namespace
{
bool hasPcdExtension(const std::string& path)
{
    if (path.size() < 4)
    {
        return false;
    }

    std::string ext = path.substr(path.size() - 4);
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    return ext == ".pcd";
}

std::string joinPath(const std::string& dir, const std::string& file)
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

std::vector<std::string> listPcdFiles(const std::string& dir)
{
    std::vector<std::string> files;
    const std::string searchPath = joinPath(dir, "*.pcd");

    WIN32_FIND_DATAA findData;
    HANDLE findHandle = FindFirstFileA(searchPath.c_str(), &findData);
    if (findHandle == INVALID_HANDLE_VALUE)
    {
        return files;
    }

    do
    {
        if ((findData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) == 0)
        {
            files.push_back(joinPath(dir, findData.cFileName));
        }
    } while (FindNextFileA(findHandle, &findData));

    FindClose(findHandle);
    std::sort(files.begin(), files.end());
    return files;
}
}

// Measurement method is configured in config.txt: plane_fit or direct_points.
int main()
{
    const std::string cloudDir = "data/test_data";
    const std::string configPath = "config.txt";
    //const std::string templateDir = "D:/3 Data/11 LanYou_S1/0 Template/Po_Kou_Ce_Liang/";

    std::vector<bevel::CloudT::Ptr> rawClouds;

    const std::vector<std::string> pcdFiles = listPcdFiles(cloudDir);
    if (pcdFiles.empty())
    {
        std::cerr << "No PCD files found in: " << cloudDir << "\n";
        return 2;
    }

    for (std::vector<std::string>::const_iterator it = pcdFiles.begin(); it != pcdFiles.end(); ++it)
    {
        if (!hasPcdExtension(*it))
        {
            continue;
        }

        bevel::CloudT::Ptr cloud(new bevel::CloudT);
        if (pcl::io::loadPCDFile<bevel::PointT>(*it, *cloud) != 0)
        {
            std::cerr << "Failed to load PCD: " << *it << "\n";
            return 2;
        }

        if (cloud->empty())
        {
            std::cerr << "Loaded PCD is empty: " << *it << "\n";
            return 2;
        }

        rawClouds.push_back(cloud);
        std::cout << "Loaded PCD: " << *it << ", points=" << cloud->size() << "\n";
    }

    if (rawClouds.empty())
    {
        std::cerr << "Loaded PCD files, but no valid point cloud exists: " << cloudDir << "\n";
        return 2;
    }

    std::cout << "Total loaded PCD files: " << rawClouds.size() << "\n";

	float angleDeg_avr = 0.0f;
	float length_avr   = 0.0f;
	int   count_t      = rawClouds.size();
    for (size_t i = 0; i < rawClouds.size(); ++i)
    {
        // Compute bevel angle and blunt edge length.
        const bevel::BevelMeasurementResult result = bevel::solveBevelFromRawCloud(rawClouds[i], configPath);
        if (!result.ok)
        {
            std::cerr << "Solve failed: " << result.message << "\n";
            return 3;
        }

		angleDeg_avr += result.angleDeg;
		length_avr   += result.length;

        std::cout << "Cloud " << (i + 1) << "/" << rawClouds.size() << "\n";
        std::cout << "angle_deg=" << result.angleDeg << "\n";                // Bevel angle
        std::cout << "length=" << result.length << "\n";                     // Blunt edge length
        std::cout << "icp_fitness=" << result.icpFitness << "\n";
    }

	if (count_t > 0)
	{
		angleDeg_avr /= (float)count_t;
		length_avr   /= (float)count_t;
	}

	// Final bevel measurement result
	std::cout << "Final bevel measurement result:" << std::endl;
	std::cout << "angle_deg = "      << angleDeg_avr << std::endl;              // Bevel angle
	std::cout << "length = "         << length_avr   << std::endl;              // Blunt edge length
    
	return 0;
}