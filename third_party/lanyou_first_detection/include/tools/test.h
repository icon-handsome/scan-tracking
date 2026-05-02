
#ifndef TEST_H
#define TEST_H

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include "utils/Utils.h"
#include "log_manager/LogMacros.h"

using namespace std;

#define FILE_DIR "../data/"

/****************************
 * @brief 生成指定轴向、半径和高度的圆柱点云
 * @param center        圆柱中心点
 * @param radius        圆柱半径
 * @param height        圆柱高度
 * @param axis          圆柱轴向
 * @param num_points    点云数量
 * @return pcl::PointCloud<pcl::PointXYZ>::Ptr 圆柱点云指针
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr generateCylinderPointCloud(
                                    const pcl::PointXYZ& center,
                                    float radius,
                                    float height,
                                    const pcl::PointXYZ& axis) 
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    int step = 1;    //角度步长


    for (float h = -height/2; h < height/2; h+=5.0f)
    {
        for(int angle = 0; angle < 360; angle+=step)
        {
            float angle_rad = angle * M_PI / 180.0f;
            float x = center.x + h;
            float y = center.y + radius * cos(angle_rad);
            float z = center.z + radius * sin(angle_rad);;
            cloud->push_back(pcl::PointXYZ(x, y, z));
        }
    
    }


    return cloud;
}

/************************************************************
 * @brief 生成指定平面参数、范围的平面点云
 * @param plane_params 平面参数 (ax + by + cz + d = 0)
 * @param min_range    点云范围最小值 (x, y, z)
 * @param max_range    点云范围最大值 (x, y, z)
 * @return pcl::PointCloud<pcl::PointXYZ>::Ptr 平面点云指针
 ************************************************************/
pcl::PointCloud<pcl::PointXYZ>::Ptr generatePlanePointCloud(
                        const Eigen::Vector4f& plane_params,
                        const Eigen::Vector3f& min_range,
                        const Eigen::Vector3f& max_range)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);


    for (float x = min_range.x(); x < max_range.x(); x+=0.02f)
    {
        for(float y = min_range.y(); y < max_range.y(); y+=0.02f)
        {
            float z = (-plane_params(0) * x - plane_params(1) * y - plane_params(3)) / plane_params(2);
            for(float _z = z-0.03f; _z < z+0.03f; _z+=0.01f)
            {
                cloud->push_back(pcl::PointXYZ(x, y, _z));
            }
        }
    }


    return cloud;
}   

pcl::PointCloud<pcl::PointXYZ>::Ptr generateCircularPointCloud(
                                    const pcl::PointXYZ& center,
                                    float radius,
                                    float radius_step)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    float center_y = center.y;
    float center_z = center.z;

    for(int _x = 4500; _x < 7000; _x+=450)
    {
        for(int angle = 0; angle < 360; angle+=radius_step)
        {
            float angle_rad = angle * M_PI / 180.0f;
            for (int _r = radius; _r < radius+120; _r+=2)
            {
                
                    float x = _x;
                    float y = center.y + _r * cos(angle_rad);
                    float z = center_z + _r * sin(angle_rad);
                    cloud->push_back(pcl::PointXYZ(x, y, z));
                    cloud->push_back(pcl::PointXYZ(x+1.0f, y, z));
                    cloud->push_back(pcl::PointXYZ(x+2.0f, y, z));
                
            }
            
            
        }
    }


    return cloud;
}


void generateCylinder()
{
    // 定义圆柱参数
    pcl::PointXYZ center(5752.0f, 5052.85f, 1227.12f);
    float radius = 520.0f;
    float height = 3366.0f;
    pcl::PointXYZ axis(1.0f, 0.0f, 0.0f); // Z轴

    // 生成圆柱点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_cloud = generateCylinderPointCloud(center, radius, height, axis);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_cloud1 = generateCylinderPointCloud(center, radius+0.15, height, axis);

    // pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud1 = generatePlanePointCloud(Eigen::Vector4f(0.0f, 0.0f, 1.0f, 2.0f),
    //                                                                             Eigen::Vector3f(-1.0f, -1.0f, -1.0f),
    //                                                                             Eigen::Vector3f(1.0f, 1.0f, 1.0f));

    // pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud2 = generatePlanePointCloud(Eigen::Vector4f(0.0f, 0.0f, 1.0f, -2.0f),
    //                                                                             Eigen::Vector3f(-1.0f, -1.0f, -1.0f),
    //                                                                             Eigen::Vector3f(1.0f, 1.0f, 1.0f));                                                                            
                                                                                

    pcl::PointCloud<pcl::PointXYZ>::Ptr circular_cloud = generateCircularPointCloud(pcl::PointXYZ(4400.0f, 5052.85f, 1227.12f), radius, 1.0f);

    *cylinder_cloud += *circular_cloud;

    pcl::io::savePLYFileASCII(FILE_DIR"cylinder_point_cloud2.ply", *cylinder_cloud);

    // // 合并点云
    // *cylinder_cloud += *cylinder_cloud1;
    // *cylinder_cloud += *plane_cloud1;
    // *cylinder_cloud += *plane_cloud2;
    
    // // 保存点云到PLY文件
    
    // pcl::io::savePLYFileASCII(FILE_DIR"cylinder_point_cloud1.ply", *cylinder_cloud);
    // cout << "Saved " << cylinder_cloud->points.size() << " data points to cylinder_point_cloud.ply." << endl;
}
                                    


void addPointCloud()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);

    if(pcl::io::loadPLYFile(FILE_DIR"cylinder_point_cloud2.ply", *cloud) == -1)
    {
        LOG_ERROR("Failed to load cylinder_point_cloud2.ply.");
        return;
    }
    if(pcl::io::loadPLYFile(FILE_DIR"vertice.ply", *cloud1) == -1)
    {
        LOG_ERROR("Failed to load vertice.ply.");
        return;
    }


    *cloud += *cloud1;
    pcl::io::savePLYFileASCII(FILE_DIR"sample.ply", *cloud);
}


// void testverticaledgetilt(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
// {
//     //高程滤波
//     pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cylinder_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr raw_sphere_cloud(new pcl::PointCloud<pcl::PointXYZ>);

//     //获取点云三围
//     Eigen::Vector4f min_pt, max_pt;
//     pcl::getMinMax3D(*cloud, min_pt, max_pt);

//     ElevationBilateralFilter(cloud, raw_cylinder_cloud, 0.5f, 10.0f);



// }


// 测试沿圆柱截面

#endif // TEST_H









