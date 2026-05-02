#include "detection/third/ThirdOutSurfaceDetection.h"

#include <algorithm>
#include <vector>
#include <utility>
#include <string>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "utils/Utils.h"
#include "log_manager/LogMacros.h"

// 构造函数：初始化第三检测位参数缓存。
ThirdOutSurfaceDetection::ThirdOutSurfaceDetection() : thirdpose_params_(), points_model_() {}

bool ThirdOutSurfaceDetection::Detect(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    // 1) 输入点云有效性检查。
    if (!cloud || cloud->empty()) {
        LOG_ERROR("Third out-surface cloud is empty!");
        return false;
    }

    // 2) 体素下采样并计算法向，为后续圆柱分割准备输入。
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    Utils::VoxelGridDownSample(cloud, cloud_filtered, 5.0f);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setInputCloud(cloud_filtered);
    ne.setSearchMethod(tree);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    // 3) RANSAC + 法向约束分割圆筒主体。
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);

    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg_cylinder;
    seg_cylinder.setModelType(pcl::SACMODEL_CYLINDER);
    seg_cylinder.setMethodType(pcl::SAC_RANSAC);
    seg_cylinder.setDistanceThreshold(2.0);
    seg_cylinder.setInputCloud(cloud_filtered);
    seg_cylinder.setInputNormals(cloud_normals);
    seg_cylinder.segment(*inliers_cylinder, *coefficients_cylinder);
    if (inliers_cylinder->indices.empty()) {
        LOG_ERROR("未能检测到圆筒主体！");
        return false;
    }

    // 4) 提取圆柱轴向，作为后续圆环法向与间距的参考方向。
    Eigen::Vector3f cylinder_axis(coefficients_cylinder->values[3],
                                  coefficients_cylinder->values[4],
                                  coefficients_cylinder->values[5]);
    cylinder_axis.normalize();

    // 5) 从点云中剔除圆筒内点，得到圆环候选点云。
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rings_raw(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(true);
    extract.filter(*cloud_rings_raw);

    // 6) 统计滤波去除离群点。
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rings(new pcl::PointCloud<pcl::PointXYZ>);
    Utils::StatisticalOutlierRemoval(cloud_rings_raw, cloud_rings, 50);

    // 7) 保存预处理后的圆环候选点云。
    pcl::io::savePLYFile("circle_raw.ply", *cloud_rings);

    // 8) 欧式聚类分离候选圆环。
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr cluster_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    cluster_tree->setInputCloud(cloud_rings);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(20.0);
    ec.setMinClusterSize(500);
    ec.setMaxClusterSize(50000);
    ec.setSearchMethod(cluster_tree);
    ec.setInputCloud(cloud_rings);
    ec.extract(cluster_indices);

    LOG_INFO("聚类完成，检测到候选聚类数量：" << cluster_indices.size());

    // 9) 对每个聚类执行 3D 圆拟合，并按法向一致性筛选。
    pcl::SACSegmentation<pcl::PointXYZ> seg_circle;
    seg_circle.setOptimizeCoefficients(true);
    seg_circle.setModelType(pcl::SACMODEL_CIRCLE3D);
    seg_circle.setMethodType(pcl::SAC_RANSAC);
    seg_circle.setDistanceThreshold(20.0);
    seg_circle.setRadiusLimits(600.0f, 650.0f);

    std::vector<pcl::ModelCoefficients> all_circle_coefficients;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> all_circle_clouds;
    int ring_count = 0;

    for (const auto& indices : cluster_indices) {
        // 9.1) 组装单个聚类点云。
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (int idx : indices.indices) {
            cloud_cluster->points.push_back(cloud_rings->points[idx]);
        }

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        pcl::ModelCoefficients::Ptr coefficients_circle(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_circle(new pcl::PointIndices);

        // 9.2) 在聚类内部拟合圆环模型。
        seg_circle.setInputCloud(cloud_cluster);
        seg_circle.segment(*inliers_circle, *coefficients_circle);

        if (inliers_circle->indices.empty()) {
            continue;
        }

        Eigen::Vector3f circle_normal(coefficients_circle->values[4],
                                      coefficients_circle->values[5],
                                      coefficients_circle->values[6]);
        circle_normal.normalize();

        // 9.3) 校验圆环法向与圆柱轴向夹角，过滤异常聚类。
        float angle_diff = 0.0f;
        Utils::ComputeAngleBetweenVectors(cylinder_axis, circle_normal, angle_diff);

        if (angle_diff > 15.0f && angle_diff < 165.0f) {
            LOG_WARN("聚类法向与轴向偏差过大 (" << angle_diff << " deg)，跳过该检测。");
            continue;
        }

        // 9.4) 提取当前圆环内点。
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_ring(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> cluster_extract;
        cluster_extract.setInputCloud(cloud_cluster);
        cluster_extract.setIndices(inliers_circle);
        cluster_extract.setNegative(false);
        cluster_extract.filter(*current_ring);

        if (current_ring->size() > 500) {
            ++ring_count;
            LOG_INFO("检测到有效圆环 " << ring_count << "，内点数：" << current_ring->size()
                     << "，偏角：" << angle_diff);
            all_circle_coefficients.push_back(*coefficients_circle);
            all_circle_clouds.push_back(current_ring);
        }
    }

    // 10) 保存每个有效圆环点云。
    for (size_t i = 0; i < all_circle_clouds.size(); ++i) {
        pcl::io::savePLYFile("../data/circle/circle_" + std::to_string(i) + ".ply", *all_circle_clouds[i]);
    }

    // 11) 计算并回写钢圈间距结果。
    thirdpose_params_.steel_ring_distance_tol.clear();
    if (all_circle_coefficients.size() < 2) {
        LOG_WARN("检测到的圆环数量不足（<2），无法计算间距。");
        return true;
    }

    // 11.1) 使用圆心在轴向上的投影进行排序。
    std::vector<std::pair<float, int>> projections;
    for (size_t i = 0; i < all_circle_coefficients.size(); ++i) {
        Eigen::Vector3f center(all_circle_coefficients[i].values[0],
                               all_circle_coefficients[i].values[1],
                               all_circle_coefficients[i].values[2]);
        float proj = center.dot(cylinder_axis);
        projections.push_back({proj, static_cast<int>(i)});
    }

    std::sort(projections.begin(), projections.end());

    // 11.2) 计算相邻圆环轴向间距。
    float total_distance = 0.0f;
    for (size_t i = 0; i + 1 < projections.size(); ++i) {
        float dist = std::abs(projections[i + 1].first - projections[i].first);
        LOG_INFO("圆环 " << projections[i].second << " 与圆环 " << projections[i + 1].second
                 << " 之间的轴向间距: " << dist);
        thirdpose_params_.steel_ring_distance_tol.push_back(dist);
        total_distance += dist;
    }
    if (projections.size() > 2) {
        LOG_INFO("平均间距: " << total_distance / static_cast<float>(projections.size() - 1));
    }

    return true;
}

// 只读访问参数。
const ThirdPoseDetectionParams& ThirdOutSurfaceDetection::GetParams() const
{
    return thirdpose_params_;
}

// 可写访问参数。
ThirdPoseDetectionParams& ThirdOutSurfaceDetection::GetParams()
{
    return thirdpose_params_;
}
