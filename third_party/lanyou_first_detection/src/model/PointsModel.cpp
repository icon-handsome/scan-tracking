#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/mls.h>
#include <iomanip> 
#include <cmath>    
#include <pcl/common/transforms.h> 
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h> 
#include <Eigen/Eigenvalues>

#include "model/PointsModel.h"
#include "utils/Utils.h"
#include "utils/tic_toc.h"
#include "log_manager/LogMacros.h"

using namespace std;

namespace {

inline bool IsFiniteXYZ(const float x, const float y, const float z)
{
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

float MedianOf(std::vector<float> values)
{
    if (values.empty()) return 0.0f;
    const size_t n = values.size();
    std::nth_element(values.begin(), values.begin() + n / 2, values.end());
    float med = values[n / 2];
    if ((n % 2) == 0)
    {
        std::nth_element(values.begin(), values.begin() + (n / 2 - 1), values.end());
        med = 0.5f * (med + values[n / 2 - 1]);
    }
    return med;
}

inline float HuberWeight(const float abs_residual, const float delta)
{
    return (abs_residual <= delta || abs_residual < 1e-8f) ? 1.0f : (delta / abs_residual);
}

inline float HuberCost(const float abs_residual, const float delta)
{
    if (abs_residual <= delta) return 0.5f * abs_residual * abs_residual;
    return delta * (abs_residual - 0.5f * delta);
}

void BuildOrthonormalBasis(const Eigen::Vector3f& d, Eigen::Vector3f& u, Eigen::Vector3f& v)
{
    Eigen::Vector3f helper = (std::abs(d.z()) < 0.9f) ? Eigen::Vector3f::UnitZ() : Eigen::Vector3f::UnitX();
    u = d.cross(helper);
    if (u.norm() < 1e-6f) u = d.cross(Eigen::Vector3f::UnitY());
    u.normalize();
    v = d.cross(u).normalized();
}

inline float RadialResidual(const Eigen::Vector3f& p, const Eigen::Vector3f& c, const Eigen::Vector3f& d, const float r)
{
    const Eigen::Vector3f q = p - c;
    const float t = q.dot(d);
    return (q - t * d).norm() - r;
}

bool ComputeTrimmedProjectionRange(std::vector<float>& projections,
                                   float& min_proj_out,
                                   float& max_proj_out,
                                   size_t& used_points)
{
    if (projections.size() < 20) return false;
    std::sort(projections.begin(), projections.end());

    const size_t n = projections.size();
    const size_t trim_by_ratio = static_cast<size_t>(static_cast<double>(n) * 0.001); // 两端各0.1%
    const size_t trim = std::min<size_t>(50, trim_by_ratio); // 限制最大裁剪，避免长度被明显缩短
    if (2 * trim >= n) return false;

    min_proj_out = projections[trim];
    max_proj_out = projections[n - 1 - trim];
    used_points = n - 2 * trim;
    return min_proj_out <= max_proj_out;
}

} // namespace

/*******************************************
 * @brief                   欧式聚类提取
 * @param cloud             输入点云指针
 * @param cluster_clouds    输出聚类点云指针向量
 * @param cluster_tolerance 聚类容差，默认 0.02
 * @param min_cluster_size  最小聚类点数，默认 100
 * @param max_cluster_size  最大聚类点数，默认 20000
 *******************************************/
bool PointsModel::EuclideanClusterExtraction(
    const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud,
    std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>& cluster_clouds,
    float cluster_tolerance,
    int min_cluster_size)
{

    npt_tree_->setInputCloud(cloud);

    // 创建聚类对象
    pcl::EuclideanClusterExtraction<pcl::PointNormal> ec;
    std::vector<pcl::PointIndices> cluster_indices; // 存储每个聚类的点索引
    
    ec.setInputCloud(cloud);
    ec.setSearchMethod(npt_tree_);
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_cluster_size);
    ec.extract(cluster_indices);    // 执行聚类获取索引

    if (cluster_indices.empty()) {
        LOG_WARN("未检测到有效聚类！");
        return false; // 无聚类但逻辑正常，返回false
    }

    // 遍历所有聚类索引，提取点云
    pcl::ExtractIndices<pcl::PointNormal> extract;
    extract.setInputCloud(cloud);
    extract.setNegative(false); // false=提取索引内的点，true=提取索引外的点

    // 从聚类索引中提取点云
    for (const auto& indices : cluster_indices)
    {   
        // 当前聚类点云指针
        auto cluster = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();
        // 设置提取的索引
        pcl::PointIndices::Ptr indices_ptr(new pcl::PointIndices(indices));
        extract.setIndices(indices_ptr);

        // 执行提取
        extract.filter(*cluster);
        cluster_clouds.push_back(cluster);
    }
    return true;
}

/**
 * @brief 将圆柱点云的轴向旋转到与目标轴对齐
 * 
 * @param cloud_in 输入点云 (会被直接修改，如果想保留原数据请先拷贝)
 * @param cylinder_model 圆柱模型
 * @param target_axis 目标轴向 (例如: [1, 0, 0] 代表X轴)
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr PointsModel::RotatePointCloudAlignToAxis(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
    const CylinderModel& cylinder_model,
    const Eigen::Vector3f& target_axis
    )
{
     if (!cloud_in || cloud_in->empty()) {
        LOG_WARN("Input point cloud is empty or null.");
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>()); // 返回空点云对象
    }

    // 归一化（防御性编程：避免零向量）
    if (cylinder_model.axis.norm() < 1e-6 || target_axis.norm() < 1e-6) {
        LOG_ERROR("Invalid axis vector (near zero norm).");
        return cloud_in; // 或抛异常
    }

    Eigen::Vector3f v_start = cylinder_model.axis.normalized();
    Eigen::Vector3f v_end = target_axis.normalized();


    // 构建平移变换：T = -origin （所有点减去坐标轴上点的偏移）
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() = -cylinder_model.center; // 核心：平移向量 = -center

    // 输出点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud_in, *cloud_out, transform);


    
    // 处理平行/反向情况（避免Quaternion数值不稳定）
    if (v_start.dot(v_end) > 0.9999f) { // 几乎同向
        // 无需旋转，直接返回副本
        return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(*cloud_in));
    }
    if (v_start.dot(v_end) < -0.9999f) { // 几乎反向
        // 绕任意垂直轴旋转180度（例如X轴）
        transform = Eigen::Affine3f::Identity();
        transform.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()));
        auto result = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud_out, *result, transform);
        return result;
    }

    // 核心旋转
    transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::Quaternionf().setFromTwoVectors(v_start, v_end));
    
    auto aligned_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud_out, *aligned_cloud, transform);
    return aligned_cloud;
}

/**
 * @brief 从对齐后的圆柱点云中提取圆环切片
 * 
 * @param aligned_cloud 对齐后的圆柱点云指针
 * @param slice_thickness 每个圆环轴向厚度，默认 5.0f
 * @param slice_center_step 切片中心间隔，默认 10.0f
 * 
 * @return std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> 圆环切片点云指针向量
 */
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> PointsModel::ExtractRingSlices(
const pcl::PointCloud<pcl::PointXYZ>::Ptr& aligned_cloud,
float slice_thickness, 
float slice_center_step)
{
     if (!aligned_cloud || aligned_cloud->empty()) {
        LOG_ERROR("Input cloud is empty!");
        return {};
    }

    // 1. 计算Z轴范围（自动跳过NaN点）
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*aligned_cloud, min_pt, max_pt);
    
    LOG_INFO(std::fixed << std::setprecision(2)
             << "[Slice] Z range: [" << min_pt.z << ", " << max_pt.z
             << "] mm | Total length: " << (max_pt.z - min_pt.z) << " mm");

    // 2. 验证切片可行性
    if ((max_pt.z - min_pt.z) < slice_thickness) {
        LOG_WARN("Cloud length (" << (max_pt.z - min_pt.z)
                 << "mm) < slice thickness (" << slice_thickness << "mm). No slices generated.");
        return {};
    }

    // 3. 计算切片参数
    float half_thickness = slice_thickness / 2.0f;
    float first_center = min_pt.z + half_thickness; // 首个切片中心（确保下边界≥min_z）
    float last_center = max_pt.z - half_thickness;  // 末个切片中心（确保上边界≤max_z）
    
    if (first_center > last_center) {
        LOG_WARN("Insufficient range for even one slice.");
        return {};
    }

    // 4. 计算切片数量（向上取整确保覆盖）
    int num_slices = static_cast<int>(std::ceil((last_center - first_center) / slice_center_step)) + 1;
    LOG_INFO("[Slice] Generating " << num_slices << " ring slices (thickness="
             << slice_thickness << "mm, step=" << slice_center_step << "mm)");

    // 5. 生成切片
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> ring_slices;
    ring_slices.reserve(num_slices);
    
    for (int i = 0; i < num_slices; ++i) {
        float center_z = first_center + i * slice_center_step;
        float z_low = center_z - half_thickness;
        float z_high = center_z + half_thickness;
        
        // 边界保护（防止浮点误差越界）
        z_low = std::max(z_low, min_pt.z);
        z_high = std::min(z_high, max_pt.z);
        
        // PassThrough滤波（高效提取Z范围点）
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(aligned_cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(z_low, z_high);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr ring(new pcl::PointCloud<pcl::PointXYZ>);
        pass.filter(*ring);
        
        // 可选：记录切片信息（调试用）
        if (i % 10 == 0 || i == num_slices - 1) {
            LOG_INFO("Slice " << i << ": Z=[" << z_low << ", " << z_high
                     << "] mm | Points: " << ring->points.size());
        }
        
        ring_slices.push_back(ring);
    }
    
    LOG_INFO("[Slice] Done. Total slices: " << ring_slices.size());
    return ring_slices;
}

/**
 * @brief               将点云在Z轴方向上展平
 * @param cloud_in         输入点云指针
 * @return                 展平后的点云指针
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr PointsModel::FlattenZ_Simple(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in)
{
    auto cloud_out = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(*cloud_in)); // 深拷贝避免修改原数据
    for (auto& pt : cloud_out->points) {
        pt.z = 0.0f; // 仅修改 z，X/Y 保留
    }
    return cloud_out;
}


/*****************************************
 * @brief           计算点云法向
 * @param cloud     输入点云指针
 * @param normals   输出法向点云指针
 *
 * @return void
 *****************************************/
void PointsModel::ComputePointCloudNormal(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, 
                                    pcl::PointCloud<pcl::Normal>::Ptr &_normal, 
                                    pcl::PointCloud<pcl::PointNormal>::Ptr &_ncloud,
                                    const float radius)
{
    Utils::IsEmptyPoint(cloud);
    TicToc timer;
    timer.tic(); // 开始计时
    // 初始化输出点云指针
    if (!_normal) _normal.reset(new pcl::PointCloud<pcl::Normal>);
    if (!_ncloud) _ncloud.reset(new pcl::PointCloud<pcl::PointNormal>);
    _normal->clear();  // 清空残留数据
    _ncloud->clear();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_valid(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_valid->points.reserve(cloud->points.size());
    for (const auto& point : cloud->points) {
        if (IsFiniteXYZ(point.x, point.y, point.z) &&
            std::abs(point.x) < 5000.0f && std::abs(point.y) < 5000.0f && std::abs(point.z) < 5000.0f) {
            cloud_valid->points.push_back(point);
        }
    }
    cloud_valid->width = static_cast<std::uint32_t>(cloud_valid->points.size());
    cloud_valid->height = 1;
    cloud_valid->is_dense = true;
    if (cloud_valid->empty()) {
        LOG_ERROR("法向计算失败，过滤无效点后点云为空");
        return;
    }
    
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr local_tree(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setInputCloud(cloud_valid);
    ne.setSearchMethod(local_tree);
    ne.setKSearch(20);
    ne.compute(*_normal);
    if (_normal->points.empty())
    {
        LOG_ERROR("法向计算失败，点云为空或半径过小");
        return;
    }

    // 合并有效点和法向
    _ncloud->points.reserve(cloud_valid->points.size());
    pcl::concatenateFields(*cloud_valid, *_normal, *_ncloud);

    LOG_INFO("法向计算成功，耗时: " << timer.toc() << " 毫秒");
}

/*****************************************
 * @brief                对点云进行MLS平滑
 * @param cloud_in       输入点云指针
 * @param cloud_out      输出平滑后点云指针
 * @param search_radius  MLS邻域搜索半径
 * @param polynomial_order 多项式阶数（>=1）
 * @param polynomial_fit 是否启用多项式拟合（false时退化为一阶）
 * @return bool          是否平滑成功
 *****************************************/
bool PointsModel::SmoothPointCloudMLS(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,
                                      float search_radius,
                                      int polynomial_order,
                                      bool polynomial_fit)
{
    if (!cloud_in || cloud_in->empty()) {
        LOG_ERROR("SmoothPointCloudMLS 输入点云为空");
        return false;
    }
    if (search_radius <= 0.0f) {
        LOG_ERROR("SmoothPointCloudMLS search_radius 无效: " << search_radius);
        return false;
    }
    if (!cloud_out) {
        cloud_out.reset(new pcl::PointCloud<pcl::PointXYZ>);
    }

    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    mls.setInputCloud(cloud_in);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(search_radius);
    mls.setPolynomialOrder(polynomial_fit ? std::max(1, polynomial_order) : 1);
    mls.setComputeNormals(false);
    mls.process(*cloud_out);

    if (!cloud_out || cloud_out->empty()) {
        LOG_WARN("SmoothPointCloudMLS 输出为空，返回失败");
        return false;
    }
    return true;
}

/*****************************************
 * @brief                       从点云提取焊缝
 * @param cloud                 输入点云指针
 * @param weld_seam_cloud       输出焊缝点云指针
 * @param weld_seam_outliner    输出焊缝外点云指针
 * @param n_radius              法向计算半径(默认5.0)
 * @param curvature_threshold   输入曲率阈值(默认0.1)
 * @return bool                 是否成功提取焊缝
 *****************************************/
bool PointsModel::ExtractWeldSeamByCurvature(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr& weld_seam_cloud,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr& weld_seam_outliner,
                                const float n_radius,
                                const float curvature_threshold)
{
    // 初始化输出点云指针
    if (!weld_seam_cloud) weld_seam_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    if (!weld_seam_outliner) weld_seam_outliner.reset(new pcl::PointCloud<pcl::PointXYZ>);
    weld_seam_cloud->clear();  // 清空残留数据
    weld_seam_outliner->clear();

    // 计算点云法向
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr ncloud(new pcl::PointCloud<pcl::PointNormal>);
    ComputePointCloudNormal(cloud, normals, ncloud, n_radius);

     // 根据曲率阈值提取点
    pcl::PointCloud<pcl::PointXYZ>::Ptr seam_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t i = 0; i < normals->points.size(); ++i) {
        // normals->points[i].curvature 值域通常在 0-1 之间
        // 平面曲率为0，尖锐边缘曲率大
        if (normals->points[i].curvature > curvature_threshold) {
            seam_cloud->points.push_back(cloud->points[i]);
        }
    }

    seam_cloud->width = seam_cloud->points.size();
    seam_cloud->height = 1;
    seam_cloud->is_dense = true;

    if (seam_cloud->empty()) {
        LOG_WARN("未提取到任何点，请尝试降低曲率阈值或调整搜索半径。");
        return false;
    }

    LOG_INFO("提取焊缝点数: " << seam_cloud->points.size());

    // 保存结果
    std::string output_file = "../data/weld_seam.ply";
    pcl::io::savePLYFileASCII(output_file, *seam_cloud);
    LOG_INFO("结果已保存至: " << output_file);

    return true;
}

/*****************************************
 * @brief                   拟合圆柱模型
 * 
 * @param cloud             输入点云指针
 * @param cylinder_cloud    输出圆柱点云指针
 * @param cylinder_model    输出圆柱模型(轴上点，轴向，半径)
 * @param dist_threshold    点到圆柱距离阈值
 * @param radius_limits     半径范围限制
 * 
 * @return bool             是否成功拟合圆柱模型
 *****************************************/
bool PointsModel::FitCylinder(const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr& cylinder_cloud,
                        CylinderModel& cylinder_model,
                        const float dist_threshold,
                        std::vector<float> radius_limits)
{
    if (!cloud || cloud->empty()) {
        LOG_ERROR("输入点云为空，无法拟合圆柱");
        return false;
    }
    if (radius_limits.size() < 2 || radius_limits[0] <= 0.0f || radius_limits[1] <= radius_limits[0]) {
        LOG_ERROR("radius_limits 参数无效，需满足 [min, max] 且 min>0");
        return false;
    }

    // 声明点云和法向
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud(new pcl::PointCloud<pcl::PointXYZ>);
    normals->points.reserve(cloud->points.size());
    _cloud->points.reserve(cloud->points.size());
    pcl::copyPointCloud(*cloud, *_cloud);
    pcl::copyPointCloud(*cloud, *normals);

    TicToc timer;
    timer.tic();

    // RANSAC 初始拟合
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setRadiusLimits(radius_limits[0], radius_limits[1]);
    seg.setDistanceThreshold(dist_threshold);
    seg.setNormalDistanceWeight(0.5f);
    seg.setMaxIterations(200);
    seg.setProbability(0.99f);
    seg.setInputCloud(_cloud);
    seg.setInputNormals(normals);
    seg.segment(*inliers, *coefficients);

    if (coefficients->values.size() != 7) {
        LOG_ERROR("圆柱系数无效（需7个参数，实际" << coefficients->values.size() << "个）");
        return false;
    }
    if (inliers->indices.size() < 1000) {
        LOG_ERROR("圆柱内点数量过低，拟合失败!");
        return false;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr noisy_cylinder(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*_cloud, inliers->indices, *noisy_cylinder);
    Utils::StatisticalOutlierRemoval(noisy_cylinder, cylinder_cloud);

    // RANSAC 初始参数
    Eigen::Vector3f center(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    Eigen::Vector3f axis(coefficients->values[3], coefficients->values[4], coefficients->values[5]);
    if (!axis.allFinite() || axis.norm() <= 1e-6f) {
        LOG_ERROR("圆柱轴向无效！");
        return false;
    }
    axis.normalize();
    float radius = coefficients->values[6];
    if (!std::isfinite(radius) || radius <= 1e-6f) {
        LOG_ERROR("圆柱半径无效（" << radius << "）");
        return false;
    }

    // // 关闭 Huber 优化，避免二次优化导致轴线偏移；直接使用 RANSAC 拟合结果。
    // const bool huber_used = false;
    // const char* huber_status_reason = "Huber优化已禁用，使用RANSAC结果";

    // 回写优化后的圆柱模型
    cylinder_model.center = center;
    cylinder_model.axis = axis.normalized();
    cylinder_model.radius = radius;
    if (cylinder_model.radius <= 1e-6) {
        LOG_ERROR("圆柱半径无效（" << cylinder_model.radius << "）");
        return false;
    }

    // 投影计算圆柱高度
    Eigen::Vector3f origin(cylinder_model.center.x(), cylinder_model.center.y(), cylinder_model.center.z());
    Eigen::Vector3f dir(cylinder_model.axis.x(), cylinder_model.axis.y(), cylinder_model.axis.z());
    dir.normalize();

    auto computeProjectionRangeOnSurfaceRobust = [&](const pcl::PointCloud<pcl::PointNormal>::Ptr& src_cloud,
                                                     float radial_tol,
                                                     float normal_cos_thresh,
                                                     float& min_proj_out,
                                                     float& max_proj_out,
                                                     size_t& used_points) -> bool
    {
        if (!src_cloud || src_cloud->empty()) return false;
        std::vector<float> projections;
        projections.reserve(src_cloud->points.size());

        for (const auto& pt : src_cloud->points) {
            if (!IsFiniteXYZ(pt.x, pt.y, pt.z)) continue;
            Eigen::Vector3f vec(pt.x - origin.x(), pt.y - origin.y(), pt.z - origin.z());
            float proj = vec.dot(dir);
            Eigen::Vector3f radial_vec = vec - proj * dir;
            float radial_dist = radial_vec.norm();
            if (radial_dist < 1e-6f) continue;
            if (std::abs(radial_dist - static_cast<float>(cylinder_model.radius)) > radial_tol) continue;

            if (IsFiniteXYZ(pt.normal_x, pt.normal_y, pt.normal_z)) {
                Eigen::Vector3f n(pt.normal_x, pt.normal_y, pt.normal_z);
                float n_norm = n.norm();
                if (n_norm > 1e-6f) {
                    n /= n_norm;
                    Eigen::Vector3f radial_dir = radial_vec / radial_dist;
                    float cos_val = std::abs(n.dot(radial_dir));
                    if (cos_val < normal_cos_thresh) continue;
                }
            }

            projections.push_back(proj);
        }

        return ComputeTrimmedProjectionRange(projections, min_proj_out, max_proj_out, used_points);
    };

    auto computeProjectionRangeRawRobust = [&](const pcl::PointCloud<pcl::PointXYZ>::Ptr& src_cloud,
                                               float& min_proj_out,
                                               float& max_proj_out,
                                               size_t& used_points) -> bool
    {
        if (!src_cloud || src_cloud->empty()) return false;
        std::vector<float> projections;
        projections.reserve(src_cloud->points.size());
        for (const auto& pt : src_cloud->points) {
            if (!IsFiniteXYZ(pt.x, pt.y, pt.z)) continue;
            Eigen::Vector3f vec(pt.x - origin.x(), pt.y - origin.y(), pt.z - origin.z());
            projections.push_back(vec.dot(dir));
        }
        return ComputeTrimmedProjectionRange(projections, min_proj_out, max_proj_out, used_points);
    };

    const float radial_tolerance = std::max(2.0f * dist_threshold, 1.0f);
    const float normal_cos_threshold = 0.75f; // 法向与径向夹角<=约41度

    float min_proj = 0.0f, max_proj = 0.0f;
    size_t used_points = 0;

    bool has_range = computeProjectionRangeOnSurfaceRobust(
        cloud, radial_tolerance, normal_cos_threshold, min_proj, max_proj, used_points);
    if (!has_range || used_points < 100)
    {
        has_range = computeProjectionRangeRawRobust(noisy_cylinder, min_proj, max_proj, used_points);
    }
    if (!has_range || used_points < 100)
    {
        has_range = computeProjectionRangeRawRobust(cylinder_cloud, min_proj, max_proj, used_points);
    }
    if (!has_range)
    {
        LOG_ERROR("圆柱高度计算失败，未找到有效投影范围！");
        return false;
    }

    cylinder_model.height = max_proj - min_proj; // 圆柱高度（长度）

    // 更新 center 为几何中心
    float center_proj = (min_proj + max_proj) / 2.0f;
    Eigen::Vector3f center_projected = origin + center_proj * dir;
    cylinder_model.center.x() = center_projected.x();
    cylinder_model.center.y() = center_projected.y();
    cylinder_model.center.z() = center_projected.z();

    // // 生成拟合圆柱参数对应的规则圆柱模型点云（用于可视化对比）
    // auto generateCylinderModelCloud = [&](const CylinderModel& model) -> pcl::PointCloud<pcl::PointXYZ>::Ptr
    // {
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //     const Eigen::Vector3f axis_unit = model.axis.normalized();
    //     Eigen::Vector3f helper = (std::abs(axis_unit.z()) < 0.9f) ? Eigen::Vector3f::UnitZ() : Eigen::Vector3f::UnitX();
    //     Eigen::Vector3f u = axis_unit.cross(helper);
    //     if (u.norm() < 1e-6f) u = axis_unit.cross(Eigen::Vector3f::UnitY());
    //     u.normalize();
    //     const Eigen::Vector3f v = axis_unit.cross(u).normalized();

    //     const float half_h = static_cast<float>(model.height) * 0.5f;
    //     const int ring_steps = std::max(20, static_cast<int>(std::ceil(model.height / 2.0)));
    //     const int angle_steps = 360;
    //     model_cloud->points.reserve(static_cast<size_t>(ring_steps + 1) * static_cast<size_t>(angle_steps));

    //     for (int i = 0; i <= ring_steps; ++i)
    //     {
    //         const float t = static_cast<float>(i) / static_cast<float>(ring_steps);
    //         const float s = -half_h + t * (2.0f * half_h);
    //         const Eigen::Vector3f ring_center = model.center + s * axis_unit;
    //         for (int j = 0; j < angle_steps; ++j)
    //         {
    //             const float ang = 2.0f * static_cast<float>(M_PI) * static_cast<float>(j) / static_cast<float>(angle_steps);
    //             const Eigen::Vector3f radial = std::cos(ang) * u + std::sin(ang) * v;
    //             const Eigen::Vector3f p = ring_center + static_cast<float>(model.radius) * radial;
    //             model_cloud->points.emplace_back(p.x(), p.y(), p.z());
    //         }
    //     }
    //     model_cloud->width = model_cloud->points.size();
    //     model_cloud->height = 1;
    //     model_cloud->is_dense = true;
    //     return model_cloud;
    // };

    // pcl::PointCloud<pcl::PointXYZ>::Ptr fitted_cylinder_model_cloud = generateCylinderModelCloud(cylinder_model);
    // const std::string model_output_path = "../data/fit_cylinder_model.ply";
    // if (!fitted_cylinder_model_cloud->empty())
    // {
    //     pcl::io::savePLYFileBinary(model_output_path, *fitted_cylinder_model_cloud);
    //     LOG_INFO("拟合圆柱参数模型已保存: " << model_output_path
    //              << ", points=" << fitted_cylinder_model_cloud->points.size());
    // }

    cylinder_model_ = cylinder_model; // 存储拟合的圆柱模型

    LOG_INFO("\n----------圆柱拟合成功-----------\n center=(" << cylinder_model.center.x() 
             << ", " << cylinder_model.center.y() << ", " << cylinder_model.center.z()
             << ")\n axis=(" << cylinder_model.axis.x() << ", "
             << cylinder_model.axis.y() << ", " << cylinder_model.axis.z()
             << ")\n radius=" << cylinder_model.radius
             << ", height=" << cylinder_model.height
             << "\n fit_time_=" << timer.toc()<<"ms"<<"\n"
             <<"--------------------------------");

    return true;
}

/*****************************************
 * @brief 计算所有点云投影到圆柱轴线的投影高度
 * @param cloud  输入点云指针
 * @param center 轴线上的一点（原点）
 * @param axis   轴线方向（自动归一化）
 * @param height 输出投影高度（沿轴向的标量距离）
 *
 * @return void
 *****************************************/
void PointsModel::GetProjectedHeightOnAxis(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                                     const Eigen::Vector3f& center, 
                                     const Eigen::Vector3f& axis,
                                     float& height)
{
    // 检查点云是否为空
    if (cloud->empty())
    {
        height = 0.0f;
        return;
    }

    Eigen::Vector3f axis_unit = axis.normalized();
    float min_proj = std::numeric_limits<float>::max();
    float max_proj = -std::numeric_limits<float>::max();

    for (const auto& p : cloud->points)
    {
        Eigen::Vector3f p_vec(p.x, p.y, p.z);
        float proj = (p_vec - center).dot(axis_unit);
        if (proj < min_proj) min_proj = proj;
        if (proj > max_proj) max_proj = proj;
    }

    height = max_proj - min_proj;
    LOG_INFO("封头投影高度为：" << height << " min_proj:" << min_proj << " max_proj:" << max_proj);
}

/*****************************************
 * @brief  计算圆柱筒体表面点云距离圆柱轴线的径向距离（单个点）
 * @param point                 输入点云指针
 * @param cylinder_model        圆柱模型
 */
float PointsModel::ComputeRadialDistanceToAxis(const pcl::PointXYZ &point, const CylinderModel &cylinder_model)
{
    Eigen::Vector3f p_vec(point.x, point.y, point.z);
    Eigen::Vector3f center(cylinder_model.center.x(), cylinder_model.center.y(), cylinder_model.center.z());
    Eigen::Vector3f axis(cylinder_model.axis.x(), cylinder_model.axis.y(), cylinder_model.axis.z());
    axis.normalize();
    float t = (p_vec - center).dot(axis); // 点在轴线的投影距离(带符号);
    Eigen::Vector3f radial_vec = p_vec - center - t * axis; // 径向向量
    return radial_vec.norm();
}


/*****************************************
 * @brief                   根据圆柱表面点云到轴线距离过滤焊缝
 * 
 * @param cloud             输入点云指针
 * @param cylinder_model    圆柱模型
 * @param distance_tolerance 距离容差
 * @param seam_cloud        输出焊缝点云指针
 * @param surface_cloud     输出圆柱表面点云指针
 * @return true 成功。
 * @return false 失败。
 *****************************************/
bool PointsModel::ExtractWeldSeamByDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, 
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr &seam_cloud, 
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr &surface_cloud,
                                            const float distance_tolerance_)
{
    if (cloud->empty())
    {
        LOG_ERROR("ExtractWeldSeamByDistance  输入点云为空");
        return false;
    }
    for (const auto &p : cloud->points)
    {
        float radial_dist = ComputeRadialDistanceToAxis(p, cylinder_model_);
        float radius_diff = std::abs(radial_dist - cylinder_model_.radius);
        if (std::abs(radius_diff) <= distance_tolerance_)
        {
            surface_cloud->points.push_back(p);
        }
        else
        {
            seam_cloud->points.push_back(p);
        }
    }
    return true;
}

/*****************************************
 * @brief               圆柱沿轴向按指定步长横截面
 * 
 * @param cloud         输入圆柱点云指针
 * @param center        圆柱中心（3D向量）
 * @param axis          圆柱轴向
 * @param radius        圆柱半径
 * @param height        圆柱高度
 * @param dist_step     采样步长
 * @param cross_section 输出横截面点云指针
 * 
 * @return void
 *****************************************/
void PointsModel::ClipCylinderCrossSection(
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                        const Eigen::Vector3f& center,
                        const Eigen::Vector3f& axis,      
                        const float radius,
                        const float height,
                        const float dist_step,
                        std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>& cross_sections
)
{
    // 检查点云为空
    Utils::IsEmptyPoint(cloud);

    // 计算圆柱半高
    const float half_height = height * 0.5f;
    const int num_sections = static_cast<int>(std::ceil(height / dist_step))-2; // 截面数减2，排除首尾边缘截面(可能有异常点)
    LOG_INFO("ClipCylinderCrossSection height:" << height << " num_sections:" << num_sections);
    // 初始化每个截面的点云容器
    for (int i = 0; i < num_sections; ++i) {
        cross_sections[i] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    }

    // 单位化轴向
    const Eigen::Vector3f axis_unit = axis.normalized();
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // 遍历圆柱轴上的点
    for (auto& p : cloud->points)
    {
        Eigen::Vector3f p_vec(p.x, p.y, p.z);

        // 计算点沿轴向的投影
        float proj = (p_vec - center).dot(axis_unit); // 范围应在 [-half_height, +half_height]
        
        // 不是截面附近的点则跳过
        if (std::abs(std::fmod(proj+half_height, dist_step)) >= 1.0f) continue;
        int section_index = static_cast<int>((proj+half_height) / dist_step);
        if (section_index <= 0 || section_index > num_sections) continue; // 排除首尾截面
        cross_sections[section_index-1]->points.push_back(p);
        filtered_cloud->points.push_back(p);
    }
    pcl::io::savePLYFileASCII("../data/cross_point.ply", *filtered_cloud);
}

/****************************************
 * @brief                   根据圆柱不同截面点云计算圆柱内径和圆度公差
 *   
 * @param cross_sections    输入圆柱不同截面点云指针
 * @param center            输入圆柱中心（3D向量）
 * @param axis              输入圆柱轴向
 * @param roundnessError    输出圆柱圆度误差(不同方向最大最小内径差) 
 * 
 * @return void
 *****************************************/
void PointsModel::GetDiameterRoundness(std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>& cross_sections,
                                const Eigen::Vector3f& center,
                                const Eigen::Vector3f& axis,
                                float& roundnessError)
{
    // 计算每个截面的最小半径和最大半径
    std::vector<float> min_radius_list;
    std::vector<float> max_radius_list;

    // 单位化轴向
    const Eigen::Vector3f axis_unit = axis.normalized();
    // 定义一个 lambda 函数，计算点到轴的距离
    auto dist_to_axis = [&center, &axis_unit](const pcl::PointXYZ& p) -> float
    {
        Eigen::Vector3f v = Eigen::Vector3f(p.x, p.y, p.z) - center;
        float axial_proj = v.dot(axis_unit);               // 圆环上点在轴上距center的投影距离(带符号)
        Eigen::Vector3f radial_vec = v - axial_proj * axis_unit;
        float distance_to_axis = radial_vec.norm();   // 点到轴距离
        return distance_to_axis;    
    };
    LOG_INFO("GetDiameterRoundness 截面数量:" << cross_sections.size());
    // 遍历每个截面
    for (auto& section_cloud : cross_sections)
    {   
        pcl::PointCloud<pcl::PointXYZ>::Ptr section_cloud_valid(new pcl::PointCloud<pcl::PointXYZ>);
        Utils::StatisticalOutlierRemoval(section_cloud.second, section_cloud_valid, 10);

        // 计算截面圆的最小半径和最大半径
        float min_radius = FLT_MAX;
        float max_radius = FLT_MIN;

        // 遍历截面点云，计算最小半径和最大半径
        for (auto& p : section_cloud_valid->points)
        {
            float radius = dist_to_axis(p);
            min_radius = std::min(min_radius, radius);
            max_radius = std::max(max_radius, radius);
        }

        min_radius_list.push_back(min_radius);
        max_radius_list.push_back(max_radius); 
    }

    // 计算圆度误差
    roundnessError = 2*(*std::max_element(max_radius_list.begin(), max_radius_list.end()) - 
                     *std::min_element(min_radius_list.begin(), min_radius_list.end()));
    // 最小半径为圆柱的内半径
    // inline_radius = 2*(*std::min_element(min_radius_list.begin(), min_radius_list.end()));
    // std::cout<<"inline_radius:"<<inline_radius<<" roundnessError:"<<roundnessError<<std::endl;
}

/***************************************
 * @brief                   每隔指定度数沿圆柱轴向切平面，将圆柱表面上的点云投影到该轴向平面上
 * @param cloud             输入点云指针
 * @param segment_cloud     输出分割后的点云指针
 * @param cylinder_model    圆柱模型
 * @param vertical_projected_points     输出垂直投影到该平面的2D点集
 * @param horizontal_projected_points   输出水平投影到该平面的2D点集
 * @param angle_step        切割平面角度步长，默认30度
 * @return void
 *****************************************/
void PointsModel::SegmentCylindAlignAxis(
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& vertical_cloud,
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& horizontal_cloud,
                        const CylinderModel& cylinder_model,
                        std::vector<std::vector<pcl::PointXY>>& vertical_projected_points_per_plane,
                        std::vector<std::vector<pcl::PointXY>>& horizontal_projected_points_per_plane,
                        const float angle_step_deg,
                        const float distance_tolerance               
)
{
    Eigen::Vector3f unit_axis = cylinder_model.axis.normalized();

    // 构建初始径向参考方向（⊥ axis）
    Eigen::Vector3f ref_dir;
    if (std::abs(unit_axis.dot(Eigen::Vector3f::UnitZ())) < 0.99f)
        ref_dir = unit_axis.cross(Eigen::Vector3f::UnitZ()).normalized();
    else
        ref_dir = unit_axis.cross(Eigen::Vector3f::UnitX()).normalized();

    int num_planes = static_cast<int>(360.0f / angle_step_deg + 0.5f);
    if (num_planes <= 0) num_planes = 12;


    vertical_projected_points_per_plane.clear();
    horizontal_projected_points_per_plane.clear();
    vertical_projected_points_per_plane.resize(num_planes);
    horizontal_projected_points_per_plane.resize(num_planes);


    const float deg2rad = M_PI / 180.0f;

    int segment_id = 0;

    // 遍历每个切割平面
    for (int i = 0; i < num_planes; ++i) {
        float angle_rad = i * angle_step_deg * deg2rad;
        Eigen::AngleAxisf rot(angle_rad, unit_axis);
        Eigen::Vector3f radial = (rot * ref_dir).normalized(); // 当前径向方向

        // 切割平面的法向量：垂直于 (radial, axis) 平面
        Eigen::Vector3f plane_normal = radial.cross(unit_axis).normalized();

        auto& vertical_current_2d = vertical_projected_points_per_plane[i];
        auto& horizontal_current_2d = horizontal_projected_points_per_plane[i];

        // 遍历所有垂直面的点云
        for (const auto& pt : vertical_cloud->points) {
            Eigen::Vector3f p(pt.x, pt.y, pt.z);
            Eigen::Vector3f dp = p - cylinder_model.center;

            // 计算点到平面的距离（带符号）
            float dist = plane_normal.dot(dp);
            if (std::abs(dist) > distance_tolerance) continue;

            // 正交投影到平面
            Eigen::Vector3f p_proj = p - dist * plane_normal;

            // 在平面局部坐标系下计算2D坐标
            Eigen::Vector3f local_dp = p_proj - cylinder_model.center;
            float u = local_dp.dot(radial);      // 径向
            float v = local_dp.dot(unit_axis);   // 轴向

            vertical_current_2d.push_back({u, v});
        }
        // 遍历所有水平面的点云
        for (const auto& pt : horizontal_cloud->points) {
            Eigen::Vector3f p(pt.x, pt.y, pt.z);
            Eigen::Vector3f dp = p - cylinder_model.center;

            // 计算点到平面的距离（带符号）
            float dist = plane_normal.dot(dp);
            if (std::abs(dist) > distance_tolerance) continue;

            // 正交投影到平面
            Eigen::Vector3f p_proj = p - dist * plane_normal;

            // 在平面局部坐标系下计算2D坐标
            Eigen::Vector3f local_dp = p_proj - cylinder_model.center;
            float u = local_dp.dot(radial);      // 径向
            float v = local_dp.dot(unit_axis);   // 轴向

            horizontal_current_2d.push_back({u, v});
        }
        if (vertical_current_2d.empty()&&horizontal_current_2d.empty()) continue;
        WritePointsToTxt(vertical_current_2d, "../data/2D_Points/segment_vertical_cloud_" + std::to_string(segment_id) + ".txt");
        WritePointsToTxt(horizontal_current_2d, "../data/2D_Points/segment_horizontal_cloud_" + std::to_string(segment_id) + ".txt");
        segment_id++;
    }

    // 移除空的2D点集
    for (int j = (int)vertical_projected_points_per_plane.size() - 1; j >= 0; j--) 
    {
        if (vertical_projected_points_per_plane[j].empty() &&
            horizontal_projected_points_per_plane[j].empty()) 
            {
                vertical_projected_points_per_plane.erase(vertical_projected_points_per_plane.begin() + j);
                horizontal_projected_points_per_plane.erase(horizontal_projected_points_per_plane.begin() + j);
            }
    }
}



/*****************************************
 * @brief                   从圆柱不同轴向截面切平面拟合多个直线模型
 * @param cloud             输入点云指针
 * @param cylinder_model    圆柱模型
 * @param line_coeffs       输出直线参数向量
 * @param line_clouds       输出直线内点云指针向量
 * @param angle_step_deg    切割平面角度步长，默认10度
 * @param slice_distance_tolerance  切平面距离阈值，默认0.2mm
 * @param line_fit_distance_tolerance  直线拟合距离阈值，默认0.2mm
 * @return void
 *****************************************/
void PointsModel::FitLinesOnCylindricalSlices(
    const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud,
    const CylinderModel& cylinder_model,
    std::vector<std::vector<pcl::ModelCoefficients>>& line_coeffs,
    std::vector<std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>>& line_clouds,
    float angle_step_deg,
    float slice_distance_tolerance,
    float line_fit_distance_tolerance)
{
    using PointT = pcl::PointNormal;

    // 确保轴向单位化
    Eigen::Vector3f axis = cylinder_model.axis.normalized();

    // 构建正交基：找两个正交于axis的向量 u, v
    Eigen::Vector3f arbitrary;
    if (std::abs(axis.dot(Eigen::Vector3f::UnitX())) < 0.9f)
        arbitrary = Eigen::Vector3f::UnitX();
    else
        arbitrary = Eigen::Vector3f::UnitY();

    Eigen::Vector3f u = axis.cross(arbitrary).normalized();
    Eigen::Vector3f v = axis.cross(u).normalized(); // already normalized

    // 角度范围 [0, 360)
    int num_slices = static_cast<int>(360.0f / angle_step_deg);
    if (num_slices <= 0) num_slices = 1;

    // 预清空输出
    line_coeffs.clear();
    line_clouds.clear();
    int kk=0;
    for (int i = 0; i < num_slices/2; ++i)
    {
        float angle_rad = static_cast<float>(i) * angle_step_deg * M_PI / 180.0f;
        Eigen::Vector3f normal_in_plane = std::cos(angle_rad) * u + std::sin(angle_rad) * v; // 切平面法向（径向）

        // 提取距离当前切平面在 slice_distance_tolerance 内的点
        pcl::PointCloud<PointT> slice_points;
        for (const auto& pt : cloud->points)
        {
            if (!pcl::isFinite(pt)) continue;
            Eigen::Vector3f p(pt.x, pt.y, pt.z);
            float dist = std::abs((p - cylinder_model.center).dot(normal_in_plane));
            if (dist <= slice_distance_tolerance)
            {
                slice_points.push_back(pt);
            }
        }

        if (slice_points.empty()) continue;

        // 欧式聚类（阈值 15.0）
        // typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        npt_tree_->setInputCloud(slice_points.makeShared());
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(15.0);
        ec.setMinClusterSize(10);
        ec.setMaxClusterSize(std::numeric_limits<int>::max());
        ec.setSearchMethod(npt_tree_);
        ec.setInputCloud(slice_points.makeShared());
        ec.extract(cluster_indices);

        // 遍历每个聚类
        for (const auto& indices : cluster_indices)
        {
            pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
            for (const auto idx : indices.indices)
                cluster->push_back(slice_points[idx]);

            // 分割为水平 vs 垂直（基于法向与圆柱轴夹角）
            pcl::PointCloud<PointT>::Ptr horizontal_pts(new pcl::PointCloud<PointT>);
            pcl::PointCloud<PointT>::Ptr vertical_pts(new pcl::PointCloud<PointT>);

            float cos_threshold = std::cos(60.0f * M_PI / 180.0f); // 25 degrees
            for (const auto& pt : cluster->points)
            {
                if (!pcl::isFinite(pt)) continue;
                Eigen::Vector3f normal(pt.normal_x, pt.normal_y, pt.normal_z);
                if (normal.norm() == 0) continue;
                normal.normalize();
                float dot = std::abs(normal.dot(axis));
                if (dot > cos_threshold)
                    horizontal_pts->push_back(pt);
                else
                    vertical_pts->push_back(pt);
            }

            if (horizontal_pts->empty() || vertical_pts->empty())
                continue;

            // 切平面局部坐标系：e1 = axis（轴向），e2 = normal_in_plane × axis（切平面内垂直轴向）
            Eigen::Vector3f e1 = axis;
            Eigen::Vector3f e2 = normal_in_plane.cross(axis).normalized();

            // === Lambda: 拟合2D直线并映射回3D ===
            auto fitLine2D = [&](const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud2d,
                                const std::vector<PointT>& original_points,
                                pcl::ModelCoefficients& coeffs,
                                pcl::PointCloud<PointT>::Ptr& inliers_3d) -> bool {
                if (cloud2d->size() < 2) return false;

                pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
                pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

                pcl::SACSegmentation<pcl::PointXYZ> seg;
                seg.setOptimizeCoefficients(true);
                seg.setModelType(pcl::SACMODEL_LINE);
                seg.setMethodType(pcl::SAC_RANSAC);
                seg.setDistanceThreshold(line_fit_distance_tolerance);
                seg.setInputCloud(cloud2d);

                seg.segment(*inliers, *coefficients); // 返回 void

                if (inliers->indices.empty() || coefficients->values.size() < 6)
                    return false;

                // 2D 直线参数: [x0, y0, 0, dx, dy, 0]
                Eigen::Vector3f pt2d(coefficients->values[0], coefficients->values[1], 0.0f);
                Eigen::Vector3f dir2d(coefficients->values[3], coefficients->values[4], 0.0f);
                dir2d.normalize();

                // 映射回3D世界坐标
                Eigen::Vector3f pt3d = cylinder_model.center + pt2d.x() * e1 + pt2d.y() * e2;
                Eigen::Vector3f dir3d = dir2d.x() * e1 + dir2d.y() * e2;
                dir3d.normalize();

                coeffs.values = {pt3d.x(), pt3d.y(), pt3d.z(), dir3d.x(), dir3d.y(), dir3d.z()};

                // 恢复带法向的内点
                inliers_3d.reset(new pcl::PointCloud<PointT>);
                for (int idx : inliers->indices) {
                    inliers_3d->push_back(original_points[idx]);
                }
                return true;
            };
            pcl::io::savePLYFileASCII("../data/2D_Points/vertical_cloud_"+std::to_string(kk)+".ply", *vertical_pts);
            pcl::io::savePLYFileASCII("../data/2D_Points/horizontal_cloud_"+std::to_string(kk)+".ply", *horizontal_pts);
            

            // === 投影水平点到2D平面 ===
            pcl::PointCloud<pcl::PointXYZ>::Ptr horiz_2d(new pcl::PointCloud<pcl::PointXYZ>);
            std::vector<PointT> horiz_orig;
            for (const auto& pt : horizontal_pts->points) {
                if (!pcl::isFinite(pt)) continue;
                Eigen::Vector3f p(pt.x, pt.y, pt.z);
                Eigen::Vector3f vec = p - cylinder_model.center;
                float x2d = vec.dot(e1);
                float y2d = vec.dot(e2);
                horiz_2d->push_back(pcl::PointXYZ(x2d, y2d, 0.0f));
                horiz_orig.push_back(pt); // 保留完整点（含法向）
            }

            // === 投影垂直点到2D平面 ===
            pcl::PointCloud<pcl::PointXYZ>::Ptr vert_2d(new pcl::PointCloud<pcl::PointXYZ>);
            std::vector<PointT> vert_orig;
            for (const auto& pt : vertical_pts->points) {
                if (!pcl::isFinite(pt)) continue;
                Eigen::Vector3f p(pt.x, pt.y, pt.z);
                Eigen::Vector3f vec = p - cylinder_model.center;
                float x2d = vec.dot(e1);
                float y2d = vec.dot(e2);
                vert_2d->push_back(pcl::PointXYZ(x2d, y2d, 0.0f));
                vert_orig.push_back(pt);
            }

            pcl::io::savePLYFileASCII("../data/2D_Points/2D_vertical_cloud_"+std::to_string(kk)+".ply", *vert_2d);
            pcl::io::savePLYFileASCII("../data/2D_Points/2D_horizontal_cloud_"+std::to_string(kk)+".ply", *horiz_2d);
            kk++;

            // === 拟合直线 ===
            pcl::ModelCoefficients horiz_coeff;
            pcl::PointCloud<PointT>::Ptr horiz_inliers(new pcl::PointCloud<PointT>);
            bool horiz_success = fitLine2D(horiz_2d, horiz_orig, horiz_coeff, horiz_inliers);

            pcl::ModelCoefficients vert_coeff;
            pcl::PointCloud<PointT>::Ptr vert_inliers(new pcl::PointCloud<PointT>);
            bool vert_success = fitLine2D(vert_2d, vert_orig, vert_coeff, vert_inliers);

            if (horiz_success && vert_success)
            {
                line_coeffs.push_back({horiz_coeff, vert_coeff});
                line_clouds.push_back({horiz_inliers, vert_inliers});
            }
        }
    }
}

/***************************************************** 
 * @brief                   计算圆柱不同截面垂直边缘的高度
 * @param line_clouds       每个直边拟合直线的内点
 * @param line_coeffs       每个直边拟合直线的参数
 * @param vertical_edge_height  输出垂直边缘的高度
 * @return void
****************************************************/
void PointsModel::ComputeVerticalEdgeHeight(
    const std::vector<std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>>& line_clouds,
    const std::vector<std::vector<pcl::ModelCoefficients>>& line_coeffs,
    std::vector<float>& vertical_edge_height)
{
    vertical_edge_height.clear();

    // 遍历每个切片
    for (size_t i = 0; i < line_coeffs.size(); ++i) {
        // 每个切片应有两条直线，仅计算索引为1的垂直边缘的高度
        for (size_t j = 1; j < line_coeffs[i].size(); ++j) {
            const auto& cloud = line_clouds[i][j];
            const auto& coeff = line_coeffs[i][j];

            // 安全检查
            if (!cloud || cloud->empty()) {
                vertical_edge_height.push_back(0.0f);
                continue;
            }

            if (coeff.values.size() != 6) {
                LOG_WARN("Invalid line coefficients (not 6 values).");
                vertical_edge_height.push_back(0.0f);
                continue;
            }

            // 提取直线参数
            Eigen::Vector3f p0(coeff.values[0], coeff.values[1], coeff.values[2]); // 直线上一点
            Eigen::Vector3f dir(coeff.values[3], coeff.values[4], coeff.values[5]); // 方向向量

            // 归一化方向向量
            if (dir.norm() < 1e-8f) {
                vertical_edge_height.push_back(0.0f);
                continue;
            }
            dir.normalize();

            // 投影所有点到直线方向，计算 t = (p - p0) · dir
            std::vector<float> projections;
            projections.reserve(cloud->points.size());

            for (const auto& pt : cloud->points) {
                Eigen::Vector3f p(pt.x, pt.y, pt.z);
                float t = (p - p0).dot(dir);
                projections.push_back(t);
            }

            // 计算投影范围
            float min_t = *std::min_element(projections.begin(), projections.end());
            float max_t = *std::max_element(projections.begin(), projections.end());
            float length = max_t - min_t;
            std::string file_name = (j%2==1? "vertical_line_" : "horizontal_line_") + std::to_string(i) + ".obj";
            std::ofstream file(("../data/2D_Points/" + file_name));
            if (!file.is_open()) {
                LOG_ERROR(std::string("Error: Could not open file ") + file_name + " for writing.");
                return;
            }
            Eigen::Vector3f max_coord = p0 + max_t * dir;
            Eigen::Vector3f min_coord = p0 + min_t * dir;
            file << "v " << max_coord.x() << " " << max_coord.y() << " " << max_coord.z() << std::endl;
            file << "v " << min_coord.x() << " " << min_coord.y() << " " << min_coord.z() << std::endl;
            file << "l 1 2" << std::endl;
            file.close();

            // 防御性处理（理论上 length >= 0）
            vertical_edge_height.push_back(std::max(0.0f, length));
        }
    }
}


/*****************************************
 * @brief                   分类判断圆柱垂直边缘的倾斜方向
 * @param line_coeff        输入直线参数
 * @param cylinder_model    圆柱模型
 * @return std::string      倾斜方向（outward, inward, circumferential, vertical）
 *****************************************/
std::string PointsModel::classifyTiltDirection(
    const pcl::ModelCoefficients& line_coeff,
    const CylinderModel& cylinder_model)
{
    using Vec3 = Eigen::Vector3f;

    Vec3 pt = Vec3(line_coeff.values[0], line_coeff.values[1], line_coeff.values[2]); // 直线上一点
    Vec3 d = Vec3(line_coeff.values[3], line_coeff.values[4], line_coeff.values[5]).normalized(); // 方向向量
    Vec3 axis = cylinder_model.axis.normalized();

    // 1. 去除轴向分量
    Vec3 d_perp = d - d.dot(axis) * axis;
    if (d_perp.norm() < 1e-6) {
        return "vertical"; // 平行于轴
    }
    d_perp.normalize(); //单位径向分量

    // 2. 计算局部径向方向（向外）
    Vec3 oc = pt - cylinder_model.center;
    float t = oc.dot(axis);
    Vec3 closest_on_axis = cylinder_model.center + t * axis;
    Vec3 radial_out = (pt - closest_on_axis).normalized(); // 向外！

    // 3: 点积判断
    float dot = d_perp.dot(radial_out);

    if (dot > 0.1f) {
        return "outward";  // 向外倾斜
    } else if (dot < -0.1f) {
        return "inward";   // 向内倾斜
    } else {
        return "circumferential"; // 近似周向
    }
}


/*****************************************
 * @brief                       计算封头直边的斜度
 * @param vertical_edge_height  输入封头直边高度
 * @param line_coeffs           输入直边拟合直线的参数
 * @param cylinder_model        圆柱模型
 * @param vertical_edge_tilt    输出直边斜度(内倾为负，外倾为正)
 * @return void
 *****************************************/
void PointsModel::ClassifyVerticalEdgeTilt(
                        const std::vector<float>& vertical_edge_height,
                        const std::vector<std::vector<pcl::ModelCoefficients>>& line_coeffs,
                        const CylinderModel& cylinder_model,
                        std::vector<float>& vertical_edge_tilt
)
{
    vertical_edge_tilt.resize(line_coeffs.size());

    std::vector<float> line_tilt_angle(line_coeffs.size(), 0.0f); // 直边相对直边圆柱的倾斜角度

    int vec_idx = 0;
    for (auto& line_coeff: line_coeffs)
    {
        float angle; // 每个切平面上的拟合直线与圆柱轴线的夹角
        Eigen::Vector3f line_dir = Eigen::Vector3f(line_coeff[1].values[3], line_coeff[1].values[4], line_coeff[1].values[5]);
        Eigen::Vector3f axis = line_dir.dot(cylinder_model.axis.normalized())*cylinder_model.axis.normalized();

        Utils::ComputeAngleBetweenVectors(line_dir, axis, angle); // 计算直边与圆柱轴线的夹角
        float line_tilt = vertical_edge_height[vec_idx]*std::sin(angle*M_PI/180.0f);

        std::string tilt_direction = classifyTiltDirection(line_coeff[1], cylinder_model); // 分类判断直边的倾斜方向

        if(tilt_direction == "outward")
        {
            vertical_edge_tilt[vec_idx] = line_tilt;
        }
        else if(tilt_direction == "inward")
        {
            vertical_edge_tilt[vec_idx] = -line_tilt;
        }
        else if(tilt_direction == "vertical")
        {
            vertical_edge_tilt[vec_idx] = 0.0f;
        }
        else
        {
            vertical_edge_tilt[vec_idx] = INVALID_VALUE;
            LOG_WARN("直边" << vec_idx << "提取出错");

        }
        vec_idx++;
    }
}

/*****************************************
 * @brief                       对直边高度进行过滤均值化
 * @param vertical_edge_height  输入封头直边高度(内倾为负，外倾为正)
 * @param vertical_edge_tilt    输入直边拟合直边的斜度
 * @param filter_ratio          输入两端过滤比例
 * @param median_edge_height    输出直边高度的中位数
 * @param median_edge_tilt      输出直边斜度中位数(内倾为负，外倾为正)
 * @return void
 *****************************************/
 void PointsModel::GetMedianEdgeHeightAndTilt(
                        const std::vector<float>& vertical_edge_height,
                        const std::vector<float>& vertical_edge_tilt,
                        float& median_edge_height,
                        float& median_edge_tilt,
                        const float filter_ratio)
{
    if (vertical_edge_height.empty() || vertical_edge_height.size() != vertical_edge_tilt.size())
    {
        median_edge_height = INVALID_VALUE;
        median_edge_tilt = INVALID_VALUE;
        return;
    }

    // 将高度和斜度配对，以便排序时斜度跟随高度，同时过滤掉无效值
    std::vector<std::pair<float, float>> height_tilt_pairs;
    height_tilt_pairs.reserve(vertical_edge_height.size());
    for (size_t i = 0; i < vertical_edge_height.size(); ++i)
    {
        if (vertical_edge_height[i] != INVALID_VALUE && vertical_edge_tilt[i] != INVALID_VALUE)
        {
            height_tilt_pairs.push_back({vertical_edge_height[i], vertical_edge_tilt[i]});
        }
    }

    if (height_tilt_pairs.empty())
    {
        median_edge_height = INVALID_VALUE;
        median_edge_tilt = INVALID_VALUE;
        return;
    }

    // 按高度进行排序
    std::sort(height_tilt_pairs.begin(), height_tilt_pairs.end(), 
              [](const std::pair<float, float>& a, const std::pair<float, float>& b) {
                  return a.first < b.first;
              });

    // 计算需要剔除的点数
    int total_points = height_tilt_pairs.size();
    int remove_count = static_cast<int>(total_points * filter_ratio);

    // 移除两端filter_ratio比例的点
    if (total_points > 2 * remove_count)
    {
        // 先删除末尾，避免影响前端删除的索引
        height_tilt_pairs.erase(height_tilt_pairs.end() - remove_count, height_tilt_pairs.end());
        height_tilt_pairs.erase(height_tilt_pairs.begin(), height_tilt_pairs.begin() + remove_count);
    }

    // 计算中位数
    if (!height_tilt_pairs.empty())
    {
        int mid_idx = height_tilt_pairs.size() / 2;
        median_edge_height = height_tilt_pairs[mid_idx].first;
        median_edge_tilt = height_tilt_pairs[mid_idx].second;
    }
    else
    {
        median_edge_height = INVALID_VALUE;
        median_edge_tilt = INVALID_VALUE;
    }
}



/*****************************************
 * @brief                       计算封头坡口的角度
 * @param line_coeffs           输入封头直边和坡口斜边直线参数
 * @param head_angle            输出封头坡口角度滤波均值化角度[0,90]
 * @param filter_ratio          输入两端过滤比例

 * @return void
 *****************************************/
void PointsModel::GetHeadAngle(std::vector<std::vector<pcl::ModelCoefficients>>& line_coeffs, float& head_angle, const float filter_ratio)
{
    if (line_coeffs.empty())
    {
        head_angle = INVALID_VALUE;
        return;
    }
    std::vector<float> head_angles;
    head_angles.reserve(line_coeffs.size());
    for (const auto& line_coeff : line_coeffs)
    {
        if (line_coeff.size() < 2)
        {
            continue;
        }
        if (line_coeff[0].values.size() < 6 || line_coeff[1].values.size() < 6) continue;
        Eigen::Vector3f horizontal_vec{line_coeff[0].values[3], line_coeff[0].values[4], line_coeff[0].values[5]};
        Eigen::Vector3f vertical_vec{line_coeff[1].values[3], line_coeff[1].values[4], line_coeff[1].values[5]};
        if (!horizontal_vec.allFinite() || !vertical_vec.allFinite()) continue;
        if (horizontal_vec.norm() < 1e-6f || vertical_vec.norm() < 1e-6f) continue;
        float angle = 0.0f;
        Utils::ComputeAngleBetweenVectors(horizontal_vec, vertical_vec, angle);
        if (std::isfinite(angle)) head_angles.push_back(angle);
    }
    if (head_angles.empty())
    {
        head_angle = INVALID_VALUE;
        return;
    }
    std::sort(head_angles.begin(), head_angles.end());
    
    // 计算需要剔除的点数
    int total_angles = head_angles.size();
    int remove_count = static_cast<int>(total_angles * std::clamp(filter_ratio, 0.0f, 0.49f));

    // 移除两端filter_ratio比例的点
    if (total_angles > 2 * remove_count)
    {
        // 先删除末尾，避免影响前端删除的索引
        head_angles.erase(head_angles.end() - remove_count, head_angles.end());
        head_angles.erase(head_angles.begin(), head_angles.begin() + remove_count);
    }

    // 计算中位数
    if (!head_angles.empty())
    {
        int mid_idx = head_angles.size() / 2;
        head_angle = head_angles[mid_idx];
    }
    else
    {
        head_angle = INVALID_VALUE;
    }
}
