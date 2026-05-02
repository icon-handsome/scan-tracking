#include <pcl/io/ply_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/surface/mls.h>
#include <pcl/common/pca.h>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <limits>
#include <cmath>
#include <iostream>

#include "utils/Utils.h"
#include "utils/tic_toc.h"

using namespace std;



/*******************************************
 * @brief                   体素滤波下采样
 * @param cloud             输入点云指针
 * @param downsampled_cloud 输出下采样点云指针
 * @param leaf_size         下采样体素，默认 0.05
 *******************************************/
void Utils::VoxelGridDownSample(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &downsampled_cloud, float leaf_size)
{
    // 检查点云为空
    IsEmptyPoint(cloud);

    // 创建下采样滤波器
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid.filter(*downsampled_cloud);
    pcl::io::savePLYFile("../data/downsample_cloud.ply", *downsampled_cloud);
}

/*******************************************
 * @brief                   均匀下采样
 * @param cloud             输入点云指针
 * @param downsampled_cloud 输出下采样点云指针
 * @param leaf_size         下采样体素，默认 0.05
 *******************************************/
void Utils::UniformDownSample(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& downsampled_cloud,
    float leaf_size)
{
    // 检查点云为空
    IsEmptyPoint(cloud);
    // 创建下采样滤波器
    pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
    uniform_sampling.setInputCloud(cloud);
    uniform_sampling.setRadiusSearch(leaf_size);
    uniform_sampling.filter(*downsampled_cloud);
    pcl::io::savePLYFile("../data/downsample_cloud.ply", *downsampled_cloud);
}



/*******************************************
 * @brief                   统计滤波
 * @param cloud             输入点云指针
 * @param downsampled_cloud 输出下采样点云指针
 * @param kNearest          最近邻点数量，默认 20
 *******************************************/
void Utils::StatisticalOutlierRemoval(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& downsampled_cloud,
    int kNearest
)
{
    // 检查点云为空
    IsEmptyPoint(cloud);

    // 创建统计滤波对象
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(kNearest);
    sor.setStddevMulThresh(1.0);
    sor.filter(*downsampled_cloud);
}

  


/******************************************
 * @brief                   高程带通滤波
 * @param cloud             输入点云指针
 * @param filtered_cloud    输出滤波点云指针
 * @param filtered_cloud_elevation  输出滤波点云高程向量(min,max)
 * @param axis              滤波轴，默认 z 轴
 *
 * @return void 
 ******************************************/
void Utils::ElevationBilateralFilter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud,
    const std::vector<float>& filtered_cloud_elevation,
    const std::string& axis
)
{
    // 检查点云为空
    IsEmptyPoint(cloud);
    // 创建带通滤波器进行高程滤波
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName(axis);
    pass.setFilterLimits(filtered_cloud_elevation.front(), filtered_cloud_elevation.back());
    pass.filter(*filtered_cloud);
    
}


/*****************************************
 * @brief                   求解两个轴的交点角度
 * 
 * @param _axis1            输入轴1（3D向量）
 * @param _axis2            输入轴2（3D向量）
 * @param angle             输出夹角角度（角度[0,90°]）
 * 
 * @return void
 *****************************************/
void Utils::ComputeAngleBetweenVectors(const Eigen::Vector3f& _axis1,
                            const Eigen::Vector3f& _axis2,
                            float& angle)
{
    angle =pcl::getAngle3D(_axis1, _axis2, true);  // 获取夹角[0,180°]
    angle = (angle>90.0f )? (180.0f-angle) : angle; // 转换为[0,90°]
}

/******************************************
 * @brief               计算圆柱不同截面拟合圆半径平均值     
 *   
 * @param cross_sections 输入不同截面点云指针
 * @param av_radius      输出平均圆半径
 * @param axis           圆柱轴向（必须是单位向量）
 * 
 * @return void
 *****************************************/

void Utils::ComputeAverageCircleRadius(
                        std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>& cross_sections,
                        double& av_radius,
                        const Eigen::Vector3f& axis
)
{
    // 在每个截面上将点云拟合圆，计算圆的半径
    int valid_sections = 0;

    for (int i = 0; i < cross_sections.size(); ++i) {
        auto& section_cloud = cross_sections[i];
        // 设置点云元数据
        section_cloud->width = static_cast<uint32_t>(section_cloud->points.size());
        section_cloud->height = 1;
        section_cloud->is_dense = false;

        // 跳过点数不足的截面
        if (section_cloud->size() < 3) {
            LOG_WARN("截面" << i << "点数不足");
            continue;
        }

        // 拟合 3D 圆并获取半径
        double radius_temp;
        fitCircle3DAndGetRadius(section_cloud, axis, radius_temp);

        // 半径累加
        av_radius += radius_temp;
        // 有效截面计数器
        valid_sections++;
    }
    // 计算平均半径
    av_radius /= valid_sections;

}


/*****************************************
 * @brief               圆柱沿轴向按指定步长横截面
 * 
 * @param cloud         输入点云指针
 * @param center        圆柱中心（3D向量）
 * @param axis          圆柱轴向
 * @param radius        圆柱半径
 * @param height        圆柱高度
 * @param dist_step     采样步长
 * @param cross_section 输出横截面点云指针
 * 
 * @return void
 *****************************************/
void Utils::ClipCylinderCrossSection(
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
    IsEmptyPoint(cloud);

    // 计算圆柱半高
    const float half_height = height * 0.5f;
    const int num_sections = static_cast<int>(std::ceil(height / dist_step))-2; // 截面数减2，排除首尾边缘截面(可能有异常点)
    LOG_INFO("ClipCylinderCrossSection height=" << height << " num_sections=" << num_sections);
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
 * 
 * @brief                   根据圆柱不同截面点云计算圆柱内径和圆度公差
 *   
 * @param cross_sections    输入圆柱不同截面点云指针
 * @param radius            输出圆柱最小内径(直径)
 * @param roundnessError    输出圆柱圆度误差(不同方向最大最小内径差) 
 * 
 * @return void
 *****************************************/
void Utils::GetCylinderRadius(std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>& cross_sections,
                                const Eigen::Vector3f& center,
                                const Eigen::Vector3f& axis,
                                float& inline_radius,
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
    LOG_INFO("GetCylinderRadius 截面数量:" << cross_sections.size());
    // 遍历每个截面
    for (auto& section_cloud : cross_sections)
    {   
        pcl::PointCloud<pcl::PointXYZ>::Ptr section_cloud_valid(new pcl::PointCloud<pcl::PointXYZ>);
        StatisticalOutlierRemoval(section_cloud.second, section_cloud_valid, 10);

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
    inline_radius = 2*(*std::min_element(min_radius_list.begin(), min_radius_list.end()));
    LOG_INFO("inline_radius:" << inline_radius << " roundnessError:" << roundnessError);
}

/**************************************
 *  @brief               计算圆柱单个截面拟合圆半径
 * 
 *  @param cloud         输入点云指针
 *  @param center        圆柱中心（3D向量）
 *  @param axis          圆柱轴向（必须是单位向量）
 *  @param radius        圆柱半径
 *  @param height        圆柱高度
 *  @param dist_step     采样步长
 *  @param roundness     输出圆柱圆度公差
 * 
 *  @return void
 *****************************************/
void Utils::fitCircle3DAndGetRadius(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const Eigen::Vector3f& axis,
    double& radius
)
{
    //统计滤波，移除离群点
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    StatisticalOutlierRemoval(cloud, cloud_filtered, 10);

    // Step 1: 归一化输入轴向（确保是单位向量）
    Eigen::Vector3f normal = axis.normalized();

    // Step 2: 计算点云fitCircle3DAndGetRadius质心
    Eigen::Vector3d centroid(0, 0, 0);
    for (const auto& pt : cloud_filtered->points) {
        centroid += Eigen::Vector3d(pt.x, pt.y, pt.z);
    }
    centroid /= static_cast<double>(cloud_filtered->size());

    // Step 3: 构造平面上的两个正交基向量 u, v
    Eigen::Vector3f u_axis, v_axis;

    // 找一个不与 normal 平行的向量（避免叉积为零）
    Eigen::Vector3f arbitrary;
    if (std::abs(normal.z()) < 0.99) {
        arbitrary = Eigen::Vector3f(0, 0, 1); // 通常安全
    } else {
        arbitrary = Eigen::Vector3f(1, 0, 0); // 当 normal 接近 z 轴时
    }

    u_axis = normal.cross(arbitrary).normalized();
    v_axis = normal.cross(u_axis).normalized(); // 确保正交且右手系

    // Step 4: 投影到 uv 平面（2D 点）
    std::vector<Eigen::Vector2d> points2D; // 使用 double 提高精度
    points2D.reserve(cloud->size());

    for (const auto& pt : cloud_filtered->points) {
        Eigen::Vector3d p(pt.x, pt.y, pt.z);
        Eigen::Vector3d diff = p - centroid;
        double u = diff.dot(u_axis.cast<double>());
        double v = diff.dot(v_axis.cast<double>());
        points2D.emplace_back(u, v);
    }
    Eigen::Vector3d fitCircle2D_result = fitCircle2D(points2D);

    radius = std::sqrt(fitCircle2D_result[2]);
}

/*****************************************
 * @brief                   拟合球模型
 * @param cloud             输入点云指针
 * @param sphere_cloud      输出球点云指针
 * @param center            输出球中心
 * @param radius            输出球半径
 * 
 * ***************************************/
void Utils::FitSphere(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr& sphere_cloud,
                        Eigen::Vector3f& center,
                        double& radius,
                        const float dist_threshold)
{
   // 1. 创建球体模型并执行 RANSAC
    pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr model(
        new pcl::SampleConsensusModelSphere<pcl::PointXYZ>(cloud));
    
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
    ransac.setDistanceThreshold(dist_threshold);
    ransac.setMaxIterations(5000); 

    if (!ransac.computeModel()) {
        // RANSAC 失败：返回空结果
        sphere_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        center.setZero();
        radius = 0.0;
        LOG_ERROR("[FitSphere] RANSAC failed to find a sphere model.");
        return;
    }

    // 2. 获取内点
    std::vector<int> inliers;
    ransac.getInliers(inliers);

    if (inliers.empty()) {
        sphere_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        center.setZero();
        radius = 0.0;
        LOG_ERROR("[FitSphere] No inliers found.");
        return;
    }

    // 3. 提取内点点云
    sphere_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, inliers, *sphere_cloud);

    // 4.  RANSAC 拟合参数
    Eigen::VectorXf coefficients;
    ransac.getModelCoefficients(coefficients);

    if (coefficients.size() != 4) {
        sphere_cloud->clear();
        center.setZero();
        radius = 0.0;
        LOG_ERROR("[FitSphere] Invalid coefficients size.");
        return;
    }

    // 非线性优化
    Eigen::VectorXf optimized_coefficients;
    model->optimizeModelCoefficients(inliers, coefficients, optimized_coefficients);

    center = Eigen::Vector3f(optimized_coefficients[0], optimized_coefficients[1], optimized_coefficients[2]);
    radius = static_cast<double>(optimized_coefficients[3]);
}

// 绘制球面点云
void Utils::PlotSphere(const Eigen::Vector3f& center,
                        const double radius)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr sphere_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    const int num_points = 50000;
    for (int i = 0; i < num_points; ++i) {
        float theta = static_cast<float>(rand()) / RAND_MAX * 2.0f * M_PI; // 0 到 2π
        float phi = static_cast<float>(rand()) / RAND_MAX * M_PI;          // 0 到 π

        float x = center.x() + radius * std::sin(phi) * std::cos(theta);
        float y = center.y() + radius * std::sin(phi) * std::sin(theta);
        float z = center.z() + radius * std::cos(phi);

        sphere_cloud->points.emplace_back(x, y, z);
    }
    pcl::io::savePLYFileASCII("../data/fit_sphere_cloud.ply", *sphere_cloud);
}

/*****************************************
 * @brief                   2D点集拟合圆（Pratt 方法）
 * 
 * @param points            输入二维点集
 * @return Eigen::Vector3f  输出圆心(x,y)和半径(r)
 *****************************************/
Eigen::Vector3d Utils::fitCircle2D(const std::vector<Eigen::Vector2d>& points) {
    int n = points.size();
    if (n < 3) return Eigen::Vector3d(0,0,0);

    Eigen::MatrixXd A(n, 3);
    Eigen::VectorXd b(n);
    for (int i = 0; i < n; ++i) {
        double x = points[i][0], y = points[i][1];
        A(i, 0) = 2 * x;
        A(i, 1) = 2 * y;
        A(i, 2) = 1;
        b(i) = x*x + y*y;
    }

    Eigen::Vector3d sol = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    double cx = sol[0], cy = sol[1];
    double r = sqrt(sol[2] + cx*cx + cy*cy);
    return Eigen::Vector3d(cx, cy, r);
}

/*****************************************
 * @brief                   迭代精拟合圆柱轴向/中心/半径（切片拟圆 + 圆心拟轴线）
 * @param cloud             输入待拟合点云
 * @param radius            输出拟合圆柱半径
 * @param axis              输入粗轴向先验并输出精化轴向（单位向量）
 * @param center            输出精化后的轴线参考中心点
 * @return bool             true=拟合成功，false=拟合失败
 *****************************************/
bool Utils::RefineCylinderAxisAndRadius(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    double& radius,
    Eigen::Vector3f& axis,
    Eigen::Vector3f& center)
{
    // 1) 输入有效性检查：点云为空或点数过少时直接失败。
    if (!cloud || cloud->size() < 1000) {
        LOG_WARN("RefineCylinderAxisAndRadius failed: input cloud too small.");
        return false;
    }

    // 2) 过滤非有限点，后续几何计算只使用有效点。
    std::vector<Eigen::Vector3f> finite_points;
    finite_points.reserve(cloud->size());
    for (const auto& p : cloud->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        finite_points.emplace_back(p.x, p.y, p.z);
    }
    if (finite_points.size() < 1000) {
        LOG_WARN("RefineCylinderAxisAndRadius failed: finite points too small.");
        return false;
    }

    // 3) 中位数 + MAD（中位绝对偏差）。
    auto median_of = [](std::vector<float> values) -> float {
        if (values.empty()) return 0.0f;
        const size_t n = values.size() / 2;
        std::nth_element(values.begin(), values.begin() + n, values.end());
        float med = values[n];
        if (values.size() % 2 == 0) {
            std::nth_element(values.begin(), values.begin() + n - 1, values.end());
            med = 0.5f * (med + values[n - 1]);
        }
        return med;
    };

    auto mad_of = [&](const std::vector<float>& values, float med) -> float {
        if (values.empty()) return 0.0f;
        std::vector<float> abs_dev;
        abs_dev.reserve(values.size());
        for (const float v : values) abs_dev.push_back(std::abs(v - med));
        return median_of(std::move(abs_dev));
    };

    // 4) 给定轴向构建切片平面的局部二维正交基(u,v)。
    auto build_basis = [](const Eigen::Vector3f& axis_unit, Eigen::Vector3f& u, Eigen::Vector3f& v) {
        Eigen::Vector3f helper = (std::abs(axis_unit.z()) < 0.9f)
                                     ? Eigen::Vector3f::UnitZ()
                                     : Eigen::Vector3f::UnitX();
        u = axis_unit.cross(helper);
        if (u.norm() < 1e-6f) u = axis_unit.cross(Eigen::Vector3f::UnitY());
        u.normalize();
        v = axis_unit.cross(u).normalized();
    };

    // 5) 计算点集质心，作为迭代初始中心参考。
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (const auto& p : finite_points) centroid += p;
    centroid /= static_cast<float>(finite_points.size());

    // 6) 初始化轴向：
    //    - 若外部给了有效粗轴向，则归一化直接用；
    //    - 否则用 PCA 主方向做初始化。
    Eigen::Vector3f refined_axis = axis;
    if (!refined_axis.allFinite() || refined_axis.norm() <= 1e-6f) {
        Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
        for (const auto& p : finite_points) {
            const Eigen::Vector3f d = p - centroid;
            cov += d * d.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(cov);
        if (eig.info() != Eigen::Success) {
            LOG_WARN("RefineCylinderAxisAndRadius failed: initial PCA eigen solve failed.");
            return false;
        }
        refined_axis = eig.eigenvectors().col(2).normalized();
        if (!refined_axis.allFinite() || refined_axis.norm() <= 1e-6f) {
            LOG_WARN("RefineCylinderAxisAndRadius failed: initial axis invalid.");
            return false;
        }
    } else {
        refined_axis.normalize();
    }
    // 记录初始轴向用于“最大偏差约束”，防止细化跑偏到错误主方向。
    const Eigen::Vector3f initial_axis = refined_axis;

    // 7) 迭代中心初值设置为质心。
    Eigen::Vector3f refined_center = centroid;

    // 8) 细化过程参数（切片拟圆 + 圆心拟线 + 鲁棒阈值）。
    constexpr float kSliceStep = 5.0f;
    constexpr float kSliceThickness = 0.6f;
    constexpr float kHalfThickness = kSliceThickness * 0.5f;
    constexpr int kMinSlicePoints = 80;
    constexpr int kMinInlierPoints = 50;
    constexpr float kCircleInlierSigmaScale = 2.5f;
    constexpr float kCircleInlierThrMin = 0.5f;
    constexpr float kCircleInlierThrMax = 2.0f;
    constexpr float kCenterlineInlierSigmaScale = 2.0f;
    constexpr float kCenterlineInlierThrMin = 0.5f;
    constexpr float kCenterlineInlierThrMax = 3.0f;
    constexpr int kMaxAxisRefineIter = 5;
    constexpr float kAxisConvergeRad = 1e-4f;
    constexpr float kMaxAxisDeviationDeg = 10.0f;
    const float kMaxAxisDeviationCos = std::cos(kMaxAxisDeviationDeg * static_cast<float>(M_PI) / 180.0f);

    // 9) 子过程A：在给定(axis, center)下做轴向切片拟圆，收集每片圆心(3D)。
    auto fit_circle_centers_for_axis =
        [&](const Eigen::Vector3f& axis_fit,
            const Eigen::Vector3f& center_fit,
            std::vector<Eigen::Vector3f>& circle_centers) -> bool {
            circle_centers.clear();

            struct Sample2D {
                float t = 0.0f;
                float x = 0.0f;
                float y = 0.0f;
            };

            Eigen::Vector3f u, v;
            build_basis(axis_fit, u, v);

            // 9.1) 预计算所有点在当前轴系下的轴向坐标 t 与径向二维坐标(x,y)。
            std::vector<Sample2D> samples;
            samples.reserve(finite_points.size());
            float t_min = std::numeric_limits<float>::max();
            float t_max = std::numeric_limits<float>::lowest();

            for (const auto& pt : finite_points) {
                const Eigen::Vector3f d = pt - center_fit;
                const float t = d.dot(axis_fit);
                const Eigen::Vector3f radial = d - t * axis_fit;
                samples.push_back({t, radial.dot(u), radial.dot(v)});
                t_min = std::min(t_min, t);
                t_max = std::max(t_max, t);
            }

            if (samples.size() < 1000) return false;
            const float t_range = t_max - t_min;
            if (!std::isfinite(t_range) || t_range < kSliceStep) return false;

            // 9.2) 沿轴向等步长取切片：每片先拟合粗圆，再用MAD残差筛内点重拟合。
            for (float t_center = t_min; t_center <= t_max; t_center += kSliceStep) {
                std::vector<Eigen::Vector2d> pts2d;
                pts2d.reserve(256);
                for (const auto& s : samples) {
                    if (std::abs(s.t - t_center) <= kHalfThickness) {
                        pts2d.emplace_back(static_cast<double>(s.x), static_cast<double>(s.y));
                    }
                }
                if (static_cast<int>(pts2d.size()) < kMinSlicePoints) continue;

                // 9.2.1) 首次拟合圆（粗圆）。
                Eigen::Vector3d c0 = fitCircle2D(pts2d);
                if (!std::isfinite(c0[2]) || c0[2] <= 1e-6) continue;

                // 9.2.2) 计算每个点到粗圆的径向残差。
                std::vector<float> residuals;
                residuals.reserve(pts2d.size());
                for (const auto& q : pts2d) {
                    const float dx = static_cast<float>(q.x() - c0[0]);
                    const float dy = static_cast<float>(q.y() - c0[1]);
                    residuals.push_back(std::abs(std::sqrt(dx * dx + dy * dy) - static_cast<float>(c0[2])));
                }

                // 9.2.3) 根据 MAD 自适应阈值筛选圆内点。
                const float med = median_of(residuals);
                const float mad = mad_of(residuals, med);
                const float sigma = std::max(1e-4f, 1.4826f * mad);
                const float inlier_thr = std::clamp(kCircleInlierSigmaScale * sigma,
                                                    kCircleInlierThrMin,
                                                    kCircleInlierThrMax);

                std::vector<Eigen::Vector2d> inlier_pts2d;
                inlier_pts2d.reserve(pts2d.size());
                for (size_t i = 0; i < pts2d.size(); ++i) {
                    if (residuals[i] <= inlier_thr) inlier_pts2d.push_back(pts2d[i]);
                }
                if (static_cast<int>(inlier_pts2d.size()) < kMinInlierPoints) continue;

                // 9.2.4) 用内点重拟合圆，得到更稳定圆心。
                Eigen::Vector3d c1 = fitCircle2D(inlier_pts2d);
                if (!std::isfinite(c1[2]) || c1[2] <= 1e-6) continue;

                // 9.2.5) 将切片二维圆心回投到三维，加入圆心集合。
                const Eigen::Vector3f circle_center_3d =
                    center_fit + t_center * axis_fit
                    + static_cast<float>(c1[0]) * u
                    + static_cast<float>(c1[1]) * v;
                circle_centers.push_back(circle_center_3d);
            }
            // 至少需要足够多切片圆心才认为该轮可用。
            return circle_centers.size() >= 6;
        };

    // 10) 子过程B：对切片圆心拟合轴线（PCA主方向 + MAD离群抑制）。
    auto fit_line_from_centers =
        [&](const std::vector<Eigen::Vector3f>& centers_in,
            const Eigen::Vector3f& axis_ref,
            Eigen::Vector3f& line_point,
            Eigen::Vector3f& line_dir) -> bool {
            if (centers_in.size() < 3) return false;
            std::vector<Eigen::Vector3f> working = centers_in;

            // 两轮：第一轮拟线并剔除离群点；第二轮在净化数据上定最终轴线。
            for (int pass = 0; pass < 2; ++pass) {
                Eigen::Vector3f mu = Eigen::Vector3f::Zero();
                for (const auto& c : working) mu += c;
                mu /= static_cast<float>(working.size());

                // 10.1) PCA 提取圆心集合的主方向作为轴线候选。
                Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
                for (const auto& c : working) {
                    const Eigen::Vector3f d = c - mu;
                    cov += d * d.transpose();
                }

                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(cov);
                if (eig.info() != Eigen::Success) return false;
                Eigen::Vector3f dir = eig.eigenvectors().col(2).normalized();
                if (!dir.allFinite() || dir.norm() <= 1e-6f) return false;

                // 10.2) 方向同向化：保持与参考轴同向，减少符号抖动。
                if (dir.dot(axis_ref) < 0.0f) dir = -dir;

                // 限制细化轴向相对初始轴向的偏差，防止跑到错误主方向。
                if (std::abs(dir.dot(initial_axis)) < kMaxAxisDeviationCos) {
                    LOG_WARN("RefineCylinderAxisAndRadius failed: axis deviates too much from initial axis.");
                    return false;
                }

                // 10.3) 计算每个圆心到候选轴线的距离。
                std::vector<float> dists;
                dists.reserve(working.size());
                for (const auto& c : working) {
                    const Eigen::Vector3f v = c - mu;
                    dists.push_back((v - v.dot(dir) * dir).norm());
                }

                // 10.4) 依据 MAD 设定离群阈值并过滤圆心。
                const float med = median_of(dists);
                const float mad = mad_of(dists, med);
                const float sigma = std::max(1e-4f, 1.4826f * mad);
                const float dist_thr = std::clamp(kCenterlineInlierSigmaScale * sigma,
                                                  kCenterlineInlierThrMin,
                                                  kCenterlineInlierThrMax);

                std::vector<Eigen::Vector3f> filtered;
                filtered.reserve(working.size());
                for (size_t i = 0; i < working.size(); ++i) {
                    if (dists[i] <= dist_thr) filtered.push_back(working[i]);
                }

                // 10.5) 第1轮若确实过滤到异常点，则用过滤后数据再来一轮。
                if (pass == 0 && filtered.size() >= 3 && filtered.size() < working.size()) {
                    working.swap(filtered);
                    continue;
                }

                // 10.6) 输出轴线：线上一点(mu) + 方向(dir)。
                line_point = mu;
                line_dir = dir;
                return true;
            }
            return false;
        };

    // 11) 主迭代：交替执行“切片拟圆心”和“圆心拟轴线”直到收敛。
    bool refined_ok = false;
    for (int iter = 0; iter < kMaxAxisRefineIter; ++iter) {
        std::vector<Eigen::Vector3f> circle_centers;
        if (!fit_circle_centers_for_axis(refined_axis, refined_center, circle_centers)) break;

        Eigen::Vector3f line_point, line_dir;
        if (!fit_line_from_centers(circle_centers, refined_axis, line_point, line_dir)) break;

        // 11.1) 计算本轮轴向变化角，作为收敛判据。
        const float dot_val = std::clamp(refined_axis.dot(line_dir), -1.0f, 1.0f);
        const float axis_delta = std::acos(dot_val);

        // 11.2) 更新轴向与中心（中心投影到新轴线上，减少漂移）。
        refined_axis = line_dir;
        refined_center = line_point + refined_axis * (refined_center - line_point).dot(refined_axis);
        refined_ok = true;
        if (axis_delta < kAxisConvergeRad) break;
    }

    if (!refined_ok) {
        LOG_WARN("RefineCylinderAxisAndRadius failed: axis refinement did not converge.");
        return false;
    }

    // 12) 在最终轴线上统计所有点径向距离，中位数作为稳健半径估计。
    std::vector<float> radial_distances;
    radial_distances.reserve(finite_points.size());
    for (const auto& pt : finite_points) {
        const Eigen::Vector3f d = pt - refined_center;
        const float t = d.dot(refined_axis);
        const float radial = (d - t * refined_axis).norm();
        radial_distances.push_back(radial);
    }
    if (radial_distances.size() < 1000) return false;

    const float radius_med = median_of(radial_distances);
    if (!std::isfinite(radius_med) || radius_med <= 1e-6f) return false;

    // 13) 输出结果：精化轴向、中心、半径。
    axis = refined_axis.normalized();
    center = refined_center;
    radius = static_cast<double>(radius_med);
    return true;
}

/*****************************************
 * @brief                   检测曲面上的孔洞
 * 
 * @param cloud             输入点云
 * @param center            输出孔洞中心
 * @param radius            输出孔洞半径
 * @param axis              输出孔洞轴
 * 
 * @return void
 *****************************************/
// void Utils::DetectHoleInCurvedSurface(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
//                                     Eigen::Vector3f& center,float& radius,
//                                     Eigen::Vector3f& axis)
// {
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_valid(new pcl::PointCloud<pcl::PointXYZ>);
//     std::vector<int> valid_indices;

//     pcl::removeNaNFromPointCloud(*cloud, *cloud_valid, valid_indices);

//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smooth(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
//     // 1. MLS 平滑（抑制波浪噪声）
//     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//     tree->setInputCloud(cloud_valid);

//     pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
//     mls.setInputCloud(cloud_valid);
//     mls.setSearchMethod(tree);
//     mls.setComputeNormals(true);
//     mls.setSearchRadius(5.0f);    // 1mm
//     mls.setPolynomialOrder(2);      // 2次多项式拟合
//     mls.setNumberOfThreads(8);      // 8线程并行处理
//     mls.process(*cloud_with_normals);
//     pcl::io::savePLYFileASCII("../data/mls_cloud.ply", *cloud_with_normals);

    

//     // 2: 提取高曲率点（孔边缘）
//     pcl::PointCloud<pcl::PointXYZ>::Ptr hole_edge(new pcl::PointCloud<pcl::PointXYZ>);
//     float curvature_threshold = 0.005f;
//     for (const auto& pt : cloud_with_normals->points) {
//         if (!std::isnan(pt.curvature) && pt.curvature > curvature_threshold) {
//             hole_edge->push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
//         }
//     }

//     if (hole_edge->empty()) {
//         std::cerr << "No high-curvature points found." << std::endl;
//         return;
//     }
//     pcl::io::savePLYFileASCII("../data/hole_edge_cloud.ply", *hole_edge);

//     // 3: 欧氏聚类（分离孔边缘）
//     tree->setInputCloud(hole_edge);
//     std::vector<pcl::PointIndices> clusters;
//     pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
//     ec.setInputCloud(hole_edge);
//     ec.setClusterTolerance(2.0f);   // 0.1mm
//     ec.setMinClusterSize(100);
//     ec.setMaxClusterSize(100000);
//     ec.setSearchMethod(tree);
//     ec.extract(clusters);

//     // 找最大簇（只有一个孔）
//     pcl::PointCloud<pcl::PointXYZ>::Ptr largest_cluster(new pcl::PointCloud<pcl::PointXYZ>);
//     size_t max_size = 0;
//     for (const auto& cluster : clusters) {
//         if (cluster.indices.size() > max_size) {
//             max_size = cluster.indices.size();
//             largest_cluster->clear();
//             for (size_t idx : cluster.indices) {
//                 largest_cluster->push_back(hole_edge->points[idx]);
//             }
//         }
//     }


//     if (largest_cluster->size() < 10) {
//         std::cerr << "No valid hole edge cluster found." << std::endl;
//         return;
//     }
//     pcl::io::savePLYFileASCII("../data/largest_hole_edge_cloud.ply", *largest_cluster);

//     pcl::PointCloud<pcl::PointXYZ>::Ptr sphere_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     double radius_double, height_double;
//     std::vector<float> radius_limits = {8.0f, 10.0f};
//     pcl::PointCloud<pcl::PointNormal>::Ptr n_cloud(new pcl::PointCloud<pcl::PointNormal>);
//     pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//     ComputePointCloudNormal(largest_cluster, normals, n_cloud, 3.0f);
//     FitCylinder(n_cloud, sphere_cloud, center, radius_double, height_double, axis, 2.0f, radius_limits);
// }

/*********************************************************
 * @brief 裁剪圆柱表面范围内的障碍物点云
 * 
 * @param cloud 输入点云
 * @param center 圆柱中心点
 * @param axis 圆柱轴向
 * @param radius 圆柱半径
 * @param height 圆柱高度
 * @param margin 外边距（默认 0.3）
 * @param barrier_cloud 输出裁剪后的点云
 * 
 * @return void
 *********************************************************/
void Utils::ClipPointCloudInCylinderShell(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const Eigen::Vector3f& center,
    const Eigen::Vector3f& axis,     
    float radius,
    float height,
    float margin,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& barrier_cloud)
{
    barrier_cloud->clear();
    if (!cloud || cloud->empty()) return;

    Eigen::Vector3f a = axis.normalized(); // 确保单位向量
    float h_half = height * 0.5f;

    for (const auto& pt : cloud->points) {
        if (!pcl::isFinite(pt)) continue;

        Eigen::Vector3f p(pt.x, pt.y, pt.z);
        Eigen::Vector3f d = p - center;
        float t = a.dot(d); // 轴向坐标（以 center 为原点）

        // 径向向量（垂直于轴）
        Eigen::Vector3f radial = d - t * a;
        float radial_dist = radial.norm();      // 径向距离（到圆柱轴向的距离）

        float distance = std::numeric_limits<float>::max();

        if (t >= -h_half && t <= h_half) {
            // 情况 A：在圆柱高度范围内 → 最近点在侧面
            distance = std::abs(radial_dist - radius);
            // 注意：若 radial_dist < radius，则点在内部
        }
        else if (t > h_half) {
            // 情况 B：在顶部上方
            float dz = t - h_half;
            if (radial_dist <= radius) {
                distance = dz; // 正对顶面
            } else {
                float dr = radial_dist - radius;
                distance = std::sqrt(dz * dz + dr * dr); // 到顶面边缘
            }
        }
        else { // t < -h_half
            // 情况 C：在底部下方
            float dz = -h_half - t;
            if (radial_dist <= radius) {
                distance = dz;
            } else {
                float dr = radial_dist - radius;
                distance = std::sqrt(dz * dz + dr * dr);
            }
        }

        // 判断：是否在外部且距离 <= margin
        bool inside = false;
        if (t >= -h_half-0.03 && t <= h_half+0.03) { // 允许点误差范围在圆柱高度范围内 3cm 内(根据Mid360手册误差设置)
            inside = (radial_dist <= radius+0.03);
        } 

        // 不在圆柱内部且距离 <= margin 时，加入障碍物点云中
        if (!inside && distance <= margin) {
            barrier_cloud->push_back(pt);
        }
    }
}


/**
 * @brief                   旋转点云使其轴与 z 轴对齐
 * 
 * @param cloud             输入点云指针
 * @param rotated_cloud     输出旋转点云指针
 * @param axis              旋转轴，默认 z 轴
 * 
 * @return void
 */
void Utils::RotatePointsAlignedWithZAxis(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                        pcl::PointCloud<pcl::PointXYZ>::Ptr& rotated_cloud,
                                        const Eigen::Vector3f& axis)
{                                         
    // 1. 归一化输入轴向
    Eigen::Vector3f src_axis = axis.normalized();
  
    Eigen::Vector3f tgt_axis = Eigen::Vector3f::UnitZ();

    // 3. 构造旋转四元数
    Eigen::Quaternionf q;
    q.setFromTwoVectors(src_axis, tgt_axis);
    Eigen::Matrix3f R = q.toRotationMatrix();

    // 4. 构造 4x4 仿射变换矩阵（无平移）
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.linear() = R;

    // 5. 应用变换
    pcl::transformPointCloud(*cloud, *rotated_cloud, transform);

    LOG_INFO("Cylinder axis aligned to Z-axis.");
}




/***************************************
 * @brief               每隔指定度数沿圆柱轴向切平面，将圆柱表面上的点云投影到该轴向平面上
 * @param cloud         输入点云指针
 * @param segment_cloud 输出分割后的点云指针
 * @param center        圆柱中心（3D向量）
 * @param axis          圆柱轴向（必须是单位向量）
 * @param projected_points 输出投影到该平面的2D点集
 * @param angle_step    切割平面角度步长，默认30度
 * @return void
 *****************************************/
void Utils::SegmentCylindAlignAxis(
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& segment_cloud,
                        const Eigen::Vector3f& center,
                        const Eigen::Vector3f& axis,
                        std::vector<std::vector<pcl::PointXY>>& projected_points_per_plane,
                        const float angle_step_deg,
                        const float distance_tolerance               
)
{
    Eigen::Vector3f unit_axis = axis.normalized();

    // 构建初始径向参考方向（⊥ axis）
    Eigen::Vector3f ref_dir;
    if (std::abs(unit_axis.dot(Eigen::Vector3f::UnitZ())) < 0.99f)
        ref_dir = unit_axis.cross(Eigen::Vector3f::UnitZ()).normalized();
    else
        ref_dir = unit_axis.cross(Eigen::Vector3f::UnitX()).normalized();

    int num_planes = static_cast<int>(360.0f / angle_step_deg + 0.5f);
    if (num_planes <= 0) num_planes = 12;

    segment_cloud.clear();
    projected_points_per_plane.clear();
    segment_cloud.resize(num_planes);
    projected_points_per_plane.resize(num_planes);

    for (int i = 0; i < num_planes; ++i) {
        segment_cloud[i] = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    }

    const float deg2rad = M_PI / 180.0f;
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    int segment_id = 0;

    // 遍历每个切割平面
    for (int i = 0; i < num_planes; ++i) {
        float angle_rad = i * angle_step_deg * deg2rad;
        Eigen::AngleAxisf rot(angle_rad, unit_axis);
        Eigen::Vector3f radial = (rot * ref_dir).normalized(); // 当前径向方向

        // 切割平面的法向量：垂直于 (radial, axis) 平面
        Eigen::Vector3f plane_normal = radial.cross(unit_axis).normalized();

        auto& current_cloud = *segment_cloud[i];
        auto& current_2d = projected_points_per_plane[i];

        // 遍历所有输入点
        for (const auto& pt : cloud->points) {
            Eigen::Vector3f p(pt.x, pt.y, pt.z);
            Eigen::Vector3f dp = p - center;

            // 计算点到平面的距离（带符号）
            float dist = plane_normal.dot(dp);
            if (std::abs(dist) > distance_tolerance) continue;

            // 保留该点（3D）
            current_cloud.push_back(pt);

            // 正交投影到平面
            Eigen::Vector3f p_proj = p - dist * plane_normal;

            // 在平面局部坐标系下计算2D坐标
            Eigen::Vector3f local_dp = p_proj - center;
            float u = local_dp.dot(radial);      // 径向
            float v = local_dp.dot(unit_axis);   // 轴向

            current_2d.push_back({u, v});
        }
        *temp_cloud += current_cloud;
        if (current_2d.empty()) continue;
        WritePointsToTxt(current_2d, "../data/2D_Points/segment_cylind_cloud_" + std::to_string(segment_id) + ".txt");
        segment_id++;
    }

    // 移除空的2D点集
    projected_points_per_plane.erase(
        std::remove_if(
            projected_points_per_plane.begin(),
            projected_points_per_plane.end(),
            [](const std::vector<pcl::PointXY>& v) {
                return v.empty();
            }
        ),
        projected_points_per_plane.end()
    );

    pcl::io::savePLYFileASCII("../data/segment_cylind_cloud.ply", *temp_cloud);
}

/***************************************
 * @brief               每隔指定度数沿圆柱轴向切平面，将圆柱表面上的点云投影到该轴向平面上
 * @param cloud         输入点云指针
 * @param segment_cloud 输出分割后的点云指针
 * @param center        圆柱中心（3D向量）
 * @param axis          圆柱轴向（必须是单位向量）
 * @param projected_points 输出投影到该平面的2D点集
 * @param angle_step    切割平面角度步长，默认30度
 * @return void
 *****************************************/
void Utils::SegmentCylindAlignAxis(
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& vertical_cloud,
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& horizontal_cloud,
                        const Eigen::Vector3f& center,
                        const Eigen::Vector3f& axis,
                        std::vector<std::vector<pcl::PointXY>>& vertical_projected_points_per_plane,
                        std::vector<std::vector<pcl::PointXY>>& horizontal_projected_points_per_plane,
                        const float angle_step_deg,
                        const float distance_tolerance               
)
{
    Eigen::Vector3f unit_axis = axis.normalized();

    // 构建初始径向参考方向（axis）
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

        // 遍历所有输入点
        for (const auto& pt : vertical_cloud->points) {
            Eigen::Vector3f p(pt.x, pt.y, pt.z);
            Eigen::Vector3f dp = p - center;

            // 计算点到平面的距离（带符号）
            float dist = plane_normal.dot(dp);
            if (std::abs(dist) > distance_tolerance) continue;

            // 正交投影到平面
            Eigen::Vector3f p_proj = p - dist * plane_normal;

            // 在平面局部坐标系下计算2D坐标
            Eigen::Vector3f local_dp = p_proj - center;
            float u = local_dp.dot(radial);      // 径向
            float v = local_dp.dot(unit_axis);   // 轴向

            vertical_current_2d.push_back({u, v});
        }
        for (const auto& pt : horizontal_cloud->points) {
            Eigen::Vector3f p(pt.x, pt.y, pt.z);
            Eigen::Vector3f dp = p - center;

            // 计算点到平面的距离（带符号）
            float dist = plane_normal.dot(dp);
            if (std::abs(dist) > distance_tolerance) continue;

            // 正交投影到平面
            Eigen::Vector3f p_proj = p - dist * plane_normal;

            // 在平面局部坐标系下计算2D坐标
            Eigen::Vector3f local_dp = p_proj - center;
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
    for (int j = (int)vertical_projected_points_per_plane.size() - 1; j >= 0; j--) {
        if (vertical_projected_points_per_plane[j].empty() &&
            horizontal_projected_points_per_plane[j].empty()) {
            vertical_projected_points_per_plane.erase(vertical_projected_points_per_plane.begin() + j);
            horizontal_projected_points_per_plane.erase(horizontal_projected_points_per_plane.begin() + j);
        }
    }

}


/*****************************************
 * @brief               从轴平面投影2D点集中拟合直线
 * @param points         输入二维点集
 * @param is_vertical    是否为垂直方向直线
 * @param line_coeffs    输出直线参数
 * @param segment_id     切片id
 * @return void
 *****************************************/
void Utils::FitLineFromPoints(
                        const std::vector<pcl::PointXY>& points,
                        std::vector<pcl::ModelCoefficients>& line_coeffs,
                        int& segment_id,
                        bool is_vertical)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr line_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& pt : points) {
        line_cloud->push_back(pcl::PointXYZ(pt.x, pt.y, 0.0f));   
    }
    // Utils::StatisticalOutlierRemoval(line_cloud, line_cloud, 3);

    std::vector<float> line_lengths; 
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

    // 拟合直线
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.2f);
    seg.setInputCloud(line_cloud);

    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        return;
    }

    // 提取直线参数
    const auto& c = coefficients->values;
    if (c.size() != 6) {
        LOG_ERROR("Invalid line model coefficients!");
        return;
    }

    Eigen::Vector3f P0(c[0], c[1], c[2]);          // 直线上一点
    Eigen::Vector3f direction(c[3], c[4], c[5]);   // 方向向量

    float dir_norm = direction.norm();
    if (dir_norm < 1e-8f) {
        LOG_ERROR("Degenerate line direction!");
        return;
    }

    Eigen::Vector3f d_unit = direction / dir_norm;

    // 计算所有内点在直线上的投影参数 t
    std::vector<float> t_values;
    t_values.reserve(inliers->indices.size());

    for (const auto& idx : inliers->indices) {
        const pcl::PointXYZ& pt = line_cloud->points[idx];
        Eigen::Vector3f v(pt.x - P0.x(), pt.y - P0.y(), pt.z - P0.z());
        float t = v.dot(d_unit);  // 投影标量（沿单位方向）
        t_values.push_back(t);
    }

    // 计算投影区间长度
    auto [t_min_it, t_max_it] = std::minmax_element(t_values.begin(), t_values.end());
    float length = *t_max_it - *t_min_it;

    Eigen::Vector3f max_coord = P0 + d_unit*(*t_max_it);
    Eigen::Vector3f min_coord = P0 + d_unit*(*t_min_it);
    std::ofstream file(
    std::string("../data/2D_Points/") + (is_vertical ? "vertical" : "horizontal") +
                "_segment_line_cloud_" + std::to_string(segment_id) + "_.obj");
    if (!file.is_open()) {
        LOG_ERROR(std::string("Error: Could not open file ") + (is_vertical ? "vertical" : "horizontal") +
        "_segment_line_cloud_" + std::to_string(segment_id) + ".obj for writing.");
        return;
    }
    file << "v " << max_coord.x() << " " << max_coord.y() << " " << max_coord.z() << std::endl;
    file << "v " << min_coord.x() << " " << min_coord.y() << " " << min_coord.z() << std::endl;
    file << "l 1 2" << std::endl;
    file.close();

    LOG_INFO("ID " << segment_id << " 拟合成功直线: P0=("
             << P0.x() << ", " << P0.y() << ", " << P0.z() << "), dir=("
             << d_unit.x() << ", " << d_unit.y() << ", " << d_unit.z()
             << "), length=" << length);
    // 保存结果
    line_coeffs.push_back(*coefficients);
    line_lengths.push_back(length);
}



// 过滤主曲率大于阈值的点云
pcl::PointCloud<pcl::PointXYZ>::Ptr Utils::FilterPrincipalCurvature(
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                        const float curvature_threshold)
{
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    // ne.setNumberOfThreads(8);
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(5.0f);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normals);

    // 计算主曲率
    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> pce;
    pce.setInputCloud(cloud);
    pce.setInputNormals(normals);
    pce.setSearchMethod(tree);
    pce.setRadiusSearch(5.0f);
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principal_curvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>);
    pce.compute(*principal_curvatures);

    auto [t_min_it, t_max_it] = std::minmax_element(principal_curvatures->points.begin(), principal_curvatures->points.end(),
        [](const pcl::PrincipalCurvatures& a, const pcl::PrincipalCurvatures& b) {
            return fabs(a.pc1) < fabs(b.pc1);
        });
    LOG_INFO("主曲率范围: [" << t_min_it->pc1 << ", " << t_max_it->pc1 << "]");

    // 过滤主曲率大于阈值的点
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        if (fabs(principal_curvatures->points[i].pc1) < curvature_threshold) {
            filtered_cloud->push_back(cloud->points[i]);
        }
    }
    return filtered_cloud;

}

/*****************************************
 * @brief                   计算点云到圆柱轴的径向距离的中位数和均值
 * 
 * @param cloud             输入点云指针
 * @param center            圆柱轴上一点（3D向量）
 * @param axis              圆柱轴向
 * @param mean_dis          输出均值
 * @return float            输出中位数
 *****************************************/
float Utils::MeanDistanceToAxis(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const Eigen::Vector3f& center,
    const Eigen::Vector3f& axis,
    float& mean_dis
)
{
    Eigen::Vector3f a = axis.normalized();
    std::vector<float> distances;
    distances.reserve(cloud->points.size());

    for (const auto& pt : cloud->points) {
        if (!pcl::isFinite(pt)) continue;

        Eigen::Vector3f p(pt.x, pt.y, pt.z);
        Eigen::Vector3f d = p - center;
        float t = a.dot(d); // 轴向坐标（以 center 为原点）

        // 径向向量（垂直于轴）
        Eigen::Vector3f radial = d - t * a;
        float radial_dist = radial.norm();      // 径向距离（到圆柱轴心的距离）

        distances.push_back(radial_dist);
    }

    std::sort(distances.begin(), distances.end());
    std::vector<float> filtered_distances;
    filtered_distances.assign(distances.begin() + distances.size() * 0.1, distances.end() - distances.size() * 0.1);

    mean_dis = filtered_distances.empty() ? 0.0f :
        std::accumulate(filtered_distances.begin(), filtered_distances.end(), 0.0f) / filtered_distances.size();

    return distances[distances.size() / 2]; // 返回中位数
}

/*****************************************
 * @brief                   计算点云在圆柱轴向的高度
 * 
 * @param cloud             输入点云指针
 * @param center            圆柱轴上一点（3D向量）
 * @param axis              圆柱轴向
 * @param radius            圆柱半径
 * @param height            输出高度
 * @return float            输出高度范围（max - min）
 *****************************************/
void Utils::ComputeCylinderHeightAlongAxis(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const Eigen::Vector3f& center,
    const Eigen::Vector3f& axis,
    const double& radius,
    float& height
)
{
    Eigen::Vector3f origin(center.x(), center.y(), center.z());
    Eigen::Vector3f dir(axis.x(), axis.y(), axis.z());
    dir.normalize(); // 确保是单位向量

    std::vector<float> projections;
    projections.reserve(cloud->points.size());

    for (const auto& pt: cloud->points) {
        Eigen::Vector3f vec(pt.x - origin.x(), pt.y - origin.y(), pt.z - origin.z());
        float proj = vec.dot(dir);  // proj 是有符号距离
        float radial_dist = (vec - proj * dir).norm();
        if (radial_dist > radius + 0.5f) continue; //
        projections.push_back(proj);
    }

    float min_proj = *std::min_element(projections.begin(), projections.end());
    float max_proj = *std::max_element(projections.begin(), projections.end());
    height = max_proj - min_proj; // 圆柱高度（长度）
}

/***********************************
 * @brief               根据点云法向过滤水平和垂直点云
 * @param cloud         输入点云指针
 * @param hrizontal_cloud 输出水平点云指针
 * @param vertical_cloud  输出垂直点云指针
 * 
 * @return void
 *****************************************/
void Utils::FilteredPointsByNormal(
                        const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr& hrizontal_cloud,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr& vertical_cloud)
{
    hrizontal_cloud->clear();
    vertical_cloud->clear();

    for (const auto& pt : cloud->points) {
        if (!pcl::isFinite(pt)) continue;

        Eigen::Vector3f normal(pt.normal_x, pt.normal_y, pt.normal_z);
        if (normal.dot(Eigen::Vector3f(0, 0, 1)) < 0.8f && normal.dot(Eigen::Vector3f(0, 0, 1)) > -0.8f) {
            vertical_cloud->push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
        } else {
            hrizontal_cloud->push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
        }
    }
}
