#ifndef POINTSMODEL_H
#define POINTSMODEL_H

#include <memory>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/search/kdtree.h>

#include <utility>   
#include <algorithm>  
#include <vector>   


#define INVALID_VALUE -9999.0f // 无效值

// 圆柱模型信息
struct CylinderModel
{
    Eigen::Vector3f center; // 圆柱中心
    Eigen::Vector3f axis; // 圆柱轴方向
    double radius; // 圆柱半径
    double height; // 圆柱高度
};


class PointsModel
{
public:


/*******************************************
 * @brief                   欧式聚类提取
 * @param cloud             输入点云指针
 * @param cluster_clouds    输出聚类点云指针向量
 * @param cluster_tolerance 聚类容差，默认 0.02
 * @param min_cluster_size  最小聚类点数，默认 100
 * @param max_cluster_size  最大聚类点数，默认 20000
 *******************************************/
bool EuclideanClusterExtraction(
    const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud,
    std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>& cluster_clouds,
    float cluster_tolerance = 0.5f,
    int min_cluster_size = 20);

/*****************************************
 * @brief                   拟合圆柱模型
 * 
 * @param cloud             输入点云指针
 * @param cylinder_cloud    输出圆柱点云指针
 * @param cylinder_model    输出圆柱模型
 * @param dist_threshold    点到圆柱距离阈值
 * @param radius_limits     半径范围限制
 * 
 * @return bool             是否成功拟合圆柱模型
 *****************************************/
bool FitCylinder(const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr& cylinder_cloud,
                        CylinderModel& cylinder_model,
                        const float dist_threshold,
                        std::vector<float> radius_limits);

/**
 * @brief 将圆柱点云的轴向旋转到与目标轴对齐
 * 
 * @param cloud_in 输入点云 (会被直接修改，如果想保留原数据请先拷贝)
 * @param cylinder_model 圆柱模型
 * @param target_axis 目标轴向 (例如: [1, 0, 0] 代表X轴)
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr RotatePointCloudAlignToAxis(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
    const CylinderModel& cylinder_model,
    const Eigen::Vector3f& target_axis);


/**
 * @brief 从对齐后的圆柱点云中提取圆环切片
 * 
 * @param aligned_cloud 对齐后的圆柱点云指针
 * @param slice_thickness 每个圆环轴向厚度，默认 5.0f
 * @param slice_center_step 切片中心间隔，默认 10.0f
 * 
 * @return std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> 圆环切片点云指针向量
 */
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> ExtractRingSlices(
const pcl::PointCloud<pcl::PointXYZ>::Ptr& aligned_cloud,
float slice_thickness = 5.0f, 
float slice_center_step = 10.0f);

/*****************************************
 * @brief           计算点云法向
 * @param cloud     输入点云指针
 * @param normals   输出法向点云指针
 *
 * @return void
 *****************************************/
void ComputePointCloudNormal(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, 
            pcl::PointCloud<pcl::Normal>::Ptr &_normal, 
            pcl::PointCloud<pcl::PointNormal>::Ptr &_ncloud,
            const float radius);  

/*****************************************
 * @brief 对点云进行MLS平滑（用于拟合前去噪稳健化）
 * @param cloud_in         输入点云
 * @param cloud_out        输出平滑点云
 * @param search_radius    MLS搜索半径
 * @param polynomial_order MLS多项式阶数
 * @param polynomial_fit   是否启用多项式拟合
 * @return bool            是否成功
 *****************************************/
bool SmoothPointCloudMLS(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,
                         float search_radius,
                         int polynomial_order = 2,
                         bool polynomial_fit = true);

/*****************************************
 * @brief 计算所有点云投影到圆柱轴线的投影高度
 * @param cloud  输入点云指针
 * @param center 轴线上的一点（原点）
 * @param axis   轴线方向（自动归一化）
 * @param height 输出投影高度（沿轴向的标量距离）
 *
 * @return void
 *****************************************/
void GetProjectedHeightOnAxis(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                                     const Eigen::Vector3f& center, 
                                     const Eigen::Vector3f& axis,
                                     float& height);

/*****************************************
 * @brief  计算圆柱筒体表面点云距离圆柱轴线的径向距离（单个点）
 * @param point                 输入点云指针
 * @param cylinder_model        圆柱模型
 *****************************************/
float ComputeRadialDistanceToAxis(const pcl::PointXYZ& point, const CylinderModel& cylinder_model);


/*****************************************
 * @brief                   根据圆柱表面点云到轴线距离过滤焊缝
 * 
 * @param cloud             输入点云指针
 * @param cylinder_model    圆柱模型
 * @param distance_tolerance 距离容差
 * @param seam_cloud        输出焊缝点云指针
 * @param surface_cloud     输出圆柱表面点云指针
 * @return void
 *****************************************/
bool ExtractWeldSeamByDistance(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& seam_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& surface_cloud,
    const float distance_tolerance_);

/*
 * @brief               将点云在Z轴方向上展平
 * @param cloud_in         输入点云指针
 * @return                 展平后的点云指针
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr FlattenZ_Simple(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in);


/***************************************
 * @brief               求焊缝点云的主方向
 * 
****************************************/
void GetMainDirection(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                Eigen::Vector3f& main_direction);

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
void ClipCylinderCrossSection(
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                        const Eigen::Vector3f& center,
                        const Eigen::Vector3f& axis,      
                        const float radius,
                        const float height,
                        const float dist_step,
                        std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>& cross_sections);

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
void GetDiameterRoundness(std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>& cross_sections,
                                const Eigen::Vector3f& center,
                                const Eigen::Vector3f& axis,
                                float& roundnessError);


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
void SegmentCylindAlignAxis(
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& vertical_cloud,
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& horizontal_cloud,
                        const CylinderModel& cylinder_model,
                        std::vector<std::vector<pcl::PointXY>>& vertical_projected_points_per_plane,
                        std::vector<std::vector<pcl::PointXY>>& horizontal_projected_points_per_plane,
                        const float angle_step_deg,
                        const float distance_tolerance               
);

/*****************************************
 * @brief                   从圆柱不同截面切平面拟合多个直线模型
 * @param cloud             输入点云指针
 * @param cylinder_model    圆柱模型
 * @param line_coeffs       输出直线参数向量
 * @param line_clouds       输出直线内点云指针向量
 * @param angle_step_deg    切割平面角度步长，默认10度
 * @param slice_distance_tolerance  切平面距离阈值，默认0.2mm
 * @param line_fit_distance_tolerance  直线拟合距离阈值，默认0.2mm
 * @return void
 *****************************************/
void FitLinesOnCylindricalSlices(
    const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud,
    const CylinderModel& cylinder_model,
    std::vector<std::vector<pcl::ModelCoefficients>>& line_coeffs,
    std::vector<std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>>& line_clouds,
    float angle_step_deg=10.0f,
    float slice_distance_tolerance=0.2f,
    float line_fit_distance_tolerance=0.2f
);


/***************************************************** 
 * @brief                   计算圆柱不同截面垂直边缘的高度
 * @param line_clouds       每个直边拟合直线的内点
 * @param line_coeffs       每个直边拟合直线的参数
 * @param vertical_edge_height  输出垂直边缘的高度
 * @return void
****************************************************/
void ComputeVerticalEdgeHeight(
    const std::vector<std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>>& line_clouds,
    const std::vector<std::vector<pcl::ModelCoefficients>>& line_coeffs,
    std::vector<float>& vertical_edge_height);

/*****************************************
 * @brief                   分类判断圆柱垂直边缘的倾斜方向
 * @param line_coeff        输入直线参数
 * @param cylinder_model    圆柱模型
 * @return std::string      倾斜方向（outward, inward, circumferential, vertical）
 *****************************************/
std::string classifyTiltDirection(const pcl::ModelCoefficients& line_coeff,const CylinderModel& cylinder_model);

/*****************************************
 * @brief                       计算封头直边的斜度
 * @param vertical_edge_height  输入封头直边高度
 * @param line_coeffs           输入直边拟合直线的参数
 * @param cylinder_model        圆柱模型
 * @param vertical_edge_tilt    输出直边斜度(内倾为负，外倾为正)
 * @return void
 *****************************************/
void ClassifyVerticalEdgeTilt(
                        const std::vector<float>& vertical_edge_height,
                        const std::vector<std::vector<pcl::ModelCoefficients>>& line_coeffs,                      
                        const CylinderModel& cylinder_model,
                        std::vector<float>& vertical_edge_tilt
);

/*****************************************
 * @brief                       对直边高度进行过滤均值化
 * @param vertical_edge_height  输入封头直边高度
 * @param vertical_edge_tilt    输入直边拟合直边的斜度(内倾为负，外倾为正)
 * @param filter_ratio          输入两端过滤比例(默认0.2)
 * @param median_edge_height    输出直边高度的中位数
 * @param median_edge_tilt      输出直边斜度中位数(内倾为负，外倾为正)
 * @return void
 *****************************************/
 void GetMedianEdgeHeightAndTilt(
                        const std::vector<float>& vertical_edge_height,
                        const std::vector<float>& vertical_edge_tilt,
                        float& median_edge_height,
                        float& median_edge_tilt,
                        const float filter_ratio=0.2f);

/*****************************************
 * @brief                       计算封头坡口的角度
 * @param line_coeffs           输入封头直边和坡口斜边直线参数
 * @param head_angle            输出封头坡口角度滤波均值化角度[0,90]
 * @param filter_ratio          输入两端过滤比例(默认0.2)
 * @return void
 *****************************************/
void GetHeadAngle(std::vector<std::vector<pcl::ModelCoefficients>>& line_coeffs, float& head_angle, const float filter_ratio=0.2f);


/*****************************************
 * @brief                       从点云提取焊缝
 * @param cloud                 输入点云指针
 * @param weld_seam_cloud       输出焊缝点云指针
 * @param weld_seam_outliner    输出焊缝外点云指针
 * @param n_radius              法向计算半径(默认5.0)
 * @param curvature_threshold   输入曲率阈值(默认0.1)
 * @return bool                 是否成功提取焊缝
 *****************************************/
bool ExtractWeldSeamByCurvature(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr& weld_seam_cloud,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr& weld_seam_outliner,
                                const float n_radius=5.0f,
                                const float curvature_threshold=0.1f);
private:

    pcl::search::KdTree<pcl::PointXYZ>::Ptr pt_tree_ = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>(); // 点云搜索树
    pcl::search::KdTree<pcl::PointNormal>::Ptr npt_tree_ = std::make_shared<pcl::search::KdTree<pcl::PointNormal>>(); // 点云搜索树
    CylinderModel cylinder_model_; // 圆柱模型
};


#endif // POINTSMODEL_H
