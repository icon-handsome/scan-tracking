#ifndef UTILS_H
#define UTILS_H
#include <string>
#include <map>
#include <iostream>
#include <fstream>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "log_manager/LogMacros.h"

// struct _2D_Point
// {
//     float x;
//     float y;
// };

class Utils
{
private:
public:

// 检查点云是否为空
static inline void IsEmptyPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    if(cloud->points.size() == 0)
    {
        LOG_ERROR("input cloud is empty!");
        return;
    }
}
static inline void IsEmptyPoint(const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud)
{
    if(cloud->points.size() == 0)
    {
        LOG_ERROR("input cloud is empty!");
        return;
    }
}

/*******************************************
 * @brief                   体素滤波下采样
 * @param cloud             输入点云指针
 * @param downsampled_cloud 输出下采样点云指针
 * @param leaf_size         下采样体素，默认 0.05
 *******************************************/
static void VoxelGridDownSample(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& downsampled_cloud,
    float leaf_size = 0.05f
);

/*******************************************
 * @brief                   均匀下采样
 * @param cloud             输入点云指针
 * @param downsampled_cloud 输出下采样点云指针
 * @param leaf_size         下采样体素，默认 0.05
 *******************************************/
static void UniformDownSample(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& downsampled_cloud,
    float leaf_size = 0.1f
);

/*******************************************
 * @brief                   统计滤波
 * @param cloud             输入点云指针
 * @param downsampled_cloud 输出下采样点云指针
 * @param kNearest          最近邻点数量，默认 20
 *******************************************/
static void StatisticalOutlierRemoval(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& downsampled_cloud,
    int kNearest = 20
);


/******************************************
 * @brief                   高程带通滤波
 * @param cloud             输入点云指针
 * @param filtered_cloud    输出滤波点云指针
 * @param filtered_cloud_elevation  输出滤波点云高程向量(min,max)
 * @param axis              滤波轴，默认 z 轴
 *
 * @return void 
 ******************************************/
static void ElevationBilateralFilter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud,
    const std::vector<float>& filtered_cloud_elevation,
    const std::string& axis = "z"
);


/*****************************************
 * @brief                   拟合球模型
 * @param cloud             输入法带向点云指针
 * @param sphere_cloud      输出球点云指针
 * @param center            输出球中心
 * @param radius            输出球半径
 * 
 * ***************************************/
static void FitSphere(const pcl::PointCloud<pcl::PointXYZ>::Ptr& n_cloud,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr& sphere_cloud,
                        Eigen::Vector3f& center,
                        double& radius,
                        const float dist_threshold = 0.5f); 


/******************************************
 * @brief                   计算点云质心
 * 
 * @param cloud             输入点云指针
 * @param centroid          输出质心坐标
 * 
 * @return void
 *****************************************/
static void PlotSphere(const Eigen::Vector3f& center,
                        const double radius);


/*****************************************
 * @brief                   2D点集拟合圆
 * 
 * @param points            输入2D点集
 * 
 * @return Eigen::Vector3d  输出圆中心(x,y,r)
 *****************************************/
static Eigen::Vector3d fitCircle2D(const std::vector<Eigen::Vector2d>& points);

/*****************************************
 * @brief                   迭代精拟合圆柱轴向/中心/半径（切片拟圆 + 圆心拟轴线）
 * @param cloud             输入待拟合点云
 * @param radius            输出拟合圆柱半径
 * @param axis              输入粗轴向先验并输出精化轴向（单位向量）
 * @param center            输出精化后的轴线参考中心点
 * @return bool             true=拟合成功，false=拟合失败
 *****************************************/
static bool RefineCylinderAxisAndRadius(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    double& radius,
    Eigen::Vector3f& axis,
    Eigen::Vector3f& center
);


/*****************************************
 * @brief                   检测曲面上的孔洞
 * 
 * @param cloud             输入点云
 * @param center            输出孔洞中心
 * @param radius            输出孔洞半径
 * @param axis              输出孔洞轴
 * 
 * @return void
//  *****************************************/
// static void DetectHoleInCurvedSurface(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
//                                     Eigen::Vector3f& center,float& radius,
//                                     Eigen::Vector3f& axis);

/***
 * @brief                   旋转点云使其轴与 z 轴对齐
 * 
 * @param cloud             输入点云指针
 * @param rotated_cloud     输出旋转点云指针
 * @param axis              旋转轴，默认 z 轴
 * 
 * @return void
 */
static void RotatePointsAlignedWithZAxis(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                        pcl::PointCloud<pcl::PointXYZ>::Ptr& rotated_cloud,
                                        const Eigen::Vector3f& axis);


/*****************************************
 * @brief               圆柱沿轴向按指定步长截取横截面圆环点云
 * B
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
static void ClipCylinderCrossSection(
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                        const Eigen::Vector3f& center,
                        const Eigen::Vector3f& axis,      
                        const float radius,
                        const float height,
                        const float dist_step,
                        std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>& cross_sections
);

/****************************************
 * 
 * @brief                   根据圆柱不同截面拟合数据计算圆柱内径和圆度公差
 *   
 * @param cross_sections    输入圆柱不同截面点云指针
 * @param radius            输出圆柱最小内径(直径)
 * @param roundnessError    输出圆柱圆度误差(不同方向最大最小内径差) 
 * 
 * @return void
 *****************************************/
 static void GetCylinderRadius(std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>& cross_sections,
                                const Eigen::Vector3f& center,
                                const Eigen::Vector3f& axis,
                                float& inline_radius,
                                float& roundnessError);


/*****************************************
 * @brief                   求解两个轴的交点角度
 * 
 * @param _axis1            输入轴1（3D向量）
 * @param _axis2            输入轴2（3D向量）
 * @param angle             输出夹角角度（角度[0,90°]）
 * 
 * @return void
 *****************************************/
static void ComputeAngleBetweenVectors(const Eigen::Vector3f& _axis1,
                            const Eigen::Vector3f& _axis2,
                            float& angle);

/*****************************************
 * @brief               裁剪圆柱表面范围内的障碍物点云
 * 
 * @param cloud         输入点云指针
 * @param center        圆柱中心（3D向量）
 * @param axis          圆柱轴向（必须是单位向量）
 * @param radius        圆柱半径
 * @param height        圆柱高度
 * @param margin        圆柱表面范围扩展 margin
 * @param barrier_cloud 输出圆柱表面范围内的障碍物点云指针
 * 
 * @return void
 *****************************************/
static void ClipPointCloudInCylinderShell(
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                        const Eigen::Vector3f& center,
                        const Eigen::Vector3f& axis,      
                        float radius,
                        float height,
                        float margin,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr& barrier_cloud);
    

/*****************************************
 * @brief               判断是否存在障碍物碰撞风险
 * 
 * @param cylinder_axis 输入圆柱轴向（3D向量）
 * @param scan_axis     输入扫描轴（3D向量）
 * 
 * @return bool         是否存在障碍物碰撞风险
 *****************************************/
static bool IsObstacleCollision(const Eigen::Vector3f& cylinder_axis,const Eigen::Vector3f& scan_axis)
{   
    // 计算圆柱和扫描轴的夹角
    Eigen::Vector3f axis = cylinder_axis.normalized();
    Eigen::Vector3f scan = scan_axis.normalized();
    float cos_theta = axis.dot(scan);
    if(cos_theta > -1.732 && cos_theta < 1.732)    //夹角在30度到150度之间碰撞
    {
        LOG_WARN("Scan Collision Warning! cos_theta=" << cos_theta);
        return true;
    }
    return false;
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

 static void ComputeAverageCircleRadius(
                        std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>& cross_sections,
                        double& av_radius,
                        const Eigen::Vector3f& axis
);

/**************************************
 * @brief               计算圆柱单个截面拟合圆半径
 * 
 * @param cloud         输入点云指针
 * @param radius        输出圆环半径
 * @param axis          圆柱轴向（必须是单位向量）
 * 
 * @return void
 **************************************/
static void fitCircle3DAndGetRadius(
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                        const Eigen::Vector3f& axis,
                        double& radius
);

/***************************************
 * @brief               每隔指定度数沿圆柱轴向切平面，将圆柱表面上的点云投影到该平面上
 * @param cloud         输入点云指针
 * @param segment_cloud 输出分割后的点云指针
 * @param center        圆柱中心（3D向量）
 * @param axis          圆柱轴向（必须是单位向量）
 * @param projected_points_per_plane 输出投影到该平面的2D点集
 * @param angle_step_deg    切割平面角度步长，默认30度
 * @return void
 *****************************************/
static void SegmentCylindAlignAxis(
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& segment_cloud,
                        const Eigen::Vector3f& center,
                        const Eigen::Vector3f& axis,
                        std::vector<std::vector<pcl::PointXY>>& projected_points_per_plane,
                        const float angle_step_deg = 30.0f,
                        const float distance_tolerance = 1.0f                
);

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
static void SegmentCylindAlignAxis(
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& vertical_cloud,
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& horizontal_cloud,
                        const Eigen::Vector3f& center,
                        const Eigen::Vector3f& axis,
                        std::vector<std::vector<pcl::PointXY>>& vertical_projected_points_per_plane,
                        std::vector<std::vector<pcl::PointXY>>& horizontal_projected_points_per_plane,
                        const float angle_step_deg,
                        const float distance_tolerance               
);



/***********************************
 * @brief               根据点云法向过滤水平和垂直点云
 * @param cloud         输入点云指针
 * @param hrizontal_cloud 输出水平点云指针
 * @param vertical_cloud  输出垂直点云指针
 * 
 * @return void
 *****************************************/
static void FilteredPointsByNormal(
                        const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr& hrizontal_cloud,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr& vertical_cloud);

/*****************************************
 * @brief               从2D点集中拟合直线
 * @param points         输入二维点集
 * @param is_vertical    是否为垂直方向直线
 * @param line_coeffs    输出直线参数（斜率k和截距b）
 * @param segment_id     切片id
 * @return void
 *****************************************/
static void FitLineFromPoints(
                        const std::vector<pcl::PointXY>& points,
                        std::vector<pcl::ModelCoefficients>& line_coeffs,
                        int& segment_id,
                        bool is_vertical);


// 过滤主曲率大于阈值的点云
static pcl::PointCloud<pcl::PointXYZ>::Ptr FilterPrincipalCurvature(
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                        const float curvature_threshold = 0.1f);


// 计算拟合圆柱内点到轴向的距离的中位数
static float MeanDistanceToAxis(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const Eigen::Vector3f& center,
    const Eigen::Vector3f& axis,
    float& meanDistance
);


/*********************************** 
* @brief                   计算圆柱高度沿轴向投影范围
 * 
 * @param cloud             输入点云指针
 * @param center            圆柱中心（3D向量）
 * @param axis              圆柱轴向
 * @param radius            圆柱半径
 * @param height            输出圆柱高度
 * 
 * @return float            返回圆柱高度
 **********************************
*/
static void ComputeCylinderHeightAlongAxis(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const Eigen::Vector3f& center,
    const Eigen::Vector3f& axis,
    const double& radius,
    float& height
);
};




static void WritePointsToTxt( 
                        const std::vector<pcl::PointXY>& points,
                        const std::string& filename
)
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        LOG_ERROR("Error: Could not open file " << filename << " for writing.");
        return;
    }

    for (const auto& pt : points) {
        file << pt.x << " " << pt.y << " " << 0.0 << std::endl;
    }

    file.close();
}
 





#endif
