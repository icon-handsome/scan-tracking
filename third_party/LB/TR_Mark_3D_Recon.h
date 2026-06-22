#pragma once
#include "AppConfig.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <opencv2/opencv.hpp>

// 标记点三维重建
class TR_INSPECT_3D_Recon_Marker
{
public:
	// 参数配置
	struct Config
	{
		double epipolar_threshold;          // 极线匹配精度约束：极线距离阈值通常在 0.5 - 2.0 像素之间 2.0
		float min_z_range;                  // 深度过滤
		float max_z_range;
		double max_reproj_err;              // 最大重投影误差约束, 0.2 1.0
		double max_ratio;                   // 唯一性比率测试，有助于提升稳定性
		cv::Mat I1, D1, E1, I2, D2, E2;     // 标定参数,内参、畸变、外参

		Config()                            // init from AppConfig
		{
			const AppConfig::Recon& recon = AppConfig::Instance().recon;
			epipolar_threshold = recon.epipolar_threshold;
			min_z_range        = recon.min_z_range;
			max_z_range        = recon.max_z_range;
			max_reproj_err     = recon.max_reproj_err;
			max_ratio          = recon.max_ratio;
			I1                 = recon.I1.clone();
			D1                 = recon.D1.clone();
			E1                 = recon.E1.clone();
			I2                 = recon.I2.clone();
			D2                 = recon.D2.clone();
			E2                 = recon.E2.clone();
		}
	} config;

	std::vector<cv::Point3f> frame_3d_points; // 3D标记点

	
	TR_INSPECT_3D_Recon_Marker()
	{
		;
	}
	~TR_INSPECT_3D_Recon_Marker()
	{
		;
	}

	// 三维重建标定参数配置
	int Set_Calib_Config(cv::Mat I1_t,
		                 cv::Mat D1_t,
				         cv::Mat E1_t,
				         cv::Mat I2_t, 
				         cv::Mat D2_t, 
				         cv::Mat E2_t);
	// 2D检测参数配置
	int Set_2D_Config(double     epipolar_threshold,
		              float      min_z_range,
		              float      max_z_range,
		              double     max_reproj_err,
	                  double     max_ratio);

	// left_cam  左相机图像
	// right_cam 左相机图像
	int Get_3D_Recon_Marker(cv::Mat &left_cam,
		                    cv::Mat &right_cam);
private:
	// 计算重投影误差
	// p3d       三角化得到的3D点（世界/相机坐标系）
	// pt2d      原始图像上的2D像素点
	// projMat   投影矩阵 P = K [R|t]
	double calculateReprojectionError(const cv::Point3f& p3d,
		                              const cv::Point2f& pt2d,
		                              const cv::Mat& projMat);



    // 在左右图上绘制极线
    // img1 左图
    // img2 右图
    // F 基础矩阵
    // pts1 左图特征点集合
    // pts2 右图特征点集合 (与 pts1 一一对应)
    void drawEpipolarLines(const cv::Mat& img1,
    	                   const cv::Mat& img2,
    	                   const cv::Mat& F,
    	                   const std::vector<cv::Point2f>& pts1,
    	                   const std::vector<cv::Point2f>& pts2);
	// 在左右图上绘制极线
	// 在目标点集中寻找最佳匹配点
	// pt            源图像中的点
	// candidates    目标图像中的候选点集
	// F             基础矩阵
	// isLeftToRight true表示左搜右(L->R)，false表示右搜左(R->L)
	// threshold     极线距离阈值
	// 注意：左搜右用Func，右搜左用 FuncT
	int findBestEpipolarMatch(const cv::Point2f& pt,
		                                                  const std::vector<cv::Point2f>& candidates,
		                                                  const cv::Mat& F,
		                                                  bool isLeftToRight,
		                                                  double threshold);

	// 简化的像素重投影误差计算
	// p3d    三角化得到的 3D 点 (在左相机坐标系下)
	// p2d    原始图像上的观察点 (带畸变的像素坐标)
	// K      内参矩阵
	// D      畸变系数
	// R      相对于左相机的旋转矩阵 (左相机传单位阵)
	// t      相对于左相机的平移向量 (左相机传零向量)
	double computePixelErrorSimple(const cv::Point3f& p3d,
		                           const cv::Point2f& p2d,
		                           const cv::Mat& K,
		                           const cv::Mat& D,
		                           const cv::Mat& R,
		                           const cv::Mat& t);

	// 寻找最优匹配点
	int findBestMatchRefined(const cv::Point2f& ptL,
		                     const std::vector<cv::Point2f>& resultsR,
		                     const cv::Mat& F,
		                     const cv::Mat& projL, // 仅用于三角化
		                     const cv::Mat& projR, // 仅用于三角化
		                     const cv::Mat& I1, const cv::Mat& D1, // 左内参、畸变
		                     const cv::Mat& I2, const cv::Mat& D2, // 右内参、畸变
		                     const cv::Mat& R1, const cv::Mat& t1, // 左相机的 R 和 t
		                     const cv::Mat& R2, const cv::Mat& t2, // 右相机相对于左相机的 R 和 t
		                     const double epipolar_threshold,
		                     const double max_reproj_err,
		                     const double max_ratio,
		                     const float min_z_range,
		                     const float max_z_range,
		                     cv::Point3f& out_p3d);

    double calculateDistance(const cv::Point2f& p1, const cv::Point2f& p2);
    
    cv::Point2f calculateCentroid(const std::vector<cv::Point2f>& points);
    
    // 过滤离群点
    // 参数说明：
    //   points: 输入的原始点集
    //   std_factor: 标准差系数（建议2或3，值越大过滤越宽松）
    // 返回值：过滤后的点集
    // 优化：
    // 1)迭代式：先过滤掉明显的离群点，再重新计算中心和阈值，重复 1-2 次，避免初始离群点影响中心计算
    // 2)原始算法假设点集围绕几何中心正态分布，但实际场景中可能不成立，可针对性优化：改用分位数法（四分位距 IQR）
    // 适用场景：点集分布非正态（比如均匀分布、偏态分布）
    // 核心逻辑：
    // 计算所有距离的四分位数 Q1（25%）、Q3（75%）
    // 计算四分位距 IQR = Q3 - Q1
    // 阈值 = Q3 + 1.5 * IQR（经典 IQR 离群点判定规则）
    int filterOutliers(const std::vector<cv::Point2f> &points,
    	               std::vector<cv::Point2f>       &filtered_points,
    	               double                         std_factor = 2.0);
    
    
    // 考虑到标记点的聚集性，使用descan进行滤波
    /**
    * DBSCAN 聚类并计算最大簇的中心点
    * @param points    输入的点集（cv::Point2f 格式）
    * @param eps       邻域半径（两个点被视为邻居的最大像素距离）
    * @param minPts    最少点数（少于此数量的点将被视为噪声）
    * @return          是否成功找到有效的聚集点
    */
    bool filterOutlies_Debscan(const std::vector<cv::Point2f> &points,
    	                      std::vector<cv::Point2f>       &filtered_points,
    						  float eps,
    						  int minPts);
};

