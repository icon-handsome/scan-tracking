#pragma once
#include "AppConfig.h"
#include <opencv2/opencv.hpp>
#include <vector>

// 单个标记点的跟踪信息
struct TrackedPoint 
{
	cv::Point2f pos2d;                          // 当前亚像素坐标
	bool        is_lost;                        // 跟踪是否丢失
	int         consecutive_frames;             // 连续跟踪帧数

	TrackedPoint() : pos2d(0, 0), is_lost(true), consecutive_frames(0) {}
};

class MarkPointDetector
{
public:
	// 参数配置
	struct Config
	{
		int   pyramid_levels;                    // 金字塔层数
		int   min_area;                          // 识别标记点的最小面积
		int   max_area;                          // 识别标记点的最小面积
		int   ROI_w;                             // 局部搜索ROI
		int   ROI_h;
		double perimeter_radius_px;
		double min_circularity;
		int   intensity_threshold;
		int   debscan_min_pts;

		Config()                                 // init from AppConfig
		{
			const AppConfig::Detector& detector = AppConfig::Instance().detector;
			pyramid_levels      = detector.pyramid_levels;
			min_area            = detector.min_area;
			max_area            = detector.max_area;
			ROI_w               = detector.ROI_w;
			ROI_h               = detector.ROI_h;
			perimeter_radius_px = detector.perimeter_radius_px;
			min_circularity     = detector.min_circularity;
			intensity_threshold = detector.intensity_threshold;
			debscan_min_pts     = detector.debscan_min_pts;
		}
	} config;

	cv::Rect    roi;                             // 预测的搜索区域
	std::vector<TrackedPoint> tracked_points;    // 跟踪标记点容器
	bool is_initialized      = false;            // 跟踪的标记点是否被初始化

	MarkPointDetector() 
	{
		tracked_points.reserve(AppConfig::Instance().limits.mark_point_size_max);
	}
 
	/**************************************************************************************
	*功  能：标记点图像预处理函数入口
	*参  数：
	*       img_in                      I         输入的高分辨率图
	*       results                     O         输出的标记点圆心亚像素坐标
	*返回值：状态码
	*备  注：该函数后续扩展成全局与局部搜索切换的函数
	**************************************************************************************/
	bool ProcessFrame(const cv::Mat& img_in, 
		              std::vector<cv::Point2f>& results);

private:
	/**************************************************************************************
	*功  能：图像金字塔由粗到精 (用于初始化或跟踪丢失)
	*参  数：
	*       img_in                      I         输入的高分辨率图
	*       results                     O         输出的标记点圆心亚像素坐标
	*返回值：状态码
	*备  注：
	**************************************************************************************/
	bool GlobalSearch(const cv::Mat& img_in, std::vector<cv::Point2f>& results);
	 
	cv::Point2f RefineSubpixel(const cv::Mat &img,
	                                              cv::Point2f   approx_pos);

	float GetSubpixelGray(const cv::Mat& img, float x, float y);
	 
	cv::Point2f RefineCenter(const cv::Mat& img, cv::Point2f approx_pos, float radius,
		                     cv::Mat K = cv::Mat(), cv::Mat distCoeffs = cv::Mat());

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

// 扫描头标记点模板匹配
// 哈希表条目：存储角度余弦值和中心点A的索引
struct Entry
{
	float l1;
	float l2;
	float angleA;               // 不添加就得使用（L1,L2,cosA）,三个维度在较大的L_BINS下，哈希桶数量巨大，内存爆了
	uint8_t pointIdA;
};

// 几何哈希：二维数组查找表，扫描仪上标记点相对距离和角度不变
// 建表、查询、投票、识别
class FastGeoHash
{
public:
	float               minDistance;   // 模板点最小距离阈值(低于该值的不参与计算特征和查询)
	float               maxDistance;   // 模板点最大距离阈值(大于该值的不参与计算特征和查询)
	float               minDistanceSq; // 预计算平方值
	
	float angleTolerance;
	float lengthTolerance;              // Max length deviation in mm
	float minPercent;
	cv::Mat scan_to_marker_RT;        // 扫描仪到扫描头标记点变换

	// 根据L1和L2方向来建立二维栅格，再结合cosA确定第三维度方向上点的位置
	// maxDist-扫描头上标记点的最大间距
	// minDist-低于这个距离的点不进行特征计算和查询
	FastGeoHash(float maxDist, float minDist = 60.0f) : maxDistance(maxDist), minDistance(minDist)
	{
		angleTolerance = AppConfig::Instance().geo_hash.angle_tolerance_deg * 3.14159265358979323846f / 180.0f;
		minPercent   = AppConfig::Instance().geo_hash.min_percent;
		lengthTolerance = AppConfig::Instance().geo_hash.length_tolerance;
		minDistanceSq = minDist * minDist;           // 预计算
		step          = maxDist / L_BINS;            // maxDist为400mm时候step==0.1mm，4000x4000 = 16,000,000 个int，约 64MB
		counts        = new int[L_BINS * L_BINS]();
		offsets       = new int[L_BINS * L_BINS]();

		Rt_global = (cv::Mat_<double>(4, 4) << 1.0, 0.0, 0.0, 0.0,
			                                   0.0, 1.0, 0.0, 0.0,
			                                   0.0, 0.0, 1.0, 0.0,
			                                   0.0, 0.0, 0.0, 1.0);

		scan_to_marker_RT = (cv::Mat_<double>(4, 4) << 1.0, 0.0, 0.0, 0.0,
			                                           0.0, 1.0, 0.0, 0.0,
			                                           0.0, 0.0, 1.0, 0.0,
			                                           0.0, 0.0, 0.0, 1.0);
	}

	~FastGeoHash()
	{
		delete[] counts;
		delete[] offsets;
	}

	std::vector<cv::Point3f> template_pnts;             // 模板点
	std::vector<cv::Point3f> filtered_frame_3d_points;  // 滤波后的3D标记点
	std::vector<int>         corres_template_points_ID; // 滤波后对应的模板3D点ID
	cv::Mat                  Rt_global;                 // 全局坐标系下点云位姿

	// 最近一次 Get_Track_Pose 诊断（失败时供上层日志使用）
	int track_pose_last_recon_count = 0;
	int track_pose_last_neighbor_filtered_count = 0;
	int track_pose_last_matched_count = 0;
	int track_pose_last_min_pose_points = 0;

	// 设置参数
	int set_template_config(float   minDistance_t,
		                    float   maxDistance_t);

	// 设置查询参数
	int set_query_config(float   angleToleranceDeg_t,
                         float   minPercent_t,
						 float   lengthTolerance_t = -1.0f);

	// 设置扫描仪到扫描头标记点的标定结果
	int set_scan_to_marker_RT(cv::Mat &scan_to_marker_RT_t);

	// 拿到模板点
	int read_template_pnts(const char *file_name);

	// 离线建表
	// 传入130个模型点，构建哈希表
	int build();

	// 稳健识别点
	// sA                目标点
	// otherCandidates  场景中的其他三维重建点
	// angleToleranceDeg     角度容差
	// minPercent       最低票数占比，占所有查询数量的百分比
	// 返回查找到的id
	int query(const cv::Point3f& sA,
		      const std::vector<cv::Point3f>& otherCandidates,
		      float angleToleranceDeg,
			  float minPercent);

	// 计算点云的变换矩阵Rt
	int computeRigidTransformSVD(const std::vector<cv::Point3f>& src, 
	                         const std::vector<cv::Point3f>& dst, 
							 cv::Mat &Rt);

	// 点云位置跟踪函数接口
	// frame_3d_points         待计算的点云（双目跟踪的标记点点云）
	int Get_Track_Pose(std::vector<cv::Point3f>& frame_3d_points,
	                   float angleToleranceDeg,
                       float minPercent);

private:
	static const int    L_BINS = 1300;  // 400mm / 0.1mm = 4000格    650/0.5=1300
	float               step;          // 每个桶的长度

	// 考虑到传统哈希表查询中，底层内存需要指针跳转
	// 需要用CSR(Compressed Sparse Row) 结构提速：counts/offsets/entries
	int                *counts;
	int                *offsets;
	std::vector<Entry>  entries;

	// 私有投票箱
	std::vector<int> votes;

	// 辅助函数：将长度映射到索引
	inline int getIdx(float len) const;

	// 辅助函数：计算3D点特征（长度和余弦值）
	// 注意：这里直接返回长度，因为栅格是基于长度mm划分的
	inline bool calcFeature(const cv::Point3f &A,
		                    const cv::Point3f &B,
							const cv::Point3f &C,
		                    float& lAB, 
							float& lAC, 
							float& cosA) const;

	// --- 第二部分：在线查询 ---
	// 输入场景中的三个点 A, B, C，单次三角形进行投票
	int addVote(const cv::Point3f &sA,
		        const cv::Point3f &sB,
			    const cv::Point3f &sC,
			    float             angleTolerance,
				float             lengthTolerance,
				int               *valid_count);
	 

	void clearVotes();

	// count - 总票数
	// minPercent - 选中id的最小占比
	int getResult(int count, float minPercent);
};