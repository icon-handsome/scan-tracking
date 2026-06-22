#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include<iostream>
#include <stdio.h>
#include "MvCameraControl.h"
#include <time.h>
#include <random>
#include "TR_Mark_Track.h"
#include "AppConfig.h"
#include "TR_Mark_3D_Recon.h"
using namespace std;

// 读取3D点云
static int Read_TXT_File(char                         *file_name,
                         std::vector<cv::Point3f>     &pnts_3D)
{
	FILE          *infile     = NULL;
	char           buff[2048] = { 0 };
	float          x          = 0.0f;
	float          y          = 0.0f;
	float          z          = 0.0f;
	infile = fopen(file_name, "r");
	if (NULL == infile)
	{
		printf("File not found\n");
		return 1;
	}
	while (!feof(infile))
	{
		if (!fscanf(infile, "%f %f %f\n", &x, &y, &z/*, &intensity*/)) // 遇到回车才换行 
		{
			break;
		}
		pnts_3D.push_back(cv::Point3f(x,y,z));
	}
	 
	fclose(infile);

	return 0;
}


int main()
{
	std::cout << "OpenCV version: " << CV_VERSION << std::endl;
	AppConfig& app_config = AppConfig::Instance();
	if (!app_config.Load())
	{
		return -1;
	}

	// 1. 设置文件夹路径（替换为实际的文件夹路径）
	// 注意：路径末尾要加上 /*.bmp 来匹配所有的 BMP 文件
	std::string folderPath1 = app_config.paths.left_images;
	std::string folderPath2 = app_config.paths.right_images;
 
	// 刚性距离角度约束：扫描仪上标记点相对距离和角度不变（类似几何哈希的二维数组查找表）
	// 建表、查询、投票、识别
	float angleToleranceDeg = app_config.geo_hash.angle_tolerance_deg;
	float minPercent = app_config.geo_hash.min_percent;                              // 最低票数占比，占所有查询数量的百分比
	FastGeoHash Geo_Hash(app_config.geo_hash.max_distance, app_config.geo_hash.min_distance);                  // 扫描头上标记点的最大间距 和  低于最小距离的点不进行特征计算和查询
	int status  = Geo_Hash.read_template_pnts(app_config.paths.template_points.c_str());
	if (status != 0)
	{
		return status;
	}
	Geo_Hash.build();

	//// 基于模板点几何哈希的三维点滤波 D:/3 Data/4 Track_Match/0 EH-Calib/17-3D-markers-scanner-cut.txt
	//std::vector<cv::Point3f>            frame_3d_points_t;    // 不超过130个点，不然调大votes
	//int status1 = Read_TXT_File("D:/3 Data/4 Track_Match/0 EH-Calib/17-3D-markers-scanner-cut.txt", frame_3d_points_t);
	//if (status1 != 0)
	//{
	//	return status1;
	//}
	//size_t pnt_size = frame_3d_points_t.size();
	//std::vector<cv::Point3f> filter_frame_3d_points;            // 滤波点
	//std::vector<cv::Point3f> corres_template_points;                    // 对应滤波点的模板点
	//filter_frame_3d_points.reserve(pnt_size);
	//corres_template_points.reserve(pnt_size);
	//int id_A = -1;
	//int count_set = VOTE_PNT_SIZE_MAX;                         // 选出参与投票的点数
	//if (count_set > frame_3d_points_t.size())
	//{
	//	count_set = frame_3d_points_t.size() - 1;
	//}
	//if (count_set < 1)
	//{
	//	return -1;
	//}
	//// 2. 选择随机数引擎（常用的 Mersenne Twister 算法：mt19937）
	//// 使用 rd() 作为种子
	//std::random_device rd;
	//std::mt19937 gen(rd());
	//// 3. 等概率打乱次序
	//std::shuffle(frame_3d_points_t.begin(), frame_3d_points_t.end(), gen);
	//for (size_t ii = 0; ii < pnt_size; ii++)
	//{
	//	std::vector<cv::Point3f> otherCandidates;
	//	std::cout << ii << std::endl;
	//	std::cout << frame_3d_points_t[ii].x << " " << frame_3d_points_t[ii].y << " " << frame_3d_points_t[ii].z << std::endl;

	//	for (size_t jj = 0; jj < count_set; jj++)
	//	{
	//		size_t id_A = 0;
	//		id_A = (ii + jj + 1) % pnt_size;
	//		otherCandidates.push_back(frame_3d_points_t[id_A]);
	//	}

	//	int id_fit = Geo_Hash.query(frame_3d_points_t[ii],
	//		otherCandidates,
	//		minPercent);
	//	if (id_fit > 0)
	//	{
	//		filter_frame_3d_points.push_back(frame_3d_points_t[ii]);
	//		corres_template_points.push_back(Geo_Hash.template_pnts[id_fit]);

	//		std::cout << frame_3d_points_t[ii].x << " "
	//			      << frame_3d_points_t[ii].y << " "
	//			      << frame_3d_points_t[ii].z << std::endl;
	//		std::cout << Geo_Hash.template_pnts[id_fit].x << " "
	//			      << Geo_Hash.template_pnts[id_fit].y << " "
	//			      << Geo_Hash.template_pnts[id_fit].z << std::endl;
	//	}

	//	std::cout << std::endl;
	//}
	//std::cout << "The Number of Filtered Points: " << filter_frame_3d_points.size() << std::endl;

	//if (filter_frame_3d_points.size() < VOTE_FILTER_PNT_SIZE_MIN)
	//{
	//	std::cout << " ERR: The Number of Filtered Points Is Insufficient." << std::endl;
	//	return -1;
	//}

	//cv::Mat Template_to_Tracker_RT;               // 扫描头模板到双目的变换矩阵

	//int res_1 = Geo_Hash.computeRigidTransformSVD(filter_frame_3d_points,
	//	                                          corres_template_points,
	//	                                          Template_to_Tracker_RT);
	//if (res_1 != 0)
	//{
	//	return res_1;
	//}
	//std::cout << "对应点数量： " << filter_frame_3d_points.size() << std::endl;
	//std::cout << " Template_to_Tracker_RT is: " << std::endl;
	//std::cout << std::fixed << std::setprecision(8);  // 强制保留 8 位小数
	//for (int i = 0; i < 4; i++)
	//{
	//	for (int j = 0; j < 4; j++)
	//	{
	//		std::cout << std::setw(8) << Template_to_Tracker_RT.at<double>(i, j) << " ";
	//	}
	//	std::cout << std::endl;
	//}

	//system("pause");

	// 三维重建
	// 1. 使用 cv::glob 获取所有符合条件的完整文件路径
	std::vector<cv::String> fileNames1;
	std::vector<cv::String> fileNames2;
	cv::glob(folderPath1, fileNames1, false); // false 表示不进行递归子目录搜索
	cv::glob(folderPath2, fileNames2, false); // false 表示不进行递归子目录搜索

	if (fileNames1.empty() || fileNames2.empty())
	{
		std::cout << "错误：未在文件夹中找到 BMP 图像！" << std::endl;
		return -1;
	}

	std::cout << "左相机图像数量: " << fileNames1.size() << std::endl;
	std::cout << "右相机图像数量: " << fileNames2.size() << std::endl;


	// 2. 循环读取每一张图像
	int fileCnt1 = fileNames1.size();
	int fileCnt2 = fileNames2.size();

	if (fileCnt1 != fileCnt2)
	{
		std::cout << "Err: 左右相机图像数量不相等" << std::endl;
		return false;
	}

	std::vector<cv::Mat> imgs1;
	std::vector<cv::Mat> imgs2;
	imgs1.reserve(100);
	imgs2.reserve(100);
	for (size_t i = 0; i < fileCnt1; i++)
	{
		// 读取图像
		cv::Mat img1 = cv::imread(fileNames1[i], cv::IMREAD_UNCHANGED);

		if (img1.empty())
		{
			std::cout << "无法读取图像: " << fileNames1[i] << std::endl;
			continue;
		}
		imgs1.push_back(img1);
		// --- 在这里进行你的处理 ---
		// 例如：打印文件名和大小
		std::cout << "正在处理 [" << i + 1 << "/" << fileNames1.size() << "]: "
			<< fileNames1[i] << " (" << img1.cols << "x" << img1.rows << ")" << std::endl;

		// 读取图像
		cv::Mat img2 = cv::imread(fileNames2[i], cv::IMREAD_UNCHANGED);

		if (img2.empty())
		{
			std::cout << "无法读取图像: " << fileNames2[i] << std::endl;
			continue;
		}
		imgs2.push_back(img2);
		// --- 在这里进行你的处理 ---
		// 例如：打印文件名和大小
		std::cout << "正在处理 [" << i + 1 << "/" << fileNames2.size() << "]: "
			<< fileNames2[i] << " (" << img2.cols << "x" << img2.rows << ")" << std::endl;
	}

	// 3. 标记点三维重建
	TR_INSPECT_3D_Recon_Marker  Mark_Recon_3D;
	int calib_res = Mark_Recon_3D.Set_Calib_Config(app_config.recon.I1,
	                                             app_config.recon.D1,
	                                             app_config.recon.E1,
	                                             app_config.recon.I2,
	                                             app_config.recon.D2,
	                                             app_config.recon.E2);
	if (calib_res != 0)
	{
		return calib_res;
	}

	Geo_Hash.set_scan_to_marker_RT(app_config.geo_hash.scan_to_marker_RT);

	double epipolar_threshold = app_config.recon.epipolar_threshold;
	float min_z_range         = app_config.recon.min_z_range;
	float max_z_range         = app_config.recon.max_z_range;
	double max_reproj_err     = app_config.recon.max_reproj_err;
	double max_ratio          = app_config.recon.max_ratio;

	int res = Mark_Recon_3D.Set_2D_Config(epipolar_threshold,
	                                      min_z_range,
	                                      max_z_range,
	                                      max_reproj_err,
	                                      max_ratio);
	for (int ii = 0; ii < 9; ii++)
	{
		res = Mark_Recon_3D.Get_3D_Recon_Marker(imgs1[ii],
		                                        imgs2[ii]);
	    if (res != 0)
	    {
	    	return res;
	    }
		//system("pause");
	
	
	// 输出结果
	//std::cout << "三维重建标记点数量: " << Mark_Recon_3D.frame_3d_points.size() << std::endl;
	//for (size_t ii = 0; ii < Mark_Recon_3D.frame_3d_points.size(); ii++)
	//{
	//	std::cout << Mark_Recon_3D.frame_3d_points[ii].x << ","
	//		      << Mark_Recon_3D.frame_3d_points[ii].y << "," 
	//			  << Mark_Recon_3D.frame_3d_points[ii].z << std::endl;
	//}
	//system("pause");

	// 4. 双目跟踪，输出位姿
	// TODO: 多筛选两次，比较匹配误差，防止随机选点出现匹配失误的问题
	res = Geo_Hash.Get_Track_Pose(Mark_Recon_3D.frame_3d_points,
	                              angleToleranceDeg,
                                  minPercent);
	if (res != 0)
	{
		return res;
	}
	 
	//// 9. 输出结果
	//std::cout << "最后成功重建三维标记点数量: " << Geo_Hash.filtered_frame_3d_points.size() << std::endl;
	//for (size_t i = 0; i < Geo_Hash.filtered_frame_3d_points.size(); ++i)
	//{
	//	std::cout << Geo_Hash.filtered_frame_3d_points[i].x << "," 
	//		      << Geo_Hash.filtered_frame_3d_points[i].y  << "," 
	//			  << Geo_Hash.filtered_frame_3d_points[i].z << std::endl;
	//}
	}
	return 0;
}

