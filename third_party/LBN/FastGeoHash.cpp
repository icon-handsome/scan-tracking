#include "FastGeoHash.h"
#include <algorithm>
#include <numeric>
#include <random>

// Silence LBN console spam (2D centers / Rt matrices). Set LBN_ENABLE_CONSOLE_LOG=1 to restore.
#ifndef LBN_ENABLE_CONSOLE_LOG
#define LBN_ENABLE_CONSOLE_LOG 0
#endif
#if LBN_ENABLE_CONSOLE_LOG
#define LBN_COUT std::cout
#else
#include <iostream>
struct LbnNullBuf : public std::streambuf {
	int overflow(int c) override { return c; }
};
inline std::ostream& lbnNullOut()
{
	static LbnNullBuf buf;
	static std::ostream out(&buf);
	return out;
}
#define LBN_COUT lbnNullOut()
#endif


namespace lbn_pose {

/**************************************************************************************
*功  能：标记点图像预处理函数入口
*参  数：
*       img_in                      I         输入的高分辨率图
*       results                     O         输出的标记点圆心亚像素坐标
*返回值：状态码
*备  注：该函数后续扩展成全局与局部搜索切换的函数
**************************************************************************************/
bool MarkPointDetector::ProcessFrame(const cv::Mat& img_in, 
		                             std::vector<cv::Point2f>& results) 
{
	bool status = GlobalSearch(img_in, results);
	return status;
}

/**************************************************************************************
*功  能：图像金字塔由粗到精 (用于初始化或跟踪丢失)
*参  数：
*       img_in                      I         输入的高分辨率图
*       results                     O         输出的标记点圆心亚像素坐标
*返回值：状态码
*备  注：
**************************************************************************************/
bool MarkPointDetector::GlobalSearch(const cv::Mat&            img_in, 
	                                 std::vector<cv::Point2f>  &results)
{
	std::vector<cv::Point2f> circle_centers;
	circle_centers.reserve(300);
	std::vector<float> mark_area;
	mark_area.resize(1000);

	// 图像平滑
	cv::Mat blurred;
	cv::GaussianBlur(img_in, blurred, cv::Size(5, 5), 1.5);

	// 1. 下采样 (例如缩小到 1/64)
	cv::Mat small_img;
	int level_cnt = 0;
	//cv::pyrDown(blurred, small_img);   // Level 1
	//cv::pyrDown(small_img, small_img); // Level 2
	//cv::pyrDown(small_img, small_img); // Level 3
	//cv::imwrite("0 small-IMG.jpg", small_img, { cv::IMWRITE_JPEG_QUALITY, 90 });

	// 2. 粗提取 (简单的阈值 + 轮廓查找)
	cv::Mat binary;
	//cv::threshold(small_img, binary, 30, 200, cv::THRESH_BINARY);
	cv::Mat binary_raw;
	cv::adaptiveThreshold(blurred, binary, 255,
		                  cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 101, -5);

	//cv::adaptiveThreshold(blurred, binary_raw, 255,
	//	                  cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 101, -5);
	//const cv::Mat close_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(17, 17));
	//const cv::Mat open_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
	//cv::morphologyEx(binary_raw, binary, cv::MORPH_CLOSE, close_kernel);
	//cv::morphologyEx(binary, binary, cv::MORPH_OPEN, open_kernel);

	//cv::Mat flood = binary.clone();
	//cv::copyMakeBorder(flood, flood, 1, 1, 1, 1, cv::BORDER_CONSTANT, cv::Scalar(0));
	//cv::floodFill(flood, cv::Point(0, 0), cv::Scalar(255));
	//flood = flood(cv::Rect(1, 1, binary.cols, binary.rows));
	//cv::bitwise_not(flood, flood);
	//binary |= flood;
	//cv::threshold(small_img, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
	////cv::bitwise_not(binary, binary);
	//cv::imwrite("contours_binary.jpg", binary, { cv::IMWRITE_JPEG_QUALITY, 90 });

	std::vector<std::vector<cv::Point>> contours;
	////cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	// cv::CHAIN_APPROX_NONE  cv::CHAIN_APPROX_SIMPLE
	cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);    // RETR_EXTERNAL keeps only the filled marker outer contour.
	
	// 画出轮廓并标上序号
	cv::Mat color_result;
	if (0)
	{
		std::vector<std::vector<cv::Point>> scaled_contours = contours; // 不要修改原始的contours
/* 		for (size_t i = 0; i < scaled_contours.size(); i++)
		{
			for (size_t j = 0; j < scaled_contours[i].size(); j++)
			{
				// 因为做了 3 次 pyrDown，所以坐标要乘以 2^3 = 8
				scaled_contours[i][j].x *= pow(2, level_cnt);
				scaled_contours[i][j].y *= pow(2, level_cnt);
			}
		} */
		// 2. 准备彩色画布
		if (img_in.channels() == 1)
			cv::cvtColor(img_in, color_result, cv::COLOR_GRAY2BGR);
		else
			color_result = img_in.clone();
		// 3. 绘制轮廓
		// 参数：画布, 轮廓集合, 绘制全部(-1), 颜色(绿色), 线条宽度(10)
		cv::drawContours(color_result, scaled_contours, -1, cv::Scalar(0, 255, 0), 2);
		//// 4. (可选) 同时画出之前算出的亚像素质心 circle_centers
		//for (const auto& pt : circle_centers)
		//{
		//	cv::circle(color_result, pt, 25, cv::Scalar(0, 0, 255), -1); // 画红色实心原点
		//}
		for (int i = 0; i < scaled_contours.size(); i++)
		{
			std::string text = std::to_string(i);
			cv::putText(color_result, text, scaled_contours[i][0], cv::FONT_HERSHEY_SIMPLEX,
				1.0, cv::Scalar(0, 0, 255), 1);
		}
		// 5. 保存结果
		// 25MP 图像建议保存为 JPG 以节省空间
		cv::imwrite("contours_result.jpg", color_result, { cv::IMWRITE_JPEG_QUALITY, 90 });
	}
	
	tracked_points.clear();
	int ii = 0;
	double perim_t = 5.0 / pow(2, level_cnt) * 6.28;     // 半径5个像素
	for (auto& cnt : contours)
	{
		//if (ii >= 339)
		//{
		//	int aaa = 0;
		//}
		//ii++;

		double area = cv::contourArea(cnt);
		if (area > (config.min_area / pow(4, level_cnt)) && area < (config.max_area / pow(4, level_cnt)))
		{
			// 圆度过滤 (Circularity)
			// 判定是否够圆，防止提取到墙缝、门框等杂质
			double perimeter = cv::arcLength(cnt, true);
			if (perimeter < perim_t)            // 半径小于5像素的都剔除，5/64 * 2 * 3.14 = 0.49
			{
				continue;
			}
			double circularity = (4 * CV_PI * area) / (perimeter * perimeter);
			if (circularity > 0.5) // 越接近1越圆，工业标记点通常 > 0.8
			{
				// 缩小的面积阈值
				cv::Moments mu = cv::moments(cnt);
				cv::Point2f center(mu.m10 / mu.m00, mu.m01 / mu.m00);
			 
				// 还原到高分辨率坐标
				cv::Point2f high_res_pos(center.x * pow(2, level_cnt), center.y * pow(2, level_cnt));
				// float radius = sqrtf(area / 3.14159265); // 1/3.14159265 * 0.5
				float radius = perimeter * 0.159155 * pow(2, level_cnt); // 1/3.14159265 * 0.5

				if (1)                  // 亮度滤波
				{
					int x_f = 0;
					int y_f = 0;
					int x_c = 0;
					int y_c = 0;
					if ((int)high_res_pos.x > (img_in.cols - 5) || (int)high_res_pos.y > (img_in.rows - 5) ||
						(int)high_res_pos.x < 5                 || (int)high_res_pos.y < 5)
					{
						continue;
					}

					x_f = floorf(high_res_pos.x);
					x_c = ceilf(high_res_pos.x);
					y_f = floorf(high_res_pos.y);
					y_c = ceilf(high_res_pos.y);

					const int intensity_c = config.intensityThreshold;
					if ((int)img_in.at<uchar>(y_f, x_f) < intensity_c ||
						(int)img_in.at<uchar>(y_c, x_f) < intensity_c ||
						(int)img_in.at<uchar>(y_f, x_c) < intensity_c ||
						(int)img_in.at<uchar>(y_c, x_c) < intensity_c)
					{
						continue;
					}

					//// 边缘点亮度值滤波（黑色）
					//int intensity_r = 30;
					//int round_x     = x_c;
					//int round_y     = y_c;
					//int radius_int  = floorf(radius) + 3;
					//if ((int)img_in.at<uchar>(round_y, round_x + radius_int) > intensity_r ||
					//	(int)img_in.at<uchar>(round_y, round_x - radius_int) > intensity_r ||
					//	(int)img_in.at<uchar>(round_y + radius_int, round_x) > intensity_r ||
					//	(int)img_in.at<uchar>(round_y - radius_int, round_x) > intensity_r)
					//{
					//	continue;
					//}

				}

				// 进一步在原图中进行亚像素精修
				cv::Point2f refined_pos;
				//cv::Mat I1 = (cv::Mat_<double>(3, 3) << 5.078966884152220e+03, 1.144606033008170, 2.731696208984134e+03,
	   //                                       0.0,                   5.076838475599669e+03,  1.832530560892915e+03,
				//							  0.0,                   0.0,                    1.0);
				//cv::Mat D1 = (cv::Mat_<double>(1, 5) << -0.061814641591874,       // k1
	   //                                       0.134149054707469,        // k2
				//							  -1.780078171601695e-04,   // p1
				//							  -5.409994817555799e-04,   // p2
				//							  -0.101638840770598);      // k3


				//refined_pos = RefineCenter(blurred, high_res_pos, radius,I1,D1);
			 
				refined_pos = RefineCenter(blurred, high_res_pos, radius);

				//if (cnt.size() < 5)
				//{
				//	//refined_pos = RefineSubpixel(blurred, high_res_pos);   // img_in
				//	continue;
				//}
				//else
				//{
				//	cv::RotatedRect ellipse = cv::fitEllipse(cnt);
				//	refined_pos = ellipse.center * 8;
				//}
				

				//TrackedPoint tp;
				//tp.pos2d = refined_pos;
				//tp.is_lost = false;
				//if (!tp.is_lost)
				//{
				//	tp.consecutive_frames++;
				//}
				//tracked_points.push_back(tp);
				circle_centers.push_back(refined_pos); // refined_pos
			}
		}
	}

	//// 4. 画出算出的亚像素质心 circle_centers
	//for (const auto& pt : circle_centers)
	//{
	//	cv::circle(color_result, pt, 1, cv::Scalar(0, 0, 255), -1); // 画红色实心原点
	//}
	//// 5. 保存结果
	//// 25MP 图像建议保存为 JPG 以节省空间
	//cv::imwrite("0centers_result.jpg", color_result, { cv::IMWRITE_JPEG_QUALITY, 90 });

	is_initialized = !circle_centers.empty();

	LBN_COUT << "Raw marker candidates before DBSCAN: " << circle_centers.size() << std::endl;

	// 注意要做离群点统计滤波
	 filterOutlies_Debscan(circle_centers,
		                  results,
						  config.debscanFilterDistPx,
		                  5);

	if (results.size() < 4)
	{
		std::vector<cv::Point2f> results_2;
		filterOutliers(circle_centers, results, 2.0);
		filterOutliers(results, results_2, 1.5);
		results.clear();
		filterOutliers(results_2, results, 1.0);
	}

	//// 对粗定位结果进行精细标记点圆心提取
	//for (int ii = 0; ii < results.size(); ii++)
	//{
	//	cv::Point2f refined_pos;
	//	RefineCenter(blurred, refined_pos, )

	//}


	//// 6. 输出结果
	//std::cout << "circle_centers:              " << circle_centers.size() << std::endl;
	//for (size_t i = 0; i < circle_centers.size(); ++i)
	//{
	//	std::cout << circle_centers[i].x << "," << circle_centers[i].y << "," << 0.0f<< std::endl;
	//}
	LBN_COUT << "Count of 2D marker centers:  " << results.size() << std::endl;
	for (size_t i = 0; i < results.size(); ++i)
	{
		LBN_COUT << results[i].x << "," << results[i].y << "," << 0.0f << std::endl;
	}

	if (0)
	{
		// 保存中心点到图像
		// 1. 准备绘制：将灰度图转为彩色图，以便画红点
		cv::Mat colorImg1;
		cv::cvtColor(binary, colorImg1, cv::COLOR_GRAY2BGR);
		// 2. 在图上画出所有找到的中心点
		for (size_t jj = 0; jj < results.size(); jj++)
		{
			// 针对 25MP 图像，半径设为 20，线条粗细设为 5
			//cv::circle(colorImg, results1[j], 20, cv::Scalar(0, 0, 255), 5);

			// 可选：画一个中心十字
			cv::drawMarker(colorImg1, results[jj], cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 1, 2);
		}
		// 3. 生成保存的文件名
		// 例如：原名 Image_01.bmp -> 保存为 Result_Image_01.jpg
		// 提示：25MP 保存为 BMP 非常大且慢，建议保存为 JPG (质量设高点)
		std::string saveName1 = "right_centers.jpg";
		// 4. 保存图像，设置 JPG 质量为 95%
		std::vector<int> params1;
		params1.push_back(cv::IMWRITE_JPEG_QUALITY);
		params1.push_back(95);
		cv::imwrite(saveName1, colorImg1, params1);
	}


	return is_initialized;
}
 
/**************************************************************************************
*功  能：像素重心提取
*参  数：
*       img                         I         输入的高分辨率图像
*       approx_pos                  I         粗提取得到的像素级中心坐标
*返回值：亚像素精修后的中心坐标
*备  注：灰度重心法，TODO: 椭圆拟合、二阶矩法
**************************************************************************************/
cv::Point2f MarkPointDetector::RefineSubpixel(const cv::Mat &img,
	                                          cv::Point2f   approx_pos)
{
	// 1. 设置精修参数
	const int radius = 5;        // 精修窗口半径，窗口大小为 (2*radius + 1)
	const int x0 = cvRound(approx_pos.x);
	const int y0 = cvRound(approx_pos.y);

	// 2. 确定搜索窗口边界，防止越界
	int left   = x0 - radius > 0 ? x0 - radius : 0;
	int top    = y0 - radius > 0 ? y0 - radius : 0;
	int right  = x0 + radius < img.cols - 1 ? x0 + radius : img.cols - 1;
	int bottom = y0 + radius < img.rows - 1 ? y0 + radius : img.rows - 1;

	double sum_gray = 0;         // 灰度值总和
	double sum_x = 0;            // x方向加权总和
	double sum_y = 0;            // y方向加权总和

	// 3. 背景抑制：计算窗口内的灰度阈值
	// 工业场景下，标记点通常比背景亮，建议只取灰度值高于一定阈值的像素
	// 这里简单采用固定阈值或自适应计算窗口内最小/平均灰度
	uchar threshold = 80;        // 经验值，根据实际反光情况调整

	// 4. 遍历窗口计算重心
	for (int i = top; i <= bottom; ++i)
	{
		// 使用指针加速像素访问
		const uchar* ptr = img.ptr<uchar>(i);
		for (int j = left; j <= right; ++j)
		{
			uchar gray = ptr[j];
			if (gray > threshold)
			{
				// 减去阈值（或者使用平方加权）可以进一步提高抗噪能力和灵敏度
				double weight = (double)gray;
				sum_gray += weight;
				sum_x += weight * j;
				sum_y += weight * i;
			}
		}
	}

	// 5. 计算最终亚像素位置
	if (sum_gray > 0)
	{
		return cv::Point2f((float)(sum_x / sum_gray), (float)(sum_y / sum_gray));
	}
	else
	{
		// 如果窗口内没有任何点高于阈值，返回原位置
		return approx_pos;
	}
}

/**************************************************************************************
*功  能：灰度线性插值
*参  数：
*       img                         I         输入的高分辨率图像
*       x                           I         x坐标
*       y                           I         y坐标
*返回值：灰度值
*备  注：
**************************************************************************************/
float MarkPointDetector::GetSubpixelGray(const cv::Mat& img, float x, float y)
{
	int x1 = floor(x);
	int y1 = floor(y);
	int x2 = x1 + 1;
	int y2 = y1 + 1;

	if (x1 < 0 || x2 >= img.cols || y1 < 0 || y2 >= img.rows) 
		return 0;

	float dx = x - x1;
	float dy = y - y1;

	float val = (1 - dx) * (1 - dy) * img.at<uchar>(y1, x1) +
		        dx * (1 - dy) * img.at<uchar>(y1, x2) +
		        (1 - dx) * dy * img.at<uchar>(y2, x1) +
		        dx * dy * img.at<uchar>(y2, x2);
	return val;
}

	/**
	* @brief 工业级精修：梯度重心法 + 鲁棒性过滤
	*/
/**************************************************************************************
*功  能：亚像素插值
*参  数：
*       img                         I         输入的高分辨率图像
*       x                           I         x坐标
*       y                           I         y坐标
*返回值：灰度值
*备  注：
**************************************************************************************/
cv::Point2f MarkPointDetector::RefineCenter(const cv::Mat& img, cv::Point2f approx_pos, float radius,
		                                    cv::Mat K, cv::Mat distCoeffs)
{
		const int num_rays = 72;           // 增加采样密度 (每5度一条)
		const float search_range = 6.0f;   // 稍大的搜索范围确保覆盖边缘
		const float step = 0.5f;

		std::vector<cv::Point2f> edge_points;
		std::vector<float> edge_weights;   // 存储梯度强度作为拟合权重

		for (int i = 0; i < num_rays; ++i)
		{
			float angle = i * (2.0f * (float)CV_PI / num_rays);
			float dx = cos(angle);
			float dy = sin(angle);

			std::vector<float> profile;
			// 1. 采样剖面
			for (float r = radius - search_range; r <= radius + search_range; r += step)
			{
				profile.push_back(GetSubpixelGray(img, approx_pos.x + r * dx, approx_pos.y + r * dy));
			}

			// 2. 计算一阶差分梯度 (使用稍大的算子抗噪)
			std::vector<float> grads;
			float max_g = -1;
			int max_idx = -1;
			for (int j = 2; j < (int)profile.size() - 2; ++j)
			{
				// 使用 [j-2, j-1, j+1, j+2] 计算梯度，比简单差分更稳
				float g = abs(profile[j - 2] + profile[j - 1] - profile[j + 1] - profile[j + 2]);
				grads.push_back(g);
				if (g > max_g) {
					max_g = g;
					max_idx = j - 2; // 对应grads的索引
				}
			}

			// 3. 核心改进：梯度重心法定位 (比抛物线插值更准)
			// 取梯度峰值及其左右各2个点，计算重心
			if (max_idx >= 2 && max_idx < (int)grads.size() - 2 && max_g > 10.0f)
			{
				double sum_gr = 0, sum_g = 0;
				for (int k = max_idx - 2; k <= max_idx + 2; ++k)
				{
					float weight = grads[k];
					float r_val = (radius - search_range) + (k + 2) * step; // 回算当前的半径距离
					sum_gr += (double)weight * r_val;
					sum_g += (double)weight;
				}

				if (sum_g > 0)
				{
					float best_r = (float)(sum_gr / sum_g);
					edge_points.push_back(cv::Point2f(approx_pos.x + best_r * dx,
						approx_pos.y + best_r * dy));
					edge_weights.push_back(max_g);
				}
			}
		}

		if (edge_points.size() < 10)
		{
			return approx_pos;
		}
		// 4. 异常值过滤：初步拟合后剔除误差过大的点 (防止脏污点干扰)
		std::vector<cv::Point2f> undistorted_points;
		if (!K.empty() && !distCoeffs.empty())
		{
			cv::undistortPoints(edge_points, undistorted_points, K, distCoeffs, cv::noArray(), K);
		}
		else
		{
			undistorted_points = edge_points;
		}

		// 第一次拟合
		cv::RotatedRect ell = cv::fitEllipseDirect(undistorted_points);

		// 鲁棒性剔除：计算每个点到椭圆边缘的距离，剔除偏离过大的点
		std::vector<cv::Point2f> final_points;
		for (const auto& p : undistorted_points)
		{
			// 简化的距离检查：到中心的距离与平均半径对比
			float dist = cv::norm(p - ell.center);
			float expected_r = (ell.size.width + ell.size.height) / 4.0f;
			if (abs(dist - expected_r) < 2.0f)       // 只保留距离偏差小于2像素的点
			{ 
				final_points.push_back(p);
			}
		}

		if (final_points.size() < 10)
		{
			return ell.center;
		}

		// 5. 最终拟合
		cv::RotatedRect final_ell = cv::fitEllipseDirect(final_points);
		return final_ell.center;
}


// 计算两个二维点之间的欧氏距离
double MarkPointDetector::calculateDistance(const cv::Point2f& p1, const cv::Point2f& p2)
{
	double dx = p1.x - p2.x;
	double dy = p1.y - p2.y;
	return std::sqrt(dx * dx + dy * dy);
}

// 计算点集的几何中心点
cv::Point2f MarkPointDetector::calculateCentroid(const std::vector<cv::Point2f>& points)
{
	if (points.empty()) 
	{
		return cv::Point2f(0, 0);
	}

	double sum_x = 0.0, sum_y = 0.0;
	for (const auto& p : points) 
	{
		sum_x += p.x;
		sum_y += p.y;
	}

	return cv::Point2f(sum_x / points.size(), sum_y / points.size());
}

// 过滤离群点
// 参数说明：
//   points: 输入的原始点集
//   std_factor: 标准差系数（建议2或3，值越大过滤越宽松）
// 返回值：过滤后的点集
// 优化：
// 1)迭代式：先过滤掉明显的离群点，再重新计算中心和阈值，重复 2-3 次，避免初始离群点影响中心计算
// 2)原始算法假设点集围绕几何中心正态分布，但实际场景中可能不成立，可针对性优化：改用分位数法（四分位距 IQR）
// 适用场景：点集分布非正态（比如均匀分布、偏态分布）
// 核心逻辑：
// 计算所有距离的四分位数 Q1（25%）、Q3（75%）
// 计算四分位距 IQR = Q3 - Q1
// 阈值 = Q3 + 1.5 * IQR（经典 IQR 离群点判定规则）
int MarkPointDetector::filterOutliers(const std::vector<cv::Point2f> &points,
                                std::vector<cv::Point2f>       &filtered_points,
	                            double                          std_factor)
{
	if (points.size() <= 1) 
	{
		filtered_points.clear();
		for (size_t i = 0; i < points.size(); ++i)
		{
			filtered_points.push_back(points[i]);
		}
		return 0; // 点太少无需过滤
	}

	// 1. 计算中心点
	 cv::Point2f centroid = calculateCentroid(points);

	// 2. 计算每个点到中心点的距离
	std::vector<double> distances;
	distances.reserve(points.size());
	for (const auto& p : points)
	{
		distances.push_back(calculateDistance(p, centroid));
	}

	// 3. 计算距离的均值
	double mean_distance = std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();

	// 4. 计算距离的标准差
	double sum_squared_diff = 0.0;
	for (double d : distances) 
	{
		sum_squared_diff += std::pow(d - mean_distance, 2);
	}
	double std_distance = std::sqrt(sum_squared_diff / distances.size());

	// 5. 计算阈值（均值 + std_factor * 标准差）
	double threshold = mean_distance + std_factor * std_distance;

	// 6. 过滤离群点
	for (size_t i = 0; i < points.size(); ++i) 
	{
		if (distances[i] <= threshold)
		{
			filtered_points.push_back(points[i]);
		}
	}

	//// 输出过滤信息（可选）
	//std::cout << "=== 离群点过滤信息 ===" << std::endl;
	//std::cout << "原始点数: " << points.size() << std::endl;
	//std::cout << "移除的离群点数: " << points.size() - filtered_points.size() << std::endl;
	//std::cout << "过滤后点数: " << filtered_points.size() << std::endl;
	//std::cout << "距离阈值: " << threshold << std::endl;
	return 0;
}



// 考虑到标记点的聚集性和均匀性，使用descan进行滤波
// points             输入的点集（cv::Point2f 格式）
// filtered_points    滤波后的点集（cv::Point2f 格式）
// eps                邻域半径（两个点被视为邻居的最大像素距离）
// minPts             最少点数（少于此数量的点将被视为噪声）
bool MarkPointDetector::filterOutlies_Debscan(const std::vector<cv::Point2f> &points,
	                                     std::vector<cv::Point2f>       &filtered_points,
						                 float eps,
						                 int minPts)
{
	if (points.empty())
	{
		return false;
	}

	float eps2 = eps * eps;
	int n = points.size();
	std::vector<int> labels(n, -1); // -1: 未处理, 0: 噪声, >0: 簇ID
	int clusterId = 0;

	// 1. 执行 DBSCAN 聚类
	for (int i = 0; i < n; i++) 
	{
		if (labels[i] != -1)
		{
			continue;
		}

		// 寻找邻居
		std::vector<int> neighbors;
		for (int j = 0; j < n; j++) 
		{
			if ((powf((points[i].x - points[j].x), 2) + powf((points[i].y - points[j].y), 2)) <= eps2)
			{
				neighbors.push_back(j);
			}
		}

		if (neighbors.size() < (size_t)minPts) 
		{
			labels[i] = 0; // 标记为噪声
		}
		else 
		{
			clusterId++;
			labels[i] = clusterId;

			// 扩展簇 (使用队列模拟递归)
			std::vector<int> seeds = neighbors;
			for (size_t k = 0; k < seeds.size(); k++) 
			{
				int currIdx = seeds[k];
				if (labels[currIdx] == 0)
				{
					labels[currIdx] = clusterId; // 噪声点变边界点
				}
				if (labels[currIdx] != -1)
				{
					continue;
				}

				labels[currIdx] = clusterId;
				std::vector<int> currNeighbors;
				for (int j = 0; j < n; j++) 
				{
					
					if ((powf((points[currIdx].x - points[j].x), 2) + powf((points[currIdx].y - points[j].y), 2)) <= eps2)
					{
						currNeighbors.push_back(j);
					}
				}
				if (currNeighbors.size() >= (size_t)minPts)
				{
					seeds.insert(seeds.end(), currNeighbors.begin(), currNeighbors.end());
				}
			}
		}
	}

	// 2. 统计哪个簇的点数最多，聚集点
	if (clusterId == 0)
	{
		return false; // 全是噪声
	}

	std::vector<int> counts(clusterId + 1, 0);
	for (int k : labels) 
	{
		if (k > 0)
		{
			counts[k]++;
		}
	}

	auto maxIt = std::max_element(counts.begin() + 1, counts.end());
	int targetId = std::distance(counts.begin(), maxIt);
	int maxPointsCount = *maxIt;

	// 3. 计算目标簇的中心
	cv::Point2f sum(0, 0);
	filtered_points.clear();
	for (int i = 0; i < n; i++)
	{
		if (labels[i] == targetId) 
		{
			filtered_points.push_back(points[i]);
		}
	}

	return true;
}



// 辅助函数：将长度映射到索引
inline int FastGeoHash::getIdx(float len) const
{
	if (step < 1e-8f)
	{
		return -1;
	}

	int idx = (int)(len / step);    // step:每个桶的长度
	if (idx < 0)
	{
		return 0;
	}
	if (idx >= L_BINS)
	{
		return L_BINS - 1;
	}
	return idx;
}

// 辅助函数：计算3D点特征（长度和余弦值）
// 注意：这里直接返回长度，因为栅格是基于长度mm划分的
inline bool FastGeoHash::calcFeature(const cv::Point3f& A,
	const cv::Point3f& B,
	const cv::Point3f& C,
	float& lAB,
	float& lAC,
	float& cosA) const
{
	float abx = B.x - A.x, aby = B.y - A.y, abz = B.z - A.z;
	float acx = C.x - A.x, acy = C.y - A.y, acz = C.z - A.z;

	float d2AB = abx * abx + aby * aby + abz * abz;
	// 阈值检查：如果 AB 太近，直接跳过
	if (d2AB < minDistanceSq)
	{
		return false;
	}
	float d2AC = acx * acx + acy * acy + acz * acz;
	// 阈值检查：如果 AB 太近，直接跳过
	if (d2AC < minDistanceSq)
	{
		return false;
	}
	lAB = std::sqrt(d2AB);
	lAC = std::sqrt(d2AC);

	float dot = abx * acx + aby * acy + abz * acz;
	cosA = dot / (lAB * lAC + 1e-8f);

	return true;
}

// --- 第一部分：离线建表 ---
// 传入130个模型点，构建哈希表
int FastGeoHash::build()
{
	int N = template_pnts.size();
	if (N < 3)
	{
		return 401;
	}

	// 1. 第一轮遍历：统计每个桶的大小
	std::memset(counts, 0, sizeof(int) * L_BINS * L_BINS);
	for (int i = 0; i < N; ++i)              // 点A
	{
		for (int j = 0; j < N; ++j)          // 点B
		{
			if (i == j)
			{
				continue;
			}

			for (int k = j + 1; k < N; ++k)  // 点C (j+1 保证BC组合不重复)
			{
				if (i == k)
				{
					continue;
				}


				float l1, l2, c;
				if (!calcFeature(template_pnts[i], template_pnts[j], template_pnts[k], l1, l2, c))
				{
					continue;              // 距离太近，忽略该特征
				}
				if (l1 > l2)
				{
					std::swap(l1, l2);
				}
				int key = getIdx(l1) * L_BINS + getIdx(l2);            // 由l1和l2构成的二维栅格桶
				counts[key]++;
			}
		}
	}

	// 2. 计算偏移量 (前缀和)
	offsets[0] = 0;
	int totalEntries = counts[0];
	int total_c = L_BINS * L_BINS;
	for (int i = 1; i < total_c; ++i)
	{
		offsets[i] = offsets[i - 1] + counts[i - 1];
		totalEntries += counts[i];
	}

	// 3. 第二轮遍历：填入真实数据
	entries.resize(totalEntries);
	int* currentPos = new int[L_BINS * L_BINS];
	std::memcpy(currentPos, offsets, sizeof(int) * L_BINS * L_BINS);
	for (int i = 0; i < N; ++i)
	{
		for (int j = 0; j < N; ++j)
		{
			if (i == j)
			{
				continue;
			}
			for (int k = j + 1; k < N; ++k)
			{
				if (i == k)
				{
					continue;
				}
				float l1, l2, c;
				if (!calcFeature(template_pnts[i], template_pnts[j], template_pnts[k], l1, l2, c))
				{
					continue;
				}
				if (l1 > l2)
				{
					std::swap(l1, l2);
				}
				int key = getIdx(l1) * L_BINS + getIdx(l2);
				entries[currentPos[key]++] = { c, (uint8_t)i };
			}
		}
	}
	delete[] currentPos;
	LBN_COUT << "Build finished. Total combinations: " << totalEntries << std::endl;
	return 0;
}

// --- 第二部分：在线查询 ---
// 输入场景中的三个点 A, B, C，单次三角形进行投票
int FastGeoHash::addVote(const cv::Point3f& sA,
	const cv::Point3f& sB,
	const cv::Point3f& sC,
	float             cosTolerance,
	int* valid_count)
{
	int count_t = (*valid_count);
	bool is_valid = false;
	float l1, l2, targetCosA;
	if (!calcFeature(sA, sB, sC, l1, l2, targetCosA))
	{
		return -1;                      // 输入点对太近，无法构成稳健特征
	}
	if (l1 > l2)
	{
		std::swap(l1, l2);
	}
	int i1 = getIdx(l1);
	int i2 = getIdx(l2);

	// 邻居搜索 (抗0.1mm的位移噪声)
	for (int di = -1; di <= 1; ++di)
	{
		for (int dj = -1; dj <= 1; ++dj)
		{
			int ni = i1 + di;
			int nj = i2 + dj;
			if (ni < 0 || ni >= L_BINS || nj < 0 || nj >= L_BINS)
			{
				continue;
			}
			int key = ni * L_BINS + nj;
			int start = offsets[key];
			int end = start + counts[key];

			// 连续内存扫描：为了适配targetCosA找到对应点
			for (int k = start; k < end; ++k)
			{
				if (std::abs(entries[k].cosA - targetCosA) < cosTolerance)
				{
					int id_t = entries[k].pointIdA;
					votes[id_t]++;
					// 本次投票有效，不管这轮投出了几张票
					// 可能存在候选点的一套参数（L1,L2,cosA）在模板上能对应多个三角形，但是每轮count_t只加一
					is_valid = true;
				}
			}
		}
	}
	if (is_valid)
	{
		count_t++;
	}

	*valid_count = count_t;

	return 0;
}

// -----------------------------------------------------------------------------
// getResult：从三角形投票箱中判定「当前帧 3D 点对应哪个模板点 ID」
//
// 历史问题（已修复）：
//   曾有一版实现要求 maxV>=6 且 secondV 倍率>1.5，且完全忽略入参 minPercent，
//   导致帧内仅 7~10 个 3D 点时永远无法通过 query()，Get_Track_Pose 恒为 code=500。
//
// 当前策略（相对文件内上方注释掉的旧版）：
//   1) 主路径：恢复 minPercent 占比 + 最低票数 voteThreshold=max(3, minPercent*count)
//   2) 次显著性：maxV 须不低于 secondV 的 1.25 倍（旧活跃版为 1.5 且须 maxV>=6）
//   3) 兜底：点数较多时沿用「maxV>4 && count>4 && maxV>minPercent*count」
//
// 生产注意：门槛低于「maxV>=6」版，点少/噪声时更易 success，但也更易误匹配；
//   上线前请多工况验证 Rt；若误匹增多可收紧 voteThreshold 或倍率（见 docs/station1/算法使用API.md）
// -----------------------------------------------------------------------------
//// count - 总票数
//// minPercent - 选中id的最小占比
//int FastGeoHash::getResult(int count, float minPercent)
//{
//	// 查找最高票
//	int bestId = -1;
//	int maxV   = 0;
//	int id     = -1;
//
//	for (int i = 0; i < 130; ++i)
//	{
//		if (votes[i] > maxV)
//		{
//			maxV = votes[i];
//			bestId = i;
//		}
//	}
//
//	if (maxV > 4 && count > 4)            // 得票数至少为5，且总投票数最少为5次，否则受噪声影响较大
//	{
//		int count_t = (int)(minPercent * count);
//		if (maxV > count_t)
//		{
//			id = bestId;
//		}
//	}
//	if (id < 0)
//	{
//		std::cout << "投票总数：" << count << "  最大得票数：" << maxV <<" No"<< std::endl;
//	}
//	else
//	{
//		std::cout << "投票总数：" << count << "  最大得票数：" << maxV << " 找到了对应点" << std::endl;
//	}
//	return id;
//}

int FastGeoHash::getResult(int count, float minPercent)
{
	int bestId = -1;
	int secondId = -1;
	int maxV = 0;
	int secondV = 0;

	for (int i = 0; i < 130; ++i)
	{
		if (votes[i] > maxV)
		{
			secondV = maxV;
			secondId = bestId;
			maxV = votes[i];
			bestId = i;
		}
		else if (votes[i] > secondV)
		{
			secondV = votes[i];
			secondId = i;
		}
	}

	if (bestId < 0 || count < 1 || maxV < 1)
	{
		return -1;
	}

	// 主路径：minPercent 必须参与判定（query() 传入，与 config.ini [LbnPose] 一致）
	const int voteThreshold = std::max(3, static_cast<int>(minPercent * static_cast<float>(count)));
	if (maxV >= voteThreshold && count >= 3)
	{
		// 无第二名或第一名显著领先，避免多模板 ID 争抢时误选
		if (secondV == 0 || static_cast<float>(maxV) >= static_cast<float>(secondV) * 1.25f)
		{
			return bestId;
		}
	}

	// 兜底：与上方注释掉的原始逻辑一致，帧内有效三角形投票较多时启用
	if (maxV > 4 && count > 4 && maxV > static_cast<int>(minPercent * static_cast<float>(count)))
	{
		return bestId;
	}

	return -1;
}

// 稳健识别点
// sA                目标点
// otherCandidates  场景中的其他三维重建点
// cosTolerance     角度容差
// minPercent       最低票数占比，占所有查询数量的百分比
int FastGeoHash::query(const cv::Point3f& sA,
	const std::vector<cv::Point3f>& otherCandidates,
	float cosTolerance,
	float minPercent)
{
	// 1. 初始化
	clearVotes();
	// 预计算平方距离阈值，避免在循环中反复计算 sqrt
	float maxDistSq = maxDistance * maxDistance;

	// 2. 过滤掉离 sA 太远的无效点，减少无效组合
	// 在 400mm 的场景下，离 A 超过 400mm 的点无法组成哈希表能检索的三角形
	std::vector<cv::Point3f> validNeighbors;
	validNeighbors.reserve(otherCandidates.size());

	for (const auto& p : otherCandidates)
	{
		if (p == sA)
		{
			continue;
		}
		float dx = p.x - sA.x;
		float dy = p.y - sA.y;
		float dz = p.z - sA.z;
		float d2 = dx * dx + dy * dy + dz * dz;

		// 只有在 [minDist, maxDist] 范围内的点才有意义
		if (d2 <= maxDistSq && d2 >= minDistanceSq)
		{
			validNeighbors.push_back(p);
		}
	}

	if (validNeighbors.size() < 2)
	{
		return -1;
	}

	// 3. 执行多重投票
	// 为了性能，如果 validNeighbors 太多（比如超过 6 个），建议只取前 6 个
	size_t cut_size = VOTE_PNT_SIZE_MAX;
	int    count = 0;
	if (cut_size > validNeighbors.size())
	{
		cut_size = validNeighbors.size();
	}
	for (size_t i = 0; i < cut_size; ++i)
	{
		for (size_t j = i + 1; j < cut_size; ++j)
		{
			// 调用之前定义的单次投票函数
			addVote(sA, validNeighbors[i], validNeighbors[j], cosTolerance, &count);
		}
	}

	// 4. 获取并返回最终最高票结果
	int id_t = getResult(count, minPercent);

	return id_t;
}

void FastGeoHash::clearVotes()
{
	std::memset(votes, 0, sizeof(votes));
}

// 计算点云的变换矩阵Rt
int FastGeoHash::computeRigidTransformSVD(const std::vector<cv::Point3f>& src,
	const std::vector<cv::Point3f>& dst,
	cv::Mat& Rt)
{
	int n = src.size();
	if (n < 3)
	{
		return -1;
	}
	// 1. 计算重心
	cv::Point3f centerSrc(0, 0, 0), centerDst(0, 0, 0);
	for (int i = 0; i < n; i++)
	{
		centerSrc += src[i];
		centerDst += dst[i];
	}
	centerSrc *= (1.0 / n);
	centerDst *= (1.0 / n);

	// 2. 去重心化并计算协方差矩阵 H
	cv::Mat H = cv::Mat::zeros(3, 3, CV_64F);
	for (int i = 0; i < n; i++)
	{
		cv::Mat s = (cv::Mat_<double>(3, 1) << src[i].x - centerSrc.x, src[i].y - centerSrc.y, src[i].z - centerSrc.z);
		cv::Mat d = (cv::Mat_<double>(3, 1) << dst[i].x - centerDst.x, dst[i].y - centerDst.y, dst[i].z - centerDst.z);
		H += d * s.t();
	}

	// 3. SVD 分解
	cv::SVD svd(H);
	cv::Mat R = svd.u * svd.vt;

	// 4. 检查行列式，防止出现镜像反射
	if (cv::determinant(R) < 0)
	{
		cv::Mat V = svd.vt.t();
		V.col(2) *= -1;         // 翻转最后一列
		R = V.t() * svd.u.t();  // 重新计算 R
		R = R.t();
	}

	// 5. 计算平移 t
	cv::Mat cSrc = (cv::Mat_<double>(3, 1) << centerSrc.x, centerSrc.y, centerSrc.z);
	cv::Mat cDst = (cv::Mat_<double>(3, 1) << centerDst.x, centerDst.y, centerDst.z);
	cv::Mat t = cDst - R * cSrc;

	// 6. 构造 4x4 矩阵
	Rt = cv::Mat::eye(4, 4, CV_64F);
	R.copyTo(Rt.rowRange(0, 3).colRange(0, 3));
	t.copyTo(Rt.rowRange(0, 3).col(3));

	return 0;
}

// 设置参数
int FastGeoHash::set_template_config(float   minDistance_t,
	float   maxDistance_t)
{
	minDistance = minDistance_t;
	maxDistance = maxDistance_t;
	return 0;
}

// 设置查询参数
int FastGeoHash::set_query_config(float   cosTolerance_t,
	float   minPercent_t)
{
	cosTolerance = cosTolerance_t;
	minPercent = minPercent_t;
	return 0;
}

// 设置扫描仪到扫描头标记点的标定结果
int FastGeoHash::set_scan_to_marker_RT(cv::Mat& scan_to_marker_RT_t)
{
	// 检查矩阵是否为空
	if (scan_to_marker_RT_t.empty())
	{
		std::cerr << "ERRs: scan_to_marker_RT_t is empty." << std::endl;
		return 400; // 返回100表示参数错误
	}

	// 深拷贝，使用 cv::Mat::clone() 深拷贝，避免外部矩阵释放导致野指针
	scan_to_marker_RT = scan_to_marker_RT_t.clone();
	return 0;
}

// 拿到模板点
int FastGeoHash::read_template_pnts(char* file_name)
{
	FILE* infile = NULL;
	char           buff[2048] = { 0 };
	float          x = 0.0f;
	float          y = 0.0f;
	float          z = 0.0f;

	template_pnts.reserve(10000);
	infile = fopen(file_name, "r");
	if (NULL == infile)
	{
		printf("File not found\n");
		return 1;
	}
	while (!feof(infile))
	{
		if (!fscanf(infile, "%f %f %f\n", &x, &y, &z)) // 遇到回车才换行 
		{
			break;
		}
		template_pnts.push_back(cv::Point3f(x, y, z));
	}
	fclose(infile);

	return 0;
}

// 点云位置跟踪函数接口
// srcPoints         待计算的点云（双目跟踪的标记点点云）
// 刚性距离角度约束：扫描仪上标记点相对距离和角度不变（类似几何哈希的二维数组查找表）
int FastGeoHash::Get_Track_Pose(std::vector<cv::Point3f>& frame_3d_points,
	                            float cosTolerance,
	                            float minPercent)
{
	filtered_frame_3d_points.clear();
	corres_template_points_ID.clear();
	// 基于模板点几何哈希的三维点滤波
	size_t pnt_size = frame_3d_points.size();
	std::vector<int> corres_template_points_ID;     // 对应滤波点的模板点
	filtered_frame_3d_points.reserve(pnt_size);
	corres_template_points_ID.reserve(pnt_size);
	int id_A = -1;
	int count_set = VOTE_PNT_SIZE_MAX;             // 选出参与投票的点数
	if (frame_3d_points.size() < count_set)
	{
		count_set = frame_3d_points.size() - 1;
	}
	if (count_set < 1)
	{
		return 400;
	}
	// 2. 选择随机数引擎（常用的 Mersenne Twister 算法：mt19937）
	// 使用 rd() 作为种子
	std::random_device rd;
	std::mt19937 gen(rd());
	// 3. 等概率打乱次序
	std::shuffle(frame_3d_points.begin(), frame_3d_points.end(), gen);
	for (size_t ii = 0; ii < pnt_size; ii++)
	{
		std::vector<cv::Point3f> otherCandidates;
		for (size_t jj = 0; jj < count_set; jj++)
		{
			size_t id_A = 0;
			id_A = (ii + jj + 1) % pnt_size;
			otherCandidates.push_back(frame_3d_points[id_A]);
		}

		int id_fit = query(frame_3d_points[ii],
			otherCandidates,
			cosTolerance,
			minPercent);
		if (id_fit >= 0)
		{
			filtered_frame_3d_points.push_back(frame_3d_points[ii]);
			corres_template_points_ID.push_back(id_fit);
		}
	}

	////int aa = 0;
	//// 时空一致性测试 (Spatial-Temporal Consistency)
	//// 基于查找表，当前帧点云与全局缓存点进行匹配
	//current_frame_idx++; // 累加全局帧序号
	//std::vector<cv::Point3f> filter_stable_pnts;    // 本帧最终通过校验的点
	//filter_stable_pnts.reserve(filter_frame_3d_points.size());
	//std::vector<int>         filter_stable_pnts_ID; // 本帧最终通过校验的点ID，值是filter_frame_3d_points中的序号
	//filter_stable_pnts_ID.reserve(filter_frame_3d_points.size());

	//// 记录本帧活跃的模板索引，用于后续清理未匹配点
	//std::set<int> current_frame_active_indices;

	//for (size_t k = 0; k < filter_frame_3d_points.size(); ++k)
	//{
	//	cv::Point3f curr_p3d = filter_frame_3d_points[k];
	//	// 获取对应的模板索引（注意：确保 Geo_Hash.query 返回的是 template_pnts 的索引）
	//	int template_idx = corres_template_points_ID[k];
	//	 

	//	// 查找该模板点是否已在追踪地图中
	//	auto it = global_track_map.find(template_idx);
	//	if (it != global_track_map.end())
	//	{
	//		// 1. 如果该点在上一帧也出现了（连续性判断）
	//		if (it->second.last_frame_idx == current_frame_idx - 1)
	//		{
	//			it->second.hit_count++;
	//		}
	//		else
	//		{
	//			// 如果中间断帧了，重置计数（或者允许少量丢帧，这里采用严格连续）
	//			it->second.hit_count = 1;
	//			it->second.pos_sum = cv::Point3f(0, 0, 0);
	//		}

	//		it->second.last_frame_idx = current_frame_idx;
	//		it->second.pos_sum += curr_p3d; // 用于后续求均值（可选）

	//		// 2. 稳定性判断
	//		if (it->second.hit_count >= STABLE_FRAME_THRESH)
	//		{
	//			it->second.is_stable = true;
	//			// 使用当前坐标，或者使用平均坐标以平滑抖动
	//			// filter_stable_pnts.push_back(it->second.pos_sum * (1.0f / it->second.hit_count));
	//			filter_stable_pnts.push_back(curr_p3d);
	//			filter_stable_pnts_ID.push_back(k);
	//		}
	//	}
	//	else
	//	{
	//		// 3. 新出现的点，记录到地图
	//		TrackedPoint_t new_track(curr_p3d, current_frame_idx);
	//		global_track_map.insert(std::make_pair(template_idx, new_track));
	//	}

	//	current_frame_active_indices.insert(template_idx);
	//}

	//// 4. 清理长期未出现的点（内存优化）
	//for (auto it = global_track_map.begin(); it != global_track_map.end();)
	//{
	//	if (current_frame_idx - it->second.last_frame_idx > STABLE_FRAME_THRESH) // 超过10帧没看到就删除
	//	{
	//		it = global_track_map.erase(it);
	//	}
	//	else
	//	{
	//		++it;
	//	}
	//}

	//std::cout << ">>> [时空校验后] 稳定点数量: " << filter_stable_pnts.size() << std::endl;
	//for (const auto& pt : filter_stable_pnts)
	//{
	//	std::cout << "Stable Point: " << pt.x << ", " << pt.y << ", " << pt.z << std::endl;
	//}

	// 输出位姿
	cv::Mat                  Rt;
	std::vector<cv::Point3f> corres_template_pnts;    // 本帧最终通过校验的点对应的模板点
	corres_template_pnts.reserve(corres_template_points_ID.size());
	for (int ii = 0; ii < corres_template_points_ID.size(); ii++)
	{
		int id_temp = corres_template_points_ID[ii];
		corres_template_pnts.push_back(template_pnts[id_temp]);
	}
	if (filtered_frame_3d_points.size() < 3)
	{
		LBN_COUT << "Errs: The Number of Template Matching is not Enough: " << filtered_frame_3d_points.size() << std::endl;
		return 500;
	}
	int res_err = computeRigidTransformSVD(filtered_frame_3d_points,
		                                   corres_template_pnts,
		                                   Rt);
	if (res_err != 0 || Rt.empty())
	{
		return res_err;
	}

	Rt_global = Rt * scan_to_marker_RT;

	LBN_COUT << "对应点数量： " << filtered_frame_3d_points.size() << std::endl;
	LBN_COUT << "标记点： " << std::endl;
	for (int i = 0; i < filtered_frame_3d_points.size(); i++)
	{
		LBN_COUT << filtered_frame_3d_points[i].x << " " <<
			         filtered_frame_3d_points[i].y << " " <<
			         filtered_frame_3d_points[i].z << std::endl;
	}
	LBN_COUT << std::endl;

	LBN_COUT << "模板点： " << std::endl;
	for (int i = 0; i < filtered_frame_3d_points.size(); i++)
	{
		LBN_COUT << corres_template_pnts[i].x << " " <<
			         corres_template_pnts[i].y << " " <<
			         corres_template_pnts[i].z << std::endl;
	}
	LBN_COUT << std::endl;

	LBN_COUT << " Realtime Rt is: " << std::endl;
	LBN_COUT << std::fixed << std::setprecision(8);  // 强制保留 8 位小数
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			LBN_COUT << std::setw(8) << Rt.at<double>(i, j) << " ";
		}
		LBN_COUT << std::endl;
	}

	LBN_COUT << " Realtime Rt_global is: " << std::endl;
	LBN_COUT << std::fixed << std::setprecision(8);  // 强制保留 8 位小数
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			LBN_COUT << std::setw(8) << Rt_global.at<double>(i, j) << " ";
		}
		LBN_COUT << std::endl;
	}

	//// 9. 输出结果
	//std::cout << "最后成功重建三维标记点数量: " << frame_3d_points.size() << std::endl;
	//for (size_t i = 0; i < frame_3d_points.size(); ++i)
	//{
	//	std::cout << frame_3d_points[i].x << "," << frame_3d_points[i].y << "," << frame_3d_points[i].z << std::endl;
	//}
	//int aa = 0;
	return 0;
}

}  // namespace lbn_pose
