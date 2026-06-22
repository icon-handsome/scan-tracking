#include "TR_Mark_3D_Recon.h"
#include "AppConfig.h"
#include "TR_Mark_Track.h"

// 计算两个二维点之间的欧氏距离
double TR_INSPECT_3D_Recon_Marker::calculateDistance(const cv::Point2f& p1, const cv::Point2f& p2)
{
	double dx = p1.x - p2.x;
	double dy = p1.y - p2.y;
	return std::sqrt(dx * dx + dy * dy);
}

// 计算点集的几何中心点
cv::Point2f TR_INSPECT_3D_Recon_Marker::calculateCentroid(const std::vector<cv::Point2f>& points)
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
int TR_INSPECT_3D_Recon_Marker::filterOutliers(const std::vector<cv::Point2f> &points,
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
bool TR_INSPECT_3D_Recon_Marker::filterOutlies_Debscan(const std::vector<cv::Point2f> &points,
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

// 三维重建标定参数配置
int TR_INSPECT_3D_Recon_Marker::Set_Calib_Config(cv::Mat I1_t,
                                                 cv::Mat D1_t,
                                                 cv::Mat E1_t,
                                                 cv::Mat I2_t,
                                                 cv::Mat D2_t,
                                                 cv::Mat E2_t)
{
	// 检查所有矩阵是否为空
	if (I1_t.empty() || D1_t.empty() || E1_t.empty() ||
		I2_t.empty() || D2_t.empty() || E2_t.empty())
	{
		std::cerr << "[错误] 标定参数矩阵不能为空！" << std::endl;
		return 100; // 返回100表示参数错误
	}

	// 检查内参矩阵必须是 3x3 浮点型（相机标准内参格式）
	if (I1_t.rows != 3 || I1_t.cols != 3 || I1_t.type() != CV_64F ||
		I2_t.rows != 3 || I2_t.cols != 3 || I2_t.type() != CV_64F)
	{
		std::cerr << "[错误] 相机内参矩阵必须是3x3双精度浮点型(CV_64F)！" << std::endl;
		return 200;
	}

	// 深拷贝，使用 cv::Mat::clone() 深拷贝，避免外部矩阵释放导致野指针
	config.I1 = I1_t.clone();
	config.D1 = D1_t.clone();
	config.E1 = E1_t.clone();

	config.I2 = I2_t.clone();
	config.D2 = D2_t.clone();
	config.E2 = E2_t.clone();

	return 0;
}

// 2D检测参数配置
int TR_INSPECT_3D_Recon_Marker::Set_2D_Config(double     epipolar_threshold,
	                                          float      min_z_range,
	                                          float      max_z_range,
	                                          double     max_reproj_err,
	                                          double     max_ratio)
{
	config.epipolar_threshold = epipolar_threshold;
	config.min_z_range        = min_z_range;
	config.max_z_range        = max_z_range;
	config.max_reproj_err     = max_reproj_err;
	config.max_ratio          = max_ratio;

	return 0;
}

// 计算重投影误差
// p3d       三角化得到的3D点（世界/相机坐标系）
// pt2d      原始图像上的2D像素点
// projMat   投影矩阵 P = K [R|t]
double  TR_INSPECT_3D_Recon_Marker::calculateReprojectionError(const cv::Point3f& p3d,
	                                                           const cv::Point2f& pt2d,
															   const cv::Mat& projMat)
{
	cv::Mat X = (cv::Mat_<double>(4, 1) << p3d.x, p3d.y, p3d.z, 1.0);
	cv::Mat x_proj = projMat * X;
	double w = x_proj.at<double>(2, 0);

	if (std::abs(w) < 1e-9)
	{
		return 1e10;
	}

	cv::Point2f projected_pt(static_cast<float>(x_proj.at<double>(0, 0) / w),
		static_cast<float>(x_proj.at<double>(1, 0) / w));

	return cv::norm(projected_pt - pt2d);
}

// 在左右图上绘制极线
// img1 左图
// img2 右图
// F 基础矩阵
// pts1 左图特征点集合
// pts2 右图特征点集合 (与 pts1 一一对应)
void TR_INSPECT_3D_Recon_Marker::drawEpipolarLines(const cv::Mat& img1,
	                                               const cv::Mat& img2,
	                                               const cv::Mat& F,
	                                               const std::vector<cv::Point2f>& pts1,
	                                               const std::vector<cv::Point2f>& pts2)
{
	cv::Mat outImg1, outImg2;
	// 转换为彩色以便画线
	if (img1.channels() == 1) cv::cvtColor(img1, outImg1, cv::COLOR_GRAY2BGR);
	else outImg1 = img1.clone();

	if (img2.channels() == 1) cv::cvtColor(img2, outImg2, cv::COLOR_GRAY2BGR);
	else outImg2 = img2.clone();

	// 1. 计算极线
	std::vector<cv::Vec3f> lines1, lines2;
	// computeCorrespondEpilines 输入必须是 float 类型
	cv::computeCorrespondEpilines(pts1, 1, F, lines2); // 左点在右图的线
	cv::computeCorrespondEpilines(pts2, 2, F, lines1); // 右点在左图的线

	cv::RNG rng(12345);

	int cnt = pts1.size() > pts2.size() ? pts2.size() : pts1.size();
	for (size_t i = 0; i < cnt; i++)
	{
		cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));

		// --- 在左图画线 (对应右图的点) ---
		float a1 = lines1[i][0], b1 = lines1[i][1], c1 = lines1[i][2];
		cv::line(outImg1, cv::Point(0, -c1 / b1), cv::Point(outImg1.cols, -(c1 + a1*outImg1.cols) / b1), color, 4); // 线宽加大
		cv::circle(outImg1, pts1[i], 15, color, -1); // 圆圈加大

		// --- 在右图画线 (对应左图的点) ---
		float a2 = lines2[i][0], b2 = lines2[i][1], c2 = lines2[i][2];
		cv::line(outImg2, cv::Point(0, -c2 / b2), cv::Point(outImg2.cols, -(c2 + a2*outImg2.cols) / b2), color, 4);
		cv::circle(outImg2, pts2[i], 15, color, -1);
	}

	// 2. 拼接图像
	cv::Mat combined;
	cv::hconcat(outImg1, outImg2, combined);

	// 3. 窗口处理 (关键修改)
	std::string winName = "Epipolar Check (Press any key to close)";
	// WINDOW_NORMAL 允许你用鼠标拖动窗口边缘来调整大小
	cv::namedWindow(winName, cv::WINDOW_NORMAL);

	// 根据显示器分辨率自动调整窗口大小 (例如调整到 1280 宽)
	float displayScale = 1280.0f / combined.cols;
	cv::resizeWindow(winName, 1280, (int)(combined.rows * displayScale));

	cv::imshow(winName, combined);
	cv::waitKey(0);
	cv::destroyWindow(winName);
}

	// 在左右图上绘制极线
	// 在目标点集中寻找最佳匹配点
	// pt            源图像中的点
	// candidates    目标图像中的候选点集
	// F             基础矩阵
	// isLeftToRight true表示左搜右(L->R)，false表示右搜左(R->L)
	// threshold     极线距离阈值
	// 注意：左搜右用Func，右搜左用 FuncT
	int TR_INSPECT_3D_Recon_Marker::findBestEpipolarMatch(const cv::Point2f& pt,
		                                                  const std::vector<cv::Point2f>& candidates,
		                                                  const cv::Mat& F,
		                                                  bool isLeftToRight,
		                                                  double threshold)
	{
		cv::Mat p_mat = (cv::Mat_<double>(3, 1) << pt.x, pt.y, 1.0);
		cv::Mat line;

		if (isLeftToRight)
		{
			line = F * p_mat; // L图点在R图的极线
		}
		else
		{
			line = F.t() * p_mat; // R图点在L图的极线 (F转置)
		}

		double a = line.at<double>(0);
		double b = line.at<double>(1);
		double c = line.at<double>(2);
		double line_norm = std::sqrt(a * a + b * b);

		int best_idx = -1;
		double min_dist = 1e10;            // 最小距离
		double sed_dist = 1e10;            // 第二小距离

		for (int j = 0; j < candidates.size(); ++j)
		{
			const auto& target_pt = candidates[j];

			// 1. 极线距离检查
			double dist = std::abs(a * target_pt.x + b * target_pt.y + c) / line_norm;

			// 2. 视差约束 (水平放置相机通常要求 pt_left.x > pt_right.x)
			// 如果是右搜左，则 target_pt.x 应大于 pt.x
			if (isLeftToRight && target_pt.x >= pt.x)
			{
				continue;
			}
			if (!isLeftToRight && target_pt.x <= pt.x)
			{
				continue;
			}


			// 2. 距离阈值初筛
			if (dist > threshold)
			{
				continue;
			}

			// 3. 核心逻辑：维护最近和次近
			if (dist < min_dist)
			{
				sed_dist = min_dist;      // 原来的第一变成第二
				min_dist = dist;          // 更新第一
				best_idx = j;
			}
			else if (dist < sed_dist)
			{
				sed_dist = dist;          // 仅更新第二
			}
		}

		// 模仿 OpenCV StereoBM 算法：如果到极线最近的点距离是 d1，次近的点距离是 d2。
		// 如果 d1 / d2 > 0.7（即第一匹配和第二匹配很接近），说明此处存在歧义，工业场景下建议舍弃该点，以保证“不出错”比“点数多”更重要。
		if (sed_dist > 0.00001)
		{
			if ((min_dist / sed_dist) > AppConfig::Instance().recon.max_ratio) // uniqueness ratio
			{
				best_idx = -1;
			}
		}

		return best_idx;
	}
	// 简化的像素重投影误差计算
	// p3d    三角化得到的 3D 点 (在左相机坐标系下)
	// p2d    原始图像上的观察点 (带畸变的像素坐标)
	// K      内参矩阵
	// D      畸变系数
	// R      相对于左相机的旋转矩阵 (左相机传单位阵)
	// t      相对于左相机的平移向量 (左相机传零向量)
	double TR_INSPECT_3D_Recon_Marker::computePixelErrorSimple(const cv::Point3f& p3d,
		                                                       const cv::Point2f& p2d,
		                                                       const cv::Mat& K,
		                                                       const cv::Mat& D,
		                                                       const cv::Mat& R,
		                                                       const cv::Mat& t)
	{
		// 将 R 转换为旋转向量 rvec (projectPoints 的要求)
		cv::Mat rvec;
		if (R.total() == 9) cv::Rodrigues(R, rvec);
		else rvec = R; // 如果传入的就是 rvec 则直接赋值

		std::vector<cv::Point3f> objPts; objPts.push_back(p3d);
		std::vector<cv::Point2f> imgPts;

		// 投影到像素平面
		cv::projectPoints(objPts, rvec, t, K, D, imgPts);

		// 返回像素距离
		return cv::norm(imgPts[0] - p2d);
	}

		// 寻找最优匹配点
	int TR_INSPECT_3D_Recon_Marker::findBestMatchRefined(const cv::Point2f& ptL,
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
		                                                 cv::Point3f& out_p3d)
	{
		int    best_idx = -1;
		double min_err = DBL_MAX;
		double second_min_err = DBL_MAX;
		double reproj_err2 = 2.0 * max_reproj_err;
		cv::Point3f best_p3d_tmp(0, 0, 0);

		// 1. 极线计算
		cv::Mat pL_mat = (cv::Mat_<double>(3, 1) << ptL.x, ptL.y, 1.0);
		cv::Mat line = F * pL_mat;
		double a = line.at<double>(0), b = line.at<double>(1), c = line.at<double>(2);
		double line_norm = std::sqrt(a * a + b * b);

		// 2. 左点去畸变（用于三角化）
		std::vector<cv::Point2f> ptsL_u;
		cv::undistortPoints(std::vector<cv::Point2f>{ptL}, ptsL_u, I1, D1, cv::noArray(), I1);

		for (int j = 0; j < (int)resultsR.size(); ++j)
		{
			const cv::Point2f& ptR = resultsR[j];

			// 3. 极线粗筛
			double dist = std::abs(a * ptR.x + b * ptR.y + c) / line_norm;
			if (dist > epipolar_threshold)
			{
				continue;
			}
			// 4. 右点去畸变
			std::vector<cv::Point2f> ptsR_u;
			cv::undistortPoints(std::vector<cv::Point2f>{ptR}, ptsR_u, I2, D2, cv::noArray(), I2);

			// 5. 三角化
			cv::Mat p4D;
			cv::triangulatePoints(projL, projR, ptsL_u, ptsR_u, p4D);
			float w = p4D.at<float>(3, 0);
			if (std::abs(w) < 1e-8f)
			{
				continue;
			}

			cv::Point3f p3d_temp(p4D.at<float>(0, 0) / w, p4D.at<float>(1, 0) / w, p4D.at<float>(2, 0) / w);
			// 6. 深度过滤
			if (p3d_temp.z < min_z_range || p3d_temp.z > max_z_range)
			{
				continue;
			}

			// 7. 计算像素重投影误差 (使用传入的 R 和 t)
			double err1 = computePixelErrorSimple(p3d_temp, ptL, I1, D1, R1, t1);
			double err2 = computePixelErrorSimple(p3d_temp, ptR, I2, D2, R2, t2);
			double total_err = err1 + err2;

			if (total_err > reproj_err2)
			{
				continue;
			}

			// 8. 记录最小和次小误差
			if (total_err < min_err)
			{
				second_min_err = min_err;
				min_err = total_err;
				best_idx = j;
				best_p3d_tmp = p3d_temp;
			}
			else if (total_err < second_min_err)
			{
				second_min_err = total_err;
			}
		}

		// 9. 唯一性比率测试
		if (best_idx != -1)
		{
			if (second_min_err != DBL_MAX)
			{
				if (min_err / (second_min_err + 1e-10) > max_ratio)
				{
					return -1;
				}
			}
			out_p3d = best_p3d_tmp;
		}

		// 10. 多帧匹配结果测试，在前n帧中是否也是该匹配点best_idx

		return best_idx;
	}
	int TR_INSPECT_3D_Recon_Marker::Get_3D_Recon_Marker(cv::Mat &left_cam,
		                                                cv::Mat &right_cam)
	{
		// 1. 提取旋转矩阵 R 和平移向量 T (从 E1, E2 转换矩阵中)
		// 用户提供的 E2 是从相机1到相机2的变换 T_c2_c1 (或者说是相机2在相机1坐标系下的位姿)
		// 根据 E1=I, E2=Extrinsic，通常 P1 = K1[I|0], P2 = K2[R|t]
		cv::Mat R = config.E2(cv::Rect(0, 0, 3, 3));
		cv::Mat t = config.E2(cv::Rect(3, 0, 1, 3));

		// 2. 构建投影矩阵 P1, P2 (用于 cv::triangulatePoints),注意，如果去畸变后点是图像坐标系的像素，那么投影矩阵不需要乘以内参矩阵
		cv::Mat proj1 = config.I1 * cv::Mat::eye(3, 4, CV_64F);
		cv::Mat Rt = cv::Mat::eye(3, 4, CV_64F);
		cv::hconcat(R, t, Rt);
		cv::Mat proj2 = config.I2 * Rt;

		// 3. 计算基础矩阵 F，用于极线约束搜索
		// F = K2^-T * [t]x * R * K1^-1
		cv::Mat t_x = (cv::Mat_<double>(3, 3) << 0, -t.at<double>(2), t.at<double>(1),
			                                     t.at<double>(2), 0, -t.at<double>(0),
			                                     -t.at<double>(1), t.at<double>(0), 0);
		cv::Mat E = t_x * R;
		cv::Mat F = config.I2.inv().t() * E * config.I1.inv();
		cv::Mat Ft = F.t();               // F转置

		// 标记点三维重建
		// 1. 标记点识别
		std::vector<cv::Point2f> results1;
		results1.reserve(1000);
		MarkPointDetector serch_Marker1;
		std::vector<cv::Point2f> results2;
		results2.reserve(1000);
		MarkPointDetector serch_Marker2;
		clock_t start, end;

		serch_Marker1.ProcessFrame(left_cam, results1);
		serch_Marker2.ProcessFrame(right_cam, results2);

		if (0)            // 显示极线
		{
			drawEpipolarLines(left_cam,
				              right_cam,
				              F,
				              results1,
				              results2);
		}

		cv::Point3f p3d_L2R;
		const float SPATIAL_MERGE_DIST = 5.0f; // 空间点合并阈值 (mm)，根据你的测量精度调整
		// 1. 匹配与重建循环
		// TODO: 极线匹配中添加相邻点辅助同名点搜索
		frame_3d_points.clear();
		for (int j = 0; j < (int)results1.size(); ++j)
		{
			cv::Point3f p3d_L2R;
			// 1. 左搜右：选出重投影误差最优的点
			int idxR = findBestMatchRefined(results1[j],
				                            results2,
				                            F,
				                            proj1,
				                            proj2,
											config.I1,
											config.D1,
											config.I2,
											config.D2,
				                            cv::Mat::eye(3, 3, CV_64F),
				                            cv::Mat::zeros(3, 1, CV_64F),
				                            R,
				                            t,
											config.epipolar_threshold,
											config.max_reproj_err,
											config.max_ratio,
											config.min_z_range,
											config.max_z_range,
				                            p3d_L2R);
			if (idxR == -1)
			{
				continue;
			}

			//// 2. 双向校验（Cross-Check）：确保逻辑闭环
			//// 注意：反向搜索也应该用 Refined 模式，以排除右图点的歧义
			//cv::Point3f p3d_R2L;
			//int idxL_back = findBestMatchRefined(results2[idxR],
			//	                                 results1,
			//									 Ft,
			//									 proj2,
			//									 proj1,
			//									 Mark_Recon_3D.I2,
			//									 Mark_Recon_3D.D2,
			//									 Mark_Recon_3D.I1,
			//									 Mark_Recon_3D.D1,
			//									 R,
			//									 t,
			//									 cv::Mat::eye(3, 3, CV_64F),
			//									 cv::Mat::zeros(3, 1, CV_64F),
			//									 epipolar_threshold,
			//									 max_reproj_err,
			//									 max_ratio,
			//									 min_z_range,
			//									 max_z_range,
			//									 p3d_R2L);

			//if (idxL_back == j)
			{
				// 匹配极其稳健，加入本帧点集
				frame_3d_points.push_back(p3d_L2R);
			}
		}

		// 6. 输出结果
		std::cout << "三维重建标记点数量: " << frame_3d_points.size() << std::endl;
		for (size_t ii = 0; ii < frame_3d_points.size(); ii++)
		{
			std::cout << frame_3d_points[ii].x << "," << frame_3d_points[ii].y << "," << frame_3d_points[ii].z << std::endl;
		}
		return 0;
	}




