#pragma once
#include "AppConfig.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <opencv2/opencv.hpp>

// ��ǵ���ά�ؽ�
class TR_INSPECT_3D_Recon_Marker
{
public:
	// ��������
	struct Config
	{
		double epipolar_threshold;          // ����ƥ�侫��Լ�������߾�����ֵͨ���� 0.5 - 2.0 ����֮�� 2.0
		float min_z_range;                  // ��ȹ���
		float max_z_range;
		double max_reproj_err;              // �����ͶӰ���Լ��, 0.2 1.0
		double max_ratio;                   // Ψһ�Ա��ʲ��ԣ������������ȶ���
		cv::Mat I1, D1, E1, I2, D2, E2;     // �궨����,�ڲΡ����䡢���

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

	std::vector<cv::Point3f> frame_3d_points; // 3D��ǵ�

	
	TR_INSPECT_3D_Recon_Marker()
	{
		;
	}
	~TR_INSPECT_3D_Recon_Marker()
	{
		;
	}

	// ��ά�ؽ��궨��������
	int Set_Calib_Config(cv::Mat I1_t,
		                 cv::Mat D1_t,
				         cv::Mat E1_t,
				         cv::Mat I2_t, 
				         cv::Mat D2_t, 
				         cv::Mat E2_t);
	// 2D����������
	int Set_2D_Config(double     epipolar_threshold,
		              float      min_z_range,
		              float      max_z_range,
		              double     max_reproj_err,
	                  double     max_ratio);

	// left_cam  �����ͼ��
	// right_cam �����ͼ��
	int Get_3D_Recon_Marker(cv::Mat &left_cam,
		                    cv::Mat &right_cam);
private:
	// ������ͶӰ���
	// p3d       ���ǻ��õ���3D�㣨����/�������ϵ��
	// pt2d      ԭʼͼ���ϵ�2D���ص�
	// projMat   ͶӰ���� P = K [R|t]
	double calculateReprojectionError(const cv::Point3f& p3d,
		                              const cv::Point2f& pt2d,
		                              const cv::Mat& projMat);



    // ������ͼ�ϻ��Ƽ���
    // img1 ��ͼ
    // img2 ��ͼ
    // F ��������
    // pts1 ��ͼ�����㼯��
    // pts2 ��ͼ�����㼯�� (�� pts1 һһ��Ӧ)
    void drawEpipolarLines(const cv::Mat& img1,
    	                   const cv::Mat& img2,
    	                   const cv::Mat& F,
    	                   const std::vector<cv::Point2f>& pts1,
    	                   const std::vector<cv::Point2f>& pts2);
	// ������ͼ�ϻ��Ƽ���
	// ��Ŀ��㼯��Ѱ�����ƥ���
	// pt            Դͼ���еĵ�
	// candidates    Ŀ��ͼ���еĺ�ѡ�㼯
	// F             ��������
	// isLeftToRight true��ʾ������(L->R)��false��ʾ������(R->L)
	// threshold     ���߾�����ֵ
	// ע�⣺��������Func���������� FuncT
	int findBestEpipolarMatch(const cv::Point2f& pt,
		                                                  const std::vector<cv::Point2f>& candidates,
		                                                  const cv::Mat& F,
		                                                  bool isLeftToRight,
		                                                  double threshold);

	// �򻯵�������ͶӰ������
	// p3d    ���ǻ��õ��� 3D �� (�����������ϵ��)
	// p2d    ԭʼͼ���ϵĹ۲�� (���������������)
	// K      �ڲξ���
	// D      ����ϵ��
	// R      ��������������ת���� (���������λ��)
	// t      ������������ƽ������ (�������������)
	double computePixelErrorSimple(const cv::Point3f& p3d,
		                           const cv::Point2f& p2d,
		                           const cv::Mat& K,
		                           const cv::Mat& D,
		                           const cv::Mat& R,
		                           const cv::Mat& t);

	// Ѱ������ƥ���
	int findBestMatchRefined(const cv::Point2f& ptL,
		                     const std::vector<cv::Point2f>& resultsR,
		                     const cv::Mat& F,
		                     const cv::Mat& projL, // ���������ǻ�
		                     const cv::Mat& projR, // ���������ǻ�
		                     const cv::Mat& I1, const cv::Mat& D1, // ���ڲΡ�����
		                     const cv::Mat& I2, const cv::Mat& D2, // ���ڲΡ�����
		                     const cv::Mat& R1, const cv::Mat& t1, // ������� R �� t
		                     const cv::Mat& R2, const cv::Mat& t2, // ����������������� R �� t
		                     const double epipolar_threshold,
		                     const double max_reproj_err,
		                     const double max_ratio,
		                     const float min_z_range,
		                     const float max_z_range,
		                     cv::Point3f& out_p3d);

    double calculateDistance(const cv::Point2f& p1, const cv::Point2f& p2);
    
    cv::Point2f calculateCentroid(const std::vector<cv::Point2f>& points);
    
    // ������Ⱥ��
    // ����˵����
    //   points: �����ԭʼ�㼯
    //   std_factor: ��׼��ϵ��������2��3��ֵԽ�����Խ���ɣ�
    // ����ֵ�����˺�ĵ㼯
    // �Ż���
    // 1)����ʽ���ȹ��˵����Ե���Ⱥ�㣬�����¼������ĺ���ֵ���ظ� 1-2 �Σ������ʼ��Ⱥ��Ӱ�����ļ���
    // 2)ԭʼ�㷨����㼯Χ�Ƽ���������̬�ֲ�����ʵ�ʳ����п��ܲ���������������Ż������÷�λ�������ķ�λ�� IQR��
    // ���ó������㼯�ֲ�����̬��������ȷֲ���ƫ̬�ֲ���
    // �����߼���
    // �������о�����ķ�λ�� Q1��25%����Q3��75%��
    // �����ķ�λ�� IQR = Q3 - Q1
    // ��ֵ = Q3 + 1.5 * IQR������ IQR ��Ⱥ���ж�����
    int filterOutliers(const std::vector<cv::Point2f> &points,
    	               std::vector<cv::Point2f>       &filtered_points,
    	               double                         std_factor = 2.0);
    
    
    // ���ǵ���ǵ�ľۼ��ԣ�ʹ��descan�����˲�
    /**
    * DBSCAN ���ಢ�������ص����ĵ�
    * @param points    ����ĵ㼯��cv::Point2f ��ʽ��
    * @param eps       ����뾶�������㱻��Ϊ�ھӵ�������ؾ��룩
    * @param minPts    ���ٵ��������ڴ������ĵ㽫����Ϊ������
    * @return          �Ƿ�ɹ��ҵ���Ч�ľۼ���
    */
    bool filterOutlies_Debscan(const std::vector<cv::Point2f> &points,
    	                      std::vector<cv::Point2f>       &filtered_points,
    						  float eps,
    						  int minPts);
};

