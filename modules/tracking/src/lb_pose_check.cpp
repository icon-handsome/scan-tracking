/**
 * @file lb_pose_check.cpp
 * @brief LB位姿检测实现文件
 *
 * 实现传统的LB位姿检测算法，包括相机标定参数配置、立体匹配、三维重建
 * 和基于模板的位姿估计等功能。
 */

#include "scan_tracking/tracking/lb_pose_check.h"

#include <array>
#include <cmath>
#include <filesystem>
#include <limits>
#include <string>
#include <vector>

#include <QDir>
#include <QFileInfo>
#include <QLoggingCategory>

#include <opencv2/opencv.hpp>

#include "TR_Mark_3D_Recon.h"
#include "TR_Mark_Track.h"

/// LB位姿检测日志类别
Q_LOGGING_CATEGORY(LOG_LB_POSE, "tracking.lb_pose")

namespace scan_tracking::tracking {
namespace {

namespace fs = std::filesystem;

/// 默认数据根目录
// @return 默认的测试数据根路径
QString defaultDataRoot()
{
    return QStringLiteral("D:/work/scan-tracking/third_party/lb_pose_detection/data");
}

/// 将配置值规范化为默认值
// @param value 用户配置的值
// @param fallback 默认值
// @return 规范化后的值（若输入为空则返回默认值）
QString normalizeOrDefault(const QString& value, const QString& fallback)
{
    return value.trimmed().isEmpty() ? fallback : value.trimmed();
}

/// 将QString路径转换为OpenCV兼容的std::string路径
// @param path Qt格式路径
// @return 转换为本地操作系统路径格式的std::string
std::string toCvPath(const QString& path)
{
    return QDir::toNativeSeparators(path).toStdString();
}

/// 配置默认的相机标定参数
//
// 设置左右相机的内参矩阵、畸变系数和外参矩阵，用于三维重建计算。
//
// @param recon 三维重建对象引用
void configureDefaultCalibration(TR_INSPECT_3D_Recon_Marker& recon)
{
    // 左相机内参矩阵 (I1) 和畸变系数 (D1)
    recon.config.I1 = (cv::Mat_<double>(3, 3) << 5.078851406536548e+03, 0.830568826844289, 2.746479519311858e+03,
                                                  0.0, 5.079564338697494e+03, 1.827274288235361e+03,
                                                  0.0, 0.0, 1.0);
    recon.config.D1 = (cv::Mat_<double>(1, 5) << -0.061121083586165,
                                                  0.174884596596884,
                                                  -1.053862530437392e-04,
                                                  -2.625558299490124e-04,
                                                  -0.174942436164493);
    // 左相机外参矩阵（单位矩阵）
    recon.config.E1 = cv::Mat::eye(4, 4, CV_64F);

    // 右相机内参矩阵 (I2) 和畸变系数 (D2)
    recon.config.I2 = (cv::Mat_<double>(3, 3) << 5.088957721152494e+03, 1.694422728104837, 2.748597487208202e+03,
                                                  0.0, 5.087725659008389e+03, 1.818343109063463e+03,
                                                  0.0, 0.0, 1.0);
    recon.config.D2 = (cv::Mat_<double>(1, 5) << -0.061336067087922,
                                                  0.140736778029161,
                                                  -2.839150977966796e-04,
                                                  0.001241546114496,
                                                  -0.079946406594583);
    // 右相机外参矩阵（相对于左相机的位姿）
    recon.config.E2 = (cv::Mat_<double>(4, 4) << 0.932342748446725, -0.009472020725314, 0.361451629187345, -5.793657636690184e+02,
                                                  -0.014055020969881, 0.997951882006392, 0.062405909859861, -13.667600451372955,
                                                  -0.361302443673362, -0.063263907745887, 0.930300071037500, 1.265698817906372e+02,
                                                  0.0, 0.0, 0.0, 1.0);
}

/// 检查RT矩阵是否有效
// @param rt 4x4变换矩阵
// @return 有效返回true，否则返回false
bool isValidRt(const cv::Mat& rt)
{
    return !rt.empty() && rt.rows == 4 && rt.cols == 4 && rt.type() == CV_64F;
}

/// 将OpenCV矩阵转换为16元素数组
// @param rt 4x4变换矩阵
// @return 标准化的16元素数组
std::array<double, 16> toRtArray(const cv::Mat& rt)
{
    std::array<double, 16> values = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    };
    if (!isValidRt(rt)) {
        return values;
    }
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            values[static_cast<std::size_t>(row * 4 + col)] = rt.at<double>(row, col);
        }
    }
    return values;
}

/// 计算平移向量的模长（毫米）
// @param rt 4x4变换矩阵
// @return 平移向量的欧几里得距离
double translationNormMm(const cv::Mat& rt)
{
    if (!isValidRt(rt)) {
        return 0.0;
    }
    const double tx = rt.at<double>(0, 3);
    const double ty = rt.at<double>(1, 3);
    const double tz = rt.at<double>(2, 3);
    return std::sqrt(tx * tx + ty * ty + tz * tz);
}

/// 规范化路径显示文本
// @param path 原始路径
// @return 规范化后的路径文本
QString summarizePath(const QString& path)
{
    return QDir::toNativeSeparators(path.trimmed());
}

}  // namespace

/// 执行传统的LB位姿检测算法
//
// 该函数执行完整的位姿检测流程：
// 1. 加载左右相机图像
// 2. 配置相机标定参数
// 3. 执行三维重建获取点云
// 4. 加载模板文件并构建哈希索引
// 5. 执行模板匹配计算位姿
//
// @param config LB位姿检测配置参数
// @return 位姿检测结果结构体
PoseCheckResult runLegacyLbPoseCheck(const scan_tracking::common::LbPoseConfig& config)
{
    PoseCheckResult result;

    // 加载并规范化配置参数
    const QString dataRootText = normalizeOrDefault(config.dataRoot, defaultDataRoot());
    const QString leftPatternText = normalizeOrDefault(
        config.leftPattern,
        QDir(dataRootText).filePath(QStringLiteral("L/*.bmp")));
    const QString rightPatternText = normalizeOrDefault(
        config.rightPattern,
        QDir(dataRootText).filePath(QStringLiteral("R/*.bmp")));
    const QString templateText = normalizeOrDefault(
        config.templateFile,
        QDir(dataRootText).filePath(QStringLiteral("template-3D-ALL-Shift-Cut-Cut.txt")));

    // 打印配置信息
    qInfo(LOG_LB_POSE).noquote()
        << "LB pose config:" 
        << "dataRoot=" << summarizePath(dataRootText)
        << "leftPattern=" << summarizePath(leftPatternText)
        << "rightPattern=" << summarizePath(rightPatternText)
        << "templateFile=" << summarizePath(templateText)
        << "minDistance=" << config.minDistance
        << "maxDistance=" << config.maxDistance
        << "cosTolerance=" << config.cosTolerance
        << "minPercent=" << config.minPercent;

    // 检查数据根目录是否存在
    const QFileInfo dataRootInfo(dataRootText);
    if (!dataRootInfo.exists()) {
        result.resultCode = 9;
        result.message = QStringLiteral("LB pose data root does not exist: %1").arg(summarizePath(dataRootText));
        qWarning(LOG_LB_POSE).noquote() << result.message;
        return result;
    }

    // 使用OpenCV glob查找匹配的图像文件
    std::vector<cv::String> leftFiles;
    std::vector<cv::String> rightFiles;
    cv::glob(toCvPath(leftPatternText), leftFiles, false);
    cv::glob(toCvPath(rightPatternText), rightFiles, false);

    // 检查左右相机图像是否都存在
    if (leftFiles.empty() || rightFiles.empty()) {
        result.resultCode = 5;
        result.message = QStringLiteral("LB pose input images missing: left=%1, right=%2")
                          .arg(static_cast<int>(leftFiles.size()))
                          .arg(static_cast<int>(rightFiles.size()));
        qWarning(LOG_LB_POSE).noquote() << result.message;
        return result;
    }

    // 检查左右图像数量是否匹配
    if (leftFiles.size() != rightFiles.size()) {
        result.resultCode = 9;
        result.message = QStringLiteral("LB pose image count mismatch: left=%1, right=%2")
                          .arg(static_cast<int>(leftFiles.size()))
                          .arg(static_cast<int>(rightFiles.size()));
        qWarning(LOG_LB_POSE).noquote() << result.message;
        return result;
    }

    // 读取第一对立体图像
    cv::Mat leftImage = cv::imread(leftFiles.front(), cv::IMREAD_UNCHANGED);
    cv::Mat rightImage = cv::imread(rightFiles.front(), cv::IMREAD_UNCHANGED);
    if (leftImage.empty() || rightImage.empty()) {
        result.resultCode = 5;
        result.message = QStringLiteral("LB pose failed to load stereo images: left=%1 right=%2")
                          .arg(QString::fromStdString(leftFiles.front()))
                          .arg(QString::fromStdString(rightFiles.front()));
        qWarning(LOG_LB_POSE).noquote() << result.message;
        return result;
    }

    result.inputPointCount = 0;
    result.invoked = true;

    try {
        // 创建三维重建对象并配置标定参数
        TR_INSPECT_3D_Recon_Marker recon;
        configureDefaultCalibration(recon);

        // 执行三维重建
        const int reconResult = recon.Get_3D_Recon_Marker(leftImage, rightImage);
        if (reconResult != 0 || recon.frame_3d_points.empty()) {
            result.resultCode = 7;
            result.message = QStringLiteral("LB pose 3D reconstruction failed, code=%1, points=%2")
                              .arg(reconResult)
                              .arg(static_cast<int>(recon.frame_3d_points.size()));
            qWarning(LOG_LB_POSE).noquote() << result.message;
            return result;
        }

        result.inputPointCount = static_cast<int>(recon.frame_3d_points.size());

        // 创建哈希索引对象并配置查询参数
        FastGeoHash geoHash(config.maxDistance, config.minDistance);
        if (geoHash.set_template_config(config.minDistance, config.maxDistance) != 0 ||
            geoHash.set_query_config(config.cosTolerance, config.minPercent) != 0) {
            result.resultCode = 7;
            result.message = QStringLiteral("LB pose failed to set hash config.");
            qWarning(LOG_LB_POSE).noquote() << result.message;
            return result;
        }

        // 加载模板文件
        std::string templatePathBytes = toCvPath(templateText);
        if (geoHash.read_template_pnts(templatePathBytes.data()) != 0) {
            result.resultCode = 9;
            result.message = QStringLiteral("LB pose template file not found or invalid: %1")
                              .arg(summarizePath(templateText));
            qWarning(LOG_LB_POSE).noquote() << result.message;
            return result;
        }

        // 构建哈希索引
        if (geoHash.build() != 0) {
            result.resultCode = 7;
            result.message = QStringLiteral("LB pose hash build failed.");
            qWarning(LOG_LB_POSE).noquote() << result.message;
            return result;
        }

        // 执行位姿估计
        const int trackResult = geoHash.Get_Track_Pose(
            recon.frame_3d_points,
            config.cosTolerance,
            config.minPercent);

        // 检查位姿估计结果
        if (trackResult != 0 || !isValidRt(geoHash.Rt)) {
            result.resultCode = 7;
            result.message = QStringLiteral("LB pose Get_Track_Pose failed, code=%1").arg(trackResult);
            qWarning(LOG_LB_POSE).noquote() << result.message;
            return result;
        }

        // 位姿检测成功，填充结果
        result.success = true;
        result.resultCode = 1;
        result.poseDeviationMm = translationNormMm(geoHash.Rt);
        result.rt = toRtArray(geoHash.Rt);
        result.message = QStringLiteral("LB pose detection succeeded.");
        qInfo(LOG_LB_POSE).noquote()
            << "LB pose succeeded"
            << "inputPoints=" << result.inputPointCount
            << "deviationMm=" << result.poseDeviationMm;
        return result;
    } catch (const std::exception& ex) {
        // 捕获标准异常
        result.resultCode = 7;
        result.message = QStringLiteral("LB pose exception: %1").arg(QString::fromLocal8Bit(ex.what()));
        qWarning(LOG_LB_POSE).noquote() << result.message;
        return result;
    } catch (...) {
        // 捕获未知异常
        result.resultCode = 7;
        result.message = QStringLiteral("LB pose unknown exception.");
        qWarning(LOG_LB_POSE).noquote() << result.message;
        return result;
    }
}

}  // namespace scan_tracking::tracking