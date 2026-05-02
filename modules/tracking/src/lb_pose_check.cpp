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

Q_LOGGING_CATEGORY(LOG_LB_POSE, "tracking.lb_pose")

namespace scan_tracking::tracking {
namespace {

namespace fs = std::filesystem;

QString defaultDataRoot()
{
    return QStringLiteral("D:/work/scan-tracking/third_party/lb_pose_detection/data");
}

QString normalizeOrDefault(const QString& value, const QString& fallback)
{
    return value.trimmed().isEmpty() ? fallback : value.trimmed();
}

std::string toCvPath(const QString& path)
{
    return QDir::toNativeSeparators(path).toStdString();
}

void configureDefaultCalibration(TR_INSPECT_3D_Recon_Marker& recon)
{
    recon.config.I1 = (cv::Mat_<double>(3, 3) << 5.078851406536548e+03, 0.830568826844289, 2.746479519311858e+03,
                                                  0.0, 5.079564338697494e+03, 1.827274288235361e+03,
                                                  0.0, 0.0, 1.0);
    recon.config.D1 = (cv::Mat_<double>(1, 5) << -0.061121083586165,
                                                  0.174884596596884,
                                                  -1.053862530437392e-04,
                                                  -2.625558299490124e-04,
                                                  -0.174942436164493);
    recon.config.E1 = cv::Mat::eye(4, 4, CV_64F);
    recon.config.I2 = (cv::Mat_<double>(3, 3) << 5.088957721152494e+03, 1.694422728104837, 2.748597487208202e+03,
                                                  0.0, 5.087725659008389e+03, 1.818343109063463e+03,
                                                  0.0, 0.0, 1.0);
    recon.config.D2 = (cv::Mat_<double>(1, 5) << -0.061336067087922,
                                                  0.140736778029161,
                                                  -2.839150977966796e-04,
                                                  0.001241546114496,
                                                  -0.079946406594583);
    recon.config.E2 = (cv::Mat_<double>(4, 4) << 0.932342748446725, -0.009472020725314, 0.361451629187345, -5.793657636690184e+02,
                                                  -0.014055020969881, 0.997951882006392, 0.062405909859861, -13.667600451372955,
                                                  -0.361302443673362, -0.063263907745887, 0.930300071037500, 1.265698817906372e+02,
                                                  0.0, 0.0, 0.0, 1.0);
}

bool isValidRt(const cv::Mat& rt)
{
    return !rt.empty() && rt.rows == 4 && rt.cols == 4 && rt.type() == CV_64F;
}

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

QString summarizePath(const QString& path)
{
    return QDir::toNativeSeparators(path.trimmed());
}

}  // namespace

PoseCheckResult runLegacyLbPoseCheck(const scan_tracking::common::LbPoseConfig& config)
{
    PoseCheckResult result;
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

    const QFileInfo dataRootInfo(dataRootText);
    if (!dataRootInfo.exists()) {
        result.resultCode = 9;
        result.message = QStringLiteral("LB pose data root does not exist: %1").arg(summarizePath(dataRootText));
        qWarning(LOG_LB_POSE).noquote() << result.message;
        return result;
    }

    std::vector<cv::String> leftFiles;
    std::vector<cv::String> rightFiles;
    cv::glob(toCvPath(leftPatternText), leftFiles, false);
    cv::glob(toCvPath(rightPatternText), rightFiles, false);

    if (leftFiles.empty() || rightFiles.empty()) {
        result.resultCode = 5;
        result.message = QStringLiteral("LB pose input images missing: left=%1, right=%2")
                          .arg(static_cast<int>(leftFiles.size()))
                          .arg(static_cast<int>(rightFiles.size()));
        qWarning(LOG_LB_POSE).noquote() << result.message;
        return result;
    }

    if (leftFiles.size() != rightFiles.size()) {
        result.resultCode = 9;
        result.message = QStringLiteral("LB pose image count mismatch: left=%1, right=%2")
                          .arg(static_cast<int>(leftFiles.size()))
                          .arg(static_cast<int>(rightFiles.size()));
        qWarning(LOG_LB_POSE).noquote() << result.message;
        return result;
    }

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
        TR_INSPECT_3D_Recon_Marker recon;
        configureDefaultCalibration(recon);
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

        FastGeoHash geoHash(config.maxDistance, config.minDistance);
        if (geoHash.set_template_config(config.minDistance, config.maxDistance) != 0 ||
            geoHash.set_query_config(config.cosTolerance, config.minPercent) != 0) {
            result.resultCode = 7;
            result.message = QStringLiteral("LB pose failed to set hash config.");
            qWarning(LOG_LB_POSE).noquote() << result.message;
            return result;
        }

        std::string templatePathBytes = toCvPath(templateText);
        if (geoHash.read_template_pnts(templatePathBytes.data()) != 0) {
            result.resultCode = 9;
            result.message = QStringLiteral("LB pose template file not found or invalid: %1")
                              .arg(summarizePath(templateText));
            qWarning(LOG_LB_POSE).noquote() << result.message;
            return result;
        }

        if (geoHash.build() != 0) {
            result.resultCode = 7;
            result.message = QStringLiteral("LB pose hash build failed.");
            qWarning(LOG_LB_POSE).noquote() << result.message;
            return result;
        }

        const int trackResult = geoHash.Get_Track_Pose(
            recon.frame_3d_points,
            config.cosTolerance,
            config.minPercent);

        if (trackResult != 0 || !isValidRt(geoHash.Rt)) {
            result.resultCode = 7;
            result.message = QStringLiteral("LB pose Get_Track_Pose failed, code=%1").arg(trackResult);
            qWarning(LOG_LB_POSE).noquote() << result.message;
            return result;
        }

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
        result.resultCode = 7;
        result.message = QStringLiteral("LB pose exception: %1").arg(QString::fromLocal8Bit(ex.what()));
        qWarning(LOG_LB_POSE).noquote() << result.message;
        return result;
    } catch (...) {
        result.resultCode = 7;
        result.message = QStringLiteral("LB pose unknown exception.");
        qWarning(LOG_LB_POSE).noquote() << result.message;
        return result;
    }
}

}  // namespace scan_tracking::tracking
