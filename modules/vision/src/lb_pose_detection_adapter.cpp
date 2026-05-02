#include "scan_tracking/vision/lb_pose_detection_adapter.h"

#include <QtCore/QByteArray>
#include <QtCore/QDateTime>
#include <QtCore/QDir>

#include <opencv2/opencv.hpp>

#include "TR_Mark_3D_Recon.h"
#include "TR_Mark_Track.h"

namespace scan_tracking {
namespace vision {

namespace {

struct DefaultCalibSet {
    cv::Mat I1;
    cv::Mat D1;
    cv::Mat E1;
    cv::Mat I2;
    cv::Mat D2;
    cv::Mat E2;
};

DefaultCalibSet makeDefaultCalibSet()
{
    DefaultCalibSet calib;
    calib.I1 = (cv::Mat_<double>(3, 3) <<
        5.078851406536548e+03, 0.830568826844289, 2.746479519311858e+03,
        0.0, 5.079564338697494e+03, 1.827274288235361e+03,
        0.0, 0.0, 1.0);
    calib.D1 = (cv::Mat_<double>(1, 5) <<
        -0.061121083586165,
        0.174884596596884,
        -1.053862530437392e-04,
        -2.625558299490124e-04,
        -0.174942436164493);
    calib.E1 = cv::Mat::eye(4, 4, CV_64F);
    calib.I2 = (cv::Mat_<double>(3, 3) <<
        5.088957721152494e+03, 1.694422728104837, 2.748597487208202e+03,
        0.0, 5.087725659008389e+03, 1.818343109063463e+03,
        0.0, 0.0, 1.0);
    calib.D2 = (cv::Mat_<double>(1, 5) <<
        -0.061336067087922,
        0.140736778029161,
        -2.839150977966796e-04,
        0.001241546114496,
        -0.079946406594583);
    calib.E2 = (cv::Mat_<double>(4, 4) <<
        0.932342748446725, -0.009472020725314, 0.361451629187345, -5.793657636690184e+02,
        -0.014055020969881, 0.997951882006392, 0.062405909859861, -13.667600451372955,
        -0.361302443673362, -0.063263907745887, 0.930300071037500, 1.265698817906372e+02,
        0.0, 0.0, 0.0, 1.0);
    return calib;
}

HikPoseCaptureResult makeFailure(
    quint64 requestId,
    const QString& cameraKey,
    const QString& logicalName,
    VisionErrorCode code,
    const QString& message)
{
    HikPoseCaptureResult result;
    result.requestId = requestId;
    result.cameraKey = cameraKey;
    result.logicalName = logicalName;
    result.errorCode = code;
    result.errorMessage = message;
    result.elapsedMs = 0;
    return result;
}

LbPoseResult makeFailure(const QString& message)
{
    LbPoseResult result;
    result.invoked = true;
    result.success = false;
    result.message = message;
    return result;
}

cv::Mat frameToGrayMat(const HikMonoFrame& frame)
{
    if (!frame.isValid() || frame.pixels == nullptr || frame.width <= 0 || frame.height <= 0) {
        return {};
    }
    if (frame.stride < frame.width) {
        return {};
    }

    cv::Mat view(frame.height, frame.width, CV_8UC1, frame.pixels->data(), static_cast<std::size_t>(frame.stride));
    return view.clone();
}

PoseMatrix4x4 toPoseMatrix(const cv::Mat& rt, const QString& sourceCameraKey, quint64 frameId)
{
    PoseMatrix4x4 pose;
    if (rt.empty() || rt.rows != 4 || rt.cols != 4 || rt.type() != CV_64F) {
        return pose;
    }
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            pose.values[static_cast<std::size_t>(row * 4 + col)] = static_cast<float>(rt.at<double>(row, col));
        }
    }
    pose.frameId = frameId;
    pose.timestampMs = QDateTime::currentMSecsSinceEpoch();
    pose.sourceCameraKey = sourceCameraKey;
    pose.valid = true;
    return pose;
}

QString buildTemplatePath(const scan_tracking::common::LbPoseConfig& config)
{
    if (!config.templateFile.trimmed().isEmpty()) {
        return config.templateFile.trimmed();
    }
    const QString dataRoot = config.dataRoot.trimmed().isEmpty()
        ? QStringLiteral("D:/work/scan-tracking/third_party/lb_pose_detection/data")
        : config.dataRoot.trimmed();
    return QDir(dataRoot).filePath(QStringLiteral("template-3D-ALL-Shift-Cut-Cut.txt"));
}

}  // namespace

LbPoseResult runLbPoseDetection(
    const HikMonoFrame& leftFrame,
    const HikMonoFrame& rightFrame,
    const scan_tracking::common::LbPoseConfig& config)
{
    LbPoseResult result;
    result.invoked = true;
    result.leftImageWidth = leftFrame.width;
    result.leftImageHeight = leftFrame.height;
    result.rightImageWidth = rightFrame.width;
    result.rightImageHeight = rightFrame.height;

    if (!leftFrame.isValid() || !rightFrame.isValid()) {
        return makeFailure(QStringLiteral("LB pose detection requires two valid grayscale frames."));
    }

    const cv::Mat leftImage = frameToGrayMat(leftFrame);
    const cv::Mat rightImage = frameToGrayMat(rightFrame);
    if (leftImage.empty() || rightImage.empty()) {
        return makeFailure(QStringLiteral("Failed to convert Hik frames to cv::Mat grayscale images."));
    }

    try {
        TR_INSPECT_3D_Recon_Marker recon;
        const auto calib = makeDefaultCalibSet();
        if (recon.Set_Calib_Config(calib.I1, calib.D1, calib.E1, calib.I2, calib.D2, calib.E2) != 0) {
            return makeFailure(QStringLiteral("Failed to configure default stereo calibration."));
        }

        if (recon.Get_3D_Recon_Marker(const_cast<cv::Mat&>(leftImage), const_cast<cv::Mat&>(rightImage)) != 0) {
            return makeFailure(QStringLiteral("TR_INSPECT_3D_Recon_Marker failed to reconstruct stereo points."));
        }

        FastGeoHash geoHash(config.maxDistance, config.minDistance);
        if (geoHash.set_template_config(config.minDistance, config.maxDistance) != 0) {
            return makeFailure(QStringLiteral("Failed to set LB template configuration."));
        }
        if (geoHash.set_query_config(config.cosTolerance, config.minPercent) != 0) {
            return makeFailure(QStringLiteral("Failed to set LB query configuration."));
        }

        const QString templatePath = buildTemplatePath(config);
        QByteArray templateBytes = templatePath.toLocal8Bit();
        if (geoHash.read_template_pnts(templateBytes.data()) != 0) {
            return makeFailure(QStringLiteral("Failed to load LB template points from %1.").arg(templatePath));
        }
        if (geoHash.build() != 0) {
            return makeFailure(QStringLiteral("Failed to build LB geometric hash table."));
        }

        const int trackResult = geoHash.Get_Track_Pose(
            recon.frame_3d_points,
            config.cosTolerance,
            config.minPercent);
        if (trackResult != 0) {
            return makeFailure(QStringLiteral("LB tracking returned error code %1.").arg(trackResult));
        }

        result.success = true;
        result.message = QStringLiteral("LB pose detection completed successfully.");
        result.framePointCount = static_cast<int>(recon.frame_3d_points.size());
        result.poseMatrix = toPoseMatrix(geoHash.Rt, QStringLiteral("lb_pose_detection"), 0);
        if (!result.poseMatrix.isValid()) {
            return makeFailure(QStringLiteral("LB pose detection produced an invalid Rt matrix."));
        }
        return result;
    } catch (const std::exception& ex) {
        return makeFailure(QStringLiteral("LB pose detection threw exception: %1").arg(QString::fromLocal8Bit(ex.what())));
    } catch (...) {
        return makeFailure(QStringLiteral("LB pose detection threw an unknown exception."));
    }
}

}  // namespace vision
}  // namespace scan_tracking
