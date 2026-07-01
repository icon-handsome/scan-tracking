#include "scan_tracking/tracking/tracking_service.h"

#include <iostream>
#include <cmath>
#include <memory>
#include <vector>

#include <QCoreApplication>
#include <QFile>
#include <QJsonObject>
#include <QTextStream>

#include "scan_tracking/common/config_manager.h"

namespace {

bool expectTrue(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "FAILED: " << message << '\n';
        return false;
    }
    return true;
}

scan_tracking::mech_eye::PointCloudFrame makePointCloudFrame(int pointCount)
{
    scan_tracking::mech_eye::PointCloudFrame frame;
    if (pointCount <= 0) {
        return frame;
    }

    auto points = std::make_shared<std::vector<float>>();
    points->reserve(static_cast<std::size_t>(pointCount * 3));
    for (int index = 0; index < pointCount; ++index) {
        points->push_back(static_cast<float>(index));
        points->push_back(static_cast<float>(index + 1));
        points->push_back(static_cast<float>(index + 2));
    }

    frame.pointsXYZ = points;
    frame.pointCount = pointCount;
    frame.width = pointCount;
    frame.height = 1;
    return frame;
}

bool writeTestConfig()
{
    const QString configPath = QCoreApplication::applicationDirPath() + QStringLiteral("/config.ini");
    QFile file(configPath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        std::cerr << "FAILED: cannot write config.ini\n";
        return false;
    }

    QTextStream stream(&file);
    stream << "[App]\nversion=0.1.0\nenvironment=test\n";
    stream << "[Logger]\nlevel=0\nrotateDays=7\n";
    stream << "[Modbus]\nhost=127.0.0.1\nport=502\nunitId=1\ntimeoutMs=1000\nreconnectIntervalMs=2000\n";
    stream << "[Camera]\ndefaultCamera=Mech-Eye Nano\nscanTimeoutMs=5000\n";
    stream << "[Vision]\nmechEyeCameraKey=Mech-Eye Nano\nmechCaptureTimeoutMs=5000\nhikConnectTimeoutMs=3000\nhikCaptureTimeoutMs=1000\nhikSdkRoot=D:/work/scan-tracking/third_party/hik_mvs\nhikCameraAName=hik_camera_a\nhikCameraAKey=192.168.10.12\nhikCameraAIp=192.168.10.12\nhikCameraASerial=\nhikCameraBName=hik_camera_b\nhikCameraBKey=192.168.10.13\nhikCameraBIp=192.168.10.13\nhikCameraBSerial=\n";
    stream << "[FlowControl]\npollIntervalMs=100\nheartbeatIntervalMs=1000\nsimulatedProcessingMs=300\n";
    stream << "[Tracking]\nscanSegmentTotal=3\n";
    stream << "[Bevel]\nconfigPath=bevel/config.txt\ntemplateDir=bevel\n";
    stream << "angleTolDeg=2.0\nlengthTolMm=1.0\ndefaultBevelType=0\ndefaultAngleDeg=45.0\ndefaultLengthMm=1.0\n";
    file.close();
    return true;
}

bool testRejectsEmptyPointCloud()
{
    scan_tracking::tracking::TrackingService service;
    const auto frame = makePointCloudFrame(0);
    const auto result = service.inspectPointCloud(frame, 0);

    bool ok = true;
    ok &= expectTrue(result.resultCode == 2, "empty cloud should return NG");
    ok &= expectTrue(result.ngReasonWord0 == (1u << 4), "empty cloud should use missing-input NG bit");
    ok &= expectTrue(result.message.contains(QStringLiteral("没有可用点云")),
                     "empty cloud should explain missing input");
    return ok;
}

bool testRejectsMissingRecipeWhenDefaultsDisabled()
{
    scan_tracking::common::ConfigManager::cleanup();
    const QString configPath = QCoreApplication::applicationDirPath() + QStringLiteral("/config.ini");
    QFile file(configPath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        std::cerr << "FAILED: cannot rewrite config.ini\n";
        return false;
    }
    QTextStream stream(&file);
    stream << "[App]\nversion=0.1.0\nenvironment=test\n";
    stream << "[Logger]\nlevel=0\nrotateDays=7\n";
    stream << "[Modbus]\nhost=127.0.0.1\nport=502\nunitId=1\ntimeoutMs=1000\nreconnectIntervalMs=2000\n";
    stream << "[Camera]\ndefaultCamera=Mech-Eye Nano\nscanTimeoutMs=5000\n";
    stream << "[Vision]\nmechEyeCameraKey=Mech-Eye Nano\nmechCaptureTimeoutMs=5000\nhikConnectTimeoutMs=3000\nhikCaptureTimeoutMs=1000\nhikSdkRoot=D:/work/scan-tracking/third_party/hik_mvs\nhikCameraAName=hik_camera_a\nhikCameraAKey=192.168.10.12\nhikCameraAIp=192.168.10.12\nhikCameraASerial=\nhikCameraBName=hik_camera_b\nhikCameraBKey=192.168.10.13\nhikCameraBIp=192.168.10.13\nhikCameraBSerial=\n";
    stream << "[FlowControl]\npollIntervalMs=100\nheartbeatIntervalMs=1000\nsimulatedProcessingMs=300\n";
    stream << "[Tracking]\nscanSegmentTotal=3\n";
    stream << "[Bevel]\nconfigPath=bevel/config.txt\ntemplateDir=bevel\n";
    stream << "angleTolDeg=2.0\nlengthTolMm=1.0\ndefaultBevelType=0\ndefaultAngleDeg=0\ndefaultLengthMm=0\n";
    file.close();

    scan_tracking::common::ConfigManager::initialize();
    if (scan_tracking::common::ConfigManager::instance()->hasActiveBevelRecipe()) {
        std::cerr << "FAILED: inactive default recipe should not be active\n";
        scan_tracking::common::ConfigManager::cleanup();
        return false;
    }

    scan_tracking::tracking::TrackingService service;
    const auto frame = makePointCloudFrame(4);
    const auto result = service.inspectPointCloud(frame, frame.pointCount);

    scan_tracking::common::ConfigManager::cleanup();

    bool ok = true;
    ok &= expectTrue(result.resultCode == 2, "missing recipe should return NG");
    ok &= expectTrue(result.message.contains(QStringLiteral("坡口配方")),
                     "missing recipe should explain bevel recipe requirement");
    return ok;
}

bool testInvokesBevelPipelineWithTinyCloud()
{
    scan_tracking::tracking::TrackingService service;
    const auto frame = makePointCloudFrame(8);
    const auto result = service.inspectPointCloud(frame, frame.pointCount);

    bool ok = true;
    ok &= expectTrue(result.resultCode == 2, "tiny synthetic cloud should fail Po_Kou pipeline");
    ok &= expectTrue(result.sourcePointCount == 8, "source point count should be preserved");
    ok &= expectTrue(
        result.ngReasonWord0 == (1u << 4) || result.ngReasonWord0 == (1u << 5),
        "failure should map to adapter or algorithm NG bit");
    ok &= expectTrue(!result.message.isEmpty(), "failure should include message");
    return ok;
}

bool nearDouble(double actual, double expected, double epsilon = 1e-3)
{
    return std::fabs(actual - expected) <= epsilon;
}

bool testHeadDisplayMetricsFields()
{
    scan_tracking::tracking::InspectionMeasurement measurement;
    measurement.headAngleTol = 45.2f;
    measurement.bluntHeightTol = 1.05f;

    QJsonObject payload;
    scan_tracking::tracking::appendHeadDisplayMetricsFields(payload, measurement);

    const QJsonObject headMetrics = payload.value(QStringLiteral("headMetrics")).toObject();
    if (headMetrics.size() != 12) {
        std::cerr << "FAILED: headMetrics should contain 12 keys\n";
        return false;
    }

    bool ok = true;
    ok &= expectTrue(
        nearDouble(headMetrics.value(QStringLiteral("bevel_angle_deg")).toDouble(), 45.2),
        "bevel_angle_deg should match Po_Kou measurement");
    ok &= expectTrue(
        nearDouble(headMetrics.value(QStringLiteral("blunt_height_mm")).toDouble(), 1.05),
        "blunt_height_mm should match Po_Kou measurement");

    const QStringList zeroKeys = {
        QStringLiteral("inner_diameter_mm"),
        QStringLiteral("roundness_tol"),
        QStringLiteral("straight_slope_tol"),
        QStringLiteral("head_depth_mm"),
        QStringLiteral("straight_height_tol"),
        QStringLiteral("inner_circumference_mm"),
        QStringLiteral("hole_opening_mm"),
        QStringLiteral("joint_fit_up_angle_deg"),
        QStringLiteral("thickness_mm"),
        QStringLiteral("head_volume_m3"),
    };
    for (const QString& key : zeroKeys) {
        ok &= expectTrue(
            nearDouble(headMetrics.value(key).toDouble(), 0.0),
            qPrintable(QStringLiteral("placeholder metric should be zero: %1").arg(key)));
    }
    return ok;
}

bool testHeadDisplayMetricsDefaultsToZero()
{
    scan_tracking::tracking::InspectionMeasurement measurement;
    QJsonObject payload;
    scan_tracking::tracking::appendHeadDisplayMetricsFields(payload, measurement);

    const QJsonObject headMetrics = payload.value(QStringLiteral("headMetrics")).toObject();
    if (headMetrics.size() != 12) {
        std::cerr << "FAILED: default headMetrics should contain 12 keys\n";
        return false;
    }

    bool ok = true;
    for (auto it = headMetrics.begin(); it != headMetrics.end(); ++it) {
        ok &= expectTrue(
            nearDouble(it.value().toDouble(), 0.0),
            qPrintable(QStringLiteral("default metric should be zero: %1").arg(it.key())));
    }
    return ok;
}

}  // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    scan_tracking::common::ConfigManager::cleanup();
    if (!writeTestConfig()) {
        return 1;
    }
    scan_tracking::common::ConfigManager::initialize();

    bool ok = true;
    ok &= testRejectsEmptyPointCloud();
    ok &= testInvokesBevelPipelineWithTinyCloud();
    ok &= testRejectsMissingRecipeWhenDefaultsDisabled();
    ok &= testHeadDisplayMetricsFields();
    ok &= testHeadDisplayMetricsDefaultsToZero();

    scan_tracking::common::ConfigManager::cleanup();

    if (!ok) {
        return 1;
    }

    std::cout << "Tracking service smoke tests passed\n";
    return 0;
}
