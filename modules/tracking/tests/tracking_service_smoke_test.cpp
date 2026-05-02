#include "scan_tracking/tracking/tracking_service.h"

#include <iostream>
#include <memory>
#include <vector>

#include <QCoreApplication>
#include <QFile>
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

scan_tracking::mech_eye::CaptureResult makeCaptureResult(int pointCount, bool success = true)
{
    scan_tracking::mech_eye::CaptureResult result;
    result.errorCode = success
        ? scan_tracking::mech_eye::CaptureErrorCode::Success
        : scan_tracking::mech_eye::CaptureErrorCode::CaptureFailed;

    if (pointCount > 0) {
        auto points = std::make_shared<std::vector<float>>();
        points->reserve(static_cast<std::size_t>(pointCount * 3));
        for (int index = 0; index < pointCount; ++index) {
            points->push_back(static_cast<float>(index));
            points->push_back(static_cast<float>(index + 1));
            points->push_back(static_cast<float>(index + 2));
        }
        result.pointCloud.pointsXYZ = points;
        result.pointCloud.pointCount = pointCount;
        result.pointCloud.width = pointCount;
        result.pointCloud.height = 1;
    }

    return result;
}

bool testRejectsWhenThereAreFewerThanThreeValidSegments()
{
    scan_tracking::tracking::TrackingService service;
    QMap<int, scan_tracking::mech_eye::CaptureResult> segments;
    segments.insert(1, makeCaptureResult(2));
    segments.insert(2, makeCaptureResult(2));

    const auto result = service.inspectSegments(segments);

    bool ok = true;
    ok &= expectTrue(result.resultCode == 2, "should return NG when fewer than three valid segments are available");
    ok &= expectTrue(result.totalPointCount == 4, "should still count all valid points");
    ok &= expectTrue(result.ngReasonWord0 == (1u << 4), "should use missing-input NG bit for insufficient segments");
    ok &= expectTrue(result.message.contains(QStringLiteral("缺少必需分段")), "should explain missing required segment count");
    return ok;
}

bool 写入测试配置(int 外表面段号, int 内表面段号, int 开孔段号)
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
    stream << "[Tracking]\n";
    stream << "firstStationOuterSegmentIndex=" << 外表面段号 << "\n";
    stream << "firstStationInnerSegmentIndex=" << 内表面段号 << "\n";
    stream << "firstStationHoleSegmentIndex=" << 开孔段号 << "\n";
    file.close();
    return true;
}

bool testUsesConfiguredSegmentMapping()
{
    scan_tracking::common::ConfigManager::cleanup();
    if (!写入测试配置(10, 20, 30)) {
        return false;
    }
    scan_tracking::common::ConfigManager::initialize();

    scan_tracking::tracking::TrackingService service;
    QMap<int, scan_tracking::mech_eye::CaptureResult> segments;
    segments.insert(10, makeCaptureResult(2));
    segments.insert(20, makeCaptureResult(2));
    segments.insert(30, makeCaptureResult(2));
    segments.insert(40, makeCaptureResult(2));

    const auto result = service.inspectSegments(segments);
    scan_tracking::common::ConfigManager::cleanup();

    bool ok = true;
    ok &= expectTrue(result.resultCode == 2, "tiny synthetic clouds should still fail the real first-station pipeline");
    ok &= expectTrue(result.totalPointCount == 8, "should count all valid points before invoking the adapter");
    ok &= expectTrue(result.ngReasonWord0 == (1u << 5), "FirstOut failure should map to the temporary FirstOut NG bit");
    ok &= expectTrue(result.message.contains(QStringLiteral("外表面算法失败")), "result message should show that FirstOut was really invoked");
    ok &= expectTrue(result.message.contains("[10,20,30]"), "result message should report the selected segment indices");
    return ok;
}

}  // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    scan_tracking::common::ConfigManager::cleanup();
    if (!写入测试配置(1, 2, 3)) {
        return 1;
    }
    scan_tracking::common::ConfigManager::initialize();

    bool ok = true;
    ok &= testRejectsWhenThereAreFewerThanThreeValidSegments();
    ok &= testUsesConfiguredSegmentMapping();

    scan_tracking::common::ConfigManager::cleanup();

    if (!ok) {
        return 1;
    }

    std::cout << "Tracking service smoke tests passed\n";
    return 0;
}
