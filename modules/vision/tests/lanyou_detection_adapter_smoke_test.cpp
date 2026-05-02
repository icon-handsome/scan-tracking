#include "scan_tracking/vision/lanyou_detection_adapter.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "scan_tracking/mech_eye/mech_eye_types.h"

namespace {

bool expectTrue(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "FAILED: " << message << '\n';
        return false;
    }
    return true;
}

scan_tracking::mech_eye::PointCloudFrame makeSyntheticFrame()
{
    scan_tracking::mech_eye::PointCloudFrame frame;
    frame.pointsXYZ = std::make_shared<std::vector<float>>(std::initializer_list<float>{
        1.0f, 2.0f, 3.0f,
        4.0f, 5.0f, 6.0f,
    });
    frame.width = 2;
    frame.height = 1;
    frame.pointCount = 2;
    return frame;
}

bool testConvertsPointCloudFrameToPcl()
{
    const auto frame = makeSyntheticFrame();
    const auto cloud = scan_tracking::vision::lanyou::toPclPointCloud(frame);

    bool ok = true;
    ok &= expectTrue(cloud != nullptr, "converted cloud should not be null");
    ok &= expectTrue(cloud->size() == 2, "converted cloud should contain two points");
    ok &= expectTrue(cloud->width == 2, "converted cloud width should preserve frame width");
    ok &= expectTrue(cloud->height == 1, "converted cloud height should preserve frame height");
    ok &= expectTrue(std::fabs(cloud->points[1].z - 6.0f) < 0.0001f, "converted z coordinate should match input");
    return ok;
}

bool testCanInvokeLanyouFirstOutDetector()
{
    const auto frame = makeSyntheticFrame();
    const auto result = scan_tracking::vision::lanyou::runFirstOutDetectionSmoke(frame);

    bool ok = true;
    ok &= expectTrue(result.invoked, "smoke call should reach the Lanyou detector");
    ok &= expectTrue(!result.success, "tiny synthetic cloud should not pass the real detector");
    ok &= expectTrue(result.inputPointCount == 2, "smoke result should report converted input point count");
    ok &= expectTrue(!result.message.isEmpty(), "smoke result should include a diagnostic message");
    ok &= expectTrue(result.issueCode == scan_tracking::vision::lanyou::LanyouIssueCode::AlgorithmFailure ||
                         result.issueCode == scan_tracking::vision::lanyou::LanyouIssueCode::ResultInvalid,
                     "smoke result should return a machine-readable failure code");
    if (!result.issueTag.isEmpty()) {
        std::cerr << "smoke issue tag: " << result.issueTag.toStdString() << '\n';
    }
    std::cerr << "smoke message: " << result.message.toStdString() << '\n';
    return ok;
}

}  // namespace

int main()
{
    bool ok = true;
    ok &= testConvertsPointCloudFrameToPcl();
    ok &= testCanInvokeLanyouFirstOutDetector();

    if (!ok) {
        return 1;
    }

    std::cout << "Lanyou adapter smoke tests passed\n";
    return 0;
}
