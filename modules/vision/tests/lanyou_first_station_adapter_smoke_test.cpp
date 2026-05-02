#include "scan_tracking/vision/lanyou_first_station_adapter.h"

#include <iostream>

namespace {

bool expectTrue(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "FAILED: " << message << '\n';
        return false;
    }
    return true;
}

bool testRejectsMissingFramesBeforeInvokingAlgorithms()
{
    scan_tracking::vision::lanyou::FirstStationFrameSet frames;
    const auto result = scan_tracking::vision::lanyou::runFirstStationDetection(frames);

    bool ok = true;
    ok &= expectTrue(!result.invoked, "pipeline should not invoke algorithms when required frames are missing");
    ok &= expectTrue(!result.firstOutInvoked, "FirstOut should not run when outer cloud is missing");
    ok &= expectTrue(!result.firstInlinerInvoked, "FirstInliner should not run when required clouds are missing");
    ok &= expectTrue(result.outerPointCount == 0, "outer point count should be zero for empty input");
    ok &= expectTrue(!result.message.isEmpty(), "result should contain a diagnostic message");
    return ok;
}

}  // namespace

int main()
{
    if (!testRejectsMissingFramesBeforeInvokingAlgorithms()) {
        return 1;
    }

    std::cout << "Lanyou first station adapter smoke tests passed\n";
    return 0;
}
