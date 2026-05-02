#pragma once

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/vision/vision_types.h"

namespace scan_tracking {
namespace vision {

LbPoseResult runLbPoseDetection(
    const HikMonoFrame& leftFrame,
    const HikMonoFrame& rightFrame,
    const scan_tracking::common::LbPoseConfig& config);

}  // namespace vision
}  // namespace scan_tracking
