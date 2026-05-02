#pragma once

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/tracking/tracking_service.h"

namespace scan_tracking::tracking {

PoseCheckResult runLegacyLbPoseCheck(const scan_tracking::common::LbPoseConfig& config);

}  // namespace scan_tracking::tracking
