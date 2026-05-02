#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <QtCore/QString>

#include "scan_tracking/mech_eye/mech_eye_types.h"
#include "scan_tracking/vision/lanyou_issue_codes.h"

namespace scan_tracking {
namespace vision {
namespace lanyou {

struct LanyouSmokeResult {
    bool invoked = false;
    bool success = false;
    int inputPointCount = 0;
    LanyouIssueCode issueCode = LanyouIssueCode::Success;
    QString issueTag;
    QString message;
};

pcl::PointCloud<pcl::PointXYZ>::Ptr toPclPointCloud(
    const scan_tracking::mech_eye::PointCloudFrame& frame);

LanyouSmokeResult runFirstOutDetectionSmoke(
    const scan_tracking::mech_eye::PointCloudFrame& frame);

}  // namespace lanyou
}  // namespace vision
}  // namespace scan_tracking
