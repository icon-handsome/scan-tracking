#pragma once

#include <QtCore/QString>

#include <vector>

namespace scan_tracking::common {

/** @brief Load xyz from PCD into exe-owned std::vector without PCL IO. */
bool loadPointCloudXyzFromPcd(
    const QString& absolutePath,
    std::vector<float>* outXyz,
    int maxPointCount = 0);

}  // namespace scan_tracking::common
