#ifndef THIRD_OUTSURFACE_DETECTION_H
#define THIRD_OUTSURFACE_DETECTION_H

#include <pcl/common/common.h>
#include "model/PointsModel.h"
#include "utils/Params.h"

class ThirdOutSurfaceDetection {
public:
    // 构造函数：初始化第三检测位参数结构体。
    ThirdOutSurfaceDetection();

    // 第三检测位外表面检测主流程。
    bool Detect(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    // 获取检测结果参数（只读/可写）。
    const ThirdPoseDetectionParams& GetParams() const;
    ThirdPoseDetectionParams& GetParams();

private:
    // 第三检测位监测结构体。
    ThirdPoseDetectionParams thirdpose_params_;
    // 每个检测类独立持有点云模型，避免外部共享实例。
    PointsModel points_model_;
};

#endif
