#ifndef SECOND_INLINERSURFACE_DETECTION_H
#define SECOND_INLINERSURFACE_DETECTION_H

#include <pcl/common/common.h>
#include "model/PointsModel.h"
#include "utils/Params.h"

class SecondInlinerSurfaceDetection {
public:
    // 构造函数：初始化第二检测位参数结构体。
    SecondInlinerSurfaceDetection();

    // 第二检测位内表面检测主流程。
    bool Detect(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    // 获取检测结果参数（只读/可写）。
    const SecondPoseDetectionParams& GetParams() const;
    SecondPoseDetectionParams& GetParams();

private:
    // 第二检测位监测结构体。
    SecondPoseDetectionParams secondpose_params_;
    // 每个检测类独立持有点云模型，避免外部共享实例。
    PointsModel points_model_;
};

#endif
