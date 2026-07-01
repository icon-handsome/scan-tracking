#pragma once

#include "HeadMeasure/Types.h"

namespace hm {

class MeasurePipeline {
public:
    explicit MeasurePipeline(MeasureConfig config);
    MeasureResult run();
    MeasureResult runWithScanCloud(const CloudConstPtr& rawScan);
    /// IPC/demo 多帧：逐帧 preprocess 后合并，与 mergeFrames 一致。
    MeasureResult runWithScanClouds(const std::vector<CloudConstPtr>& rawScans);
    /// IPC 在线路径：输入已完成 pose 矫正与降采样，跳过 preprocess 中的 SOR/二次体素。
    MeasureResult runWithPreprocessedScanCloud(const CloudConstPtr& preprocessedScan);

private:
    MeasureResult runPipelineWithPreprocessedScan(const CloudConstPtr& scan);

    MeasureConfig config_;
};

}  // namespace hm
