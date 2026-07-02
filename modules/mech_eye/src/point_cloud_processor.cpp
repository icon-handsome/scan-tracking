/**
 * @file point_cloud_processor.cpp
 * @brief PCL 点云后处理与 4×4 坐标变换实现
 *
 * 后处理流水线（config.enabled 时按序执行）：
 * PassThrough(z) → StatisticalOutlierRemoval → MovingLeastSquares → VoxelGrid
 */
#include "scan_tracking/mech_eye/point_cloud_processor.h"

#include <QLoggingCategory>

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>

#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>

#include <Eigen/Core>

Q_LOGGING_CATEGORY(LOG_POINT_CLOUD_PROC, "mech_eye.point_cloud_processor")

namespace scan_tracking::mech_eye {

/**
 * @brief 进程内 PCL/Eigen 全局互斥锁
 *
 * Windows 下多线程并发调用 PCL（尤其 MLS/VoxelGrid）会触发 Eigen aligned_free 崩溃。
 * StateMachine 分段 refinement 与坡口测量可能并行，所有 PCL 入口须持有此锁。
 */
std::mutex& pointCloudAlgorithmMutex()
{
    static std::mutex mutex;
    return mutex;
}

namespace {

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using CloudPtr = Cloud::Ptr;

/** @brief 判断三维坐标是否为有限值 */
bool isFinitePoint(float x, float y, float z)
{
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

/** @brief PointCloudFrame → PCL 点云，跳过 NaN/Inf 点 */
CloudPtr toPclCloud(const PointCloudFrame& frame)
{
    auto cloud = pcl::make_shared<Cloud>();
    if (!frame.pointsXYZ || frame.pointCount <= 0) {
        return cloud;
    }

    const auto& points = *frame.pointsXYZ;
    const int availablePointCount = static_cast<int>(points.size() / 3);
    const int pointCount = std::min(frame.pointCount, availablePointCount);
    cloud->points.reserve(static_cast<std::size_t>(pointCount));

    for (int index = 0; index < pointCount; ++index) {
        const auto base = static_cast<std::size_t>(index * 3);
        const float x = points[base];
        const float y = points[base + 1];
        const float z = points[base + 2];
        if (!isFinitePoint(x, y, z)) {
            continue;
        }
        cloud->points.emplace_back(x, y, z);
    }

    cloud->width = static_cast<std::uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
}

/** @brief PCL 点云 → PointCloudFrame，保留 frameId/timestampMs 等元数据 */
PointCloudFrame fromPclCloud(const CloudPtr& cloud, const PointCloudFrame& metadata)
{
    PointCloudFrame frame;
    frame.frameId = metadata.frameId;
    frame.timestampMs = metadata.timestampMs;
    frame.pointsXYZ = std::make_shared<std::vector<float>>();
    frame.normalsXYZ.reset();

    if (!cloud || cloud->empty()) {
        frame.width = 0;
        frame.height = 0;
        frame.pointCount = 0;
        return frame;
    }

    const int count = static_cast<int>(cloud->size());
    frame.pointsXYZ->reserve(static_cast<std::size_t>(count) * 3);
    for (const auto& point : cloud->points) {
        frame.pointsXYZ->push_back(point.x);
        frame.pointsXYZ->push_back(point.y);
        frame.pointsXYZ->push_back(point.z);
    }

    frame.pointCount = count;
    frame.width = count;
    frame.height = 1;
    return frame;
}

/** @brief 深拷贝 PointCloudFrame（含 pointsXYZ / normalsXYZ 数组） */
PointCloudFrame clonePointCloudFrameFull(const PointCloudFrame& src)
{
    PointCloudFrame dst = src;
    if (src.pointsXYZ) {
        dst.pointsXYZ = std::make_shared<std::vector<float>>(*src.pointsXYZ);
    }
    if (src.normalsXYZ) {
        dst.normalsXYZ = std::make_shared<std::vector<float>>(*src.normalsXYZ);
    }
    return dst;
}

/** Windows 上对百万级点云跑 SOR/MLS 易触发堆损坏（0xC0000374）；超限仅做深度裁剪+体素降采样 */
constexpr std::size_t kHeavyFilterMaxPoints = 500'000;

/** @brief 检查滤波后点数是否满足 minPoints 下限，不足时写入 message */
bool checkMinPoints(const CloudPtr& cloud, int minPoints, const QString& stepLabel, QString* message)
{
    if (!cloud || static_cast<int>(cloud->size()) < minPoints) {
        if (message != nullptr) {
            *message = QStringLiteral("%1 后点数不足: %2 < %3")
                           .arg(stepLabel)
                           .arg(cloud ? static_cast<int>(cloud->size()) : 0)
                           .arg(minPoints);
        }
        return false;
    }
    return true;
}

/** @brief 体素降采样；失败时写入 message */
bool applyVoxelDownsample(
    CloudPtr& cloud,
    float leafSizeMm,
    int minPoints,
    const QString& stepLabel,
    QString* message)
{
    if (leafSizeMm <= 0.0f || !cloud || cloud->empty()) {
        return true;
    }

    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(leafSizeMm, leafSizeMm, leafSizeMm);
    CloudPtr filtered = pcl::make_shared<Cloud>();
    voxel.filter(*filtered);
    cloud = filtered;
    qDebug(LOG_POINT_CLOUD_PROC).noquote()
        << QStringLiteral("体素降采样 leaf=") << leafSizeMm
        << QStringLiteral(" mm，剩余点数=") << cloud->size();
    return checkMinPoints(cloud, minPoints, stepLabel, message);
}

/** @brief 返回行优先 4×4 单位矩阵 */
std::array<float, 16> identityMatrix4x4()
{
    std::array<float, 16> matrix{};
    matrix[0] = matrix[5] = matrix[10] = matrix[15] = 1.0f;
    return matrix;
}

/** @brief 行优先 float[16] → Eigen::Matrix4f（按 row-major 索引映射） */
Eigen::Matrix4f rowMajorToEigen(const std::array<float, 16>& matrix)
{
    Eigen::Matrix4f eigenMatrix;
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            eigenMatrix(row, col) = matrix[static_cast<std::size_t>(row * 4 + col)];
        }
    }
    return eigenMatrix;
}

}  // namespace

/**
 * @brief 行优先 4×4 矩阵乘法：out = left × right
 *
 * 索引约定：matrix[row * 4 + col]，与 StateMachine、LBN 位姿链、scan_paths_config 一致。
 * 不使用 Eigen 乘法以避免额外依赖，矩阵规模固定 4×4 开销可忽略。
 */
std::array<float, 16> multiplyRowMajor4x4(
    const std::array<float, 16>& left,
    const std::array<float, 16>& right)
{
    std::array<float, 16> out{};
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            float sum = 0.0f;
            for (int k = 0; k < 4; ++k) {
                sum += left[static_cast<std::size_t>(row * 4 + k)] *
                       right[static_cast<std::size_t>(k * 4 + col)];
            }
            out[static_cast<std::size_t>(row * 4 + col)] = sum;
        }
    }
    return out;
}

/**
 * @brief 将分段点云变换到统一坐标系
 *
 * 变换矩阵：combined = calibrationMatrixT0Prime × stereoTrackingMatrixT
 * IPC 约定：
 * - LB 位姿检测成功：calibration=Rt_global(T0)，stereo=I
 * - 否则：calibration=T0'，stereo=I（兼容旧链式）
 *
 * 内部使用 pcl::transformPointCloud，持有 pointCloudAlgorithmMutex。
 */
bool transformPointCloudFrame(
    const PointCloudFrame& input,
    const std::array<float, 16>& calibrationMatrixT0Prime,
    const std::array<float, 16>& stereoTrackingMatrixT,
    PointCloudFrame* output,
    QString* message)
{
    if (output == nullptr) {
        if (message != nullptr) {
            *message = QStringLiteral("点云拼接输出指针为空。");
        }
        return false;
    }

    if (!input.isValid()) {
        if (message != nullptr) {
            *message = QStringLiteral("点云拼接输入无效。");
        }
        return false;
    }

    std::lock_guard<std::mutex> pclLock(pointCloudAlgorithmMutex());

    const auto combinedTransform =
        multiplyRowMajor4x4(calibrationMatrixT0Prime, stereoTrackingMatrixT);
    // row-major float[16] → Eigen::Matrix4f，供 pcl::transformPointCloud 使用
    const Eigen::Matrix4f eigenTransform = rowMajorToEigen(combinedTransform);

    CloudPtr cloud = toPclCloud(input);
    if (!cloud || cloud->empty()) {
        if (message != nullptr) {
            *message = QStringLiteral("点云拼接时有效点数为 0。");
        }
        return false;
    }

    CloudPtr transformedCloud = pcl::make_shared<Cloud>();
    pcl::transformPointCloud(*cloud, *transformedCloud, eigenTransform);
    *output = fromPclCloud(transformedCloud, input);

    if (!output->isValid()) {
        if (message != nullptr) {
            *message = QStringLiteral("点云拼接后输出无效。");
        }
        return false;
    }

    if (message != nullptr) {
        *message = QStringLiteral("点云拼接完成：%1 -> %2 点（T0'×T）")
                       .arg(input.pointCount)
                       .arg(output->pointCount);
    }
    return true;
}

/**
 * @brief 对 Mech-Eye 点云执行可配置 PCL 后处理流水线
 *
 * config.enabled=false 时深拷贝直通，不持有额外锁开销以上的逻辑。
 * enabled=true 时按序执行（任一步骤点数低于 minPointsAfterProcessing 则失败返回）：
 *   1. PassThrough(z)     — depthMinMm ~ depthMaxMm
 *   2. StatisticalOutlierRemoval — meanK + stddevMul
 *   3. MovingLeastSquares — 表面平滑
 *   4. VoxelGrid          — 体素降采样
 *
 * @param report 可选；写入 input/output 点数与摘要信息
 */
bool processPointCloudFrame(
    const PointCloudFrame& input,
    const common::PointCloudProcessingConfig& config,
    PointCloudFrame* output,
    PointCloudProcessReport* report)
{
    if (output == nullptr) {
        return false;
    }

    std::lock_guard<std::mutex> pclLock(pointCloudAlgorithmMutex());

    PointCloudProcessReport localReport;
    localReport.inputPointCount = input.pointCount;

    if (!input.isValid()) {
        localReport.message = QStringLiteral("输入点云无效。");
        if (report != nullptr) {
            *report = localReport;
        }
        return false;
    }

    if (!config.enabled) {
        // 后处理关闭：完整深拷贝，保留 normals 等附属数据
        *output = clonePointCloudFrameFull(input);
        localReport.outputPointCount = output->pointCount;
        localReport.message = QStringLiteral("点云后处理已禁用，直通原始点云。");
        if (report != nullptr) {
            *report = localReport;
        }
        return true;
    }

    CloudPtr cloud = toPclCloud(input);
    if (cloud->empty()) {
        localReport.message = QStringLiteral("有效点数为 0，无法后处理。");
        if (report != nullptr) {
            *report = localReport;
        }
        return false;
    }

    qDebug(LOG_POINT_CLOUD_PROC).noquote()
        << QStringLiteral("点云后处理开始，有效点数=") << cloud->size();

    const int minPoints = std::max(1, config.minPointsAfterProcessing);
    QString failMessage;
    bool voxelAlreadyApplied = false;
    const bool skipHeavyFilters = cloud->size() > kHeavyFilterMaxPoints;
    if (skipHeavyFilters) {
        qWarning(LOG_POINT_CLOUD_PROC).noquote()
            << QStringLiteral("点数=") << cloud->size()
            << QStringLiteral(" 超过 SOR/MLS 安全上限 ")
            << kHeavyFilterMaxPoints
            << QStringLiteral("，跳过离群点去除与表面平滑，优先体素降采样");
    }

    // --- 步骤 1：Z 轴深度裁剪（相机坐标系，单位 mm） ---
    if (config.depthMinMm < config.depthMaxMm) {
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(config.depthMinMm, config.depthMaxMm);
        CloudPtr filtered = pcl::make_shared<Cloud>();
        pass.filter(*filtered);
        cloud = filtered;
        qDebug(LOG_POINT_CLOUD_PROC).noquote()
            << QStringLiteral("深度裁剪 z=[") << config.depthMinMm << QStringLiteral(",") << config.depthMaxMm
            << QStringLiteral("] mm，剩余点数=") << cloud->size();
        if (!checkMinPoints(cloud, minPoints, QStringLiteral("深度裁剪"), &failMessage)) {
            localReport.message = failMessage;
            if (report != nullptr) {
                *report = localReport;
            }
            return false;
        }
    }

    // --- 超大点云：深度裁剪后先体素降采样，避免 SOR/MLS 占满内存或损坏堆 ---
    if (skipHeavyFilters) {
        const float leafSizeMm =
            (config.downsampleEnabled && config.voxelLeafSizeMm > 0.0f) ? config.voxelLeafSizeMm : 2.0f;
        if (!applyVoxelDownsample(
                cloud,
                leafSizeMm,
                minPoints,
                QStringLiteral("大体素降采样"),
                &failMessage)) {
            localReport.message = failMessage;
            if (report != nullptr) {
                *report = localReport;
            }
            return false;
        }
        voxelAlreadyApplied = true;
    }

    // --- 步骤 2：统计离群点去除（点数须大于 meanK 才有意义） ---
    if (!skipHeavyFilters &&
        config.outlierRemovalEnabled &&
        cloud->size() > static_cast<std::size_t>(config.outlierMeanK)) {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(static_cast<int>(std::max(1, config.outlierMeanK)));
        sor.setStddevMulThresh(config.outlierStddevMul);
        CloudPtr filtered = pcl::make_shared<Cloud>();
        sor.filter(*filtered);
        cloud = filtered;
        qDebug(LOG_POINT_CLOUD_PROC).noquote()
            << QStringLiteral("离群点去除 meanK=") << config.outlierMeanK
            << QStringLiteral(" stddevMul=") << config.outlierStddevMul
            << QStringLiteral("，剩余点数=") << cloud->size();
        if (!checkMinPoints(cloud, minPoints, QStringLiteral("离群点去除"), &failMessage)) {
            localReport.message = failMessage;
            if (report != nullptr) {
                *report = localReport;
            }
            return false;
        }
    }

    // --- 步骤 3：MLS 表面平滑（不计算法向，仅更新 xyz） ---
    if (!skipHeavyFilters &&
        config.smoothingEnabled &&
        config.mlsSearchRadiusMm > 0.0f &&
        !cloud->empty()) {
        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
        mls.setInputCloud(cloud);
        mls.setSearchRadius(config.mlsSearchRadiusMm);
        mls.setPolynomialOrder(std::max(1, config.mlsPolynomialOrder));
        mls.setComputeNormals(false);
        CloudPtr smoothed = pcl::make_shared<Cloud>();
        mls.process(*smoothed);
        cloud = smoothed;
        qDebug(LOG_POINT_CLOUD_PROC).noquote()
            << QStringLiteral("表面平滑 MLS 半径=") << config.mlsSearchRadiusMm
            << QStringLiteral(" mm 阶数=") << config.mlsPolynomialOrder
            << QStringLiteral("，剩余点数=") << cloud->size();
        if (!checkMinPoints(cloud, minPoints, QStringLiteral("表面平滑"), &failMessage)) {
            localReport.message = failMessage;
            if (report != nullptr) {
                *report = localReport;
            }
            return false;
        }
    }

    // --- 步骤 4：体素网格降采样（leaf 为立方体边长 mm） ---
    if (!voxelAlreadyApplied && config.downsampleEnabled && config.voxelLeafSizeMm > 0.0f) {
        if (!applyVoxelDownsample(
                cloud,
                config.voxelLeafSizeMm,
                minPoints,
                QStringLiteral("体素降采样"),
                &failMessage)) {
            localReport.message = failMessage;
            if (report != nullptr) {
                *report = localReport;
            }
            return false;
        }
    }

    // 转回 PointCloudFrame 并填充统计报告
    *output = fromPclCloud(cloud, input);
    localReport.outputPointCount = output->pointCount;
    localReport.message = QStringLiteral(
        "点云后处理完成: %1 -> %2 点")
                              .arg(localReport.inputPointCount)
                              .arg(localReport.outputPointCount);

    qInfo(LOG_POINT_CLOUD_PROC).noquote() << localReport.message;

    if (report != nullptr) {
        *report = localReport;
    }
    return output->isValid();
}

}  // namespace scan_tracking::mech_eye
