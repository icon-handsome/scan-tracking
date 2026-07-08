/**
 * @file tracking_service.h
 * @brief 跟踪服务头文件，提供检测和位姿校验功能
 *
 * 本模块负责协调点云采集、综合检测和位姿校验等核心业务逻辑，
 * 整合坡口测量（Po_Kou）算法，为状态机提供统一的检测接口。
 */

#pragma once

#include <array>
#include <functional>
#include <string>

#include <QtCore/QJsonObject>
#include <QtCore/QList>
#include <QtCore/QMetaType>
#include <QtCore/QString>
#include <QtCore/QtGlobal>

#include "scan_tracking/mech_eye/mech_eye_types.h"

namespace scan_tracking {
namespace tracking {

/// 坡口测量算法输出（对应 HMI event.inspection.finished 协议字段）
enum class InspectionAlgorithm {
    Bevel = 0,
    Hole = 1,
    Thickness = 2,
    InternalSurface = 3,
    CodeRead = 4,
    Defect = 5,
};

struct InspectionMeasurement {
    InspectionAlgorithm algorithm = InspectionAlgorithm::Bevel;
    float headAngleTol = 0.0f;   ///< head_angle_tol 坡口角（deg）
    float bluntHeightTol = 0.0f; ///< blunt_height_tol 钝边长度（mm）
    int bevelType = -1;          ///< bevel_type 坡口类型
    float icpFitness = 0.0f;     ///< icp_fitness ICP 拟合度
    int qualityCode = 10000;     ///< quality_code 0=合格
    float innerDiameterMm = 0.0f;
    float innerCircumferenceMm = 0.0f;
    float roundnessToleranceMm = 0.0f;
    float straightSideSlopeDeg = 0.0f;
    float straightSideHeightMm = 0.0f;
    float holeOpeningMm = 0.0f;
    float jointFitUpAngleDeg = 0.0f;
    float thicknessMm = 0.0f;
    float headDepthMm = 0.0f;
    float headVolumeM3 = 0.0f;
};

/// 将测量项写入 JSON payload（协议 snake_case 字段名，含与 headMetrics 重复的顶层测量值）
void appendInspectionMeasurementFields(QJsonObject& payload, const InspectionMeasurement& measurement);

/// 将算法元数据写入 payload（不含与 headMetrics 重复的测量值，供 HMI event.inspection.finished 使用）
void appendInspectionMetadataFields(QJsonObject& payload, const InspectionMeasurement& measurement);

/// 将显控 12 项指标写入 payload.headMetrics（2 项 Po_Kou 实测，其余 10 项暂为 0）
void appendHeadDisplayMetricsFields(QJsonObject& payload, const InspectionMeasurement& measurement);

/// 检测结果结构体，封装坡口综合检测的完整输出
struct InspectionResult {
    quint16 resultCode = 0;           ///< 结果码：1-成功，2-失败
    quint16 ngReasonWord0 = 0;        ///< NG 原因字 0
    quint16 ngReasonWord1 = 0;        ///< NG 原因字 1
    quint16 measureItemCount = 0;     ///< 测量项数量
    int sourcePointCount = 0;         ///< 输入点云点数
    InspectionMeasurement measurement; ///< 算法测量项（HMI 结构化上报）
    QString message;                  ///< 结果描述信息
};

/// 位姿校验结果结构体，封装LB位姿检测的输出
struct PoseCheckResult {
    bool invoked = false;             ///< 是否已调用位姿检测算法
    bool success = false;             ///< 位姿检测是否成功
    quint16 resultCode = 7;           ///< 结果码：1-成功，其他-失败
    int inputPointCount = 0;          ///< 输入点云点数
    double poseDeviationMm = 0.0;     ///< 位姿偏差（mm）
    std::array<double, 16> rt = {     ///< 4x4 位姿变换矩阵（RT）
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    };
    QString message;                  ///< 结果描述信息

    /// 判断是否有有效的位姿矩阵
    // @return 成功且结果码为1时返回true
    bool hasPoseMatrix() const { return success && resultCode == 1; }
};

/// 坡口检测结果就绪回调（供 HMI 推送等场景注入，避免 tracking 反向依赖 hmi_server）
using InspectionResultNotifier = std::function<void(const InspectionResult&)>;

/// 跟踪服务类，负责管理检测和位姿校验的核心业务逻辑
class TrackingService {
public:
    ~TrackingService();

    /// 获取服务状态文本
    // @return 服务状态描述字符串
    std::string statusText() const;

    /// 注册综合检测结果回调：inspectPointCloud 返回前必定触发
    void setInspectionResultNotifier(InspectionResultNotifier notifier);

    /// 安全释放回调（swap 清空，避免退出阶段 std::function 析构访问失效目标）
    void clearInspectionResultNotifier();

    /// 执行单点云综合检测（按路径 inspectionType 分流坡口或 Hole）
    InspectionResult inspectPointCloud(
        const scan_tracking::mech_eye::PointCloudFrame& pointCloud,
        int sourcePointCount,
        int inspectionPathId = 0,
        bool notifyListener = true) const;

    /// 内表面：从磁盘点云文件执行测量（不经过 PointCloudFrame 内存路径）
    InspectionResult inspectInternalSurfaceFromScanFile(
        const QString& scanCloudPath,
        int inspectionPathId = 0,
        bool notifyListener = true,
        bool useOfflineReplayAlgorithmConfig = false) const;

    /// 内表面正式检测：多扫描分段按序合并后落盘 PCD（与 demo 单文件输入一致）
    InspectionResult inspectInternalSurfaceFromSegmentFrames(
        const QList<scan_tracking::mech_eye::PointCloudFrame>& segmentClouds,
        int sourcePointCount,
        int inspectionPathId = 0,
        bool notifyListener = true) const;

    /// 坡口离线/文件路径测量：PCL 在 Po_Kou 模块内加载，避免跨模块堆释放崩溃
    InspectionResult inspectBevelPointCloudFile(
        const QString& cloudPath,
        int inspectionPathId = 0,
        bool notifyListener = true) const;

    /// 坡口离线目录回放：逐文件测量后取均值（与 Po_Kou demo 一致，不合并大点云）
    InspectionResult inspectBevelPointCloudFilesAveraged(
        const QStringList& cloudFiles,
        int inspectionPathId = 0,
        bool notifyListener = true) const;

    /// 坡口正式检测：逐扫描分段测量后取均值（不合并大点云）
    InspectionResult inspectBevelPointCloudFramesAveraged(
        const QList<scan_tracking::mech_eye::PointCloudFrame>& segmentClouds,
        int sourcePointCount,
        int inspectionPathId = 0,
        bool notifyListener = true) const;

    /// Hole 正式检测：逐扫描分段 preprocess 后合并（与 demo mergeFrames 一致）
    InspectionResult inspectHolePointCloudFrames(
        const QList<scan_tracking::mech_eye::PointCloudFrame>& segmentClouds,
        int sourcePointCount,
        int inspectionPathId = 0,
        bool notifyListener = true) const;

    /// Hole 正式检测：从落盘分段 PCD/PLY 逐文件加载（内存峰值更低）
    InspectionResult inspectHolePointCloudFromSegmentPcdFiles(
        const QStringList& segmentPcdPaths,
        int sourcePointCount,
        int inspectionPathId = 0,
        bool notifyListener = true) const;

    /// 执行厚度综合检测（inner/outer 双点云，按路径 inspectionType=thickness）
    InspectionResult inspectThicknessPointClouds(
        const scan_tracking::mech_eye::PointCloudFrame& innerCloud,
        const scan_tracking::mech_eye::PointCloudFrame& outerCloud,
        int innerPointCount,
        int outerPointCount,
        int inspectionPathId = 0,
        bool notifyListener = true) const;

    /// 编号识别（inspectionType=code_read，不依赖点云合并）
    InspectionResult inspectCodeRead(int inspectionPathId = 0, bool notifyListener = true) const;

    /// 表面缺陷识别（inspectionType=defect，不依赖点云合并）
    InspectionResult inspectSurfaceDefect(int inspectionPathId = 0, bool notifyListener = true) const;

    /// 执行位姿校验（LB位姿检测）
    // @return 位姿校验结果结构体
    PoseCheckResult checkPose() const;

private:
    /// 统一出口：触发回调后再把结果返回给状态机
    InspectionResult deliverInspectionResult(InspectionResult result, bool notifyListener) const;

    InspectionResultNotifier m_inspectionResultNotifier;
};

}  // namespace tracking
}  // namespace scan_tracking

Q_DECLARE_METATYPE(scan_tracking::tracking::InspectionMeasurement)
