#pragma once

/**
 * @file checkpoint_store.h
 * @brief IPC 断点续跑检查点：落盘 / 加载 / 清除（PLC 不断、仅 IPC 侧恢复）
 */

#include <QtCore/QHash>
#include <QtCore/QSet>
#include <QtCore/QString>
#include <QtCore/QVector>

namespace scan_tracking::flow_control {

struct CheckpointPathState {
    int pathId = 0;
    QString inspectionType;
    int totalPoints = 0;
    QVector<int> scannedSegments;
    /// none | pending | fake_ok | done | failed
    QString inspectionStatus;
    quint16 inspectionResultCode = 0;
    /// idle | running | done | failed（内表面/坡口后台）
    QString algoStatus;
    QString mergedInspectionPcd;
    QString sessionDir;
};

struct WorkpieceCheckpoint {
    int schemaVersion = 1;
    QString stationProfile;
    QString scanPathsFingerprint;
    QString updatedAt;
    int currentPathId = 1;
    QVector<int> completedPathIds;
    QHash<int, CheckpointPathState> paths;
    quint64 internalSurfaceGeneration = 0;
    quint64 bevelGeneration = 0;
    /// 段级落盘 session 根目录（output/session_*）
    QString sessionDir;
    /// PoseStitch/路径级融合点云 run 根目录（output/run_*）
    QString poseStitchRunRoot;

    bool isValid() const { return schemaVersion >= 1 && currentPathId > 0; }
};

class CheckpointStore {
public:
    static QString resolveCheckpointFilePath(const QString& configuredPath);

    static bool save(const WorkpieceCheckpoint& checkpoint, const QString& configuredPath, QString* errorMessage = nullptr);

    static bool load(const QString& configuredPath, WorkpieceCheckpoint* out, QString* errorMessage = nullptr);

    static bool clear(const QString& configuredPath, QString* errorMessage = nullptr);

    static bool exists(const QString& configuredPath);

    /// 由启用路径列表生成指纹，配置变更时拒绝续跑
    static QString buildScanPathsFingerprint(
        const QVector<int>& enabledPathIds,
        const QHash<int, int>& pathIdToTotalPoints,
        const QHash<int, QString>& pathIdToInspectionType);
};

}  // namespace scan_tracking::flow_control
