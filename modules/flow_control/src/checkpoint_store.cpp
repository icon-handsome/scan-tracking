#include "scan_tracking/flow_control/checkpoint_store.h"

#include <QtCore/QCoreApplication>
#include <QtCore/QDateTime>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtCore/QJsonArray>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QLoggingCategory>
#include <QtCore/QSaveFile>

namespace scan_tracking::flow_control {

namespace {
Q_LOGGING_CATEGORY(LOG_CHECKPOINT, "flow_control.checkpoint")

constexpr int kSupportedSchemaVersion = 1;

QString absoluteConfiguredPath(const QString& configuredPath)
{
    const QString trimmed = configuredPath.trimmed();
    if (trimmed.isEmpty()) {
        return QString();
    }
    const QFileInfo info(trimmed);
    if (info.isAbsolute()) {
        return QDir::cleanPath(trimmed);
    }
    const QString appDir = QCoreApplication::applicationDirPath();
    if (appDir.isEmpty()) {
        return QDir::cleanPath(trimmed);
    }
    return QDir::cleanPath(QDir(appDir).filePath(trimmed));
}

QJsonObject pathStateToJson(const CheckpointPathState& state)
{
    QJsonObject obj;
    obj.insert(QStringLiteral("pathId"), state.pathId);
    obj.insert(QStringLiteral("inspectionType"), state.inspectionType);
    obj.insert(QStringLiteral("totalPoints"), state.totalPoints);
    QJsonArray segs;
    for (int seg : state.scannedSegments) {
        segs.append(seg);
    }
    obj.insert(QStringLiteral("scannedSegments"), segs);
    obj.insert(QStringLiteral("inspectionStatus"), state.inspectionStatus);
    obj.insert(QStringLiteral("inspectionResultCode"), static_cast<int>(state.inspectionResultCode));
    obj.insert(QStringLiteral("algoStatus"), state.algoStatus);
    obj.insert(QStringLiteral("mergedInspectionPcd"), state.mergedInspectionPcd);
    obj.insert(QStringLiteral("sessionDir"), state.sessionDir);
    return obj;
}

CheckpointPathState pathStateFromJson(const QJsonObject& obj)
{
    CheckpointPathState state;
    state.pathId = obj.value(QStringLiteral("pathId")).toInt();
    state.inspectionType = obj.value(QStringLiteral("inspectionType")).toString();
    state.totalPoints = obj.value(QStringLiteral("totalPoints")).toInt();
    const QJsonArray segs = obj.value(QStringLiteral("scannedSegments")).toArray();
    state.scannedSegments.reserve(segs.size());
    for (const QJsonValue& v : segs) {
        const int seg = v.toInt();
        if (seg > 0) {
            state.scannedSegments.append(seg);
        }
    }
    state.inspectionStatus = obj.value(QStringLiteral("inspectionStatus")).toString();
    state.inspectionResultCode =
        static_cast<quint16>(obj.value(QStringLiteral("inspectionResultCode")).toInt());
    state.algoStatus = obj.value(QStringLiteral("algoStatus")).toString();
    state.mergedInspectionPcd = obj.value(QStringLiteral("mergedInspectionPcd")).toString();
    state.sessionDir = obj.value(QStringLiteral("sessionDir")).toString();
    return state;
}
}  // namespace

QString CheckpointStore::resolveCheckpointFilePath(const QString& configuredPath)
{
    return absoluteConfiguredPath(configuredPath);
}

QString CheckpointStore::buildScanPathsFingerprint(
    const QVector<int>& enabledPathIds,
    const QHash<int, int>& pathIdToTotalPoints,
    const QHash<int, QString>& pathIdToInspectionType)
{
    QStringList parts;
    parts.reserve(enabledPathIds.size());
    for (int pathId : enabledPathIds) {
        parts.append(QStringLiteral("%1:%2:%3")
                         .arg(pathId)
                         .arg(pathIdToTotalPoints.value(pathId, 0))
                         .arg(pathIdToInspectionType.value(pathId)));
    }
    return parts.join(QLatin1Char('|'));
}

bool CheckpointStore::exists(const QString& configuredPath)
{
    const QString path = resolveCheckpointFilePath(configuredPath);
    return !path.isEmpty() && QFileInfo::exists(path);
}

bool CheckpointStore::clear(const QString& configuredPath, QString* errorMessage)
{
    const QString path = resolveCheckpointFilePath(configuredPath);
    if (path.isEmpty()) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("检查点路径为空");
        }
        return false;
    }
    if (!QFileInfo::exists(path)) {
        return true;
    }
    if (!QFile::remove(path)) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("删除检查点失败：%1").arg(path);
        }
        return false;
    }
    qInfo(LOG_CHECKPOINT).noquote() << QStringLiteral("[Resume] 已清除检查点") << path;
    return true;
}

bool CheckpointStore::save(
    const WorkpieceCheckpoint& checkpoint,
    const QString& configuredPath,
    QString* errorMessage)
{
    const QString path = resolveCheckpointFilePath(configuredPath);
    if (path.isEmpty()) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("检查点路径为空");
        }
        return false;
    }

    const QFileInfo info(path);
    if (!QDir().mkpath(info.absolutePath())) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("创建检查点目录失败：%1").arg(info.absolutePath());
        }
        return false;
    }

    QJsonObject root;
    root.insert(QStringLiteral("schemaVersion"), checkpoint.schemaVersion);
    root.insert(QStringLiteral("stationProfile"), checkpoint.stationProfile);
    root.insert(QStringLiteral("scanPathsFingerprint"), checkpoint.scanPathsFingerprint);
    root.insert(
        QStringLiteral("updatedAt"),
        checkpoint.updatedAt.isEmpty()
            ? QDateTime::currentDateTime().toString(Qt::ISODateWithMs)
            : checkpoint.updatedAt);
    root.insert(QStringLiteral("currentPathId"), checkpoint.currentPathId);
    root.insert(QStringLiteral("sessionDir"), checkpoint.sessionDir);
    root.insert(QStringLiteral("poseStitchRunRoot"), checkpoint.poseStitchRunRoot);

    QJsonArray completed;
    for (int pathId : checkpoint.completedPathIds) {
        completed.append(pathId);
    }
    root.insert(QStringLiteral("completedPathIds"), completed);

    QJsonObject pathsObj;
    for (auto it = checkpoint.paths.constBegin(); it != checkpoint.paths.constEnd(); ++it) {
        pathsObj.insert(QString::number(it.key()), pathStateToJson(it.value()));
    }
    root.insert(QStringLiteral("paths"), pathsObj);

    QJsonObject asyncJobs;
    QJsonObject internalSurface;
    internalSurface.insert(QStringLiteral("generation"), static_cast<qint64>(checkpoint.internalSurfaceGeneration));
    asyncJobs.insert(QStringLiteral("internalSurface"), internalSurface);
    QJsonObject bevel;
    bevel.insert(QStringLiteral("generation"), static_cast<qint64>(checkpoint.bevelGeneration));
    asyncJobs.insert(QStringLiteral("bevel"), bevel);
    root.insert(QStringLiteral("asyncJobs"), asyncJobs);

    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("打开检查点写入失败：%1").arg(file.errorString());
        }
        return false;
    }
    const QByteArray bytes = QJsonDocument(root).toJson(QJsonDocument::Indented);
    if (file.write(bytes) != bytes.size()) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("写入检查点失败：%1").arg(file.errorString());
        }
        return false;
    }
    if (!file.commit()) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("提交检查点失败：%1").arg(file.errorString());
        }
        return false;
    }

    qInfo(LOG_CHECKPOINT).noquote()
        << QStringLiteral("[Resume] 检查点已保存") << path
        << QStringLiteral(" currentPathId=") << checkpoint.currentPathId
        << QStringLiteral(" completed=") << checkpoint.completedPathIds.size();
    return true;
}

bool CheckpointStore::load(
    const QString& configuredPath,
    WorkpieceCheckpoint* out,
    QString* errorMessage)
{
    if (out == nullptr) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("输出检查点指针为空");
        }
        return false;
    }

    const QString path = resolveCheckpointFilePath(configuredPath);
    if (path.isEmpty() || !QFileInfo::exists(path)) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("检查点不存在：%1").arg(path);
        }
        return false;
    }

    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("打开检查点失败：%1").arg(file.errorString());
        }
        return false;
    }

    QJsonParseError parseError;
    const QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !doc.isObject()) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("解析检查点 JSON 失败：%1").arg(parseError.errorString());
        }
        return false;
    }

    const QJsonObject root = doc.object();
    WorkpieceCheckpoint checkpoint;
    checkpoint.schemaVersion = root.value(QStringLiteral("schemaVersion")).toInt(0);
    if (checkpoint.schemaVersion <= 0 || checkpoint.schemaVersion > kSupportedSchemaVersion) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("不支持的检查点 schemaVersion=%1")
                                .arg(checkpoint.schemaVersion);
        }
        return false;
    }

    checkpoint.stationProfile = root.value(QStringLiteral("stationProfile")).toString();
    checkpoint.scanPathsFingerprint = root.value(QStringLiteral("scanPathsFingerprint")).toString();
    checkpoint.updatedAt = root.value(QStringLiteral("updatedAt")).toString();
    checkpoint.currentPathId = root.value(QStringLiteral("currentPathId")).toInt(1);
    checkpoint.sessionDir = root.value(QStringLiteral("sessionDir")).toString();
    checkpoint.poseStitchRunRoot = root.value(QStringLiteral("poseStitchRunRoot")).toString();

    const QJsonArray completed = root.value(QStringLiteral("completedPathIds")).toArray();
    for (const QJsonValue& v : completed) {
        const int pathId = v.toInt();
        if (pathId > 0) {
            checkpoint.completedPathIds.append(pathId);
        }
    }

    const QJsonObject pathsObj = root.value(QStringLiteral("paths")).toObject();
    for (auto it = pathsObj.begin(); it != pathsObj.end(); ++it) {
        if (!it.value().isObject()) {
            continue;
        }
        CheckpointPathState state = pathStateFromJson(it.value().toObject());
        if (state.pathId <= 0) {
            state.pathId = it.key().toInt();
        }
        if (state.pathId > 0) {
            checkpoint.paths.insert(state.pathId, state);
        }
    }

    const QJsonObject asyncJobs = root.value(QStringLiteral("asyncJobs")).toObject();
    checkpoint.internalSurfaceGeneration = static_cast<quint64>(
        asyncJobs.value(QStringLiteral("internalSurface")).toObject()
            .value(QStringLiteral("generation")).toDouble(0));
    checkpoint.bevelGeneration = static_cast<quint64>(
        asyncJobs.value(QStringLiteral("bevel")).toObject()
            .value(QStringLiteral("generation")).toDouble(0));

    if (!checkpoint.isValid()) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("检查点内容无效");
        }
        return false;
    }

    *out = checkpoint;
    qInfo(LOG_CHECKPOINT).noquote()
        << QStringLiteral("[Resume] 检查点已加载") << path
        << QStringLiteral(" currentPathId=") << checkpoint.currentPathId
        << QStringLiteral(" completed=") << checkpoint.completedPathIds.size();
    return true;
}

}  // namespace scan_tracking::flow_control
