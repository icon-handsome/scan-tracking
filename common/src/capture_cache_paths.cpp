#include "scan_tracking/common/capture_cache_paths.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>

namespace scan_tracking::common {

QString defaultCaptureCacheRoot()
{
    return QCoreApplication::applicationDirPath() + QStringLiteral("/ScanTracking_CaptureCache");
}

QString resolveCaptureCacheRoot(const QString& configuredRoot)
{
    const QString trimmed = configuredRoot.trimmed();
    if (trimmed.isEmpty()) {
        return defaultCaptureCacheRoot();
    }
    return QDir(trimmed).absolutePath();
}

QString ensureDirectoryExists(const QString& directoryPath)
{
    if (directoryPath.trimmed().isEmpty()) {
        return QString();
    }

    QDir dir;
    if (!dir.mkpath(directoryPath)) {
        return QString();
    }
    return QDir(directoryPath).absolutePath();
}

QString captureCacheMech3DDir(const QString& root)
{
    const QString resolved = ensureDirectoryExists(resolveCaptureCacheRoot(root));
    if (resolved.isEmpty()) {
        return QString();
    }
    return ensureDirectoryExists(QDir(resolved).absoluteFilePath(QStringLiteral("mech_3d")));
}

QString captureCacheMech2DDir(const QString& root)
{
    const QString resolved = ensureDirectoryExists(resolveCaptureCacheRoot(root));
    if (resolved.isEmpty()) {
        return QString();
    }
    return ensureDirectoryExists(QDir(resolved).absoluteFilePath(QStringLiteral("mech_2d")));
}

QString captureCacheHikMonoDir(const QString& root)
{
    const QString resolved = ensureDirectoryExists(resolveCaptureCacheRoot(root));
    if (resolved.isEmpty()) {
        return QString();
    }
    return ensureDirectoryExists(QDir(resolved).absoluteFilePath(QStringLiteral("hik_mono")));
}

QString captureCacheHikMonoCameraDir(const QString& root, const QString& cameraTag)
{
    const QString hikRoot = captureCacheHikMonoDir(root);
    if (hikRoot.isEmpty()) {
        return QString();
    }

    QString subDir;
    if (cameraTag.compare(QStringLiteral("hikA"), Qt::CaseInsensitive) == 0) {
        subDir = QStringLiteral("camera_a");
    } else if (cameraTag.compare(QStringLiteral("hikB"), Qt::CaseInsensitive) == 0) {
        subDir = QStringLiteral("camera_b");
    } else {
        subDir = cameraTag.trimmed();
    }

    if (subDir.isEmpty()) {
        return QString();
    }

    return ensureDirectoryExists(QDir(hikRoot).absoluteFilePath(subDir));
}

QString buildCaptureTimestamp()
{
    return QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss_zzz"));
}

}  // namespace scan_tracking::common
