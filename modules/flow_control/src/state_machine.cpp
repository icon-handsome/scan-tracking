#include "scan_tracking/flow_control/state_machine.h"

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/common/logger.h"
#include "scan_tracking/flow_control/station_trigger_policy.h"
#include "scan_tracking/flow_control/task_handler_registry.h"
#include "scan_tracking/mech_eye/mech_eye_service.h"
#include "scan_tracking/mech_eye/point_cloud_io.h"
#include "scan_tracking/mech_eye/point_cloud_processor.h"
#include "scan_tracking/vision/hik_mono_io.h"
#include "scan_tracking/vision/vision_pipeline_service.h"

#include <QtCore/QEventLoop>
#include <QtCore/QPointer>
#include <QtCore/QStringList>
#include <QtCore/QLoggingCategory>
#include <QtCore/QElapsedTimer>
#include <QtCore/QThread>
#include <QtCore/QCoreApplication>
#include <QtCore/QDateTime>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtCore/QCryptographicHash>
#include <QtCore/QTextStream>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <algorithm>
#include <exception>
#include <filesystem>
#include <memory>
#include <mutex>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>
#include <qdebug.h>
namespace scan_tracking::flow_control {

// 定义流程控制模块的日志分类
Q_LOGGING_CATEGORY(LOG_FLOW, "flow_control")

namespace {

/**
 * @brief 设备在线状态字位掩码
 * 
 * 用于标识 IPC 设备在线状态的各个比特位，包括：
 * - Bit 0: 系统就绪
 * - Bit 1: 通信正常
 * - Bit 2: 传感器在线
 * - Bit 4: 相机连接
 * - Bit 5: 跟踪服务
 * - Bit 6: Modbus 连接
 */
constexpr quint16 kDeviceOnlineWord0 =
    (1u << 0) |
    (1u << 1) |
    (1u << 2) |
    (1u << 4) |
    (1u << 5) |
    (1u << 6);

/// 自检缓存桶 ID（不在 scan_paths 中；仅 IPC 内部区分自检两点缓存）
constexpr int kSelfCheckCacheBucketId = 9001;

/// 最大扫描分段索引（从1开始计数；须 ≥ scan_paths 中单路径最大 totalPoints）
constexpr int kMaxScanSegmentIndex = 200;

/// 默认扫描分段采集超时时间（毫秒）
constexpr int kDefaultScanSegmentCaptureTimeoutMs = 30000;

/// 允许的最大连续 Modbus 通信失败次数，超过此值将进入故障状态
constexpr int kMaxConsecutiveModbusFailures = 3;
constexpr int kPollLogEveryN = 20;
constexpr int kBackgroundRefinementJoinTimeoutMs = 300000;

/// PLC `Res_Inspection`：1=OK，6=超时 NG（与 Modbus 协议一致）
constexpr quint16 kInspectionResOk = 1;
constexpr quint16 kInspectionResTimeoutNg = 6;

bool isAlgorithmBypassEnabled()
{
    const auto* configMgr = scan_tracking::common::ConfigManager::instance();
    return configMgr != nullptr && configMgr->flowControlConfig().algorithmBypassEnabled;
}

int countHikImagesInBundle(const scan_tracking::vision::MultiCameraCaptureBundle& bundle)
{
    int imageCount = 0;
    if (bundle.hikCameraAResult.success() && bundle.hikCameraAResult.frame.isValid()) {
        ++imageCount;
    }
    if (bundle.hikCameraBResult.success() && bundle.hikCameraBResult.frame.isValid()) {
        ++imageCount;
    }
    return imageCount;
}

scan_tracking::mech_eye::PointCloudFrame clonePointCloudFrame(
    const scan_tracking::mech_eye::PointCloudFrame& src)
{
    scan_tracking::mech_eye::PointCloudFrame dst = src;
    if (src.pointsXYZ) {
        dst.pointsXYZ = std::make_shared<std::vector<float>>(*src.pointsXYZ);
    }
    if (src.normalsXYZ) {
        dst.normalsXYZ = std::make_shared<std::vector<float>>(*src.normalsXYZ);
    }
    return dst;
}

scan_tracking::mech_eye::CaptureResult cloneCaptureResult(
    const scan_tracking::mech_eye::CaptureResult& src)
{
    scan_tracking::mech_eye::CaptureResult dst = src;
    dst.pointCloud = clonePointCloudFrame(src.pointCloud);
    if (src.texture2D.isValid()) {
        dst.texture2D.pixels =
            std::make_shared<std::vector<uint8_t>>(*src.texture2D.pixels);
    }
    return dst;
}

scan_tracking::vision::HikMonoFrame cloneHikMonoFrame(
    const scan_tracking::vision::HikMonoFrame& src)
{
    scan_tracking::vision::HikMonoFrame dst = src;
    if (src.pixels) {
        dst.pixels = std::make_shared<std::vector<std::uint8_t>>(*src.pixels);
    }
    return dst;
}

scan_tracking::vision::MultiCameraCaptureBundle cloneCaptureBundleForCache(
    const scan_tracking::vision::MultiCameraCaptureBundle& src,
    const scan_tracking::mech_eye::PointCloudFrame& sharedPointCloud)
{
    scan_tracking::vision::MultiCameraCaptureBundle dst = src;
    dst.mechEyeResult = src.mechEyeResult;
    dst.mechEyeResult.pointCloud = sharedPointCloud;
    if (src.mechEyeResult.texture2D.isValid()) {
        dst.mechEyeResult.texture2D.pixels =
            std::make_shared<std::vector<uint8_t>>(*src.mechEyeResult.texture2D.pixels);
    }
    dst.hikCameraAResult.frame = cloneHikMonoFrame(src.hikCameraAResult.frame);
    dst.hikCameraBResult.frame = cloneHikMonoFrame(src.hikCameraBResult.frame);
    return dst;
}

void assignSharedPointCloudToSegmentEntry(
    QMap<int, QMap<int, scan_tracking::mech_eye::CaptureResult>>& pathSegmentCaptureResults,
    QMap<int, QMap<int, scan_tracking::vision::MultiCameraCaptureBundle>>& pathSegmentCaptureBundles,
    int pathId,
    int segmentIndex,
    const scan_tracking::mech_eye::PointCloudFrame& sourceCloud)
{
    scan_tracking::mech_eye::PointCloudFrame sharedCloud = clonePointCloudFrame(sourceCloud);

    auto& resultRef = pathSegmentCaptureResults[pathId][segmentIndex];
    scan_tracking::mech_eye::releasePointCloudFrameBuffers(&resultRef.pointCloud);
    resultRef.pointCloud = sharedCloud;

    if (pathSegmentCaptureBundles.contains(pathId) &&
        pathSegmentCaptureBundles[pathId].contains(segmentIndex)) {
        auto& bundleRef = pathSegmentCaptureBundles[pathId][segmentIndex];
        scan_tracking::mech_eye::releasePointCloudFrameBuffers(&bundleRef.mechEyeResult.pointCloud);
        bundleRef.mechEyeResult.pointCloud = sharedCloud;
    }
}

std::array<float, 16> identityMatrix4x4()
{
    std::array<float, 16> m{};
    m[0] = m[5] = m[10] = m[15] = 1.0f;
    return m;
}

/** 行优先 4×4：out = left × right */
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

std::array<float, 16> poseMatrixToArray(const scan_tracking::vision::PoseMatrix4x4& pose)
{
    return pose.isValid() ? pose.values : identityMatrix4x4();
}

QString formatRowMajorMatrixBlock(const QString& title, const std::array<float, 16>& matrix)
{
    QString text = title + QStringLiteral(":\n");
    for (int row = 0; row < 4; ++row) {
        text += QStringLiteral("  [");
        for (int col = 0; col < 4; ++col) {
            text += QString::number(
                static_cast<double>(matrix[static_cast<std::size_t>(row * 4 + col)]),
                'g',
                12);
            if (col + 1 < 4) {
                text += QStringLiteral(", ");
            }
        }
        text += QStringLiteral("]\n");
    }
    return text;
}

QString poseStitchOutputTimestamp()
{
    return QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"));
}

QString absolutePathFromStdPath(const std::filesystem::path& path)
{
    std::error_code ec;
    const auto absolute = std::filesystem::absolute(path, ec);
    if (ec) {
        return {};
    }
    return QString::fromStdWString(absolute.wstring());
}

bool ensureDirectoryTreeExists(const QString& directoryPath)
{
    if (directoryPath.trimmed().isEmpty()) {
        return false;
    }

    std::error_code ec;
    const auto fsPath = std::filesystem::path(directoryPath.toStdWString());
    std::filesystem::create_directories(fsPath, ec);
    if (ec) {
        return false;
    }
    return std::filesystem::is_directory(fsPath);
}

QString createPoseStitchRunRootDirectory(const QString& timestamp)
{
    const QString appDir = QCoreApplication::applicationDirPath();
    if (appDir.isEmpty()) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("[PoseStitchOutput] applicationDirPath 为空，无法创建输出目录");
        return {};
    }

    const QString outputBase = appDir + QStringLiteral("/output");
    if (!ensureDirectoryTreeExists(outputBase)) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("[PoseStitchOutput] 创建 output 根目录失败：") << outputBase;
        return {};
    }

    const QString runRoot = outputBase + QStringLiteral("/run_") + timestamp;
    const QString matrixDir = runRoot + QStringLiteral("/matrix");
    const QString pointcloudDir = runRoot + QStringLiteral("/pointcloud");

    if (!ensureDirectoryTreeExists(matrixDir)) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("[PoseStitchOutput] 创建 matrix 目录失败：") << matrixDir;
        return {};
    }
    if (!ensureDirectoryTreeExists(pointcloudDir)) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("[PoseStitchOutput] 创建 pointcloud 目录失败：") << pointcloudDir;
        return {};
    }
    return absolutePathFromStdPath(std::filesystem::path(runRoot.toStdWString()));
}

QString poseStitchMatrixOutputDirectory(const QString& runRoot)
{
    return QDir(runRoot).filePath(QStringLiteral("matrix"));
}

QString poseStitchPointCloudOutputDirectory(const QString& runRoot)
{
    return QDir(runRoot).filePath(QStringLiteral("pointcloud"));
}

QString segmentCaptureExportGroupDirectory(const QString& sessionRoot, int pathId, int segmentIndex)
{
    return QDir(sessionRoot).filePath(
        QStringLiteral("path%1_seg%2").arg(pathId).arg(segmentIndex, 2, 10, QChar('0')));
}

bool writeTextFileIfPossible(const QString& absolutePath, const QString& text)
{
    QFile file(absolutePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        return false;
    }
    QTextStream stream(&file);
    stream.setCodec("UTF-8");
    stream << text;
    return true;
}

struct SegmentCaptureCxpImageMeta {
    bool attempted = false;
    bool saved = false;
    QString fileName;
    QString cameraKey;
    QString logicalName;
    quint64 captureRequestId = 0;
    quint64 frameId = 0;
    qint64 captureTimestampMs = 0;
    int width = 0;
    int height = 0;
    int stride = 0;
    qint64 pixelByteCount = 0;
    QString pixelMd5;
    QString firstPixelHex;
    QString centerPixelHex;
    QString lastPixelHex;
    QString savedAt;
    qint64 savedFileBytes = 0;
    QString savedFileMd5;
};

struct SegmentCaptureCxpExportMeta {
    QString exportGroupId;
    QString sessionRoot;
    QString captureFileTimestamp;
    SegmentCaptureCxpImageMeta left;
    SegmentCaptureCxpImageMeta right;
};

QString md5HexFromBytes(const std::uint8_t* data, std::size_t size)
{
    if (data == nullptr || size == 0) {
        return QStringLiteral("(empty)");
    }
    const QByteArray bytes(reinterpret_cast<const char*>(data), static_cast<int>(size));
    return QString::fromLatin1(QCryptographicHash::hash(bytes, QCryptographicHash::Md5).toHex());
}

QString md5HexFromFile(const QString& absolutePath)
{
    QFile file(absolutePath);
    if (!file.open(QIODevice::ReadOnly)) {
        return QStringLiteral("(unreadable)");
    }
    QCryptographicHash hash(QCryptographicHash::Md5);
    if (!hash.addData(&file)) {
        return QStringLiteral("(hash-failed)");
    }
    return QString::fromLatin1(hash.result().toHex());
}

QString mono8SampleHex(const scan_tracking::vision::HikMonoFrame& frame, int index)
{
    if (!frame.isValid() || frame.pixels == nullptr) {
        return QStringLiteral("na");
    }
    const auto& pixels = *frame.pixels;
    if (index < 0 || static_cast<std::size_t>(index) >= pixels.size()) {
        return QStringLiteral("na");
    }
    return QStringLiteral("0x%1").arg(pixels[static_cast<std::size_t>(index)], 2, 16, QChar('0'));
}

QString sanitizeFileNameComponent(const QString& value)
{
    QString sanitized;
    sanitized.reserve(value.size());
    for (const QChar ch : value.trimmed()) {
        if (ch.isLetterOrNumber() || ch == QLatin1Char('_') || ch == QLatin1Char('-')) {
            sanitized.append(ch);
        } else if (ch.isSpace() || ch == QLatin1Char('.')) {
            sanitized.append(QLatin1Char('_'));
        }
    }
    return sanitized.isEmpty() ? QStringLiteral("unknown") : sanitized;
}

QString resolveCxpExportCameraTag(
    const scan_tracking::vision::HikPoseCaptureResult& result,
    const QString& fallbackSide)
{
    if (!result.logicalName.trimmed().isEmpty()) {
        return sanitizeFileNameComponent(result.logicalName);
    }
    if (!result.cameraKey.trimmed().isEmpty()) {
        return sanitizeFileNameComponent(result.cameraKey);
    }
    if (result.frame.isValid() && !result.frame.sourceCameraKey.trimmed().isEmpty()) {
        return sanitizeFileNameComponent(result.frame.sourceCameraKey);
    }
    return fallbackSide;
}

QString formatCxpCaptureTimestampTag(qint64 timestampMs)
{
    const qint64 effectiveTs =
        timestampMs > 0 ? timestampMs : QDateTime::currentMSecsSinceEpoch();
    return QDateTime::fromMSecsSinceEpoch(effectiveTs)
        .toString(QStringLiteral("yyyyMMdd_HHmmss_zzz"));
}

QString resolveSegmentCaptureFlatOutputRoot()
{
    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    const QString configuredRoot = configManager != nullptr
        ? configManager->segmentCaptureExportConfig().outputRoot
        : QStringLiteral("output");
    const QString outputRoot = configuredRoot.trimmed().isEmpty()
        ? QStringLiteral("output")
        : configuredRoot;

    const QString appDir = QCoreApplication::applicationDirPath();
    return QFileInfo(outputRoot).isAbsolute() ? outputRoot : QDir(appDir).filePath(outputRoot);
}

QString buildSegmentCaptureExportGroupId(int pathId, int segmentIndex, quint64 requestId)
{
    return QStringLiteral("path%1_seg%2_req%3")
        .arg(pathId)
        .arg(segmentIndex, 2, 10, QChar('0'))
        .arg(requestId);
}

SegmentCaptureCxpImageMeta buildCxpImageMetaFromCapture(
    const scan_tracking::vision::HikPoseCaptureResult& result,
    const QString& fileName,
    bool saved,
    const QString& savedAt,
    qint64 savedFileBytes,
    const QString& savedFileMd5)
{
    SegmentCaptureCxpImageMeta meta;
    meta.attempted = true;
    meta.saved = saved;
    meta.fileName = fileName;
    meta.cameraKey = result.cameraKey;
    meta.logicalName = result.logicalName;
    meta.captureRequestId = result.requestId;
    if (result.frame.isValid() && result.frame.pixels != nullptr) {
        meta.frameId = result.frame.frameId;
        meta.captureTimestampMs = result.frame.timestampMs;
        meta.width = result.frame.width;
        meta.height = result.frame.height;
        meta.stride = result.frame.stride;
        meta.pixelByteCount = static_cast<qint64>(result.frame.pixels->size());
        meta.pixelMd5 = md5HexFromBytes(result.frame.pixels->data(), result.frame.pixels->size());
        const int centerIndex = (result.frame.height / 2) * result.frame.stride + (result.frame.width / 2);
        meta.firstPixelHex = mono8SampleHex(result.frame, 0);
        meta.centerPixelHex = mono8SampleHex(result.frame, centerIndex);
        meta.lastPixelHex =
            mono8SampleHex(result.frame, static_cast<int>(result.frame.pixels->size()) - 1);
    }
    meta.savedAt = savedAt;
    meta.savedFileBytes = savedFileBytes;
    meta.savedFileMd5 = savedFileMd5;
    return meta;
}

void appendCxpImageMetaLines(QString& text, const QString& prefix, const SegmentCaptureCxpImageMeta& meta)
{
    text += prefix + QStringLiteral(".attempted=") + (meta.attempted ? QStringLiteral("true") : QStringLiteral("false")) + QStringLiteral("\n");
    text += prefix + QStringLiteral(".saved=") + (meta.saved ? QStringLiteral("true") : QStringLiteral("false")) + QStringLiteral("\n");
    text += prefix + QStringLiteral(".fileName=") + meta.fileName + QStringLiteral("\n");
    text += prefix + QStringLiteral(".cameraKey=") + meta.cameraKey + QStringLiteral("\n");
    text += prefix + QStringLiteral(".logicalName=") + meta.logicalName + QStringLiteral("\n");
    text += prefix + QStringLiteral(".captureRequestId=") + QString::number(meta.captureRequestId) + QStringLiteral("\n");
    text += prefix + QStringLiteral(".frameId=") + QString::number(meta.frameId) + QStringLiteral("\n");
    text += prefix + QStringLiteral(".captureTimestampMs=") + QString::number(meta.captureTimestampMs) + QStringLiteral("\n");
    text += prefix + QStringLiteral(".width=") + QString::number(meta.width) + QStringLiteral("\n");
    text += prefix + QStringLiteral(".height=") + QString::number(meta.height) + QStringLiteral("\n");
    text += prefix + QStringLiteral(".stride=") + QString::number(meta.stride) + QStringLiteral("\n");
    text += prefix + QStringLiteral(".pixelByteCount=") + QString::number(meta.pixelByteCount) + QStringLiteral("\n");
    text += prefix + QStringLiteral(".pixelMd5=") + meta.pixelMd5 + QStringLiteral("\n");
    text += prefix + QStringLiteral(".firstPixel=") + meta.firstPixelHex + QStringLiteral("\n");
    text += prefix + QStringLiteral(".centerPixel=") + meta.centerPixelHex + QStringLiteral("\n");
    text += prefix + QStringLiteral(".lastPixel=") + meta.lastPixelHex + QStringLiteral("\n");
    text += prefix + QStringLiteral(".savedAt=") + meta.savedAt + QStringLiteral("\n");
    text += prefix + QStringLiteral(".savedFileBytes=") + QString::number(meta.savedFileBytes) + QStringLiteral("\n");
    text += prefix + QStringLiteral(".savedFileMd5=") + meta.savedFileMd5 + QStringLiteral("\n");
}

void logSegmentCaptureCxpImageSaved(
    const QString& sideLabel,
    const QString& exportGroupId,
    const QString& absolutePath,
    const SegmentCaptureCxpImageMeta& meta)
{
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("[SegmentCaptureExport] CXP") << sideLabel << QStringLiteral("存图")
        << QStringLiteral("group=") << exportGroupId
        << QStringLiteral("path=") << absolutePath
        << QStringLiteral("frameId=") << meta.frameId
        << QStringLiteral("captureReqId=") << meta.captureRequestId
        << QStringLiteral("captureTsMs=") << meta.captureTimestampMs
        << QStringLiteral("savedAt=") << meta.savedAt
        << QStringLiteral("pixelMd5=") << meta.pixelMd5
        << QStringLiteral("savedFileBytes=") << meta.savedFileBytes
        << QStringLiteral("savedFileMd5=") << meta.savedFileMd5
        << QStringLiteral("first/center/last=") << meta.firstPixelHex << meta.centerPixelHex
        << meta.lastPixelHex;
}

QString resolveCxpCaptureFileTimestamp(
    const scan_tracking::vision::MultiCameraCaptureBundle& bundle)
{
    const qint64 leftTs = bundle.hikCameraAResult.frame.timestampMs;
    const qint64 rightTs = bundle.hikCameraBResult.frame.timestampMs;
    qint64 captureTs = leftTs > 0 ? leftTs : rightTs;
    if (captureTs <= 0) {
        captureTs = QDateTime::currentMSecsSinceEpoch();
    }
    return formatCxpCaptureTimestampTag(captureTs);
}

QString buildCxpCaptureFileName(const QString& cameraTag, const QString& timestampTag)
{
    return QStringLiteral("%1_%2.bmp").arg(cameraTag, timestampTag);
}

SegmentCaptureCxpExportMeta saveSegmentCxp2dImagesToFlatOutput(
    const scan_tracking::vision::MultiCameraCaptureBundle& bundle)
{
    SegmentCaptureCxpExportMeta cxpExportMeta;
    cxpExportMeta.captureFileTimestamp = resolveCxpCaptureFileTimestamp(bundle);

    const QString outputRoot = resolveSegmentCaptureFlatOutputRoot();
    if (!ensureDirectoryTreeExists(outputRoot)) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("[SegmentCaptureExport] 创建 2D 输出目录失败：") << outputRoot;
        return cxpExportMeta;
    }

    if (bundle.hikCameraAResult.success() && bundle.hikCameraAResult.frame.isValid()) {
        const QString leftTag =
            resolveCxpExportCameraTag(bundle.hikCameraAResult, QStringLiteral("left"));
        const QString leftTimestamp = formatCxpCaptureTimestampTag(
            bundle.hikCameraAResult.frame.timestampMs);
        const QString leftFileName = buildCxpCaptureFileName(leftTag, leftTimestamp);
        const QString leftPath = QDir(outputRoot).filePath(leftFileName);
        const QString savedAt = QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
        const bool cxpLeftSaved = scan_tracking::vision::saveHikMonoFrameToBmp(
            bundle.hikCameraAResult.frame, leftPath);
        const qint64 savedFileBytes = cxpLeftSaved ? QFileInfo(leftPath).size() : 0;
        const QString savedFileMd5 = cxpLeftSaved ? md5HexFromFile(leftPath) : QStringLiteral("(not-saved)");
        cxpExportMeta.left = buildCxpImageMetaFromCapture(
            bundle.hikCameraAResult, leftFileName, cxpLeftSaved, savedAt, savedFileBytes, savedFileMd5);
        if (cxpLeftSaved) {
            logSegmentCaptureCxpImageSaved(
                QStringLiteral("左"), leftFileName, leftPath, cxpExportMeta.left);
        } else {
            qWarning(LOG_FLOW).noquote()
                << QStringLiteral("[SegmentCaptureExport] 左目 BMP 写入失败：") << leftPath
                << QStringLiteral("frameId=") << cxpExportMeta.left.frameId
                << QStringLiteral("pixelMd5=") << cxpExportMeta.left.pixelMd5;
        }
    }

    if (bundle.hikCameraBResult.success() && bundle.hikCameraBResult.frame.isValid()) {
        const QString rightTag =
            resolveCxpExportCameraTag(bundle.hikCameraBResult, QStringLiteral("right"));
        const QString rightTimestamp = formatCxpCaptureTimestampTag(
            bundle.hikCameraBResult.frame.timestampMs);
        const QString rightFileName = buildCxpCaptureFileName(rightTag, rightTimestamp);
        const QString rightPath = QDir(outputRoot).filePath(rightFileName);
        const QString savedAt = QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
        const bool cxpRightSaved = scan_tracking::vision::saveHikMonoFrameToBmp(
            bundle.hikCameraBResult.frame, rightPath);
        const qint64 savedFileBytes = cxpRightSaved ? QFileInfo(rightPath).size() : 0;
        const QString savedFileMd5 = cxpRightSaved ? md5HexFromFile(rightPath) : QStringLiteral("(not-saved)");
        cxpExportMeta.right = buildCxpImageMetaFromCapture(
            bundle.hikCameraBResult, rightFileName, cxpRightSaved, savedAt, savedFileBytes, savedFileMd5);
        if (cxpRightSaved) {
            logSegmentCaptureCxpImageSaved(
                QStringLiteral("右"), rightFileName, rightPath, cxpExportMeta.right);
        } else {
            qWarning(LOG_FLOW).noquote()
                << QStringLiteral("[SegmentCaptureExport] 右目 BMP 写入失败：") << rightPath
                << QStringLiteral("frameId=") << cxpExportMeta.right.frameId
                << QStringLiteral("pixelMd5=") << cxpExportMeta.right.pixelMd5;
        }
    }

    return cxpExportMeta;
}

SegmentCaptureCxpExportMeta buildCxpExportMetaFromFlatOutput(
    const scan_tracking::vision::MultiCameraCaptureBundle& bundle)
{
    SegmentCaptureCxpExportMeta cxpExportMeta;
    cxpExportMeta.captureFileTimestamp = resolveCxpCaptureFileTimestamp(bundle);
    const QString outputRoot = resolveSegmentCaptureFlatOutputRoot();

    if (bundle.hikCameraAResult.success() && bundle.hikCameraAResult.frame.isValid()) {
        const QString leftTag =
            resolveCxpExportCameraTag(bundle.hikCameraAResult, QStringLiteral("left"));
        const QString leftTimestamp = formatCxpCaptureTimestampTag(
            bundle.hikCameraAResult.frame.timestampMs);
        const QString leftFileName = buildCxpCaptureFileName(leftTag, leftTimestamp);
        const QString leftPath = QDir(outputRoot).filePath(leftFileName);
        const bool saved = QFileInfo::exists(leftPath);
        const QString savedAt =
            saved
                ? QFileInfo(leftPath).lastModified().toString(Qt::ISODateWithMs)
                : QString();
        const qint64 savedFileBytes = saved ? QFileInfo(leftPath).size() : 0;
        const QString savedFileMd5 = saved ? md5HexFromFile(leftPath) : QStringLiteral("(not-saved)");
        cxpExportMeta.left = buildCxpImageMetaFromCapture(
            bundle.hikCameraAResult, leftFileName, saved, savedAt, savedFileBytes, savedFileMd5);
    }

    if (bundle.hikCameraBResult.success() && bundle.hikCameraBResult.frame.isValid()) {
        const QString rightTag =
            resolveCxpExportCameraTag(bundle.hikCameraBResult, QStringLiteral("right"));
        const QString rightTimestamp = formatCxpCaptureTimestampTag(
            bundle.hikCameraBResult.frame.timestampMs);
        const QString rightFileName = buildCxpCaptureFileName(rightTag, rightTimestamp);
        const QString rightPath = QDir(outputRoot).filePath(rightFileName);
        const bool saved = QFileInfo::exists(rightPath);
        const QString savedAt =
            saved
                ? QFileInfo(rightPath).lastModified().toString(Qt::ISODateWithMs)
                : QString();
        const qint64 savedFileBytes = saved ? QFileInfo(rightPath).size() : 0;
        const QString savedFileMd5 = saved ? md5HexFromFile(rightPath) : QStringLiteral("(not-saved)");
        cxpExportMeta.right = buildCxpImageMetaFromCapture(
            bundle.hikCameraBResult, rightFileName, saved, savedAt, savedFileBytes, savedFileMd5);
    }

    return cxpExportMeta;
}

QString buildSegmentCaptureExportMetaText(
    int pathId,
    int segmentIndex,
    const scan_tracking::vision::MultiCameraCaptureBundle& bundle,
    const scan_tracking::mech_eye::PointCloudFrame& rawPointCloud,
    const scan_tracking::mech_eye::PointCloudFrame& stitchedPointCloud,
    const StateMachine::SegmentPoseStitchRecord& stitchRecord,
    const SegmentCaptureCxpExportMeta& cxpExportMeta)
{
    QString text;
    text += QStringLiteral("# Segment capture export metadata\n");
    text += QStringLiteral("generatedAt=") + QDateTime::currentDateTime().toString(Qt::ISODateWithMs) + QStringLiteral("\n");
    text += QStringLiteral("exportGroupId=") + cxpExportMeta.exportGroupId + QStringLiteral("\n");
    text += QStringLiteral("sessionRoot=") + cxpExportMeta.sessionRoot + QStringLiteral("\n");
    text += QStringLiteral("pathId=") + QString::number(pathId) + QStringLiteral("\n");
    text += QStringLiteral("segmentIndex=") + QString::number(segmentIndex) + QStringLiteral("\n");
    text += QStringLiteral("taskId=") + QString::number(bundle.request.taskId) + QStringLiteral("\n");
    text += QStringLiteral("requestId=") + QString::number(bundle.request.requestId) + QStringLiteral("\n");
    text += QStringLiteral("bundleSegmentIndex=") + QString::number(bundle.request.segmentIndex) + QStringLiteral("\n");
    text += QStringLiteral("needMechEye2D=") + QString(bundle.request.needMechEye2D ? "true" : "false") + QStringLiteral("\n");
    text += QStringLiteral("cxpLeftKey=") + bundle.request.hikCameraAKey + QStringLiteral("\n");
    text += QStringLiteral("cxpRightKey=") + bundle.request.hikCameraBKey + QStringLiteral("\n");
    text += QStringLiteral("cxpCaptureFileTimestamp=") + cxpExportMeta.captureFileTimestamp + QStringLiteral("\n");
    text += QStringLiteral("cxp2dOutputRoot=") + resolveSegmentCaptureFlatOutputRoot() + QStringLiteral("\n");
    text += QStringLiteral("cxpLeftFileName=") + cxpExportMeta.left.fileName + QStringLiteral("\n");
    text += QStringLiteral("cxpRightFileName=") + cxpExportMeta.right.fileName + QStringLiteral("\n");
    text += QStringLiteral("lbInvoked=") + QString(bundle.lbPoseResult.invoked ? "true" : "false") + QStringLiteral("\n");
    text += QStringLiteral("lbSuccess=") + QString(bundle.lbPoseResult.success ? "true" : "false") + QStringLiteral("\n");
    text += QStringLiteral("lbFramePointCount=") + QString::number(bundle.lbPoseResult.framePointCount) + QStringLiteral("\n");
    text += QStringLiteral("lbMessage=") + bundle.lbPoseResult.message + QStringLiteral("\n");
    text += QStringLiteral("lbTrackingValid=") + QString(stitchRecord.lbTrackingValid ? "true" : "false") + QStringLiteral("\n");
    text += QStringLiteral("rawPointCount=") + QString::number(rawPointCloud.pointCount) + QStringLiteral("\n");
    text += QStringLiteral("stitchedPointCount=") + QString::number(stitchedPointCloud.pointCount) + QStringLiteral("\n");
    text += QStringLiteral("\n# CXP image fingerprints (compare pixelMd5 / savedFileMd5 across segments)\n");
    appendCxpImageMetaLines(text, QStringLiteral("cxpLeft"), cxpExportMeta.left);
    appendCxpImageMetaLines(text, QStringLiteral("cxpRight"), cxpExportMeta.right);
    if (!bundle.lbPoseResult.diagnosticText.trimmed().isEmpty()) {
        text += QStringLiteral("\n");
        text += bundle.lbPoseResult.diagnosticText;
        if (!text.endsWith(QChar('\n'))) {
            text += QStringLiteral("\n");
        }
    }
    return text;
}

QString buildPoseStitchRtText(
    int pathId,
    int segmentIndex,
    const std::array<float, 16>& baseCalibrationT0,
    const std::array<float, 16>& calibrationT0Prime,
    const std::array<float, 16>& stereoTrackingT,
    const std::array<float, 16>& combinedOutputRt,
    bool lbValid)
{
    QString text;
    text += QStringLiteral("# Pose stitch output (row-major 4x4)\n");
    text += QStringLiteral("# generatedAt=") + QDateTime::currentDateTime().toString(Qt::ISODateWithMs) + QStringLiteral("\n");
    text += QStringLiteral("pathId=") + QString::number(pathId) + QStringLiteral("\n");
    text += QStringLiteral("segmentIndex=") + QString::number(segmentIndex) + QStringLiteral("\n");
    text += QStringLiteral("formula=p' = p x T0 ; LB成功时 T0=Rt_global，否则 T0=T0'(LBN链)\n");
    text += QStringLiteral("lbTrackingValid=") + QString(lbValid ? "true" : "false") + QStringLiteral("\n\n");
    text += formatRowMajorMatrixBlock(QStringLiteral("T0 (JSON fallback / base)"), baseCalibrationT0);
    text += QStringLiteral("\n");
    text += formatRowMajorMatrixBlock(QStringLiteral("T0 applied (Rt_global or T0')"), calibrationT0Prime);
    text += QStringLiteral("\n");
    text += formatRowMajorMatrixBlock(QStringLiteral("T (legacy slot, identity)"), stereoTrackingT);
    text += QStringLiteral("\n");
    text += formatRowMajorMatrixBlock(QStringLiteral("combined output Rt (= T0 applied)"), combinedOutputRt);
    return text;
}

QString buildSegmentPoseStitchRtText(const StateMachine::SegmentPoseStitchRecord& record)
{
    QString text;
    text += QStringLiteral("# Segment pose stitch matrix (row-major 4x4)\n");
    text += QStringLiteral("# generatedAt=") + QDateTime::currentDateTime().toString(Qt::ISODateWithMs) + QStringLiteral("\n");
    text += QStringLiteral("pathId=") + QString::number(record.pathId) + QStringLiteral("\n");
    text += QStringLiteral("segmentIndex=") + QString::number(record.segmentIndex) + QStringLiteral("\n");
    text += QStringLiteral("formula=p' = p x T0 ; LB成功时 T0=Rt_global，否则 T0=T0'(LBN链)\n");
    text += QStringLiteral("lbTrackingValid=") + QString(record.lbTrackingValid ? "true" : "false") + QStringLiteral("\n");
    text += QStringLiteral("lbRtGlobalValid=") + QString(record.lbRtGlobalValid ? "true" : "false") + QStringLiteral("\n\n");
    text += formatRowMajorMatrixBlock(QStringLiteral("T0 (JSON fallback / base)"), record.baseCalibrationT0);
    text += QStringLiteral("\n");
    text += formatRowMajorMatrixBlock(QStringLiteral("T0' (LBN chain)"), record.t0PrimeLbn);
    text += QStringLiteral("\n");
    if (record.lbRtGlobalValid) {
        text += formatRowMajorMatrixBlock(QStringLiteral("Rt_global (LB)"), record.lbRtGlobal);
    } else {
        text += QStringLiteral("Rt_global (LB): unavailable\n");
    }
    text += QStringLiteral("\n");
    text += formatRowMajorMatrixBlock(QStringLiteral("T0 applied (used for stitch)"), record.appliedT0);
    text += QStringLiteral("\n");
    text += formatRowMajorMatrixBlock(QStringLiteral("T (legacy slot, identity)"), record.stereoTrackingT);
    text += QStringLiteral("\n");
    text += formatRowMajorMatrixBlock(QStringLiteral("combined output Rt (= T0 applied)"), record.combinedOutputRt);
    return text;
}

QString buildFinalPoseStitchRtText(
    int pathId,
    int finalSegmentIndex,
    const StateMachine::SegmentPoseStitchRecord& record)
{
    QString text;
    text += QStringLiteral("# Path final pose stitch matrix (row-major 4x4)\n");
    text += QStringLiteral("# generatedAt=") + QDateTime::currentDateTime().toString(Qt::ISODateWithMs) + QStringLiteral("\n");
    text += QStringLiteral("pathId=") + QString::number(pathId) + QStringLiteral("\n");
    text += QStringLiteral("finalSegmentIndex=") + QString::number(finalSegmentIndex) + QStringLiteral("\n");
    text += QStringLiteral("note=最终矩阵取自该路径最后一个有效扫描段的 T0 applied\n\n");
    text += buildSegmentPoseStitchRtText(record);
    return text;
}

bool writeTextFile(const QString& filePath, const QString& content)
{
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        return false;
    }
    QTextStream stream(&file);
    stream.setCodec("UTF-8");
    stream << content;
    return true;
}

QVector<int> enabledScanPathIds()
{
    QVector<int> pathIds;
    const auto* configMgr = scan_tracking::common::ConfigManager::instance();
    if (configMgr == nullptr) {
        return pathIds;
    }

    const auto& pathsConfig = configMgr->scanPathsConfig();
    if (!pathsConfig.selectedPathIds.empty()) {
        for (int pathId : pathsConfig.selectedPathIds) {
            pathIds.push_back(pathId);
        }
    } else if (pathsConfig.executeAllPaths) {
        for (const auto& path : pathsConfig.scanPaths) {
            if (path.enabled) {
                pathIds.push_back(path.pathId);
            }
        }
    } else {
        for (const auto& path : pathsConfig.scanPaths) {
            if (path.enabled) {
                pathIds.push_back(path.pathId);
            }
        }
    }

    std::sort(pathIds.begin(), pathIds.end());
    pathIds.erase(std::unique(pathIds.begin(), pathIds.end()), pathIds.end());
    return pathIds;
}

int segmentTotalForPath(int pathId)
{
    if (pathId == kSelfCheckCacheBucketId) {
        const auto* configMgr = scan_tracking::common::ConfigManager::instance();
        if (configMgr != nullptr && configMgr->selfCheckConfig().totalPoints > 0) {
            return configMgr->selfCheckConfig().totalPoints;
        }
        return 2;
    }

    const auto* configMgr = scan_tracking::common::ConfigManager::instance();
    if (configMgr == nullptr || pathId <= 0) {
        return 0;
    }

    for (const auto& path : configMgr->scanPathsConfig().scanPaths) {
        if (path.enabled && path.pathId == pathId && path.totalPoints > 0) {
            return path.totalPoints;
        }
    }

    const int fallback = configMgr->trackingConfig().scanSegmentTotal;
    return fallback > 0 ? fallback : 0;
}

bool lookupNeedRotationForSegment(int pathId, int segmentIndex)
{
    const auto* configMgr = scan_tracking::common::ConfigManager::instance();
    if (configMgr == nullptr || pathId <= 0 || segmentIndex <= 0) {
        return false;
    }

    const auto& pathsConfig = configMgr->scanPathsConfig();
    for (const auto& path : pathsConfig.scanPaths) {
        if (!path.enabled || path.pathId != pathId) {
            continue;
        }
        for (const auto& point : path.points) {
            if (point.pointIndex == segmentIndex) {
                return point.needRotation;
            }
        }
    }
    return false;
}

/// 日志中展示 PLC 原始字；若与下游解码值不同则注明（如 16256→段号1）
QString formatPlcRegisterValueForLog(int modbusIndex, quint16 rawValue)
{
    namespace regs = protocol::registers;
    if (modbusIndex == regs::kScanSegmentIndex || modbusIndex == regs::kScanSegmentIndexRobot) {
        const quint16 decoded = regs::plcAnalogToUInt16(rawValue, 0);
        if (rawValue != decoded) {
            return QStringLiteral("%1 (原始PLC字=%2)").arg(decoded).arg(rawValue);
        }
        return QString::number(decoded);
    }
    if (modbusIndex == regs::kRequestTimeoutSeconds) {
        const quint16 decoded = regs::plcAnalogToUInt16(rawValue, 0);
        if (rawValue != decoded) {
            return QStringLiteral("%1s (原始PLC字=%2)").arg(decoded).arg(rawValue);
        }
        return QStringLiteral("%1").arg(decoded);
    }
    return QString::number(rawValue);
}

QString formatPlcRegisterChangeForLog(int modbusIndex, quint16 oldValue, quint16 newValue)
{
    return QStringLiteral("%1 -> %2")
        .arg(formatPlcRegisterValueForLog(modbusIndex, oldValue))
        .arg(formatPlcRegisterValueForLog(modbusIndex, newValue));
}

/**
 * @brief 将浮点数转换为 CDAB 字节序的两个寄存器值
 * 
 * Modbus 协议中 float 类型通常占用两个 16 位寄存器，采用 CDAB 字节序：
 * - 低16位在前（Little Endian 的低位字）
 * - 高16位在后（Little Endian 的高位字）
 * 
 * @param value 要转换的浮点数值
 * @return 包含两个寄存器值的向量 [low_word, high_word]
 */
QVector<quint16> floatToCdabRegisters(float value)
{
    quint32 raw = 0;
    static_assert(sizeof(raw) == sizeof(value), "Unexpected float width");
    std::memcpy(&raw, &value, sizeof(raw));

    const quint16 high = static_cast<quint16>((raw >> 16) & 0xFFFFu);
    const quint16 low = static_cast<quint16>(raw & 0xFFFFu);
    return {low, high};
}

scan_tracking::flow_control::StateMachine::PoseSourceResult parsePoseSource(
    const char* envName,
    const QString& sourceName,
    const std::array<float, 6>& fallback,
    bool treatMissingAsSimulated)
{
    scan_tracking::flow_control::StateMachine::PoseSourceResult result;
    result.available = true;
    result.sourceName = sourceName;

    const QString raw = qEnvironmentVariable(envName).trimmed();
    if (raw.isEmpty()) {
        result.success = true;
        result.message = treatMissingAsSimulated
            ? QStringLiteral("未配置外部位姿源；使用模拟回退。")
            : QStringLiteral("未配置外部位姿源。");
        result.x = fallback[0];
        result.y = fallback[1];
        result.z = fallback[2];
        result.rx = fallback[3];
        result.ry = fallback[4];
        result.rz = fallback[5];
        return result;
    }

    const auto tokens = raw.split(QRegExp(QStringLiteral("[,;\\s]+")), Qt::SkipEmptyParts);
    if (tokens.size() < 6) {
        result.success = false;
        result.message = QStringLiteral("位姿源 %1 需要 6 个值：x,y,z,rx,ry,rz。").arg(QString::fromLatin1(envName));
        result.sourceName = QStringLiteral("%1 (invalid)").arg(sourceName);
        return result;
    }

    bool ok = false;
    const float x = tokens.value(0).toFloat(&ok);
    if (!ok) {
        result.success = false;
        result.message = QStringLiteral("位姿源 %1 包含非数字值。").arg(QString::fromLatin1(envName));
        result.sourceName = QStringLiteral("%1 (invalid)").arg(sourceName);
        return result;
    }
    const float y = tokens.value(1).toFloat(&ok);
    if (!ok) {
        result.success = false;
        result.message = QStringLiteral("位姿源 %1 包含非数字值。").arg(QString::fromLatin1(envName));
        result.sourceName = QStringLiteral("%1 (invalid)").arg(sourceName);
        return result;
    }
    const float z = tokens.value(2).toFloat(&ok);
    if (!ok) {
        result.success = false;
        result.message = QStringLiteral("位姿源 %1 包含非数字值。").arg(QString::fromLatin1(envName));
        result.sourceName = QStringLiteral("%1 (invalid)").arg(sourceName);
        return result;
    }
    const float rx = tokens.value(3).toFloat(&ok);
    if (!ok) {
        result.success = false;
        result.message = QStringLiteral("位姿源 %1 包含非数字值。").arg(QString::fromLatin1(envName));
        result.sourceName = QStringLiteral("%1 (invalid)").arg(sourceName);
        return result;
    }
    const float ry = tokens.value(4).toFloat(&ok);
    if (!ok) {
        result.success = false;
        result.message = QStringLiteral("位姿源 %1 包含非数字值。").arg(QString::fromLatin1(envName));
        result.sourceName = QStringLiteral("%1 (invalid)").arg(sourceName);
        return result;
    }
    const float rz = tokens.value(5).toFloat(&ok);
    if (!ok) {
        result.success = false;
        result.message = QStringLiteral("位姿源 %1 包含非数字值。").arg(QString::fromLatin1(envName));
        result.sourceName = QStringLiteral("%1 (invalid)").arg(sourceName);
        return result;
    }

    result.success = true;
    result.message = QStringLiteral("从外部源 %1 加载位姿。").arg(QString::fromLatin1(envName));
    result.x = x;
    result.y = y;
    result.z = z;
    result.rx = rx;
    result.ry = ry;
    result.rz = rz;
    return result;
}

}  // namespace

/**
 * @brief 状态机构造函数
 * 
 * 初始化流程控制状态机，配置定时器、连接信号槽，并设置初始状态。
 * 
 * @param modbusService Modbus 通信服务指针
 * @param mechEyeService Mech-Eye 相机服务指针
 * @param trackingService 跟踪检测服务指针
 * @param parent Qt 父对象指针
 */
StateMachine::StateMachine(
    modbus::ModbusService* modbusService,
    mech_eye::MechEyeService* mechEyeService,
    vision::VisionPipelineService* visionPipelineService,
    tracking::TrackingService* trackingService,
    QObject* parent)
    : QObject(parent)
    , m_modbus(modbusService)
    , m_mechEye(mechEyeService)
    , m_visionPipeline(visionPipelineService)
    , m_tracking(trackingService)
    , m_pollTimer(new QTimer(this))
    , m_heartbeatTimer(new QTimer(this))
    , m_timeoutTimer(new QTimer(this))
    , m_handlerRegistry(std::make_unique<TaskHandlerRegistry>())
    , m_state(AppState::Init)
{
    // 从配置管理器获取流程控制配置，如果配置不存在则使用默认值
    const auto* configMgr = common::ConfigManager::instance();
    const auto flowConfig = configMgr ? configMgr->flowControlConfig()
                                      : common::FlowControlConfig{100, 1000, 300, {}};
    if (configMgr) {
        const auto& profile = configMgr->stationProfile();
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("[Station] StateMachine stationId=")
            << common::stationIdToInt(profile.stationId)
            << QStringLiteral(" workMode=")
            << common::workModeIdToString(profile.defaultWorkMode);
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("[Station] handlers=")
            << m_handlerRegistry->handlerCount()
            << QStringLiteral(" enabledTriggers=")
            << m_handlerRegistry->enabledTriggerNames(profile).join(QLatin1Char(','))
            << QStringLiteral(" (enableTelescopicScan/enableHoistAssist/enableCollisionMonitor stage3+)");
    }

    // 配置定时器间隔
    m_pollTimer->setInterval(flowConfig.pollIntervalMs);      // PLC 轮询间隔
    m_heartbeatTimer->setInterval(flowConfig.heartbeatIntervalMs);  // 心跳发送间隔
    m_timeoutTimer->setSingleShot(true);  // 超时定时器为单次触发

    // 连接定时器信号到对应的槽函数
    connect(m_pollTimer, &QTimer::timeout, this, &StateMachine::pollPlcState);
    connect(m_heartbeatTimer, &QTimer::timeout, this, &StateMachine::publishHeartbeat);
    connect(m_timeoutTimer, &QTimer::timeout, this, &StateMachine::onProcessTimeout);

    reloadCalibrationMatricesFromConfig();

    // 如果 Modbus 服务可用，连接其信号
    if (m_modbus) {
        connect(m_modbus, &modbus::ModbusService::connected, this, &StateMachine::onModbusConnected);
        connect(m_modbus, &modbus::ModbusService::disconnected, this, &StateMachine::onModbusDisconnected);
        connect(m_modbus, &modbus::ModbusService::errorOccurred, this, &StateMachine::onModbusError);
        connect(m_modbus, &modbus::ModbusService::registersRead, this, &StateMachine::handleRegistersRead);
        connect(m_modbus, &modbus::ModbusService::registerReadFailed, this, &StateMachine::onRegisterReadFailed);
        // P1修复：连接写失败信号，便于追踪具体哪个寄存器写入失败
        connect(m_modbus, &modbus::ModbusService::registerWriteFailed, this, &StateMachine::onRegisterWriteFailed);
    }

    // 如果 Mech-Eye 相机服务可用，连接其信号
    if (m_mechEye) {
        connect(
            m_mechEye,
            &mech_eye::MechEyeService::stateChanged,
            this,
            [](mech_eye::CameraRuntimeState state, QString desc) {
                qInfo(LOG_FLOW) << "[MechEye] 相机状态变更:" << static_cast<int>(state) << desc;
            });
        connect(
            m_mechEye,
            &mech_eye::MechEyeService::fatalError,
            this,
            &StateMachine::onMechEyeFatalError,
            Qt::QueuedConnection);
    }

    if (m_visionPipeline) {
        connect(
            m_visionPipeline,
            &vision::VisionPipelineService::bundleCaptureFinished,
            this,
            &StateMachine::onVisionBundleCaptureFinished,
            Qt::QueuedConnection);
        connect(
            m_visionPipeline,
            &vision::VisionPipelineService::stateChanged,
            this,
            [](vision::VisionPipelineState state, const QString& description) {
                qInfo(LOG_FLOW) << QStringLiteral("[VisionPipeline] 状态=") << static_cast<int>(state) << description;
            });
        connect(
            m_visionPipeline,
            &vision::VisionPipelineService::fatalError,
            this,
            [](vision::VisionErrorCode code, const QString& message) {
                qWarning(LOG_FLOW).noquote()
                    << QStringLiteral("[VisionPipeline] 致命错误：")
                    << static_cast<int>(code)
                    << message;
            });
    }
}

/**
 * @brief 析构函数，停止所有定时器并清理资源
 */
StateMachine::~StateMachine()
{
    stop();
}

/**
 * @brief 启动状态机
 * 
 * 重置所有状态变量，发布初始 IPC 状态，如果 Modbus 已连接则直接进入就绪状态。
 * 此方法通常在系统初始化或从错误恢复后调用。
 */
void StateMachine::start()
{
    qInfo(LOG_FLOW) << QStringLiteral("状态机启动。");
    if (isAlgorithmBypassEnabled()) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("[算法旁路] 已启用：跳过检测/位姿/点云后处理算法；Trig_ScanSegment 仍执行相机采集。");
    }
    clearActiveTask();           // 清除当前活动任务
    resetScanSegmentCache();     // 清空扫描缓存
    m_isPollingPlc = false;      // 重置 PLC 轮询标志
    m_ipcState = protocol::IpcState::Initializing;  // 设置 IPC 状态为初始化中
    m_currentStage = protocol::Stage::Idle;         // 设置当前阶段为空闲
    m_alarmLevel = 0;            // 清除报警级别
    m_alarmCode = 0;             // 清除报警代码
    m_warnCode = 0;              // 清除警告代码
    m_progress = 0;              // 重置进度
    m_dataValid = false;         // 标记数据无效
    m_consecutiveModbusFailures = 0;  // 重置 Modbus 失败计数器
    const int stalePending = m_pendingRefinementJobs.exchange(0, std::memory_order_acq_rel);
    if (stalePending != 0) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("状态机 start：复位后台 refinement 在途计数 stale=") << stalePending;
    }
    setState(AppState::Init);    // 设置应用状态为初始化
    initializePoseStitchRunOutputDirectory();
    publishIpcStatus();          // 发布 IPC 状态到 PLC

    // 如果 Modbus 已经连接，直接触发连接成功处理
    if (m_modbus && m_modbus->isConnected()) {
        onModbusConnected();
    }
}

/**
 * @brief 停止状态机
 * 
 * 停止所有定时器，清除活动任务，重置所有状态变量。
 * 此方法通常在系统关闭或需要完全重置时调用。
 */
void StateMachine::stop()
{
    const bool firstStop = !m_stopped.exchange(true);

    if (firstStop) {
        // 先断开外部信号并释放 std::function，避免退出阶段 processEvents 触发已失效回调
        {
            tracking::InspectionResultNotifier cleared;
            m_inspectionResultPublisher.swap(cleared);
        }

        if (m_modbus != nullptr) {
            disconnect(m_modbus, nullptr, this, nullptr);
        }
        if (m_mechEye != nullptr) {
            disconnect(m_mechEye, nullptr, this, nullptr);
        }
        if (m_visionPipeline != nullptr) {
            disconnect(m_visionPipeline, nullptr, this, nullptr);
        }

        if (m_pollTimer != nullptr) {
            m_pollTimer->stop();
        }
        if (m_heartbeatTimer != nullptr) {
            m_heartbeatTimer->stop();
        }
        if (m_timeoutTimer != nullptr) {
            m_timeoutTimer->stop();
        }

        // 尽早清零 IPC→PLC 结果区，避免 refinement join 期间 PLC 仍读到旧 Ack/Res
        resetPlcOutputRegisters();

        m_isPollingPlc = false;
        clearActiveTask();
    }

    if (firstStop) {
        joinAllBackgroundRefinementJobs(kShutdownRefinementJoinTimeoutMs);
        if (pendingRefinementJobCount() != 0) {
            reconcilePendingRefinementJobCounter("stop");
        }
    }

    if (firstStop) {
        resetScanSegmentCache();

        m_consecutiveModbusFailures = 0;
        m_alarmLevel = 0;
        m_alarmCode = 0;
        m_warnCode = 0;
        m_progress = 0;
        m_dataValid = false;
        m_heartbeatCounter = 0;
        m_ipcState = protocol::IpcState::Uninitialized;
        m_currentStage = protocol::Stage::Idle;
        setState(AppState::Init);
    }

    // HMI stop 后再退出进程时仍会走到此处，确保结果区再次清零
    resetPlcOutputRegisters();
}

/**
 * @brief 设置应用状态并发出状态变更信号
 * 
 * @param newState 新的应用状态
 */
void StateMachine::setState(AppState newState)
{
    if (m_state != newState) {
        m_state = newState;
        if (!m_stopped.load(std::memory_order_acquire)) {
            emit stateChanged(newState);
        }
        qInfo(LOG_FLOW) << QStringLiteral("应用状态切换为：") << static_cast<int>(newState);
    }
}

/**
 * @brief Modbus 连接成功时的回调处理
 * 
 * 重置失败计数器，设置 IPC 状态为就绪，启动定时器和心跳。
 */
void StateMachine::onModbusConnected()
{
    qInfo(LOG_FLOW) << QStringLiteral("Modbus 已连接，流程控制就绪。");
    
    // P2改进：重连后清理可能的残留状态，确保系统处于干净的初始状态
    if (m_activeTask.definition != nullptr) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("Modbus 重连后清除残留活动任务：")
            << protocol::triggerName(*m_activeTask.definition);
        clearActiveTask();
        resetScanSegmentCache();
    }
    
    m_isPollingPlc = false;           // 重置 PLC 轮询标志
    resetModbusFailureCounter();      // 重置 Modbus 失败计数器
    m_consecutiveModbusFailures = 0;  // 显式清零失败计数器
    m_ipcState = protocol::IpcState::Ready;   // 设置 IPC 状态为就绪
    m_currentStage = protocol::Stage::Idle;   // 设置当前阶段为空闲
    m_alarmLevel = 0;                 // 清除报警级别
    m_alarmCode = 0;                  // 清除报警代码
    m_warnCode = 0;                   // 清除警告代码
    m_progress = 0;                   // 重置进度
    m_dataValid = false;              // 标记数据无效
    setState(AppState::Ready);        // 设置应用状态为就绪
    publishIpcStatus();               // 发布 IPC 状态到 PLC
    if (m_unloadAreaConfigReceived) {
        updateUnloadAreaConfig(m_unloadAreaConfig);
    }
    publishHeartbeat();               // 立即发送一次心跳
    m_pollTimer->start();             // 启动 PLC 轮询定时器
    m_heartbeatTimer->start();        // 启动心跳定时器
    
    qInfo(LOG_FLOW) << QStringLiteral("Modbus 重连恢复完成，系统已回到就绪状态。");
}

/**
 * @brief Modbus 断开连接时的回调处理
 * 
 * 停止所有定时器，进入故障状态，等待重新连接。
 */
void StateMachine::onModbusDisconnected()
{
    qWarning(LOG_FLOW) << QStringLiteral("Modbus 已断开，流程控制暂停。");
    m_pollTimer->stop();       // 停止 PLC 轮询
    m_heartbeatTimer->stop();  // 停止心跳
    m_timeoutTimer->stop();    // 停止超时定时器
    m_isPollingPlc = false;    // 重置 PLC 轮询标志
    // 进入故障状态，报警代码 900，不中止当前任务（因为没有活跃任务），不通知 PLC
    enterFaultState(900, QStringLiteral("Modbus 已断开连接"), true, false);
}

/**
 * @brief Modbus 发生错误时的回调处理
 * 
 * 记录错误并增加失败计数器，连续失败达到阈值时将进入故障状态。
 * 
 * @param errorString 错误描述信息
 */
void StateMachine::onModbusError(const QString& errorString)
{
    qWarning(LOG_FLOW).noquote() << "Modbus 错误传播到流程控制：" << errorString;
    recordModbusFailure(901, errorString);  // 记录 Modbus 失败，报警代码 901
}

/**
 * @brief 轮询 PLC 状态（Server 模式下为空操作）
 * 
 * 在 Server 模式下，PLC 主动写入命令区时会触发 onDataWritten → registersRead 信号，
 * 不需要 IPC 主动轮询。保留此函数是为了兼容定时器连接，避免改动过大。
 */
void StateMachine::pollPlcState()
{
    // Server 模式：PLC 主动写入命令区，通过 registersRead 信号触发 handleRegistersRead
    // 此函数保留为空操作，轮询定时器可用于其他周期性检查（如心跳超时检测）
}

/**
 * @brief 处理从 PLC 读取的寄存器数据
 *
 * 解析命令块，检测触发信号，处理任务完成确认等。
 *
 * @param startAddress 起始寄存器地址
 * @param values 读取到的寄存器值向量
 */
void StateMachine::handleRegistersRead(int startAddress, const QVector<quint16>& values)
{
    if (m_stopped.load(std::memory_order_acquire)) {
        return;
    }

    // 验证是否是预期的命令块读取，且数据长度足够
    if (startAddress != protocol::registers::kCommandBlockStart ||
        values.size() < protocol::registers::kCommandBlockSize) {
        return;
    }

    const QVector<quint16> previousCommandBlock = m_lastCommandBlock;
    m_lastCommandBlock = values;       // 保存最新的命令块数据
    m_robotTcpPose = protocol::registers::readRobotTcpPoseFromCommandBlock(values);
    resetModbusFailureCounter();       // 通信成功，重置失败计数器

    // 判断命令块是否发生业务变化：忽略 PLC_Heartbeat，避免每 100ms 因心跳刷屏
    bool commandBlockChanged = previousCommandBlock.isEmpty();
    if (!commandBlockChanged) {
        const int compareCount = qMin(previousCommandBlock.size(), values.size());
        for (int index = 1; index < compareCount; ++index) {
            if (previousCommandBlock.value(index) != values.value(index)) {
                commandBlockChanged = true;
                break;
            }
        }
    }

    // 轮询完成日志：节流输出，避免 100ms 轮询刷屏
    if (m_activePollRequestSequence == 1 || (m_activePollRequestSequence % kPollLogEveryN) == 0) {
        qDebug(LOG_FLOW).noquote()
            << QStringLiteral("PLC 轮询完成")
            << QStringLiteral(" 请求序号=") << m_activePollRequestSequence
            << QStringLiteral(" 耗时ms=") << (m_pollRequestTimer.isValid() ? m_pollRequestTimer.elapsed() : -1);
    }

    m_activePollRequestSequence = 0;

    // 命令块快照：只在首次读取或内容变化时打印
    if (commandBlockChanged) {
        namespace regs = protocol::registers;
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("命令块快照：")
            << "PLC_Start=" << regs::holdingRegisterAddress(regs::kCommandBlockStart)
            << "Flow_Enable=" << values.value(regs::kFlowEnable)
            << "Reg04=" << values.value(regs::kSafetyStatusWord)
            << "ScanSegmentIndex=" << protocol::registers::resolveScanSegmentIndexFromBlock(values)
            << "RequestTimeout_s=" << formatPlcRegisterValueForLog(regs::kRequestTimeoutSeconds,
                                                                   values.value(regs::kRequestTimeoutSeconds))
            << "Robot_Status_Word=" << values.value(regs::kRobotStatusWord)
            << "Trig_LoadGrasp=" << values.value(regs::modbusIndexFromPlcAddress(40020))
            << "Trig_StationMaterialCheck=" << values.value(regs::modbusIndexFromPlcAddress(40021))
            << "Trig_PoseCheck=" << values.value(regs::modbusIndexFromPlcAddress(40022))
            << "Trig_ScanSegment=" << values.value(regs::modbusIndexFromPlcAddress(40023))
            << "Trig_Inspection=" << values.value(regs::modbusIndexFromPlcAddress(40024))
            << "Trig_UnloadCalc=" << values.value(regs::modbusIndexFromPlcAddress(40025))
            << "Trig_SelfCheck=" << values.value(regs::modbusIndexFromPlcAddress(40026))
            << "Trig_CodeRead=" << values.value(regs::modbusIndexFromPlcAddress(40027))
            << "Trig_ResultReset=" << values.value(regs::modbusIndexFromPlcAddress(40028))
            << "TaskIdHigh=" << values.value(regs::kTaskIdHigh)
            << "TaskIdLow=" << values.value(regs::kTaskIdLow)
            << "RobotTcp="
            << QStringLiteral("x=%1 y=%2 z=%3 rx=%4 ry=%5 rz=%6")
                   .arg(m_robotTcpPose.x, 0, 'f', 3)
                   .arg(m_robotTcpPose.y, 0, 'f', 3)
                   .arg(m_robotTcpPose.z, 0, 'f', 3)
                   .arg(m_robotTcpPose.rx, 0, 'f', 3)
                   .arg(m_robotTcpPose.ry, 0, 'f', 3)
                   .arg(m_robotTcpPose.rz, 0, 'f', 3);
    }

    // 命令块原始寄存器值：只在首次读取或内容变化时打印
    if (commandBlockChanged) {
        QStringList rawRegisters;
        rawRegisters.reserve(values.size());

        for (int index = 0; index < values.size(); ++index) {
            rawRegisters << QStringLiteral("%1=%2")
                .arg(index)
                .arg(values.value(index));
        }

        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("命令块原始寄存器：")
            << rawRegisters.join(QStringLiteral(" "));
    }

    // 打印变化字段：只在非首次读取且有字段变化时打印（带 PLC 地址和寄存器名）
    if (!previousCommandBlock.isEmpty()) {
        static const char* const kRegisterNames[] = {
            "Reserved_0",              // 0
            "PLC_Heartbeat",           // 1   40001
            "PLC_SystemState",         // 2
            "Station_WorkMode",        // 3
            "Flow_Enable",             // 4
            "Safety_Status_Word",      // 5
            "Cmd_StartAuto",           // 6
            "Cmd_Pause",               // 7
            "Cmd_Stop",                // 8
            "Cmd_Reset",               // 9
            "Cmd_ClearAlarms",         // 10
            "TaskId_H",                // 11
            "TaskId_L",                // 12
            "ProductType",             // 13
            "RecipeId",                // 14
            "ScanSegmentIndex",        // 15  40015
            "ScanSegmentIndex_Robot",  // 16  40016 机械臂/PLC 实际段号
            "RequestTimeout_s",        // 17  40017
            "Robot_Status_Word",       // 18  40018 埃斯顿 Robot 40004，PLC 转发
            "Reserved_19",             // 19
            "Trig_LoadGrasp",          // 20  40020
            "Trig_StationMaterialCheck", // 21
            "Trig_PoseCheck",          // 22
            "Trig_ScanSegment",        // 23
            "Trig_Inspection",         // 24
            "Trig_UnloadCalc",         // 25
            "Trig_SelfCheck",          // 26
            "Trig_CodeRead",           // 27
            "Trig_ResultReset",        // 28
            "RobotTcp_X_L",            // 29  40029
            "RobotTcp_X_H",            // 30  40030
            "RobotTcp_Y_L",            // 31
            "RobotTcp_Y_H",            // 32
            "RobotTcp_Z_L",            // 33
            "RobotTcp_Z_H",            // 34
            "RobotTcp_Rx_L",           // 35
            "RobotTcp_Rx_H",           // 36
            "RobotTcp_Ry_L",           // 37
            "RobotTcp_Ry_H",           // 38
            "RobotTcp_Rz_L",           // 39  40039
            "RobotTcp_Rz_H",           // 40  40040
            "TelescopicRod_Status",    // 41  40041
            "Roller_SetFreq_Hz",       // 42  40042
            "Roller_RunFreq_Hz",       // 43  40043
            "Electromagnet_Status",    // 44  40044
            "EstopButton_Status",      // 45  40045
            "LoadVision_StatusCode",   // 46  40046
            "UnloadNgArea_Full",       // 47  40047
            "UnloadOkArea_Full",       // 48  40048
            "UnloadNgArea_Count",      // 49  40049
            "UnloadOkArea_Count",      // 50  40050
        };
        constexpr int kNameCount = sizeof(kRegisterNames) / sizeof(kRegisterNames[0]);
        const int compareCount = qMin(previousCommandBlock.size(),
                                      qMin(values.size(), protocol::registers::kCommandBlockSize));

        QStringList changedFields;
        for (int index = 0; index < compareCount; ++index) {
            const quint16 oldValue = previousCommandBlock.value(index);
            const quint16 newValue = values.value(index);

            if (oldValue == newValue) {
                continue;
            }

            const char* name = (index < kNameCount) ? kRegisterNames[index] : "?";
            changedFields << QStringLiteral("  [%1] %2 (plcOffset=%3, modbusIndex=%4): %5")
                .arg(protocol::registers::holdingRegisterAddress(index))
                .arg(QString::fromLatin1(name))
                .arg(protocol::registers::plcTableOffset(index))
                .arg(index)
                .arg(formatPlcRegisterChangeForLog(index, oldValue, newValue));
        }

        if (!changedFields.isEmpty()) {
            qInfo(LOG_FLOW).noquote()
                << "=== PLC 寄存器变化 ===" << "\n" << changedFields.join(QStringLiteral("\n"));
        }
    }

    namespace regs = protocol::registers;
    const int scanTrigOffset = regs::modbusIndexFromPlcAddress(40023);
    const int inspectionTrigOffset = regs::modbusIndexFromPlcAddress(40024);
    const bool scanPending =
        scanTrigOffset < values.size() && values[scanTrigOffset] == 1;
    const bool inspectionPending =
        inspectionTrigOffset < values.size() && values[inspectionTrigOffset] == 1;

    // 如果当前有活动任务且已完成宣告，检查 PLC 是否已释放触发信号
    if (m_activeTask.definition != nullptr && m_activeTask.completionAnnounced) {
        // 多路径：检测已握手完成但 PLC 仍保持 Trig_Inspection=1，同时要继续扫下一路径 → 勿阻塞
        if (m_activeTask.definition->stage == protocol::Stage::Inspection && scanPending
            && inspectionPending) {
            qWarning(LOG_FLOW).noquote()
                << QStringLiteral("[多路径] 检测已完成但 Trig_Inspection 仍为 1，")
                << QStringLiteral("Trig_ScanSegment=1，强制收尾活动任务以继续扫描。")
                << multiPathCacheStatusText();
            clearActiveTask();
            m_ipcState = protocol::IpcState::Ready;
            m_currentStage = protocol::Stage::Idle;
            m_progress = 0;
            setState(AppState::Ready);
            publishIpcStatus();
        } else {
            finalizeCompletedTaskIfTriggerReleased(values);
        }
    }

    // 如果有活动任务但未完成宣告，等待任务执行完毕
    if (m_activeTask.definition != nullptr) {
        return;
    }

    if (const protocol::TriggerDefinition* pendingTrigger = selectPendingTrigger(values)) {
        processTrigger(*pendingTrigger, values);
    }
}
/**
 * @brief 处理寄存器读取失败的回调
 * 
 * 记录错误并重置轮询标志，允许下一次轮询继续进行。
 * 
 * @param startAddress 读取失败的起始地址
 * @param errorString 错误描述信息
 */
void StateMachine::onRegisterReadFailed(int startAddress, const QString& errorString)
{
    // 只关心命令块的读取失败
    if (startAddress != protocol::registers::kCommandBlockStart) {
        return;
    }

    if (m_isPollingPlc) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("PLC 轮询失败：")
            << errorString
            << QStringLiteral(" 请求序号=") << m_activePollRequestSequence
            << QStringLiteral(" 耗时ms=") << (m_pollRequestTimer.isValid() ? m_pollRequestTimer.elapsed() : -1);
    }
    m_isPollingPlc = false;  // 重置轮询标志，允许下次轮询
    m_activePollRequestSequence = 0;
}

/**
 * @brief 处理寄存器写入失败事件
 * 
 * P1修复：当 Modbus 写操作失败时记录详细日志，便于追踪具体哪个寄存器写入失败。
 * 此槽函数连接到 ModbusService 的 registerWriteFailed 信号。
 * 
 * @param startAddress 写入失败的起始地址
 * @param errorString 错误描述信息
 */
void StateMachine::onRegisterWriteFailed(int startAddress, const QString& errorString)
{
    qWarning(LOG_FLOW).noquote()
        << QStringLiteral("寄存器写入失败，地址=") << startAddress
        << QStringLiteral(" (0x") << QString::number(startAddress, 16) << QStringLiteral(")：")
        << errorString;
    
    // 可以根据地址范围判断是哪个业务的写入失败，采取不同的恢复策略
    // 例如：如果是结果区写入失败，可能需要重新发送结果
}

/**
 * @brief 处理 PLC 触发信号
 * 
 * 当检测到某个触发位被置为 1 时，验证触发条件，初始化任务状态，
 * 发送 ACK 响应，启动超时定时器，并执行对应的任务逻辑。
 * 
 * @param trigger 触发定义结构，包含触发偏移、阶段、超时等信息
 * @param commandBlock 完整的命令块寄存器数据
 */
void StateMachine::processTrigger(const protocol::TriggerDefinition& trigger, const QVector<quint16>& commandBlock)
{
    if (!m_modbus || !m_modbus->isConnected()) {
        return;  // Modbus 未连接，无法处理触发
    }

    if (const auto* configMgr = common::ConfigManager::instance()) {
        const auto& profile = configMgr->stationProfile();
        if (!isTriggerEnabledForProfile(profile, trigger.trigOffset)) {
            rejectDisabledTrigger(trigger);
            return;
        }
    }

    // 除了卸载计算和结果复位外，其他触发都需要 Flow_Enable=1 才能执行
    if (trigger.stage != protocol::Stage::UnloadCalc &&
        trigger.stage != protocol::Stage::ResultReset &&
        commandBlock.value(protocol::registers::kFlowEnable) == 0) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("Flow_Enable=0 时拒绝触发：")
                                     << protocol::triggerName(trigger);
        sendRes(trigger, 9);                          // 返回错误码 9（参数错误）
        sendAck(trigger, protocol::AckState::Failed); // 发送失败 ACK
        return;
    }

    // 初始化活动任务状态
    m_activeTask.definition = &trigger;                                    // 保存触发定义指针
    m_activeTask.taskId = readTaskId(commandBlock);                        // 读取任务 ID
    // 如果 PLC 指定了超时时间则使用，否则使用触发定义的默认超时
    {
        const quint16 timeoutRaw = commandBlock.value(protocol::registers::kRequestTimeoutSeconds);
        const quint16 timeoutDecoded = protocol::registers::plcAnalogToUInt16(timeoutRaw, 0);
        m_activeTask.timeoutSeconds = timeoutDecoded > 0
            ? timeoutDecoded
            : static_cast<quint16>(trigger.defaultTimeoutSeconds);
    }
    m_activeTask.scanSegmentIndex = resolveScanSegmentIndex(commandBlock); // 解析扫描分段索引

    if (trigger.stage == protocol::Stage::ScanSegment) {
        QString validationError;
        if (!validateScanSegmentRequest(commandBlock, &validationError)) {
            qWarning(LOG_FLOW).noquote()
                << QStringLiteral("拒绝触发") << protocol::triggerName(trigger)
                << validationError;
            m_activeTask.definition = nullptr;
            sendRes(trigger, 9);
            sendAck(trigger, protocol::AckState::Failed);
            return;
        }
    }

    if (trigger.stage == protocol::Stage::Inspection) {
        m_activeTask.inspectionPathId = resolvePathIdForInspection();
    }

    if (trigger.stage == protocol::Stage::Inspection && m_activeTask.inspectionPathId <= 0) {
        if (isAlgorithmBypassEnabled()) {
            m_activeTask.inspectionPathId = 1;
            qInfo(LOG_FLOW).noquote()
                << QStringLiteral("[算法旁路] Trig_Inspection 无缓存路径，使用虚拟路径 ID=1。");
        } else {
        const QString status = multiPathCacheStatusText();
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("[多路径] 拒绝 Trig_Inspection（当前无已扫满的任一路径），保留已缓存点云。")
            << status;
        writeInspectionResult({2, 1u << 4, 0, 0});
        if (m_inspectionResultPublisher) {
            tracking::InspectionResult failure;
            failure.resultCode = 2;
            failure.ngReasonWord0 = (1u << 4);
            failure.message = QStringLiteral("综合检测过早：%1").arg(status);
            m_inspectionResultPublisher(failure);
        }
        completeActiveTask(2, protocol::AckState::Completed, false);
        // 过早检测：不等 PLC 释放 Trig_Inspection，立即回到 Ready 以便继续扫下一路径
        clearActiveTask();
        m_ipcState = protocol::IpcState::Ready;
        m_currentStage = protocol::Stage::Idle;
        m_progress = 0;
        setState(AppState::Ready);
        publishIpcStatus();
        return;
        }
    }

    {
        const auto* cfgMgr = scan_tracking::common::ConfigManager::instance();
        if (m_selfCheckSessionActive) {
            m_activeTask.scanSegmentTotal = segmentTotalForPath(kSelfCheckCacheBucketId);
        } else {
            const int pathId = resolvePathIdForIncomingSegment(m_activeTask.scanSegmentIndex);
            const int pathTotal = segmentTotalForPath(pathId);
            m_activeTask.scanSegmentTotal = pathTotal > 0
                ? pathTotal
                : (cfgMgr ? cfgMgr->trackingConfig().scanSegmentTotal : 1);
        }
    }
    m_activeTask.completionAnnounced = false;  // 重置完成宣告标志
    m_activeTask.captureRequestId = 0;         // 重置采集请求 ID

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("已接受触发") << protocol::triggerName(trigger)
        << QStringLiteral(" 超时s=") << m_activeTask.timeoutSeconds
        << QStringLiteral(" 段号=") << m_activeTask.scanSegmentIndex
        << (trigger.stage == protocol::Stage::Inspection
                ? QStringLiteral(" 检测路径ID=") + QString::number(m_activeTask.inspectionPathId)
                : QString());

    // 清除之前的报警信息
    setAlarm(0, 0, QString());
    // 切换到扫描状态
    setState(AppState::Scanning);
    m_ipcState = protocol::IpcState::Busy;   // IPC 状态设为忙碌
    m_currentStage = trigger.stage;          // 设置当前阶段
    m_progress = 5;                          // 进度设为 5%（刚开始）
    m_dataValid = false;                     // 数据无效
    publishIpcStatus();                      // 发布 IPC 状态到 PLC

    sendAck(trigger, protocol::AckState::Running);  // 发送运行中 ACK

    // 如果任务 ID 不为 0，回写到 PLC 的任务 ID 回声寄存器
    if (m_activeTask.taskId != 0) {
        const bool taskIdWritten = m_modbus->writeRegisters(protocol::registers::kTaskIdEchoHigh, {
            static_cast<quint16>((m_activeTask.taskId >> 16) & 0xFFFFu),
            static_cast<quint16>(m_activeTask.taskId & 0xFFFFu),
        });
        if (!taskIdWritten) {
            qWarning(LOG_FLOW).noquote() << QStringLiteral("写入任务 ID 回声寄存器失败");
        }
    }

    // 启动超时定时器（秒转毫秒）
    m_timeoutTimer->start(static_cast<int>(m_activeTask.timeoutSeconds) * 1000);
    // 执行具体的任务逻辑
    executeActiveTask();
}

void StateMachine::rejectDisabledTrigger(const protocol::TriggerDefinition& trigger)
{
    qWarning(LOG_FLOW).noquote()
        << QStringLiteral("[Station] 触发器")
        << protocol::triggerName(trigger)
        << QStringLiteral("在当前 profile 未启用，已拒绝，Res=8");
    sendAck(trigger, protocol::AckState::Running);
    sendRes(trigger, 8);
    sendAck(trigger, protocol::AckState::Failed);
}

/**
 * @brief 执行当前活动任务
 * 
 * 根据触发定义的 trigOffset 分发到已注册的 Handler。
 */
void StateMachine::executeBypassActiveTask()
{
    if (m_activeTask.definition == nullptr) {
        return;
    }

    const protocol::TriggerDefinition& trigger = *m_activeTask.definition;
    qWarning(LOG_FLOW).noquote()
        << QStringLiteral("[算法旁路] 跳过检测/位姿等算法，PLC 握手回写成功：")
        << protocol::triggerName(trigger);

    switch (trigger.stage) {
    case protocol::Stage::LoadGrasp:
        writeLoadGraspResult();
        completeActiveTask(1, protocol::AckState::Completed, true);
        emit loadGraspFinished(1, 125.0f, 250.0f, 375.0f, 0.0f, 90.0f, 180.0f);
        break;
    case protocol::Stage::StationMaterialCheck:
        writeAsciiPlaceholder(protocol::registers::kSelfCheckFailWord0, 2, QStringLiteral("OK"));
        completeActiveTask(1, protocol::AckState::Completed, true);
        break;
    case protocol::Stage::PoseCheck: {
        const QVector<double> identityRt = {
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0,
        };
        writeFloatPlaceholder(protocol::registers::kPoseDeviationMm, 0.0f);
        completeActiveTask(1, protocol::AckState::Completed, true);
        emit poseCheckFinished(true, 1, 0.0, identityRt, QStringLiteral("算法旁路"));
        break;
    }
    case protocol::Stage::ScanSegment:
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("[算法旁路] Trig_ScanSegment：执行相机采集，跳过 LBN/LB/点云后处理算法");
        executeScanSegmentTask();
        break;
    case protocol::Stage::Inspection: {
        InspectionSummary summary;
        summary.resultCode = 1;
        writeInspectionResult(summary);
        completeActiveTask(1, protocol::AckState::Completed, true);
        tracking::InspectionMeasurement measurement;
        emit inspectionFinished(1, 0, 0, 0, measurement, QStringLiteral("算法旁路"));
        break;
    }
    case protocol::Stage::UnloadCalc:
        writeUnloadCalcResult();
        completeActiveTask(1, protocol::AckState::Completed, true);
        emit unloadCalcFinished(1, 500.0f, 600.0f, 700.0f, 0.0f, 0.0f, 90.0f);
        break;
    case protocol::Stage::SelfCheck:
        if (m_modbus && m_modbus->isConnected()) {
            m_modbus->writeRegisters(protocol::registers::kSelfCheckFailWord0, {0, 0});
            m_modbus->writeRegisters(protocol::registers::kSelfCheckFailWord1, {0});
        }
        completeActiveTask(1, protocol::AckState::Completed, true);
        emit selfCheckFinished(1, 0);
        break;
    case protocol::Stage::ResultReset:
        executeResultResetTask();
        break;
    default:
        if (trigger.trigOffset == 27) {
            if (m_modbus && m_modbus->isConnected()) {
                writeAsciiPlaceholder(
                    protocol::registers::kCodeValueAscii,
                    protocol::registers::kCodeValueRegisterCount,
                    QStringLiteral("BY"));
            }
            completeActiveTask(1, protocol::AckState::Completed, true);
            emit codeReadFinished(1, QStringLiteral("BY"));
        } else {
            completeActiveTask(1, protocol::AckState::Completed, true);
        }
        break;
    }
}

void StateMachine::executeActiveTask()
{
    if (m_activeTask.definition == nullptr) {
        return;  // 没有活动任务，直接返回
    }

    if (isAlgorithmBypassEnabled()) {
        executeBypassActiveTask();
        return;
    }

    ITaskHandler* handler = m_handlerRegistry
        ? m_handlerRegistry->handlerForOffset(m_activeTask.definition->trigOffset)
        : nullptr;
    if (handler == nullptr) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("拒绝不支持的触发")
            << protocol::triggerName(*m_activeTask.definition)
            << QStringLiteral(" trigOffset=") << m_activeTask.definition->trigOffset;
        setAlarm(2, 624, QStringLiteral("收到不支持的触发"));
        completeActiveTask(9, protocol::AckState::Failed, false);
        return;
    }

    TaskHandlerContext ctx{*this, m_lastCommandBlock, m_activeTask};
    handler->execute(ctx);
}

void StateMachine::onVisionBundleCaptureFinished(scan_tracking::vision::MultiCameraCaptureBundle bundle)
{
    if (m_stopped.load(std::memory_order_acquire)) {
        return;
    }

    if (!m_activeTask.definition
        || m_activeTask.definition->stage != protocol::Stage::ScanSegment) {
        return;
    }
    if (m_activeTask.completionAnnounced) {
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("忽略过期视觉 bundle（Trig_ScanSegment 已握手完成）")
            << QStringLiteral(" 段号=") << bundle.request.segmentIndex
            << QStringLiteral(" requestId=") << bundle.request.requestId;
        return;
    }
    if (bundle.request.taskId != m_activeTask.taskId) {
        return;
    }
    if (m_activeTask.captureRequestId != 0
        && bundle.request.requestId != m_activeTask.captureRequestId) {
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("忽略 requestId 不匹配的视觉 bundle：期望=")
            << m_activeTask.captureRequestId
            << QStringLiteral(" 实际=") << bundle.request.requestId;
        return;
    }

    if (!bundle.mechEyeResult.success()) {
        const quint16 resultCode = mapCaptureErrorToResCode(bundle.mechEyeResult.errorCode);
        finishScanSegmentFailure(
            resultCode,
            3,
            722,
            QStringLiteral("视觉组合中 MechEye 采集失败"),
            QStringLiteral("视觉组合中 MechEye 采集失败"));
        return;
    }

    // TODO: MechEye 暂时屏蔽（已验证通过），当前只测试海康 A/B
    // 当 MechEye 恢复后，需检查 bundle.mechEyeResult.success()

    // 海康 A/B 采集失败则报错终止
    if (!bundle.hikCameraAResult.success() || !bundle.hikCameraBResult.success()) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("海康 A/B 采集失败")
            << QStringLiteral(" hikA=") << bundle.hikCameraAResult.errorMessage
            << QStringLiteral(" hikB=") << bundle.hikCameraBResult.errorMessage;
        // 不阻断流程但记录失败
    }

    const auto& result = bundle.mechEyeResult;
    const int pathIdForCapture = resolvePathIdForIncomingSegment(m_activeTask.scanSegmentIndex);
    if (pathIdForCapture != m_currentPathId && m_currentPathId > 0) {
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("[多路径] 路径切换，标定矩阵重置为 T0")
            << QStringLiteral(" 旧路径ID=") << m_currentPathId
            << QStringLiteral(" 新路径ID=") << pathIdForCapture
            << QStringLiteral(" 段号=") << m_activeTask.scanSegmentIndex;
        m_currentCalibrationMatrix = m_baseCalibrationMatrix;
    }
    const bool needRotation = resolveNeedRotationForSegment(pathIdForCapture, m_activeTask.scanSegmentIndex);
    if (isAlgorithmBypassEnabled()) {
        if (!result.pointCloud.isValid()) {
            finishScanSegmentFailure(
                7,
                3,
                722,
                QStringLiteral("Mech-Eye 点云无效，无法缓存"),
                QStringLiteral("Mech-Eye 点云无效，无法缓存"));
            return;
        }

        const int segmentIndex = m_activeTask.scanSegmentIndex;
        const int pathIdForCache = resolvePathIdForIncomingSegment(segmentIndex);
        const int capturedPointCount = result.pointCloud.pointCount;
        commitBypassScanSegmentCapture(pathIdForCache, segmentIndex, capturedPointCount, bundle);
        return;
    }

    applyLbnCalibrationUpdate(pathIdForCapture, m_activeTask.scanSegmentIndex, needRotation, bundle);

    if (!result.pointCloud.isValid()) {
        finishScanSegmentFailure(
            7,
            3,
            722,
            QStringLiteral("Mech-Eye 点云无效，无法缓存"),
            QStringLiteral("Mech-Eye 点云无效，无法缓存"));
        return;
    }

    const int segmentIndex = m_activeTask.scanSegmentIndex;
    const quint32 taskId = m_activeTask.taskId;
    const scan_tracking::common::PointCloudProcessingConfig processingConfig =
        scan_tracking::common::ConfigManager::instance()
            ? scan_tracking::common::ConfigManager::instance()->pointCloudProcessingConfig()
            : scan_tracking::common::PointCloudProcessingConfig{};

    const int pathIdForCache = resolvePathIdForIncomingSegment(segmentIndex);
    commitScanSegmentCaptureImmediate(pathIdForCache, segmentIndex, result, bundle);
    if (!processingConfig.enabled) {
        applySegmentPoseStitching(pathIdForCache, segmentIndex);
    }
    startSegmentBackgroundRefinement(pathIdForCache, segmentIndex, taskId, processingConfig);
}

void StateMachine::commitScanSegmentCaptureImmediate(
    int pathId,
    int segmentIndex,
    const scan_tracking::mech_eye::CaptureResult& result,
    const scan_tracking::vision::MultiCameraCaptureBundle& bundle)
{
    if (pathId != m_currentPathId) {
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("[多路径] 切换路径")
            << QStringLiteral(" 段号=") << segmentIndex
            << QStringLiteral(" 旧路径ID=") << m_currentPathId
            << QStringLiteral(" 新路径ID=") << pathId;
        m_currentPathId = pathId;
        m_currentPathSegments.clear();
    }

    m_currentPathSegments.insert(segmentIndex);

    {
        std::lock_guard<std::mutex> lock(m_segmentCacheMutex);
        const scan_tracking::mech_eye::CaptureResult cachedResult = cloneCaptureResult(result);
        m_pathSegmentCaptureResults[pathId][segmentIndex] = cachedResult;
        m_pathSegmentCaptureBundles[pathId][segmentIndex] =
            cloneCaptureBundleForCache(bundle, cachedResult.pointCloud);
    }

    int imageCount = countHikImagesInBundle(bundle);
    if (imageCount == 0) {
        imageCount = 1;
    }
    const int cloudFrameCount = result.pointCloud.pointCount > 0 ? 1 : 0;

    writeScanSegmentResult(segmentIndex, imageCount, cloudFrameCount);
    m_progress = 100;
    publishIpcStatus();

    exportSegmentCxp2dImages(segmentIndex, bundle);

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("[SegmentCache] 原始点云已入内存（results/bundle 共享同一份点云），立即回写 PLC")
        << QStringLiteral(" [路径") << pathId << QStringLiteral("][段") << segmentIndex << QStringLiteral("]")
        << QStringLiteral(" 任务ID=") << m_activeTask.taskId
        << QStringLiteral(" 点数=") << result.pointCloud.pointCount
        << QStringLiteral(" 总缓存=") << totalCachedPointCloudCount();

    completeActiveTask(1);
    emit scanFinished(segmentIndex, 1, imageCount, cloudFrameCount);

    maybeLatchFirstPathStepPause(pathId, segmentIndex);
}

void StateMachine::exportSegmentCxp2dImages(
    int segmentIndex,
    const scan_tracking::vision::MultiCameraCaptureBundle& bundle)
{
    if (segmentIndex <= 0) {
        return;
    }
    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    if (configManager == nullptr || !configManager->segmentCaptureExportConfig().enabled) {
        return;
    }

    const auto cxpExportMeta = saveSegmentCxp2dImagesToFlatOutput(bundle);
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("[SegmentCaptureExport] 段号=") << segmentIndex
        << QStringLiteral(" 2D输出目录=") << resolveSegmentCaptureFlatOutputRoot()
        << QStringLiteral(" 左=")
        << (cxpExportMeta.left.saved ? cxpExportMeta.left.fileName : QStringLiteral("(skip)"))
        << QStringLiteral(" 右=")
        << (cxpExportMeta.right.saved ? cxpExportMeta.right.fileName : QStringLiteral("(skip)"));
}

void StateMachine::commitBypassScanSegmentCapture(
    int pathId,
    int segmentIndex,
    int capturedPointCount,
    const scan_tracking::vision::MultiCameraCaptureBundle& bundle)
{
    if (pathId != m_currentPathId) {
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("[多路径] 切换路径")
            << QStringLiteral(" 段号=") << segmentIndex
            << QStringLiteral(" 旧路径ID=") << m_currentPathId
            << QStringLiteral(" 新路径ID=") << pathId;
        m_currentPathId = pathId;
        m_currentPathSegments.clear();
    }

    m_currentPathSegments.insert(segmentIndex);

    {
        std::lock_guard<std::mutex> lock(m_segmentCacheMutex);
        m_pathSegmentCaptureResults[pathId].remove(segmentIndex);
        m_pathSegmentCaptureBundles[pathId].remove(segmentIndex);
    }

    int imageCount = countHikImagesInBundle(bundle);
    if (imageCount == 0) {
        imageCount = 1;
    }
    const int cloudFrameCount = capturedPointCount > 0 ? 1 : 0;

    writeScanSegmentResult(segmentIndex, imageCount, cloudFrameCount);
    m_progress = 100;
    publishIpcStatus();

    exportSegmentCxp2dImages(segmentIndex, bundle);

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("[算法旁路] 采集完成，点云已丢弃不入缓存")
        << QStringLiteral(" [路径") << pathId << QStringLiteral("][段") << segmentIndex << QStringLiteral("]")
        << QStringLiteral(" 任务ID=") << m_activeTask.taskId
        << QStringLiteral(" 丢弃点数=") << capturedPointCount
        << QStringLiteral(" 2D帧数=") << imageCount;

    completeActiveTask(1);
    emit scanFinished(segmentIndex, 1, imageCount, cloudFrameCount);

    maybeLatchFirstPathStepPause(pathId, segmentIndex);
}

void StateMachine::registerRefinementJob()
{
    const int previous = m_pendingRefinementJobs.fetch_add(1, std::memory_order_acq_rel);
    if (previous + 1 > kMaxReasonableRefinementJobs) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("后台 refinement 在途数偏高 pending=") << (previous + 1);
    }
}

void StateMachine::completeRefinementJob()
{
    const int previous = m_pendingRefinementJobs.fetch_sub(1, std::memory_order_acq_rel);
    if (previous <= 0) {
        m_pendingRefinementJobs.fetch_add(1, std::memory_order_relaxed);
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("后台 refinement 完成时计数已为 0（重复收尾）");
    }
}

int StateMachine::pendingRefinementJobCount() const
{
    const int count = m_pendingRefinementJobs.load(std::memory_order_acquire);
    if (count < 0 || count > kMaxReasonableRefinementJobs) {
        return -1;
    }
    return count;
}

int StateMachine::reconcilePendingRefinementJobCounter(const char* reason)
{
    const int pending = m_pendingRefinementJobs.exchange(0, std::memory_order_acq_rel);
    if (pending == 0) {
        return 0;
    }

    qCritical(LOG_FLOW).noquote()
        << QStringLiteral("后台 refinement 在途计数已强制复位 pending=") << pending
        << QStringLiteral(" 原因=") << (reason != nullptr ? QString::fromUtf8(reason) : QString())
        << QStringLiteral("（常见根因：旧版 dispatch 对已 move 的 outcome 二次 invoke；"
                           "或 register/complete 未成对；将用原始点云继续综合检测）");
    return pending;
}

void StateMachine::joinAllBackgroundRefinementJobs(int maxWaitMs)
{
    const int waitLimitMs = maxWaitMs >= 0
        ? maxWaitMs
        : kBackgroundRefinementJoinTimeoutMs;

    int remaining = pendingRefinementJobCount();
    if (remaining < 0) {
        reconcilePendingRefinementJobCounter("join 入口计数异常");
        return;
    }
    if (remaining <= 0) {
        return;
    }

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("等待后台点云 refinement 结束，剩余任务数=") << remaining
        << QStringLiteral(" 最长等待ms=") << waitLimitMs;

    int waitedMs = 0;
    while (remaining > 0 && waitedMs < waitLimitMs) {
        // 先 pump 事件，确保 QueuedConnection 的 refinement 收尾能执行 completeRefinementJob
        if (QCoreApplication::instance() != nullptr) {
            QCoreApplication::processEvents(QEventLoop::AllEvents, 50);
        }
        QThread::msleep(50);
        waitedMs += 50;
        remaining = pendingRefinementJobCount();
        if (remaining < 0) {
            reconcilePendingRefinementJobCounter("join 轮询计数异常");
            return;
        }
    }

    if (remaining > 0) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("后台点云 refinement 未在时限内结束，剩余=") << remaining
            << QStringLiteral("；综合检测将使用当前缓存（可能含未 refine 的原始点云）");
        reconcilePendingRefinementJobCounter("join 超时仍有在途任务");
    }
}

void StateMachine::dispatchSegmentRefinementFinished(SegmentProcessOutcome outcome)
{
    // 使用 shared_ptr 避免 invokeMethod 失败时对已 move 的 outcome 二次 move（UB，可破坏 m_pendingRefinementJobs）
    const auto outcomeHolder = std::make_shared<SegmentProcessOutcome>(std::move(outcome));
    QPointer<StateMachine> self(this);

    const auto deliverOnMainThread = [self, outcomeHolder]() {
        if (!self || self->m_stopped.load(std::memory_order_acquire)) {
            qWarning(LOG_FLOW).noquote()
                << QStringLiteral("后台 refinement 回调时 StateMachine 已销毁，在途计数将在 stop/reset 时复位");
            return;
        }
        self->onSegmentBackgroundRefinementFinished(std::move(*outcomeHolder));
    };

    if (QThread::currentThread() == thread()) {
        deliverOnMainThread();
        return;
    }

    if (QCoreApplication::instance() == nullptr) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("后台 refinement 无法投递到主线程（无 QCoreApplication），仅平衡在途计数");
        completeRefinementJob();
        return;
    }

    if (QMetaObject::invokeMethod(this, deliverOnMainThread, Qt::QueuedConnection)) {
        return;
    }

    qWarning(LOG_FLOW).noquote()
        << QStringLiteral("后台 refinement QueuedConnection 失败，改 BlockingQueuedConnection 段号=")
        << outcomeHolder->segmentIndex;
    QMetaObject::invokeMethod(this, deliverOnMainThread, Qt::BlockingQueuedConnection);
}

void StateMachine::startSegmentBackgroundRefinement(
    int pathId,
    int segmentIndex,
    quint32 taskId,
    const scan_tracking::common::PointCloudProcessingConfig& config)
{
    if (!config.enabled) {
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("[ScanSync] 点云后处理已禁用，跳过后台 refinement，路径=") << pathId
            << QStringLiteral(" 段号=") << segmentIndex;
        return;
    }

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("[ScanSync] 后台 refinement 已排队，路径=") << pathId
        << QStringLiteral(" 段号=") << segmentIndex
        << QStringLiteral(" 任务ID=") << taskId;

    QPointer<StateMachine> self(this);
    const qint64 refinementQueuedAtMs = QDateTime::currentMSecsSinceEpoch();
    std::thread([self, pathId, segmentIndex, taskId, config, refinementQueuedAtMs]() {
        QElapsedTimer refinementWallTimer;
        refinementWallTimer.start();

        // 多段 400 万级点云并行 refinement 会占满内存；与 PCL 全局锁一致，进程内串行执行。
        static std::mutex kSegmentRefinementSerialMutex;
        std::lock_guard<std::mutex> serialGuard(kSegmentRefinementSerialMutex);
        const qint64 queueWaitMs = refinementWallTimer.elapsed();

        if (!self || self->m_stopped.load(std::memory_order_acquire)) {
            return;
        }

        self->registerRefinementJob();

        struct RefinementPendingGuard {
            QPointer<StateMachine> owner;
            bool handedOff = false;
            void handOff() { handedOff = true; }
            ~RefinementPendingGuard()
            {
                if (!handedOff && owner) {
                    owner->completeRefinementJob();
                }
            }
        } pendingGuard{self};

        SegmentProcessOutcome outcome;
        outcome.pathId = pathId;
        outcome.segmentIndex = segmentIndex;
        outcome.taskId = taskId;

        const auto logRefinementTiming = [&](const SegmentProcessOutcome& finished) {
            qInfo(LOG_FLOW).noquote()
                << QStringLiteral("[RefinementTimer] Mech点云后台refinement")
                << QStringLiteral(" 路径=") << finished.pathId
                << QStringLiteral(" 段号=") << finished.segmentIndex
                << QStringLiteral(" 任务ID=") << finished.taskId
                << QStringLiteral(" 成功=") << (finished.success ? QStringLiteral("是") : QStringLiteral("否"))
                << QStringLiteral(" 总耗时ms=") << finished.processElapsedMs
                << QStringLiteral(" (排队等待ms=") << queueWaitMs
                << QStringLiteral(" PCL多步处理ms=") << finished.pclProcessElapsedMs
                << QStringLiteral(" 点数=") << finished.rawPointCount
                << QStringLiteral("->") << finished.processedPointCount
                << QStringLiteral(" 入队时刻ms=") << refinementQueuedAtMs;
        };

        const auto finishJob = [&](SegmentProcessOutcome finishedOutcome) {
            finishedOutcome.processElapsedMs = refinementWallTimer.elapsed();
            logRefinementTiming(finishedOutcome);
            if (self) {
                pendingGuard.handOff();
                self->dispatchSegmentRefinementFinished(std::move(finishedOutcome));
            } else {
                qWarning(LOG_FLOW).noquote()
                    << QStringLiteral("后台 refinement 完成时 StateMachine 已销毁，段号=") << segmentIndex;
            }
        };

        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("[ScanSync] 后台 refinement 开始，路径=") << pathId
            << QStringLiteral(" 段号=") << segmentIndex
            << QStringLiteral(" 任务ID=") << taskId
            << QStringLiteral(" 排队等待ms=") << queueWaitMs;

        try {
            if (!self) {
                qWarning(LOG_FLOW).noquote()
                    << QStringLiteral("后台 refinement 启动时 StateMachine 已销毁，路径=") << pathId
                    << QStringLiteral(" 段号=") << segmentIndex;
                return;
            }

            scan_tracking::mech_eye::PointCloudFrame inputCloud;
            {
                std::lock_guard<std::mutex> lock(self->m_segmentCacheMutex);
                const auto pathIt = self->m_pathSegmentCaptureResults.constFind(pathId);
                if (pathIt == self->m_pathSegmentCaptureResults.cend()) {
                    outcome.success = false;
                    outcome.errorMessage =
                        QStringLiteral("后台 refinement 时路径 %1 不存在。").arg(pathId);
                } else {
                    const auto segIt = pathIt->constFind(segmentIndex);
                    if (segIt == pathIt->cend() || !segIt->pointCloud.isValid()) {
                        outcome.success = false;
                        outcome.errorMessage =
                            QStringLiteral("后台 refinement 时路径 %1 段 %2 不存在或点云无效。")
                                .arg(pathId)
                                .arg(segmentIndex);
                    } else {
                        inputCloud = clonePointCloudFrame(segIt->pointCloud);
                        outcome.rawPointCount = inputCloud.pointCount;
                    }
                }
            }

            if (!outcome.errorMessage.isEmpty()) {
                finishJob(std::move(outcome));
                return;
            }

            QElapsedTimer pclTimer;
            pclTimer.start();

            scan_tracking::mech_eye::PointCloudProcessReport report;
            scan_tracking::mech_eye::PointCloudFrame processedCloud;
            scan_tracking::mech_eye::CaptureResult captureInput;
            captureInput.pointCloud = std::move(inputCloud);

            if (scan_tracking::mech_eye::processPointCloudFrame(
                    captureInput.pointCloud,
                    config,
                    &processedCloud,
                    &report)) {
                outcome.success = true;
                outcome.captureResult.pointCloud = std::move(processedCloud);
                outcome.processedPointCount = report.outputPointCount;
                outcome.rawPointCount = report.inputPointCount;
            } else {
                outcome.success = false;
                outcome.errorMessage = report.message.isEmpty()
                    ? QStringLiteral("后台点云 refinement 失败。")
                    : report.message;
                outcome.rawPointCount = report.inputPointCount;
            }
            outcome.pclProcessElapsedMs = pclTimer.elapsed();
            finishJob(std::move(outcome));
        } catch (const std::exception& ex) {
            qCritical(LOG_FLOW).noquote()
                << QStringLiteral("后台 refinement 异常 段号=") << segmentIndex
                << QStringLiteral(" 说明=") << QString::fromUtf8(ex.what());
            outcome.success = false;
            outcome.errorMessage = QStringLiteral("后台 refinement 异常：%1")
                                       .arg(QString::fromUtf8(ex.what()));
            finishJob(std::move(outcome));
        } catch (...) {
            qCritical(LOG_FLOW).noquote()
                << QStringLiteral("后台 refinement 未知异常 段号=") << segmentIndex;
            outcome.success = false;
            outcome.errorMessage = QStringLiteral("后台 refinement 未知异常。");
            finishJob(std::move(outcome));
        }
    }).detach();
}

void StateMachine::onSegmentBackgroundRefinementFinished(SegmentProcessOutcome outcome)
{
    completeRefinementJob();

    if (!outcome.success) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("[ScanSync] 后台 refinement 失败，保留原始缓存")
            << QStringLiteral(" 段号=") << outcome.segmentIndex
            << QStringLiteral(" 说明=") << outcome.errorMessage;
        return;
    }

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("[ScanSync] 后台 refinement 完成")
        << QStringLiteral(" 段号=") << outcome.segmentIndex
        << QStringLiteral(" 任务ID=") << outcome.taskId
        << QStringLiteral(" 总耗时ms=") << outcome.processElapsedMs
        << QStringLiteral(" PCL处理ms=") << outcome.pclProcessElapsedMs
        << QStringLiteral(" 点数=") << outcome.rawPointCount << QStringLiteral("->") << outcome.processedPointCount;

    applySegmentRefinementOutcome(outcome);
}

void StateMachine::applySegmentRefinementOutcome(const SegmentProcessOutcome& outcome)
{
    std::lock_guard<std::mutex> lock(m_segmentCacheMutex);

    const int targetPathId = outcome.pathId > 0 ? outcome.pathId : m_currentPathId;
    if (!m_pathSegmentCaptureResults.contains(targetPathId) ||
        !m_pathSegmentCaptureResults[targetPathId].contains(outcome.segmentIndex)) {
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("忽略 refinement 结果：缓存不存在，路径=") << targetPathId
            << QStringLiteral(" 段号=") << outcome.segmentIndex;
        return;
    }

    assignSharedPointCloudToSegmentEntry(
        m_pathSegmentCaptureResults,
        m_pathSegmentCaptureBundles,
        targetPathId,
        outcome.segmentIndex,
        outcome.captureResult.pointCloud);

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("[SegmentCache] refinement 已写回内存")
        << QStringLiteral(" [路径") << targetPathId << QStringLiteral("][段") << outcome.segmentIndex << QStringLiteral("]")
        << QStringLiteral(" 点数=") << outcome.processedPointCount;

    applySegmentPoseStitching(targetPathId, outcome.segmentIndex);
}

/**
 * @brief 执行综合检测任务（Trig_Inspection）
 * 
 * 调用跟踪检测服务对之前采集的所有分段点云进行综合分析，
 * 计算工件的偏移量和检测结果，并将结果写入 PLC 寄存器。
 * 
 * 关键步骤：
 * 1. 检查跟踪服务是否可用
 * 2. 调用 inspectPointCloud 进行坡口测量
 * 3. 将检测结果写入 PLC
 * 4. 根据检测结果决定任务成功或失败
 * 5. 清空点云缓存（检测完成后不再需要原始点云）
 */
void StateMachine::setInspectionResultPublisher(
    std::function<void(const tracking::InspectionResult&)> publisher)
{
    m_inspectionResultPublisher = std::move(publisher);
}

QVector<int> StateMachine::cachedScanSegmentIndices() const
{
    QVector<int> indices;
    
    // === 多路径支持：编码为 路径ID*100 + 段号，便于 HMI 显示 ===
    std::lock_guard<std::mutex> lock(m_segmentCacheMutex);
    for (auto pathIt = m_pathSegmentCaptureResults.constBegin(); 
         pathIt != m_pathSegmentCaptureResults.constEnd(); ++pathIt) {
        const int pathId = pathIt.key();
        const auto& segments = pathIt.value();
        
        for (auto segIt = segments.constBegin(); segIt != segments.constEnd(); ++segIt) {
            const int segmentIndex = segIt.key();
            // 编码：路径1段1 → 101，路径2段2 → 202
            indices.push_back(pathId * 100 + segmentIndex);
        }
    }
    
    std::sort(indices.begin(), indices.end());
    return indices;
}

bool StateMachine::isCurrentPathSegmentSetComplete() const
{
    const int scanSegmentTotal = segmentTotalForPath(m_currentPathId);
    if (scanSegmentTotal <= 0) {
        return false;
    }

    for (int segmentIndex = 1; segmentIndex <= scanSegmentTotal; ++segmentIndex) {
        if (!m_currentPathSegments.contains(segmentIndex)) {
            return false;
        }
    }
    return true;
}

bool StateMachine::isPathScanComplete(int pathId) const
{
    if (pathId <= 0) {
        return false;
    }

    const int scanSegmentTotal = segmentTotalForPath(pathId);
    if (scanSegmentTotal <= 0) {
        return false;
    }

    std::lock_guard<std::mutex> lock(m_segmentCacheMutex);
    if (!m_pathSegmentCaptureResults.contains(pathId)) {
        return false;
    }
    const auto& pathSegments = m_pathSegmentCaptureResults[pathId];
    for (int segmentIndex = 1; segmentIndex <= scanSegmentTotal; ++segmentIndex) {
        const auto segIt = pathSegments.constFind(segmentIndex);
        if (segIt == pathSegments.cend() || !segIt->pointCloud.isValid()) {
            return false;
        }
    }
    return true;
}

bool StateMachine::hasPathReadyForInspection() const
{
    return resolvePathIdForInspection() > 0;
}

int StateMachine::resolvePathIdForInspection() const
{
    const QVector<int> pathIds = enabledScanPathIds();
    int latestCompletePathId = -1;
    for (int pathId : pathIds) {
        if (isPathScanComplete(pathId)) {
            latestCompletePathId = pathId;
        }
    }
    return latestCompletePathId;
}

bool StateMachine::hasAllScanSegmentsCached() const
{
    const QVector<int> pathIds = enabledScanPathIds();
    if (pathIds.isEmpty()) {
        return false;
    }

    std::lock_guard<std::mutex> lock(m_segmentCacheMutex);
    for (int pathId : pathIds) {
        const int scanSegmentTotal = segmentTotalForPath(pathId);
        if (scanSegmentTotal <= 0) {
            return false;
        }
        if (!m_pathSegmentCaptureResults.contains(pathId)) {
            return false;
        }
        const auto& pathSegments = m_pathSegmentCaptureResults[pathId];
        for (int segmentIndex = 1; segmentIndex <= scanSegmentTotal; ++segmentIndex) {
            const auto segIt = pathSegments.constFind(segmentIndex);
            if (segIt == pathSegments.cend() || !segIt->pointCloud.isValid()) {
                return false;
            }
        }
    }
    return true;
}

QString StateMachine::multiPathCacheStatusText() const
{
    const QVector<int> pathIds = enabledScanPathIds();
    int expectedTotal = 0;
    for (int pathId : pathIds) {
        expectedTotal += segmentTotalForPath(pathId);
    }
    const int cachedTotal = totalCachedPointCloudCount();

    QStringList pathParts;
    {
        std::lock_guard<std::mutex> lock(m_segmentCacheMutex);
        for (int pathId : pathIds) {
            const int count =
                m_pathSegmentCaptureResults.contains(pathId)
                    ? m_pathSegmentCaptureResults[pathId].size()
                    : 0;
            pathParts << QStringLiteral("路径%1=%2段").arg(pathId).arg(count);
        }
    }

    return QStringLiteral("已缓存 %1/%2 段 [%3]，当前路径ID=%4")
        .arg(cachedTotal)
        .arg(expectedTotal)
        .arg(pathParts.join(QLatin1Char(',')))
        .arg(m_currentPathId);
}

int StateMachine::firstEnabledScanPathId() const
{
    const QVector<int> pathIds = enabledScanPathIds();
    if (pathIds.isEmpty()) {
        return 1;
    }
    return pathIds.front();
}

int StateMachine::configuredFirstPathPauseAfterPoint() const
{
    const auto* configMgr = scan_tracking::common::ConfigManager::instance();
    if (configMgr == nullptr) {
        return 0;
    }
    return qMax(0, configMgr->flowControlConfig().firstPathPauseAfterPoint);
}

void StateMachine::clearFirstPathStepPauseLatch()
{
    m_firstPathStepPauseLatched = false;
    m_firstPathStepPauseAtSegment = 0;
}

bool StateMachine::isFirstPathStepPauseBlocking(QString* errorMessage) const
{
    if (!m_firstPathStepPauseLatched) {
        return false;
    }

    if (errorMessage != nullptr) {
        *errorMessage = QStringLiteral(
            "路径1联调暂停：已完成第 %1 点（CXP+梅卡已落盘）。"
            "请将 config.ini [FlowControl] firstPathPauseAfterPoint 改为 0 跑完全流程，"
            "或改为更大值后重启 IPC 再拍下一点；也可 Trig_ResultReset 后重扫。")
                            .arg(m_firstPathStepPauseAtSegment);
    }
    return true;
}

void StateMachine::maybeLatchFirstPathStepPause(int pathId, int segmentIndex)
{
    const int pauseAfterPoint = configuredFirstPathPauseAfterPoint();
    if (pauseAfterPoint <= 0) {
        return;
    }
    if (pathId != firstEnabledScanPathId() || segmentIndex != pauseAfterPoint) {
        return;
    }

    m_firstPathStepPauseLatched = true;
    m_firstPathStepPauseAtSegment = segmentIndex;

    qWarning(LOG_FLOW).noquote()
        << QStringLiteral("[路径1联调] 已完成第") << segmentIndex
        << QStringLiteral("点采集与 CXP 落盘，流程暂停。")
        << QStringLiteral("后续 Trig_ScanSegment 将被拒绝，直至 firstPathPauseAfterPoint=0")
        << QStringLiteral("或调大后重启 IPC，或执行 Trig_ResultReset。");
    publishIpcStatus();
}

int StateMachine::resolvePathIdForIncomingSegment(int segmentIndex) const
{
    if (m_selfCheckSessionActive) {
        return kSelfCheckCacheBucketId;
    }

    const QVector<int> pathIds = enabledScanPathIds();
    if (pathIds.isEmpty()) {
        return m_currentPathId > 0 ? m_currentPathId : 1;
    }

    if (!m_currentPathSegments.contains(segmentIndex)) {
        return m_currentPathId > 0 ? m_currentPathId : pathIds.front();
    }

    if (!isCurrentPathSegmentSetComplete()) {
        return m_currentPathId;
    }

    const int currentIndex = pathIds.indexOf(m_currentPathId);
    if (currentIndex >= 0 && currentIndex + 1 < pathIds.size()) {
        return pathIds[currentIndex + 1];
    }
    return m_currentPathId;
}

int StateMachine::totalCachedPointCloudCount() const
{
    std::lock_guard<std::mutex> lock(m_segmentCacheMutex);
    int total = 0;
    for (auto pathIt = m_pathSegmentCaptureResults.constBegin(); 
         pathIt != m_pathSegmentCaptureResults.constEnd(); ++pathIt) {
        total += pathIt->size();
    }
    return total;
}

bool StateMachine::hasSegmentInPath(int pathId, int segmentIndex) const
{
    std::lock_guard<std::mutex> lock(m_segmentCacheMutex);
    return m_pathSegmentCaptureResults.contains(pathId) &&
           m_pathSegmentCaptureResults[pathId].contains(segmentIndex) &&
           m_pathSegmentCaptureResults[pathId][segmentIndex].pointCloud.isValid();
}

int StateMachine::selfCheckCachePathId() const
{
    return kSelfCheckCacheBucketId;
}

void StateMachine::clearPathSegmentCache(int pathId)
{
    std::lock_guard<std::mutex> lock(m_segmentCacheMutex);
    if (m_pathSegmentCaptureBundles.contains(pathId)) {
        for (auto segIt = m_pathSegmentCaptureBundles[pathId].begin();
             segIt != m_pathSegmentCaptureBundles[pathId].end(); ++segIt) {
            scan_tracking::vision::releaseHikMonoFrameBuffers(&segIt->hikCameraAResult.frame);
            scan_tracking::vision::releaseHikMonoFrameBuffers(&segIt->hikCameraBResult.frame);
            scan_tracking::mech_eye::releasePointCloudFrameBuffers(&segIt->mechEyeResult.pointCloud);
        }
        m_pathSegmentCaptureBundles.remove(pathId);
    }
    if (m_pathSegmentCaptureResults.contains(pathId)) {
        for (auto segIt = m_pathSegmentCaptureResults[pathId].begin();
             segIt != m_pathSegmentCaptureResults[pathId].end(); ++segIt) {
            scan_tracking::mech_eye::releasePointCloudFrameBuffers(&segIt->pointCloud);
        }
        m_pathSegmentCaptureResults.remove(pathId);
    }
    m_pathSegmentCalibrationMatrices.remove(pathId);
    m_pathSegmentPoseStitchRecords.remove(pathId);
    if (m_currentPathId == pathId) {
        m_currentPathSegments.clear();
    }
}

bool StateMachine::hasSelfCheckCaptureReady() const
{
    const int pathId = kSelfCheckCacheBucketId;
    const auto* configMgr = common::ConfigManager::instance();
    const int totalPoints = configMgr != nullptr && configMgr->selfCheckConfig().totalPoints > 0
        ? configMgr->selfCheckConfig().totalPoints
        : 2;

    std::lock_guard<std::mutex> lock(m_segmentCacheMutex);
    if (!m_pathSegmentCaptureBundles.contains(pathId)) {
        return false;
    }
    const auto& bundles = m_pathSegmentCaptureBundles[pathId];
    for (int segmentIndex = 1; segmentIndex <= totalPoints; ++segmentIndex) {
        if (!bundles.contains(segmentIndex)) {
            return false;
        }
    }
    return true;
}

void StateMachine::beginSelfCheckScanSession()
{
    clearPathSegmentCache(kSelfCheckCacheBucketId);
    m_selfCheckSessionActive = true;
    m_currentPathId = kSelfCheckCacheBucketId;
    m_currentPathSegments.clear();
    m_currentStage = protocol::Stage::SelfCheck;
    m_ipcState = protocol::IpcState::Ready;
    publishIpcStatus();

    const int totalPoints = common::ConfigManager::instance()
        ? common::ConfigManager::instance()->selfCheckConfig().totalPoints
        : 2;
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("[SelfCheck] 显控已请求自检，IPC 已置阶段=SelfCheck 通知 PLC；")
        << QStringLiteral("待 PLC 调度") << totalPoints
        << QStringLiteral(" 次 Trig_ScanSegment 后下发 Trig_SelfCheck");
}

void StateMachine::endSelfCheckScanSession()
{
    if (!m_selfCheckSessionActive) {
        return;
    }
    m_selfCheckSessionActive = false;
    qInfo(LOG_FLOW).noquote() << QStringLiteral("[SelfCheck] 自检扫描会话已结束");
}

const protocol::TriggerDefinition* StateMachine::selectPendingTrigger(
    const QVector<quint16>& commandBlock) const
{
    namespace regs = protocol::registers;
    const int scanTrigOffset = regs::modbusIndexFromPlcAddress(40023);
    const int inspectionTrigOffset = regs::modbusIndexFromPlcAddress(40024);
    const bool scanPending =
        scanTrigOffset < commandBlock.size() && commandBlock[scanTrigOffset] == 1;
    const bool inspectionPending =
        inspectionTrigOffset < commandBlock.size() && commandBlock[inspectionTrigOffset] == 1;

    if (scanPending && inspectionPending) {
        const int inspectPathId = resolvePathIdForInspection();
        if (inspectPathId > 0) {
            const int segmentIndex =
                protocol::registers::resolveScanSegmentIndexFromBlock(commandBlock);
            const int upcomingPathId = resolvePathIdForIncomingSegment(segmentIndex);
            if (upcomingPathId > inspectPathId) {
                qInfo(LOG_FLOW).noquote()
                    << QStringLiteral("[多路径] 路径") << inspectPathId
                    << QStringLiteral(" 已扫满，Trig_ScanSegment 将开始路径") << upcomingPathId
                    << QStringLiteral("，优先执行本路径综合检测");
                return protocol::triggerByOffset(inspectionTrigOffset);
            }
        }
        return protocol::triggerByOffset(scanTrigOffset);
    }

    for (const auto& trigger : protocol::triggerDefinitions()) {
        if (trigger.trigOffset < commandBlock.size() && commandBlock[trigger.trigOffset] == 1) {
            return &trigger;
        }
    }
    return nullptr;
}

bool StateMachine::loadMergedPointCloudForInspection(
    scan_tracking::mech_eye::PointCloudFrame* outCloud,
    int* totalPointCount,
    int* segmentCount,
    QString* errorMessage)
{
    if (outCloud == nullptr) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("综合检测失败：输出点云指针为空。");
        }
        return false;
    }

    int maxJoinMs = kBackgroundRefinementJoinTimeoutMs;
    if (m_activeTask.definition != nullptr &&
        m_activeTask.definition->stage == protocol::Stage::Inspection &&
        m_activeTask.timeoutSeconds > 0) {
        maxJoinMs = std::min(
            kBackgroundRefinementJoinTimeoutMs,
            std::max(5000, static_cast<int>(m_activeTask.timeoutSeconds) * 1000 - 5000));
    }
    joinAllBackgroundRefinementJobs(maxJoinMs);

    int pendingAfterJoin = pendingRefinementJobCount();
    if (pendingAfterJoin < 0) {
        reconcilePendingRefinementJobCounter("综合检测前计数异常");
        pendingAfterJoin = 0;
    } else if (pendingAfterJoin > 0) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("综合检测 join 超时后仍有 refinement 在途=") << pendingAfterJoin
            << QStringLiteral("，已强制复位；将使用当前缓存点云（可能为未 refine 的原始点云）");
        reconcilePendingRefinementJobCounter("综合检测前 join 未清空");
        pendingAfterJoin = 0;
    }

    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    if (configManager == nullptr) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("综合检测失败：ConfigManager 不可用。");
        }
        return false;
    }

    int inspectPathId = m_activeTask.inspectionPathId;
    if (inspectPathId <= 0) {
        inspectPathId = resolvePathIdForInspection();
    }
    if (inspectPathId <= 0) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("综合检测失败：没有已扫满的可检测路径。");
        }
        return false;
    }

    auto mergedPoints = std::make_shared<std::vector<float>>();
    int mergedPointCount = 0;
    int mergedSegmentCount = 0;

    {
        std::lock_guard<std::mutex> lock(m_segmentCacheMutex);

        if (!m_pathSegmentCaptureResults.contains(inspectPathId)) {
            if (errorMessage != nullptr) {
                *errorMessage = QStringLiteral(
                    "综合检测失败：路径 %1 不存在（尚未扫描）。").arg(inspectPathId);
            }
            return false;
        }

        const auto& pathSegments = m_pathSegmentCaptureResults[inspectPathId];
        QList<int> segmentIndices = pathSegments.keys();
        std::sort(segmentIndices.begin(), segmentIndices.end());

        for (int segmentIndex : segmentIndices) {
            const auto& captureResult = pathSegments[segmentIndex];
            if (!captureResult.success() || !captureResult.pointCloud.isValid()) {
                continue;
            }

            const auto& cloud = captureResult.pointCloud;
            if (!cloud.pointsXYZ || cloud.pointCount <= 0) {
                continue;
            }

            const int availablePointCount =
                static_cast<int>(cloud.pointsXYZ->size() / 3);
            const int pointCount = std::min(cloud.pointCount, availablePointCount);
            if (pointCount <= 0) {
                continue;
            }

            mergedPoints->reserve(
                mergedPoints->size() + static_cast<std::size_t>(pointCount * 3));
            for (int index = 0; index < pointCount; ++index) {
                const auto base = static_cast<std::size_t>(index * 3);
                mergedPoints->push_back((*cloud.pointsXYZ)[base]);
                mergedPoints->push_back((*cloud.pointsXYZ)[base + 1]);
                mergedPoints->push_back((*cloud.pointsXYZ)[base + 2]);
            }

            mergedPointCount += pointCount;
            ++mergedSegmentCount;
        }
    }

    if (mergedPointCount <= 0 || mergedSegmentCount <= 0) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral(
                "综合检测失败：路径 %1 没有可用点云段。").arg(inspectPathId);
        }
        return false;
    }

    scan_tracking::mech_eye::PointCloudFrame mergedCloud;
    mergedCloud.pointsXYZ = std::move(mergedPoints);
    mergedCloud.pointCount = mergedPointCount;
    mergedCloud.width = mergedPointCount;
    mergedCloud.height = 1;

    if (m_poseStitchRunRootDirectory.isEmpty()) {
        ensurePoseStitchRunRootDirectory();
    }
    persistMergedInspectionPointCloudToDisk(inspectPathId, mergedSegmentCount, mergedCloud);

    *outCloud = std::move(mergedCloud);
    if (totalPointCount != nullptr) {
        *totalPointCount = mergedPointCount;
    }
    if (segmentCount != nullptr) {
        *segmentCount = mergedSegmentCount;
    }

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("[多路径] 路径级点云合并 pathId=") << inspectPathId
        << QStringLiteral(" 参与段数=") << mergedSegmentCount
        << QStringLiteral(" 总点数=") << mergedPointCount;

    return true;
}

bool StateMachine::loadThicknessPointCloudsForInspection(
    scan_tracking::mech_eye::PointCloudFrame* outInnerCloud,
    scan_tracking::mech_eye::PointCloudFrame* outOuterCloud,
    int* innerPointCount,
    int* outerPointCount,
    int* innerSegmentIndex,
    int* outerSegmentIndex,
    QString* errorMessage)
{
    if (outInnerCloud == nullptr || outOuterCloud == nullptr) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("厚度检测失败：输出点云指针为空。");
        }
        return false;
    }

    int maxJoinMs = kBackgroundRefinementJoinTimeoutMs;
    if (m_activeTask.definition != nullptr &&
        m_activeTask.definition->stage == protocol::Stage::Inspection &&
        m_activeTask.timeoutSeconds > 0) {
        maxJoinMs = std::min(
            kBackgroundRefinementJoinTimeoutMs,
            std::max(5000, static_cast<int>(m_activeTask.timeoutSeconds) * 1000 - 5000));
    }
    joinAllBackgroundRefinementJobs(maxJoinMs);

    int pendingAfterJoin = pendingRefinementJobCount();
    if (pendingAfterJoin < 0) {
        reconcilePendingRefinementJobCounter("厚度检测前计数异常");
        pendingAfterJoin = 0;
    } else if (pendingAfterJoin > 0) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("厚度检测 join 超时后仍有 refinement 在途=") << pendingAfterJoin
            << QStringLiteral("，将使用当前缓存点云");
        reconcilePendingRefinementJobCounter("厚度检测前 join 未清空");
    }

    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    if (configManager == nullptr) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("厚度检测失败：ConfigManager 不可用。");
        }
        return false;
    }

    int inspectPathId = m_activeTask.inspectionPathId;
    if (inspectPathId <= 0) {
        inspectPathId = resolvePathIdForInspection();
    }
    if (inspectPathId <= 0) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("厚度检测失败：没有已扫满的可检测路径。");
        }
        return false;
    }

    const int innerSeg = configManager->innerScanSegmentIndexForPath(inspectPathId);
    const int outerSeg = configManager->outerScanSegmentIndexForPath(inspectPathId);
    if (innerSeg <= 0 || outerSeg <= 0) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral(
                "厚度检测失败：路径 %1 未配置 innerScanSegmentIndex/outerScanSegmentIndex。")
                                 .arg(inspectPathId);
        }
        return false;
    }

    scan_tracking::mech_eye::PointCloudFrame innerCloud;
    scan_tracking::mech_eye::PointCloudFrame outerCloud;
    int innerCount = 0;
    int outerCount = 0;

    {
        std::lock_guard<std::mutex> lock(m_segmentCacheMutex);

        if (!m_pathSegmentCaptureResults.contains(inspectPathId)) {
            if (errorMessage != nullptr) {
                *errorMessage = QStringLiteral(
                    "厚度检测失败：路径 %1 不存在（尚未扫描）。").arg(inspectPathId);
            }
            return false;
        }

        const auto& pathSegments = m_pathSegmentCaptureResults[inspectPathId];
        const auto loadSegment = [&](int segmentIndex, scan_tracking::mech_eye::PointCloudFrame* outCloud, int* outCount) -> bool {
            if (!pathSegments.contains(segmentIndex)) {
                return false;
            }
            const auto& captureResult = pathSegments[segmentIndex];
            if (!captureResult.success() || !captureResult.pointCloud.isValid()) {
                return false;
            }
            *outCloud = clonePointCloudFrame(captureResult.pointCloud);
            *outCount = outCloud->pointCount;
            return *outCount > 0;
        };

        if (!loadSegment(innerSeg, &innerCloud, &innerCount)) {
            if (errorMessage != nullptr) {
                *errorMessage = QStringLiteral(
                    "厚度检测失败：路径 %1 段 %2（inner）点云不可用。")
                                     .arg(inspectPathId)
                                     .arg(innerSeg);
            }
            return false;
        }
        if (!loadSegment(outerSeg, &outerCloud, &outerCount)) {
            if (errorMessage != nullptr) {
                *errorMessage = QStringLiteral(
                    "厚度检测失败：路径 %1 段 %2（outer）点云不可用。")
                                     .arg(inspectPathId)
                                     .arg(outerSeg);
            }
            return false;
        }
    }

    *outInnerCloud = std::move(innerCloud);
    *outOuterCloud = std::move(outerCloud);
    if (innerPointCount != nullptr) {
        *innerPointCount = innerCount;
    }
    if (outerPointCount != nullptr) {
        *outerPointCount = outerCount;
    }
    if (innerSegmentIndex != nullptr) {
        *innerSegmentIndex = innerSeg;
    }
    if (outerSegmentIndex != nullptr) {
        *outerSegmentIndex = outerSeg;
    }

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("[多路径] 厚度点云已加载 pathId=") << inspectPathId
        << QStringLiteral(" inner段=") << innerSeg << QStringLiteral(" 点数=") << innerCount
        << QStringLiteral(" outer段=") << outerSeg << QStringLiteral(" 点数=") << outerCount;

    return true;
}

tracking::InspectionResult StateMachine::runDebugInspectionOnCachedSegments() const
{
    tracking::InspectionResult failure;
    failure.resultCode = 2;

    if (m_tracking == nullptr) {
        failure.ngReasonWord0 = (1u << 4);
        failure.message = QStringLiteral("调试综合检测失败：Tracking 服务不可用。");
        return failure;
    }

    if (totalCachedPointCloudCount() == 0) {
        failure.ngReasonWord0 = (1u << 4);
        failure.message = QStringLiteral("调试综合检测失败：点云缓存为空，请先完成扫描分段采集。");
        return failure;
    }

    QString loadError;
    auto* mutableSelf = const_cast<StateMachine*>(this);

    int inspectPathId = m_activeTask.inspectionPathId;
    if (inspectPathId <= 0) {
        inspectPathId = mutableSelf->resolvePathIdForInspection();
    }

    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    const auto inspectionType = configManager != nullptr
        ? configManager->inspectionTypeForPath(inspectPathId)
        : scan_tracking::common::InspectionType::Bevel;

    if (inspectionType == scan_tracking::common::InspectionType::CodeRead) {
        return m_tracking->inspectCodeRead(inspectPathId, false);
    }

    if (inspectionType == scan_tracking::common::InspectionType::Defect) {
        return m_tracking->inspectSurfaceDefect(inspectPathId, false);
    }

    if (inspectionType == scan_tracking::common::InspectionType::Thickness) {
        scan_tracking::mech_eye::PointCloudFrame innerCloud;
        scan_tracking::mech_eye::PointCloudFrame outerCloud;
        int innerPointCount = 0;
        int outerPointCount = 0;
        int innerSegmentIndex = 0;
        int outerSegmentIndex = 0;
        if (!mutableSelf->loadThicknessPointCloudsForInspection(
                &innerCloud,
                &outerCloud,
                &innerPointCount,
                &outerPointCount,
                &innerSegmentIndex,
                &outerSegmentIndex,
                &loadError)) {
            failure.ngReasonWord0 = (1u << 4);
            failure.message = loadError.isEmpty()
                ? QStringLiteral("调试综合检测失败：无法加载厚度 inner/outer 点云。")
                : loadError;
            return failure;
        }

        return m_tracking->inspectThicknessPointClouds(
            innerCloud,
            outerCloud,
            innerPointCount,
            outerPointCount,
            inspectPathId,
            false);
    }

    scan_tracking::mech_eye::PointCloudFrame mergedCloud;
    int totalPointCount = 0;
    int segmentCount = 0;
    if (!mutableSelf->loadMergedPointCloudForInspection(
            &mergedCloud, &totalPointCount, &segmentCount, &loadError)) {
        failure.ngReasonWord0 = (1u << 4);
        failure.message = loadError.isEmpty()
            ? QStringLiteral("调试综合检测失败：无法加载合并点云。")
            : loadError;
        return failure;
    }

    return m_tracking->inspectPointCloud(
        mergedCloud, totalPointCount, inspectPathId, false);
}

/**
 * @brief 向 PLC 发送 ACK（应答）信号
 * 
 * 将当前的应答状态写入触发定义中指定的 ACK 寄存器地址。
 * ACK 状态包括：Idle(0)、Running(1)、Completed(2)、Failed(3)。
 * 
 * @param definition 触发定义，包含 ACK 寄存器的偏移地址
 * @param ackState 要写入的应答状态
 */
void StateMachine::sendAck(const protocol::TriggerDefinition& definition, protocol::AckState ackState)
{
    if (!m_modbus) {
        return;  // Modbus 不可用，无法发送
    }

    qDebug(LOG_FLOW).noquote() << QStringLiteral("写入 Ack") << static_cast<int>(ackState)
                               << QStringLiteral(" 至") << definition.ackOffset
                               << QStringLiteral(" 触发=") << protocol::triggerName(definition);
    const bool ackWritten = m_modbus->writeRegister(definition.ackOffset, static_cast<quint16>(ackState));
    if (!ackWritten) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("写入 Ack 状态失败");
    }
}

/**
 * @brief 向 PLC 发送 Res（结果）信号
 * 
 * 将任务执行的结果码写入触发定义中指定的 Res 寄存器地址。
 * 常见的结果码：1=成功，5=设备未就绪，6=超时，7=处理失败，9=参数错误。
 * 
 * @param definition 触发定义，包含 Res 寄存器的偏移地址
 * @param resultCode 要写入的结果码
 */
void StateMachine::sendRes(const protocol::TriggerDefinition& definition, quint16 resultCode)
{
    if (!m_modbus) {
        return;  // Modbus 不可用，无法发送
    }

    qDebug(LOG_FLOW).noquote() << QStringLiteral("写入 Res") << resultCode
                               << QStringLiteral(" 至") << definition.resOffset
                               << QStringLiteral(" 触发=") << protocol::triggerName(definition);
    const bool resWritten = m_modbus->writeRegister(definition.resOffset, resultCode);
    if (!resWritten) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("写入 Res 结果码失败");
    }
}

/**
 * @brief 发布 IPC 状态到 PLC
 * 
 * 将当前的 IPC 运行状态、报警信息、进度等写入一组连续的 Modbus 寄存器，
 * 供 PLC 实时监控 IPC 的运行情况。
 * 
 * 写入的寄存器包括：
 * - 心跳计数器
 * - IPC 状态（Ready/Busy/Fault）
 * - 当前阶段（Idle/ScanSegment/Inspection 等）
 * - 报警级别和代码
 * - 警告代码
 * - 系统就绪标志
 * - 数据有效标志
 * - 任务进度百分比
 * - 设备在线状态字
 * - 当前任务 ID（高16位和低16位）
 */
void StateMachine::resetPlcOutputRegisters()
{
    if (!m_modbus || !m_modbus->isConnected()) {
        qInfo(LOG_FLOW).noquote() << QStringLiteral("跳过程序退出寄存器复位：Modbus 未连接");
        return;
    }

    const bool cleared = m_modbus->resetIpcResultBlock();
    if (cleared) {
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("程序退出：IPC 结果区已清零（40101-40184，Ack/Res/状态/坐标等全部为 0）");
    } else {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("程序退出：IPC 结果区清零失败");
    }
}

void StateMachine::publishIpcStatus()
{
    if (!m_modbus || !m_modbus->isConnected()) {
        return;  // Modbus 未连接，无法发布状态
    }

    QVector<quint16> status = {
        m_heartbeatCounter,                                          // 0: 心跳计数器
        static_cast<quint16>(m_ipcState),                            // 1: IPC 状态
        static_cast<quint16>(m_currentStage),                        // 2: 当前阶段
        m_alarmLevel,                                                // 3: 报警级别
        m_alarmCode,                                                 // 4: 报警代码
        m_warnCode,                                                  // 5: 警告代码
        static_cast<quint16>(m_state == AppState::Ready ? 1 : 0),   // 6: 系统就绪标志
        static_cast<quint16>(m_dataValid ? 1 : 0),                  // 7: 数据有效标志
        m_progress,                                                  // 8: 任务进度（0-100）
        kDeviceOnlineWord0,                                          // 9: 设备在线状态字
        0,                                                           // 10: 保留
        0,                                                           // 11: 保留
        0,                                                           // 12: 保留
        static_cast<quint16>((m_activeTask.taskId >> 16) & 0xFFFFu), // 13: 任务 ID 高16位
        static_cast<quint16>(m_activeTask.taskId & 0xFFFFu),         // 14: 任务 ID 低16位
    };

    // 批量写入状态寄存器（从 kIpcHeartbeat 开始）
    const bool heartbeatWritten = m_modbus->writeRegisters(protocol::registers::kIpcHeartbeat, status);
    if (!heartbeatWritten) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("写入 IPC 心跳状态失败");
    }
}

/**
 * @brief 发布心跳信号
 * 
 * 递增心跳计数器并发布 IPC 状态，用于向 PLC 证明 IPC 仍在正常运行。
 * PLC 可以通过监控心跳计数器的变化来判断 IPC 是否死机或通信中断。
 */
void StateMachine::publishHeartbeat()
{
    if (!m_modbus || !m_modbus->isConnected()) {
        return;  // Modbus 未连接，无法发送心跳
    }

    ++m_heartbeatCounter;  // 递增心跳计数器
    publishIpcStatus();    // 发布包含新心跳计数的状态
}

/**
 * @brief 处理任务超时事件
 * 
 * 当任务执行时间超过设定的超时时间时触发此回调。
 * 设置超时报警，并根据任务类型采取不同的处理策略：
 * - 扫描分段任务：写入失败结果并标记为失败
 * - 其他任务：直接标记为完成但数据无效
 */
void StateMachine::onProcessTimeout()
{
    if (m_activeTask.definition == nullptr) {
        return;  // 没有活动任务，忽略超时
    }

    qWarning(LOG_FLOW).noquote() << QStringLiteral("任务超时：") << protocol::triggerName(*m_activeTask.definition);
    setAlarm(2, 610, QStringLiteral("任务超时"));  // 设置警告级别报警，代码 610
    m_activeTask.captureRequestId = 0;  // 清除采集请求 ID

    // P0修复：超时时清理已缓存的点云数据，防止内存泄漏
    if (m_activeTask.definition->stage == protocol::Stage::ScanSegment) {
        qWarning(LOG_FLOW) << QStringLiteral("任务超时，清空扫描分段内存缓存");
        resetScanSegmentCache();
    }

    // 根据任务类型采取不同的超时处理策略
    if (m_activeTask.definition->stage == protocol::Stage::ScanSegment) {
        // 扫描分段超时：写入空结果（0 图像数，0 点云帧数）
        writeScanSegmentResult(m_activeTask.scanSegmentIndex, 0, 0);
        completeActiveTask(6, protocol::AckState::Failed, false);  // Res=6 表示超时
        return;
    }
    if (m_activeTask.definition->stage == protocol::Stage::Inspection) {
        writeInspectionResult({});
        completeActiveTask(
            kInspectionResTimeoutNg,
            protocol::AckState::Failed,
            false);
        resetScanSegmentCache();
        return;
    }
    // 其他任务超时：直接完成，标记为失败
    completeActiveTask(6, protocol::AckState::Completed, false);
}

/**
 * @brief 处理相机采集完成的回调
 * 
 * 当 Mech-Eye 相机完成一次 3D 点云采集后，通过此回调接收结果。
 * 这是扫描分段任务的核心处理逻辑。
 * 
 * 处理流程：
 * 1. 验证回调是否对应当前的活动任务和请求 ID
 * 2. 检查采集是否成功以及点云数据是否有效
 * 3. 如果成功，将点云结果存入缓存，更新进度，完成任务
 * 4. 如果失败，记录错误信息并完成失败的任务
 * 
 * @param result 采集结果，包含点云数据、状态码、错误信息等
 */
void StateMachine::onCaptureFinished(mech_eye::CaptureResult result)
{
    if (m_visionPipeline != nullptr) {
        return;
    }

    // 只在扫描分段阶段且存在活动任务时才处理
    if (m_activeTask.definition == nullptr ||
        m_activeTask.definition->stage != protocol::Stage::ScanSegment) {
        return;
    }

    // 过滤掉过期的回调（任务已完成或请求 ID 不匹配）
    if (m_activeTask.completionAnnounced || result.requestId != m_activeTask.captureRequestId) {
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("忽略过期梅卡采集回调，段号=") << m_activeTask.scanSegmentIndex;
        return;
    }

    // 检查采集是否成功、点云数据是否有效
    if (!result.success() || !result.pointCloud.isValid()) {
        const QString failureMessage =
            result.errorMessage.isEmpty() ? QStringLiteral("采集失败") : result.errorMessage;
        finishScanSegmentFailure(
            mapCaptureErrorToResCode(result.errorCode),  // 映射错误码到 Res 码
            2,                                           // 报警级别：2 = 警告
            722,                                         // 报警代码：722 = 采集失败
            QStringLiteral("Trig_ScanSegment 采集失败：%1").arg(failureMessage),
            failureMessage);
        return;
    }

    scan_tracking::vision::MultiCameraCaptureBundle legacyBundle;
    legacyBundle.mechEyeResult = result;
    legacyBundle.request.segmentIndex = m_activeTask.scanSegmentIndex;
    legacyBundle.request.taskId = m_activeTask.taskId;

    const int segmentIndex = m_activeTask.scanSegmentIndex;
    const quint32 taskId = m_activeTask.taskId;
    const scan_tracking::common::PointCloudProcessingConfig processingConfig =
        scan_tracking::common::ConfigManager::instance()
            ? scan_tracking::common::ConfigManager::instance()->pointCloudProcessingConfig()
            : scan_tracking::common::PointCloudProcessingConfig{};

    const int pathIdForCache = resolvePathIdForIncomingSegment(segmentIndex);
    commitScanSegmentCaptureImmediate(pathIdForCache, segmentIndex, result, legacyBundle);
    startSegmentBackgroundRefinement(pathIdForCache, segmentIndex, taskId, processingConfig);
}

/**
 * @brief 处理 Mech-Eye 相机致命错误（legacy 单相机路径）
 * 
 * 当相机发生无法恢复的错误（如硬件故障、驱动崩溃等）时触发此回调。
 * 如果当前正在执行扫描分段任务，立即终止任务并进入故障状态。
 * 
 * @param code 错误代码
 * @param message 错误描述信息
 */
void StateMachine::onMechEyeFatalError(mech_eye::CaptureErrorCode code, QString message)
{
    qCritical(LOG_FLOW) << "[MechEye] 致命错误:" << message;
    emit protocolEvent(QStringLiteral("Mech-Eye: %1").arg(message));  // 发出协议事件通知

    // 只在扫描分段阶段且任务未完成时才处理
    if (m_activeTask.definition == nullptr ||
        m_activeTask.definition->stage != protocol::Stage::ScanSegment ||
        m_activeTask.completionAnnounced) {
        return;
    }

    // P0修复：相机致命错误时清理已缓存的点云数据，防止内存泄漏
    qWarning(LOG_FLOW) << QStringLiteral("Mech-Eye 致命错误，清空扫描分段内存缓存");
    resetScanSegmentCache();

    // 相机在扫描中途发生致命错误时，需要第一时间拉高报警并强制结束当前扫描触发。
    finishScanSegmentFailure(
        mapCaptureErrorToResCode(code),  // 映射错误码到 Res 码
        3,                               // 报警级别：3 = 严重错误
        723,                             // 报警代码：723 = 相机致命错误
        QStringLiteral("扫描中 Mech-Eye 致命错误"),
        message);
}

/**
 * @brief 完成当前活动任务
 * 
 * 这是任务完成的统一出口，负责：
 * 1. 停止超时定时器
 * 2. 更新进度和数据有效性标志
 * 3. 向 PLC 发送 Res（结果码）和 ACK（应答状态）
 * 4. 标记任务已完成宣告
 * 5. 发布更新的 IPC 状态
 * 
 * @param resultCode 任务结果码（1=成功，其他=各种失败原因）
 * @param finalAckState 最终的 ACK 状态（Completed 或 Failed）
 * @param dataValid 数据是否有效（true 表示检测结果可用）
 */
/**
 * @brief 完成活动任务并发送结果和确认
 * 
 * P1修复：使用批量写入 Res 和 Ack，确保原子性，避免 PLC 读到中间状态。
 * 由于所有触发器的 Ack 和 Res 地址都是连续的（Ack 在前，Res 在后），
 * 可以使用一次 writeRegisters 调用同时写入两个寄存器。
 * 
 * @param resultCode 结果代码
 * @param finalAckState 最终 ACK 状态（Completed=2 或 Failed=3）
 * @param dataValid 数据是否有效标志
 */
bool StateMachine::completeActiveTask(
    quint16 resultCode,
    protocol::AckState finalAckState,
    bool dataValid)
{
    if (m_activeTask.definition == nullptr || !m_modbus) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("无法完成任务：任务定义或 Modbus 为空");
        return false;  // 没有活动任务或 Modbus 不可用，无法完成
    }

    // P3改进：封装带重试的 Modbus 写入 lambda
    auto executeWithRetry = [this](auto&& writeOperation, const QString& operationName) -> bool {
        constexpr int kMaxRetries = 3;
        constexpr int kRetryIntervalMs = 100;
        
        for (int attempt = 1; attempt <= kMaxRetries; ++attempt) {
            // P3改进：每次重试前检查必要前提条件
            if (!m_modbus) {
                qWarning(LOG_FLOW).noquote()
                    << operationName << QStringLiteral(" 失败：第") << attempt << QStringLiteral(" 次重试时 Modbus 已为空");
                return false;
            }
            
            if (m_activeTask.definition == nullptr) {
                qWarning(LOG_FLOW).noquote()
                    << operationName << QStringLiteral(" 失败：第") << attempt << QStringLiteral(" 次重试时任务定义已为空");
                return false;
            }
            
            // 执行写入操作
            writeOperation();
            
            // P3改进：如果是最后一次尝试，直接返回成功（已发起写入请求）
            if (attempt == kMaxRetries) {
                return true;
            }
            
            // P3改进：非最后一次尝试，等待后继续重试
            QThread::msleep(kRetryIntervalMs);
        }
        
        return true;  // 理论上不会到达这里
    };
    
    // P1修复：批量写入 Res 和 Ack，保证原子性
    // P2改进：虽然批量写入是原子的，但为了确保语义清晰，我们明确标注写入顺序
    // 在 Modbus 批量写入中，所有寄存器在同一事务中原子更新，PLC 会同时看到新值
    // 地址布局：ackOffset (低位) -> resOffset (高位)，两者必须连续
    const int ackOffset = m_activeTask.definition->ackOffset;
    const int resOffset = m_activeTask.definition->resOffset;
    
    bool writeSuccess = false;
    auto failCompletionWrite = [this](const QString& reason) -> bool {
        qWarning(LOG_FLOW).noquote() << reason;
        enterFaultState(902, reason, false, false);
        return false;
    };
    
    // 验证地址连续性（防御性编程）
    if (resOffset == ackOffset + 1) {
        // 地址连续，使用批量写入
        QVector<quint16> batchValues = {
            static_cast<quint16>(finalAckState),  // Ack 值
            resultCode                             // Res 值
        };
        
        // P3改进：带重试的批量写入
        writeSuccess = executeWithRetry(
            [&]() {
                m_modbus->writeRegisters(ackOffset, batchValues);
            },
            QStringLiteral("批量写入寄存器"));
        
        if (!writeSuccess) {
            return failCompletionWrite(QStringLiteral(
                "批量写入重试后仍失败：Ack=%1 Res=%2 地址 %3-%4")
                .arg(static_cast<int>(finalAckState))
                .arg(resultCode)
                .arg(ackOffset)
                .arg(resOffset));
        }
        
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("原子批量写入 Ack=") << static_cast<int>(finalAckState)
            << QStringLiteral(" Res=") << resultCode
            << QStringLiteral(" 地址") << ackOffset << QStringLiteral("-") << resOffset;
    } else {
        // 地址不连续（异常情况），降级为单独写入
        // P2改进：确保先写 Res 再写 Ack，避免 PLC 读到中间状态
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("Ack 与 Res 地址不连续：")
            << QStringLiteral(" ackOffset=") << ackOffset << QStringLiteral(" resOffset=") << resOffset
            << QStringLiteral("，降级为单独写入并重试。");
        
        // P3改进：先写 Res（结果数据），带重试
        bool resSuccess = executeWithRetry(
            [&]() {
                sendRes(*m_activeTask.definition, resultCode);
            },
            QStringLiteral("发送 Res"));
        
        if (!resSuccess) {
            return failCompletionWrite(QStringLiteral(
                "Res 重试后仍发送失败：resultCode=%1")
                .arg(resultCode));
        }
        
        // P3改进：再写 Ack（完成标志），带重试
        bool ackSuccess = executeWithRetry(
            [&]() {
                sendAck(*m_activeTask.definition, finalAckState);
            },
            QStringLiteral("发送 Ack"));
        
        if (!ackSuccess) {
            return failCompletionWrite(QStringLiteral(
                "Ack 重试后仍发送失败：ackState=%1")
                .arg(static_cast<int>(finalAckState)));
        }
    }
    
    // P3改进：只有 Modbus 操作成功发起后，才更新状态
    m_timeoutTimer->stop();   // 停止超时定时器
    m_progress = 100;         // 进度设为 100%
    m_dataValid = dataValid;  // 设置数据有效性标志
    m_activeTask.completionAnnounced = true;                    // 标记已完成宣告
    m_activeTask.captureRequestId = 0;                          // 清除采集请求 ID
    publishIpcStatus();                                         // 发布更新的 IPC 状态

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("触发已完成") << protocol::triggerName(*m_activeTask.definition)
        << QStringLiteral(" Res=") << resultCode
        << QStringLiteral(" Ack=") << static_cast<int>(finalAckState);
    
    return true;
}

/**
 * @brief 在 PLC 释放触发信号后 finalize 已完成的任务
 * 
 * 当任务完成后，PLC 需要将对应的 Trig 位清零以确认收到结果。
 * 此方法检测 Trig 位是否已清零，如果是则回零 Ack/Res 并清除活动任务状态，
 * 使系统回到就绪状态等待下一个触发。
 * 
 * @param commandBlock 最新的命令块寄存器数据
 */
void StateMachine::finalizeCompletedTaskIfTriggerReleased(const QVector<quint16>& commandBlock)
{
    if (m_activeTask.definition == nullptr || !m_activeTask.completionAnnounced) {
        return;  // 没有活动任务或任务未完成宣告，无需处理
    }

    const int trigOffset = m_activeTask.definition->trigOffset;
    // 检查 Trig 位是否已清零（PLC 确认收到结果）
    if (trigOffset >= commandBlock.size() || commandBlock[trigOffset] != 0) {
        return;  // Trig 位仍为 1，PLC 尚未释放
    }

    qInfo(LOG_FLOW).noquote() << QStringLiteral("PLC 已释放触发：")
                              << protocol::triggerName(*m_activeTask.definition);

    const protocol::TriggerDefinition& definition = *m_activeTask.definition;
    if (definition.stage == protocol::Stage::ScanSegment) {
        writeScanSegmentResult(0, 0, 0);
    }

    if (m_modbus) {
        const int ackOffset = definition.ackOffset;
        const int resOffset = definition.resOffset;
        bool released = false;
        if (resOffset == ackOffset + 1) {
            released = m_modbus->writeRegisters(ackOffset, {
                static_cast<quint16>(protocol::AckState::Idle),
                0,
            });
        } else {
            sendRes(definition, 0);
            sendAck(definition, protocol::AckState::Idle);
            released = true;
        }
        if (!released) {
            qWarning(LOG_FLOW).noquote()
                << QStringLiteral("释放触发后回零 Ack/Res 失败：") << protocol::triggerName(definition);
        } else {
            qInfo(LOG_FLOW).noquote()
                << QStringLiteral("握手释放完成") << protocol::triggerName(definition)
                << QStringLiteral(" Ack=0 Res=0");
        }
    }

    clearActiveTask();                                            // 清除活动任务
    m_ipcState = protocol::IpcState::Ready;                       // IPC 状态回到就绪
    m_currentStage = protocol::Stage::Idle;                       // 当前阶段回到空闲
    m_progress = 0;                                               // 进度归零
    setState(AppState::Ready);                                    // 应用状态回到就绪
    publishIpcStatus();                                           // 发布更新的 IPC 状态
}

/**
 * @brief 清除当前活动任务的所有状态
 * 
 * 将活动任务结构体重置为默认值，释放所有相关资源。
 */
void StateMachine::clearActiveTask()
{
    m_activeTask = {};  // 重置为默认构造的空任务
}

/**
 * @brief 设置报警信息
 * 
 * 更新报警级别、报警代码和警告代码，并发出协议事件通知。
 * 
 * @param level 报警级别（0=无报警，1=提示，2=警告，3=严重错误）
 * @param code 报警代码
 * @param message 报警描述信息
 */
void StateMachine::setAlarm(quint16 level, quint16 code, const QString& message)
{
    m_alarmLevel = level;
    m_alarmCode = code;
    // 警告代码仅在报警级别为 1 或 2 时有效
    m_warnCode = level > 0 && level < 3 ? code : 0;
    if (!message.isEmpty()) {
        emit protocolEvent(message);  // 发出协议事件通知外部监听者
    }
}

bool StateMachine::writeIpcSafetyActionWord()
{
    if (!m_modbus || !m_modbus->isConnected()) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("Modbus 未连接，无法写入 IPC_SafetyAction_Word=")
            << m_ipcSafetyActionWord;
        return false;
    }

    const bool written = m_modbus->writeRegisters(
        protocol::registers::kIpcSafetyActionWord,
        {m_ipcSafetyActionWord});
    if (!written) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("写入 IPC_SafetyAction_Word 失败，值=") << m_ipcSafetyActionWord;
    } else {
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("已写入 IPC_SafetyAction_Word=") << m_ipcSafetyActionWord;
    }
    return written;
}

bool StateMachine::reportPersonZoneAlarm(bool alarm)
{
    namespace safety = protocol::safety_bits;

    if (alarm) {
        if (m_personZoneAlarmActive &&
            (m_ipcSafetyActionWord & safety::kAiPersonIntrusion) != 0) {
            return writeIpcSafetyActionWord();
        }

        m_ipcSafetyActionWord |= safety::kAiPersonIntrusion;
        const bool plcWritten = writeIpcSafetyActionWord();

        const QString message = QStringLiteral("监控区域检测到人员");
        enterFaultState(601, message, true, false);
        m_personZoneAlarmActive = true;
        publishIpcStatus();

        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("人员区域报警：已置 IPC_SafetyAction_Word Bit0，中止当前任务");
        return plcWritten;
    }

    if (!m_personZoneAlarmActive &&
        (m_ipcSafetyActionWord & safety::kAiPersonIntrusion) == 0) {
        return writeIpcSafetyActionWord();
    }

    m_ipcSafetyActionWord &= ~static_cast<quint16>(safety::kAiPersonIntrusion);
    const bool plcWritten = writeIpcSafetyActionWord();

    if (m_personZoneAlarmActive) {
        setAlarm(0, 0, QString());
        m_ipcState = protocol::IpcState::Ready;
        m_currentStage = protocol::Stage::Idle;
        m_personZoneAlarmActive = false;
        setState(AppState::Ready);
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("人员区域报警解除：已清除 IPC_SafetyAction_Word Bit0，恢复 Ready");
    }

    publishIpcStatus();
    return plcWritten;
}

bool StateMachine::updateUnloadAreaConfig(const UnloadAreaConfig& config)
{
    namespace regs = protocol::registers;
    namespace limits = protocol::unload_area;

    m_unloadAreaConfig.maxStackCount =
        static_cast<quint16>(qBound(0, static_cast<int>(config.maxStackCount), limits::kMaxStackCountLimit));
    m_unloadAreaConfig.okCount =
        static_cast<quint16>(qBound(0, static_cast<int>(config.okCount), limits::kHeadCountLimit));
    m_unloadAreaConfig.ngCount =
        static_cast<quint16>(qBound(0, static_cast<int>(config.ngCount), limits::kHeadCountLimit));
    m_unloadAreaConfig.autoClear =
        config.autoClear != 0 ? limits::kAutoClearOn : limits::kAutoClearOff;
    m_unloadAreaConfigReceived = true;

    if (!m_modbus) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("下料区封头配置已缓存但 Modbus 不可用：maxStack=")
            << m_unloadAreaConfig.maxStackCount
            << QStringLiteral(" ok=") << m_unloadAreaConfig.okCount
            << QStringLiteral(" ng=") << m_unloadAreaConfig.ngCount
            << QStringLiteral(" autoClear=") << m_unloadAreaConfig.autoClear;
        return false;
    }

    const QVector<quint16> values = {
        m_unloadAreaConfig.maxStackCount,
        m_unloadAreaConfig.okCount,
        m_unloadAreaConfig.ngCount,
        m_unloadAreaConfig.autoClear,
    };
    const bool written = m_modbus->writeRegisters(regs::kUnloadAreaMaxStackCount, values);
    if (!written) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("写入下料区封头配置失败（40176~40179）");
    } else {
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("已写入下料区封头配置 40176~40179：maxStack=")
            << m_unloadAreaConfig.maxStackCount
            << QStringLiteral(" ok=") << m_unloadAreaConfig.okCount
            << QStringLiteral(" ng=") << m_unloadAreaConfig.ngCount
            << QStringLiteral(" autoClear=") << m_unloadAreaConfig.autoClear;
    }
    return written;
}

StateMachine::UnloadAreaConfig StateMachine::unloadAreaConfig() const
{
    return m_unloadAreaConfig;
}

/**
 * @brief 向 PLC 写入浮点数占位符（CDAB 字节序）
 * 
 * 将单个 float 值转换为两个 16 位寄存器并写入指定的起始地址。
 * 用于向 PLC 传递坐标、角度等浮点数据。
 * 
 * @param startOffset 起始寄存器偏移地址
 * @param value 要写入的浮点数值
 */
void StateMachine::writeFloatPlaceholder(int startOffset, float value)
{
    if (!m_modbus) {
        return;  // Modbus 不可用，无法写入
    }

    const bool floatWritten = m_modbus->writeRegisters(startOffset, floatToCdabRegisters(value));
    if (!floatWritten) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("写入浮点占位符失败，偏移=") << startOffset;
    }
}

StateMachine::PoseSourceResult StateMachine::resolveLoadGraspPoseSource() const
{
    return parsePoseSource(
        "SCAN_TRACKING_LOAD_GRASP_POSE",
        QStringLiteral("load-grasp-provider"),
        {125.0f, 250.0f, 375.0f, 0.0f, 90.0f, 180.0f},
        true);
}

StateMachine::PoseSourceResult StateMachine::resolveUnloadCalcPoseSource() const
{
    return parsePoseSource(
        "SCAN_TRACKING_UNLOAD_CALC_POSE",
        QStringLiteral("unload-calc-provider"),
        {500.0f, 600.0f, 700.0f, 0.0f, 0.0f, 90.0f},
        true);
}

/**
 * @brief 向 PLC 写入 ASCII 字符串占位符
 * 
 * 将字符串按每两个字符打包为一个 16 位寄存器的方式写入 PLC。
 * 如果字符串长度不足，用空格填充；如果超长，则截断。
 * 
 * @param startOffset 起始寄存器偏移地址
 * @param registerCount 要写入的寄存器数量
 * @param text 要写入的文本字符串
 */
void StateMachine::writeAsciiPlaceholder(int startOffset, int registerCount, const QString& text)
{
    if (!m_modbus) {
        return;  // Modbus 不可用，无法写入
    }

    // 限制字符串长度并用空格右对齐填充
    const QString padded = text.left(registerCount * 2).leftJustified(registerCount * 2, QLatin1Char(' '));
    QVector<quint16> values;
    values.reserve(registerCount);
    for (int i = 0; i < registerCount; ++i) {
        // 每两个字符打包为一个 16 位寄存器：高8位为第一个字符，低8位为第二个字符
        const QChar first = padded.at(i * 2);
        const QChar second = padded.at(i * 2 + 1);
        const quint16 packed = (static_cast<quint16>(first.unicode()) << 8) |
                               static_cast<quint16>(second.unicode() & 0xFF);
        values.push_back(packed);
    }
    const bool asciiWritten = m_modbus->writeRegisters(startOffset, values);
    if (!asciiWritten) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("写入 ASCII 占位符失败，偏移=") << startOffset;
    }
}

/**
 * @brief 写入加载抓取任务的模拟结果
 * 
 * 向 PLC 写入预设的加载位姿数据（位置和姿态角）。
 * 这是一个占位实现，实际应用中应替换为真实的视觉定位结果。
 */
void StateMachine::writeLoadGraspResult()
{
    const auto poseSource = resolveLoadGraspPoseSource();
    writeFloatPlaceholder(protocol::registers::kLoadX, poseSource.x);
    writeFloatPlaceholder(protocol::registers::kLoadY, poseSource.y);
    writeFloatPlaceholder(protocol::registers::kLoadZ, poseSource.z);
    writeFloatPlaceholder(protocol::registers::kLoadRx, poseSource.rx);
    writeFloatPlaceholder(protocol::registers::kLoadRy, poseSource.ry);
    writeFloatPlaceholder(protocol::registers::kLoadRz, poseSource.rz);
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("LoadGrasp 位姿已写入")
        << QStringLiteral(" source=") << poseSource.sourceName
        << QStringLiteral(" x=") << poseSource.x
        << QStringLiteral(" y=") << poseSource.y
        << QStringLiteral(" z=") << poseSource.z
        << QStringLiteral(" rx=") << poseSource.rx
        << QStringLiteral(" ry=") << poseSource.ry
        << QStringLiteral(" rz=") << poseSource.rz;
}

/**
 * @brief 写入卸载计算任务的模拟结果
 * 
 * 向 PLC 写入预设的卸料位姿数据（位置和姿态角）。
 * 这是一个占位实现，实际应用中应替换为真实的路径规划结果。
 */
void StateMachine::writeUnloadCalcResult()
{
    const auto poseSource = resolveUnloadCalcPoseSource();
    writeFloatPlaceholder(protocol::registers::kUnloadX, poseSource.x);
    writeFloatPlaceholder(protocol::registers::kUnloadY, poseSource.y);
    writeFloatPlaceholder(protocol::registers::kUnloadZ, poseSource.z);
    writeFloatPlaceholder(protocol::registers::kUnloadRx, poseSource.rx);
    writeFloatPlaceholder(protocol::registers::kUnloadRy, poseSource.ry);
    writeFloatPlaceholder(protocol::registers::kUnloadRz, poseSource.rz);
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("UnloadCalc 位姿已写入")
        << QStringLiteral(" source=") << poseSource.sourceName
        << QStringLiteral(" x=") << poseSource.x
        << QStringLiteral(" y=") << poseSource.y
        << QStringLiteral(" z=") << poseSource.z
        << QStringLiteral(" rx=") << poseSource.rx
        << QStringLiteral(" ry=") << poseSource.ry
        << QStringLiteral(" rz=") << poseSource.rz;
}

/**
 * @brief 写入扫描分段结果到 PLC
 * 
 * 将当前分段的索引、采集的图像数量和点云帧数量写入 PLC 寄存器，
 * 供 PLC 跟踪扫描进度。
 * 
 * @param segmentIndex 分段索引（从1开始）
 * @param imageCount 采集的图像数量（通常为1）
 * @param cloudFrameCount 采集的点云帧数量（通常为1）
 */
void StateMachine::writeScanSegmentResult(int segmentIndex, int imageCount, int cloudFrameCount)
{
    if (!m_modbus) {
        return;  // Modbus 不可用，无法写入
    }

    const bool progressWritten = m_modbus->writeRegisters(protocol::registers::kScanSegmentDoneIndex, {
        static_cast<quint16>(segmentIndex),     // 已完成的分段索引
        static_cast<quint16>(imageCount),       // 该分段的图像数量
        static_cast<quint16>(cloudFrameCount),  // 该分段的点云帧数量
    });
    if (!progressWritten) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("写入扫描分段进度失败");
    }
}

/**
 * @brief 写入综合检测结果到 PLC
 * 
 * 将检测结果的 NG 原因字、测量项数量等写入 PLC 寄存器，
 * 供 PLC 判断工件是否合格以及获取详细的缺陷信息。
 * 
 * @param summary 检测结果摘要结构
 */
void StateMachine::writeInspectionResult(const InspectionSummary& summary)
{
    if (!m_modbus) {
        return;  // Modbus 不可用，无法写入
    }

    // 写入 NG 原因字和测量项数量
    const bool inspectionWritten = m_modbus->writeRegisters(protocol::registers::kNgReasonWord0, {
        summary.ngReasonWord0,      // NG 原因字 0（位掩码表示各种缺陷类型）
        summary.ngReasonWord1,      // NG 原因字 1（扩展缺陷类型）
        summary.measureItemCount,   // 实际测量的项目数量
    });
    if (!inspectionWritten) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("写入检测结果失败");
        return;
    }

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("检测结果寄存器已写入")
        << QStringLiteral(" ngReasonWord0=") << summary.ngReasonWord0
        << QStringLiteral(" ngReasonWord1=") << summary.ngReasonWord1
        << QStringLiteral(" measureItemCount=") << summary.measureItemCount;
}

/**
 * @brief 重置点云缓存
 * 
 * 清空所有累积的扫描分段点云数据，释放内存。
 * 在以下情况下调用：
 * - 综合检测完成后（点云已被消费）
 * - 结果复位任务执行时
 * - 系统启动或停止时
 * - 发生故障需要清理状态时
 */
void StateMachine::resetPointCloudCache()
{
    resetScanSegmentCache();
}

void StateMachine::resetScanSegmentCache()
{
    if (!m_stopped.load(std::memory_order_acquire)) {
        joinAllBackgroundRefinementJobs();
    } else if (pendingRefinementJobCount() != 0) {
        reconcilePendingRefinementJobCounter("resetScanSegmentCache(stop中)");
    }

    {
        std::lock_guard<std::mutex> lock(m_segmentCacheMutex);

        // === 多路径支持：清空二维缓存 ===
        int totalCacheSize = 0;
    for (auto pathIt = m_pathSegmentCaptureBundles.begin(); pathIt != m_pathSegmentCaptureBundles.end(); ++pathIt) {
        for (auto segIt = pathIt->begin(); segIt != pathIt->end(); ++segIt) {
            scan_tracking::vision::releaseHikMonoFrameBuffers(&segIt->hikCameraAResult.frame);
            scan_tracking::vision::releaseHikMonoFrameBuffers(&segIt->hikCameraBResult.frame);
            scan_tracking::mech_eye::releasePointCloudFrameBuffers(&segIt->mechEyeResult.pointCloud);
            totalCacheSize++;
        }
    }
    m_pathSegmentCaptureBundles.clear();

    for (auto pathIt = m_pathSegmentCaptureResults.begin(); pathIt != m_pathSegmentCaptureResults.end(); ++pathIt) {
        for (auto segIt = pathIt->begin(); segIt != pathIt->end(); ++segIt) {
            scan_tracking::mech_eye::releasePointCloudFrameBuffers(&segIt->pointCloud);
        }
    }
    m_pathSegmentCaptureResults.clear();
    
    // 重置路径上下文
    m_currentPathId = 1;
    m_currentPathSegments.clear();
    clearFirstPathStepPauseLatch();

    if (totalCacheSize > 0) {
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("[多路径] 已清空内存点云缓存，总条目数=") << totalCacheSize
            << QStringLiteral("，路径ID已重置为1");
    }

    m_pathSegmentCalibrationMatrices.clear();
    m_pathSegmentPoseStitchRecords.clear();
    m_currentCalibrationMatrix = m_baseCalibrationMatrix;
    }

    {
        std::lock_guard<std::mutex> poseStitchLock(m_lastPoseStitchMutex);
        scan_tracking::mech_eye::releasePointCloudFrameBuffers(&m_lastPoseStitchArtifact.stitchedPointCloud);
        m_lastPoseStitchArtifact = LastPoseStitchArtifact{};
    }
}

void StateMachine::reloadCalibrationMatricesFromConfig()
{
    m_baseCalibrationMatrix = identityMatrix4x4();
    m_currentCalibrationMatrix = m_baseCalibrationMatrix;

    const auto* configMgr = scan_tracking::common::ConfigManager::instance();
    if (configMgr == nullptr) {
        return;
    }

    const auto& t0 = configMgr->scanPathsConfig().calibrationMatrixT0;
    bool hasNonIdentity = false;
    for (std::size_t i = 0; i < t0.size(); ++i) {
        if (i == 0 || i == 5 || i == 10 || i == 15) {
            if (std::abs(t0[i] - 1.0f) > 1e-6f) {
                hasNonIdentity = true;
            }
        } else if (std::abs(t0[i]) > 1e-6f) {
            hasNonIdentity = true;
        }
    }
    if (hasNonIdentity) {
        m_baseCalibrationMatrix = t0;
        m_currentCalibrationMatrix = t0;
    }

    qInfo(LOG_FLOW).noquote() << QStringLiteral("已从 scan_paths_config 加载 JSON T0（LB 失败时的回退基准）");
}

bool StateMachine::resolveNeedRotationForSegment(int pathId, int segmentIndex) const
{
    return lookupNeedRotationForSegment(pathId, segmentIndex);
}

void StateMachine::applyLbnCalibrationUpdate(
    int pathId,
    int segmentIndex,
    bool needRotation,
    const scan_tracking::vision::MultiCameraCaptureBundle& bundle)
{
    if (pathId <= 0) {
        pathId = m_currentPathId > 0 ? m_currentPathId : 1;
    }

    if (!needRotation) {
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("LBN 标定：路径=") << pathId
            << QStringLiteral(" 段号=") << segmentIndex
            << QStringLiteral(" 无需转盘，保持当前 T0'");
    } else {
        const auto& lbn = bundle.lbnPoseResult;
        if (!lbn.invoked) {
            qWarning(LOG_FLOW).noquote()
                << QStringLiteral("LBN 标定：路径=") << pathId
                << QStringLiteral(" 段号=") << segmentIndex
                << QStringLiteral(" 未调用 LBN，保持当前 T0'");
        } else {
            const bool useIdentityBypass = []() {
                const auto* cfg = scan_tracking::common::ConfigManager::instance();
                return cfg && cfg->lbnPoseConfig().useIdentityRtWithoutMarkers;
            }();

            if (!lbn.success || !lbn.poseMatrix.isValid()) {
                if (!useIdentityBypass) {
                    qWarning(LOG_FLOW).noquote()
                        << QStringLiteral("LBN 标定：路径=") << pathId
                        << QStringLiteral(" 段号=") << segmentIndex
                        << QStringLiteral(" 失败，保持当前 T0'：") << lbn.message;
                } else {
                    qWarning(LOG_FLOW).noquote()
                        << QStringLiteral("LBN 标定：路径=") << pathId
                        << QStringLiteral(" 段号=") << segmentIndex
                        << QStringLiteral(" 失败，TODO(marker) 回退 Rt=单位阵：") << lbn.message;
                }
            } else {
                const auto rt = poseMatrixToArray(lbn.poseMatrix);
                m_currentCalibrationMatrix = multiplyRowMajor4x4(rt, m_currentCalibrationMatrix);

                qInfo(LOG_FLOW).noquote()
                    << QStringLiteral("LBN 标定已更新 T0'，路径=") << pathId
                    << QStringLiteral(" 段号=") << segmentIndex
                    << QStringLiteral(" 匹配点数=") << lbn.matchedPointCount;
                for (int row = 0; row < 4; ++row) {
                    qInfo(LOG_FLOW).noquote()
                        << QStringLiteral("  T0'[%1] %2 %3 %4 %5")
                               .arg(row)
                               .arg(m_currentCalibrationMatrix[static_cast<std::size_t>(row * 4 + 0)], 0, 'g', 6)
                               .arg(m_currentCalibrationMatrix[static_cast<std::size_t>(row * 4 + 1)], 0, 'g', 6)
                               .arg(m_currentCalibrationMatrix[static_cast<std::size_t>(row * 4 + 2)], 0, 'g', 6)
                               .arg(m_currentCalibrationMatrix[static_cast<std::size_t>(row * 4 + 3)], 0, 'g', 6);
                }
            }
        }
    }

    m_pathSegmentCalibrationMatrices[pathId][segmentIndex] = m_currentCalibrationMatrix;
}

void StateMachine::applySegmentPoseStitching(int pathId, int segmentIndex)
{
    if (pathId <= 0) {
        pathId = m_currentPathId > 0 ? m_currentPathId : 1;
    }

    scan_tracking::mech_eye::PointCloudFrame inputCloud;
    std::array<float, 16> calibrationMatrix = m_baseCalibrationMatrix;
    std::array<float, 16> t0PrimeLbn = m_currentCalibrationMatrix;
    std::array<float, 16> lbRtGlobal = identityMatrix4x4();
    const std::array<float, 16> stereoMatrix = identityMatrix4x4();
    bool lbValid = false;
    bool lbRtGlobalValid = false;
    bool rtGlobalAsT0 = false;
    QString lbFallbackNote;

    {
        std::lock_guard<std::mutex> lock(m_segmentCacheMutex);
        const auto pathIt = m_pathSegmentCaptureResults.constFind(pathId);
        if (pathIt == m_pathSegmentCaptureResults.cend() || !pathIt->contains(segmentIndex)) {
            qWarning(LOG_FLOW).noquote()
                << QStringLiteral("[PoseStitch] 跳过：路径") << pathId
                << QStringLiteral(" 段") << segmentIndex << QStringLiteral(" 点云未缓存");
            return;
        }

        inputCloud = clonePointCloudFrame(pathIt.value().value(segmentIndex).pointCloud);

        const auto pathCalibrationIt = m_pathSegmentCalibrationMatrices.constFind(pathId);
        if (pathCalibrationIt != m_pathSegmentCalibrationMatrices.cend()) {
            const auto segmentCalibrationIt = pathCalibrationIt->constFind(segmentIndex);
            if (segmentCalibrationIt != pathCalibrationIt->cend()) {
                t0PrimeLbn = segmentCalibrationIt.value();
            }
        }

        const auto bundlePathIt = m_pathSegmentCaptureBundles.constFind(pathId);
        if (bundlePathIt != m_pathSegmentCaptureBundles.cend()) {
            const auto bundleSegmentIt = bundlePathIt->constFind(segmentIndex);
            if (bundleSegmentIt != bundlePathIt->cend()) {
                const auto& lb = bundleSegmentIt->lbPoseResult;
                if (lb.success && lb.poseMatrix.valid) {
                    lbRtGlobal = lb.poseMatrix.values;
                    lbRtGlobalValid = true;
                    calibrationMatrix = lbRtGlobal;
                    lbValid = true;
                    rtGlobalAsT0 = true;
                    m_pathSegmentCalibrationMatrices[pathId][segmentIndex] = calibrationMatrix;
                } else if (lb.invoked) {
                    lbFallbackNote = lb.message;
                }
            }
        }

        if (!rtGlobalAsT0) {
            calibrationMatrix = t0PrimeLbn;
        }
    }

    if (!lbValid) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("[PoseStitch] LB 无效，拼接回退 T0'（JSON/LBN），路径=") << pathId
            << QStringLiteral(" 段号=") << segmentIndex
            << (lbFallbackNote.isEmpty() ? QString() : QStringLiteral(" 说明=") + lbFallbackNote);
    }

    scan_tracking::mech_eye::PointCloudFrame stitchedCloud;
    QString stitchMessage;
    if (!scan_tracking::mech_eye::transformPointCloudFrame(
            inputCloud,
            calibrationMatrix,
            stereoMatrix,
            &stitchedCloud,
            &stitchMessage)) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("[PoseStitch] 失败，路径=") << pathId
            << QStringLiteral(" 段号=") << segmentIndex
            << QStringLiteral(" 说明=") << stitchMessage;
        return;
    }

    {
        std::lock_guard<std::mutex> lock(m_segmentCacheMutex);
        if (!m_pathSegmentCaptureResults.contains(pathId) ||
            !m_pathSegmentCaptureResults[pathId].contains(segmentIndex)) {
            return;
        }

        assignSharedPointCloudToSegmentEntry(
            m_pathSegmentCaptureResults,
            m_pathSegmentCaptureBundles,
            pathId,
            segmentIndex,
            stitchedCloud);
    }

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("[PoseStitch] 点云已拼接 T0")
        << (rtGlobalAsT0 ? QStringLiteral("(Rt_global)") : QStringLiteral("(T0' fallback)"))
        << QStringLiteral(" 路径=") << pathId
        << QStringLiteral(" 段号=") << segmentIndex
        << QStringLiteral(" 点数=") << stitchedCloud.pointCount
        << QStringLiteral(" LB有效=") << lbValid
        << QStringLiteral(" 说明=") << stitchMessage;

    const auto combinedOutputRt = calibrationMatrix;
    recordSegmentPoseStitch(
        pathId,
        segmentIndex,
        lbValid,
        lbRtGlobalValid,
        m_baseCalibrationMatrix,
        t0PrimeLbn,
        lbRtGlobal,
        calibrationMatrix,
        stereoMatrix,
        combinedOutputRt);
    updateLastPoseStitchArtifact(
        pathId,
        segmentIndex,
        lbValid,
        m_baseCalibrationMatrix,
        calibrationMatrix,
        stereoMatrix,
        combinedOutputRt,
        stitchedCloud);

    SegmentPoseStitchRecord stitchRecord;
    stitchRecord.valid = true;
    stitchRecord.lbTrackingValid = lbValid;
    stitchRecord.lbRtGlobalValid = lbRtGlobalValid;
    stitchRecord.pathId = pathId;
    stitchRecord.segmentIndex = segmentIndex;
    stitchRecord.baseCalibrationT0 = m_baseCalibrationMatrix;
    stitchRecord.t0PrimeLbn = t0PrimeLbn;
    stitchRecord.lbRtGlobal = lbRtGlobal;
    stitchRecord.appliedT0 = calibrationMatrix;
    stitchRecord.stereoTrackingT = stereoMatrix;
    stitchRecord.combinedOutputRt = combinedOutputRt;

    scan_tracking::vision::MultiCameraCaptureBundle bundleSnapshot;
    {
        std::lock_guard<std::mutex> lock(m_segmentCacheMutex);
        const auto pathIt = m_pathSegmentCaptureBundles.constFind(pathId);
        if (pathIt != m_pathSegmentCaptureBundles.cend()) {
            const auto segmentIt = pathIt->constFind(segmentIndex);
            if (segmentIt != pathIt->cend()) {
                bundleSnapshot = segmentIt.value();
            }
        }
    }
    persistSegmentCaptureExportGroup(
        pathId, segmentIndex, bundleSnapshot, inputCloud, stitchedCloud, stitchRecord);
}

void StateMachine::recordSegmentPoseStitch(
    int pathId,
    int segmentIndex,
    bool lbTrackingValid,
    bool lbRtGlobalValid,
    const std::array<float, 16>& baseCalibrationT0,
    const std::array<float, 16>& t0PrimeLbn,
    const std::array<float, 16>& lbRtGlobal,
    const std::array<float, 16>& appliedT0,
    const std::array<float, 16>& stereoTrackingT,
    const std::array<float, 16>& combinedOutputRt)
{
    SegmentPoseStitchRecord record;
    record.valid = true;
    record.lbTrackingValid = lbTrackingValid;
    record.lbRtGlobalValid = lbRtGlobalValid;
    record.pathId = pathId;
    record.segmentIndex = segmentIndex;
    record.baseCalibrationT0 = baseCalibrationT0;
    record.t0PrimeLbn = t0PrimeLbn;
    record.lbRtGlobal = lbRtGlobal;
    record.appliedT0 = appliedT0;
    record.stereoTrackingT = stereoTrackingT;
    record.combinedOutputRt = combinedOutputRt;

    std::lock_guard<std::mutex> lock(m_segmentCacheMutex);
    m_pathSegmentPoseStitchRecords[pathId][segmentIndex] = record;
}

void StateMachine::updateLastPoseStitchArtifact(
    int pathId,
    int segmentIndex,
    bool lbTrackingValid,
    const std::array<float, 16>& baseCalibrationT0,
    const std::array<float, 16>& calibrationT0Prime,
    const std::array<float, 16>& stereoTrackingT,
    const std::array<float, 16>& combinedOutputRt,
    const scan_tracking::mech_eye::PointCloudFrame& stitchedPointCloud)
{
    LastPoseStitchArtifact artifact;
    artifact.valid = stitchedPointCloud.isValid();
    artifact.lbTrackingValid = lbTrackingValid;
    artifact.pathId = pathId;
    artifact.segmentIndex = segmentIndex;
    artifact.baseCalibrationT0 = baseCalibrationT0;
    artifact.calibrationT0Prime = calibrationT0Prime;
    artifact.stereoTrackingT = stereoTrackingT;
    artifact.combinedOutputRt = combinedOutputRt;
    artifact.stitchedPointCloud = clonePointCloudFrame(stitchedPointCloud);

    {
        std::lock_guard<std::mutex> lock(m_lastPoseStitchMutex);
        scan_tracking::mech_eye::releasePointCloudFrameBuffers(&m_lastPoseStitchArtifact.stitchedPointCloud);
        m_lastPoseStitchArtifact = std::move(artifact);
    }
}

void StateMachine::initializePoseStitchRunOutputDirectory()
{
    // 启动阶段不做任何目录 I/O：MechEye/Hik/LB 等 SDK 加载后，Qt QDir::mkpath 可能触发堆损坏。
    // run_* 子目录在 Trig_Inspection 落盘前由 ensurePoseStitchRunRootDirectory() 创建。
    m_poseStitchRunRootDirectory.clear();
    m_poseStitchOutputTimestamp.clear();
    m_segmentCaptureExportSessionRoot.clear();
    m_segmentCaptureExportSessionTimestamp.clear();
}

bool StateMachine::ensurePoseStitchRunRootDirectory()
{
    if (!m_poseStitchRunRootDirectory.isEmpty()) {
        return true;
    }

    m_poseStitchOutputTimestamp = poseStitchOutputTimestamp();
    m_poseStitchRunRootDirectory = createPoseStitchRunRootDirectory(m_poseStitchOutputTimestamp);
    if (m_poseStitchRunRootDirectory.isEmpty()) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("[PoseStitchOutput] 无法创建本次运行输出目录");
        m_poseStitchOutputTimestamp.clear();
        return false;
    }

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("[PoseStitchOutput] 本次运行输出目录=") << m_poseStitchRunRootDirectory;
    return true;
}

void StateMachine::persistInspectionPoseStitchOutput(int pathId) const
{
    if (pathId <= 0) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("[PoseStitchOutput] 跳过落盘：无效 pathId=") << pathId;
        return;
    }

    if (m_poseStitchRunRootDirectory.isEmpty()) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("[PoseStitchOutput] 跳过落盘：运行输出目录未就绪");
        return;
    }

    const QString timestamp = m_poseStitchOutputTimestamp.isEmpty()
        ? poseStitchOutputTimestamp()
        : m_poseStitchOutputTimestamp;
    const QString rtDirectory = poseStitchMatrixOutputDirectory(m_poseStitchRunRootDirectory);

    QMap<int, SegmentPoseStitchRecord> segmentRecords;
    {
        std::lock_guard<std::mutex> lock(m_segmentCacheMutex);
        const auto pathIt = m_pathSegmentPoseStitchRecords.constFind(pathId);
        if (pathIt == m_pathSegmentPoseStitchRecords.cend() || pathIt->isEmpty()) {
            qWarning(LOG_FLOW).noquote()
                << QStringLiteral("[PoseStitchOutput] 路径") << pathId
                << QStringLiteral(" 无段位姿矩阵记录，仅尝试落盘末段点云");
            persistLastPoseStitchArtifactToDisk();
            return;
        }
        segmentRecords = pathIt.value();
    }

    QList<int> segmentIndices = segmentRecords.keys();
    std::sort(segmentIndices.begin(), segmentIndices.end());

    QString summaryText;
    summaryText += QStringLiteral("# Path pose stitch matrix summary (row-major 4x4)\n");
    summaryText += QStringLiteral("# generatedAt=")
        + QDateTime::currentDateTime().toString(Qt::ISODateWithMs) + QStringLiteral("\n");
    summaryText += QStringLiteral("pathId=") + QString::number(pathId) + QStringLiteral("\n");
    summaryText += QStringLiteral("segmentCount=") + QString::number(segmentIndices.size()) + QStringLiteral("\n\n");

    int writtenSegmentCount = 0;
    for (int segmentIndex : segmentIndices) {
        const SegmentPoseStitchRecord& record = segmentRecords[segmentIndex];
        if (!record.valid) {
            continue;
        }

        const QString baseName = QStringLiteral("%1_path%2_seg%3")
                                     .arg(timestamp)
                                     .arg(pathId)
                                     .arg(segmentIndex);
        const QString rtFilePath = QDir(rtDirectory).filePath(baseName + QStringLiteral("_Rt.txt"));
        if (writeTextFile(rtFilePath, buildSegmentPoseStitchRtText(record))) {
            ++writtenSegmentCount;
            qInfo(LOG_FLOW).noquote()
                << QStringLiteral("[PoseStitchOutput] 段矩阵已写入") << rtFilePath;
        } else {
            qWarning(LOG_FLOW).noquote()
                << QStringLiteral("[PoseStitchOutput] 段矩阵写入失败：") << rtFilePath;
        }

        summaryText += QStringLiteral("=== segment ") + QString::number(segmentIndex) + QStringLiteral(" ===\n");
        summaryText += buildSegmentPoseStitchRtText(record);
        summaryText += QStringLiteral("\n");
    }

    const int finalSegmentIndex = segmentIndices.isEmpty() ? 0 : segmentIndices.last();
    if (finalSegmentIndex > 0 && segmentRecords.contains(finalSegmentIndex)
        && segmentRecords[finalSegmentIndex].valid) {
        const SegmentPoseStitchRecord& finalRecord = segmentRecords[finalSegmentIndex];
        const QString finalFilePath = QDir(rtDirectory).filePath(
            QStringLiteral("%1_path%2_final_Rt.txt").arg(timestamp).arg(pathId));
        if (writeTextFile(
                finalFilePath,
                buildFinalPoseStitchRtText(pathId, finalSegmentIndex, finalRecord))) {
            qInfo(LOG_FLOW).noquote()
                << QStringLiteral("[PoseStitchOutput] 最终矩阵已写入") << finalFilePath
                << QStringLiteral(" (finalSegmentIndex=") << finalSegmentIndex << QStringLiteral(")");
        } else {
            qWarning(LOG_FLOW).noquote()
                << QStringLiteral("[PoseStitchOutput] 最终矩阵写入失败：") << finalFilePath;
        }

        summaryText += QStringLiteral("=== final (segment ") + QString::number(finalSegmentIndex)
            + QStringLiteral(") ===\n");
        summaryText += buildFinalPoseStitchRtText(pathId, finalSegmentIndex, finalRecord);
        summaryText += QStringLiteral("\n");
    }

    const QString summaryFilePath = QDir(rtDirectory).filePath(
        QStringLiteral("%1_path%2_all_matrices.txt").arg(timestamp).arg(pathId));
    if (writeTextFile(summaryFilePath, summaryText)) {
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("[PoseStitchOutput] 汇总矩阵已写入") << summaryFilePath
            << QStringLiteral(" 段数=") << writtenSegmentCount;
    } else {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("[PoseStitchOutput] 汇总矩阵写入失败：") << summaryFilePath;
    }

    persistLastPoseStitchArtifactToDisk();
}

void StateMachine::persistLastPoseStitchArtifactToDisk() const
{
    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    if (configManager == nullptr || !configManager->segmentCaptureExportConfig().saveRawPointCloud) {
        return;
    }

    LastPoseStitchArtifact artifact;
    {
        std::lock_guard<std::mutex> lock(m_lastPoseStitchMutex);
        if (!m_lastPoseStitchArtifact.valid || !m_lastPoseStitchArtifact.stitchedPointCloud.isValid()) {
            qWarning(LOG_FLOW).noquote()
                << QStringLiteral("[PoseStitchOutput] 跳过落盘：尚无有效拼接结果");
            return;
        }
        artifact.pathId = m_lastPoseStitchArtifact.pathId;
        artifact.segmentIndex = m_lastPoseStitchArtifact.segmentIndex;
        artifact.lbTrackingValid = m_lastPoseStitchArtifact.lbTrackingValid;
        artifact.baseCalibrationT0 = m_lastPoseStitchArtifact.baseCalibrationT0;
        artifact.calibrationT0Prime = m_lastPoseStitchArtifact.calibrationT0Prime;
        artifact.stereoTrackingT = m_lastPoseStitchArtifact.stereoTrackingT;
        artifact.combinedOutputRt = m_lastPoseStitchArtifact.combinedOutputRt;
        artifact.stitchedPointCloud = clonePointCloudFrame(m_lastPoseStitchArtifact.stitchedPointCloud);
        artifact.valid = true;
    }

    if (m_poseStitchRunRootDirectory.isEmpty()) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("[PoseStitchOutput] 跳过落盘：运行输出目录未就绪");
        scan_tracking::mech_eye::releasePointCloudFrameBuffers(&artifact.stitchedPointCloud);
        return;
    }

    const QString timestamp = m_poseStitchOutputTimestamp.isEmpty()
        ? poseStitchOutputTimestamp()
        : m_poseStitchOutputTimestamp;
    const QString baseName = QStringLiteral("%1_path%2_seg%3")
                                 .arg(timestamp)
                                 .arg(artifact.pathId)
                                 .arg(artifact.segmentIndex);

    const QString cloudDirectory = poseStitchPointCloudOutputDirectory(m_poseStitchRunRootDirectory);
    const QString plyFilePath = QDir(cloudDirectory).filePath(baseName + QStringLiteral("_stitched.ply"));

    if (!scan_tracking::mech_eye::savePointCloudFrameToPly(artifact.stitchedPointCloud, plyFilePath)) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("[PoseStitchOutput] 写入点云失败：") << plyFilePath;
    } else {
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("[PoseStitchOutput] 拼接点云已写入") << plyFilePath
            << QStringLiteral(" 点数=") << artifact.stitchedPointCloud.pointCount;
    }

    scan_tracking::mech_eye::releasePointCloudFrameBuffers(&artifact.stitchedPointCloud);
}

void StateMachine::persistMergedInspectionPointCloudToDisk(
    int pathId,
    int mergedSegmentCount,
    const scan_tracking::mech_eye::PointCloudFrame& mergedCloud) const
{
    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    if (configManager == nullptr || !configManager->segmentCaptureExportConfig().saveRawPointCloud) {
        return;
    }

    if (pathId <= 0 || mergedSegmentCount <= 0 || !mergedCloud.isValid()) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("[PoseStitchOutput] 跳过融合点云落盘：无效输入 pathId=") << pathId
            << QStringLiteral(" 段数=") << mergedSegmentCount;
        return;
    }

    if (m_poseStitchRunRootDirectory.isEmpty()) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("[PoseStitchOutput] 跳过融合点云落盘：运行输出目录未就绪");
        return;
    }

    const QString timestamp = m_poseStitchOutputTimestamp.isEmpty()
        ? poseStitchOutputTimestamp()
        : m_poseStitchOutputTimestamp;
    const QString baseName = QStringLiteral("%1_path%2_merged_%3segs")
                                 .arg(timestamp)
                                 .arg(pathId)
                                 .arg(mergedSegmentCount);

    const QString cloudDirectory = poseStitchPointCloudOutputDirectory(m_poseStitchRunRootDirectory);
    const QString plyFilePath =
        QDir(cloudDirectory).filePath(baseName + QStringLiteral("_inspection.ply"));

    if (!scan_tracking::mech_eye::savePointCloudFrameToPly(mergedCloud, plyFilePath)) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("[PoseStitchOutput] 写入检测融合点云失败：") << plyFilePath;
        return;
    }

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("[PoseStitchOutput] 检测融合点云已写入") << plyFilePath
        << QStringLiteral(" 点数=") << mergedCloud.pointCount
        << QStringLiteral(" 参与段数=") << mergedSegmentCount;
}

bool StateMachine::ensureSegmentCaptureExportSessionRoot()
{
    if (!m_segmentCaptureExportSessionRoot.isEmpty()) {
        return true;
    }

    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    const QString configuredRoot = configManager != nullptr
        ? configManager->segmentCaptureExportConfig().outputRoot
        : QStringLiteral("output");
    const QString outputRoot = configuredRoot.trimmed().isEmpty()
        ? QStringLiteral("output")
        : configuredRoot;

    const QString appDir = QCoreApplication::applicationDirPath();
    const QString baseDir = QFileInfo(outputRoot).isAbsolute()
        ? outputRoot
        : QDir(appDir).filePath(outputRoot);
    if (!ensureDirectoryTreeExists(baseDir)) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("[SegmentCaptureExport] 创建输出根目录失败：") << baseDir;
        return false;
    }

    m_segmentCaptureExportSessionTimestamp = poseStitchOutputTimestamp();
    const QString sessionRoot = QDir(baseDir).filePath(
        QStringLiteral("session_") + m_segmentCaptureExportSessionTimestamp);
    if (!ensureDirectoryTreeExists(sessionRoot)) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("[SegmentCaptureExport] 创建会话目录失败：") << sessionRoot;
        m_segmentCaptureExportSessionTimestamp.clear();
        return false;
    }

    m_segmentCaptureExportSessionRoot = sessionRoot;
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("[SegmentCaptureExport] 会话输出目录=") << m_segmentCaptureExportSessionRoot;
    return true;
}

void StateMachine::persistSegmentCaptureExportGroup(
    int pathId,
    int segmentIndex,
    const scan_tracking::vision::MultiCameraCaptureBundle& bundle,
    const scan_tracking::mech_eye::PointCloudFrame& rawPointCloud,
    const scan_tracking::mech_eye::PointCloudFrame& stitchedPointCloud,
    const SegmentPoseStitchRecord& stitchRecord)
{
    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    if (configManager == nullptr || !configManager->segmentCaptureExportConfig().enabled) {
        return;
    }
    if (pathId <= 0 || segmentIndex <= 0) {
        return;
    }
    if (!ensureSegmentCaptureExportSessionRoot()) {
        return;
    }

    const QString groupDir =
        segmentCaptureExportGroupDirectory(m_segmentCaptureExportSessionRoot, pathId, segmentIndex);
    if (!ensureDirectoryTreeExists(groupDir)) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("[SegmentCaptureExport] 创建分组目录失败：") << groupDir;
        return;
    }

    SegmentCaptureCxpExportMeta cxpExportMeta = buildCxpExportMetaFromFlatOutput(bundle);
    cxpExportMeta.exportGroupId =
        buildSegmentCaptureExportGroupId(pathId, segmentIndex, bundle.request.requestId);
    cxpExportMeta.sessionRoot = m_segmentCaptureExportSessionRoot;

    const QString matrixPath = QDir(groupDir).filePath(QStringLiteral("matrix.txt"));
    if (!writeTextFileIfPossible(matrixPath, buildSegmentPoseStitchRtText(stitchRecord))) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("[SegmentCaptureExport] 矩阵写入失败：") << matrixPath;
    }

    const QString metaPath = QDir(groupDir).filePath(QStringLiteral("meta.txt"));
    if (!writeTextFileIfPossible(
            metaPath,
            buildSegmentCaptureExportMetaText(
                pathId,
                segmentIndex,
                bundle,
                rawPointCloud,
                stitchedPointCloud,
                stitchRecord,
                cxpExportMeta))) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("[SegmentCaptureExport] meta 写入失败：") << metaPath;
    }

    bool rawSaved = false;
    bool stitchedSaved = false;
    if (configManager->segmentCaptureExportConfig().saveRawPointCloud) {
        if (rawPointCloud.isValid()) {
            const QString rawPath = QDir(groupDir).filePath(QStringLiteral("pointcloud_raw.ply"));
            rawSaved = scan_tracking::mech_eye::savePointCloudFrameToPly(rawPointCloud, rawPath);
            if (!rawSaved) {
                qWarning(LOG_FLOW).noquote()
                    << QStringLiteral("[SegmentCaptureExport] 原始点云写入失败：") << rawPath;
            }
        }

        if (stitchedPointCloud.isValid()) {
            const QString stitchedPath = QDir(groupDir).filePath(QStringLiteral("pointcloud_stitched.ply"));
            stitchedSaved =
                scan_tracking::mech_eye::savePointCloudFrameToPly(stitchedPointCloud, stitchedPath);
            if (!stitchedSaved) {
                qWarning(LOG_FLOW).noquote()
                    << QStringLiteral("[SegmentCaptureExport] 拼接点云写入失败：") << stitchedPath;
            }
        }
    }

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("[SegmentCaptureExport] 分组已写入") << groupDir
        << QStringLiteral(" group=") << cxpExportMeta.exportGroupId
        << QStringLiteral(" 2D输出=") << resolveSegmentCaptureFlatOutputRoot()
        << QStringLiteral(" CXP左=") << (cxpExportMeta.left.saved ? QStringLiteral("OK") : QStringLiteral("FAIL"))
        << QStringLiteral(" CXP右=") << (cxpExportMeta.right.saved ? QStringLiteral("OK") : QStringLiteral("FAIL"))
        << QStringLiteral(" 左pixelMd5=") << cxpExportMeta.left.pixelMd5
        << QStringLiteral(" 右pixelMd5=") << cxpExportMeta.right.pixelMd5
        << QStringLiteral(" 原始PLY=") << (rawSaved ? QStringLiteral("OK") : QStringLiteral("SKIP"))
        << QStringLiteral(" 拼接PLY=") << (stitchedSaved ? QStringLiteral("OK") : QStringLiteral("SKIP"));
}

/**
 * @brief 记录 Modbus 通信失败
 * 
 * 递增连续失败计数器，当达到阈值时自动进入故障状态。
 * 这种机制可以容忍偶发的通信干扰，但对持续故障做出快速响应。
 * 
 * @param alarmCode 报警代码
 * @param message 错误描述信息
 */
void StateMachine::recordModbusFailure(quint16 alarmCode, const QString& message)
{
    ++m_consecutiveModbusFailures;
    qWarning(LOG_FLOW).noquote()
        << QStringLiteral("记录 Modbus 失败")
        << m_consecutiveModbusFailures << QStringLiteral("/") << kMaxConsecutiveModbusFailures
        << QStringLiteral(" alarmCode=") << alarmCode
        << QStringLiteral(" reason=") << message;

    // 如果连续失败次数达到阈值，进入故障状态
    if (m_consecutiveModbusFailures >= kMaxConsecutiveModbusFailures) {
        enterFaultState(alarmCode, message, true, true);
    }
}

/**
 * @brief 重置 Modbus 失败计数器
 * 
 * 在成功通信后调用，将连续失败计数器归零。
 */
void StateMachine::resetModbusFailureCounter()
{
    if (m_consecutiveModbusFailures > 0) {
        qInfo(LOG_FLOW) << QStringLiteral("通信成功后重置 Modbus 失败计数器。");
    }
    m_consecutiveModbusFailures = 0;
}

/**
 * @brief 进入故障状态
 * 
 * 设置报警信息，切换应用状态为 Error，并根据参数决定是否中止当前任务。
 * 这是系统处理严重错误的统一入口。
 * 
 * @param alarmCode 报警代码
 * @param message 错误描述信息
 * @param abortCurrentTask 是否中止当前活动任务
 * @param notifyPlc 是否通知 PLC（通过发送 Res/Ack）
 */
void StateMachine::enterFaultState(
    quint16 alarmCode,
    const QString& message,
    bool abortCurrentTask,
    bool notifyPlc)
{
    setAlarm(3, alarmCode, message);       // 设置严重错误级别报警
    m_ipcState = protocol::IpcState::Fault; // IPC 状态设为故障
    setState(AppState::Error);              // 应用状态设为错误

    if (abortCurrentTask) {
        abortActiveTaskForFault(7);  // 中止当前任务，Res=7 表示处理失败
    } else {
        m_timeoutTimer->stop();      // 停止超时定时器
        m_progress = 0;              // 进度归零
        m_currentStage = protocol::Stage::Idle;  // 阶段回到空闲
        publishIpcStatus();          // 发布故障状态
    }

    if (!notifyPlc) {
        clearActiveTask();           // 清除活动任务
        m_currentStage = protocol::Stage::Idle;
    }
}

/**
 * @brief 因故障中止当前活动任务
 * 
 * 根据任务类型采取不同的中止策略：
 * - 扫描分段任务：先写入空结果再完成
 * - 其他任务：直接清理状态
 * 如果 Modbus 可用，通过正常的 completeActiveTask 流程通知 PLC；
 * 否则直接清理本地状态。
 * 
 * @param resultCode 任务结果码（通常为 7 = 处理失败）
 */
void StateMachine::abortActiveTaskForFault(quint16 resultCode)
{
    if (m_activeTask.definition == nullptr) {
        // 没有活动任务，只需清理基本状态
        m_timeoutTimer->stop();
        m_progress = 0;
        m_dataValid = false;
        m_currentStage = protocol::Stage::Idle;
        publishIpcStatus();
        return;
    }

    // 如果是扫描分段任务，先写入空结果
    if (m_activeTask.definition->stage == protocol::Stage::ScanSegment) {
        writeScanSegmentResult(m_activeTask.scanSegmentIndex, 0, 0);
    }

    // 如果 Modbus 可用，通过正常流程通知 PLC
    if (m_modbus && m_modbus->isConnected()) {
        completeActiveTask(resultCode, protocol::AckState::Failed, false);
        return;
    }

    // Modbus 不可用时，直接清理本地状态
    m_timeoutTimer->stop();
    m_progress = 0;
    m_dataValid = false;
    m_activeTask.captureRequestId = 0;
    m_activeTask.completionAnnounced = false;
    clearActiveTask();
    m_currentStage = protocol::Stage::Idle;
    publishIpcStatus();
}

/**
 * @brief 将相机采集错误码映射为 PLC 结果码
 * 
 * 将 Mech-Eye 相机的内部错误码转换为 PLC 能理解的结果码：
 * - 1: 成功
 * - 5: 设备未就绪（连接失败、忙等）
 * - 6: 超时
 * - 7: 处理失败（未知错误）
 * - 9: 参数错误（无效请求）
 * 
 * @param errorCode 相机采集错误码
 * @return 对应的 PLC 结果码
 */
quint16 StateMachine::mapCaptureErrorToResCode(mech_eye::CaptureErrorCode errorCode) const
{
    switch (errorCode) {
    case mech_eye::CaptureErrorCode::Success:
        return 1;   // 成功
    case mech_eye::CaptureErrorCode::NotStarted:
    case mech_eye::CaptureErrorCode::NotConnected:
    case mech_eye::CaptureErrorCode::Busy:
    case mech_eye::CaptureErrorCode::DiscoverFailed:
    case mech_eye::CaptureErrorCode::ConnectFailed:
    case mech_eye::CaptureErrorCode::DisconnectFailed:
        return 5;   // 设备未就绪
    case mech_eye::CaptureErrorCode::Timeout:
        return 6;   // 超时
    case mech_eye::CaptureErrorCode::InvalidRequest:
        return 9;   // 参数错误
    default:
        return 7;   // 其他错误归为处理失败
    }
}

/**
 * @brief 从命令块中读取任务 ID
 * 
 * 任务 ID 是一个 32 位整数，存储在两个 16 位寄存器中（高16位和低16位）。
 * 
 * @param commandBlock 命令块寄存器数据
 * @return 32 位任务 ID
 */
quint32 StateMachine::readTaskId(const QVector<quint16>& commandBlock) const
{
    const quint32 high = static_cast<quint32>(commandBlock.value(protocol::registers::kTaskIdHigh));
    const quint32 low = static_cast<quint32>(commandBlock.value(protocol::registers::kTaskIdLow));
    return (high << 16) | low;  // 组合高16位和低16位
}

/**
 * @brief 解析扫描分段索引
 * 
 * 从命令块中读取当前请求的扫描分段索引。
 * 40015 为段号；PLC 常把 REAL（如 1.0f）以未转字序的原始字写入（如 16256=0x3F80）。
 * 通过 plcAnalogToUInt16 解码，不再固定按 40015/40016 高低字拼接。
 *
 * @param commandBlock 命令块寄存器数据
 * @return 扫描分段索引（从1开始）
 */
quint16 StateMachine::resolveScanSegmentIndex(const QVector<quint16>& commandBlock) const
{
    return protocol::registers::resolveScanSegmentIndexFromBlock(commandBlock);
}

/**
 * @brief 验证扫描分段请求的合法性
 * 
 * 检查分段索引是否在有效范围内，以及是否已经采集过该分段（重复检测）。
 * 
 * @param commandBlock 命令块寄存器数据
 * @param errorMessage 输出参数，如果验证失败则填充错误描述信息
 * @return true 如果请求合法，false 如果请求非法
 */
bool StateMachine::validateScanSegmentRequest(const QVector<quint16>& commandBlock, QString* errorMessage)
{
    if (isFirstPathStepPauseBlocking(errorMessage)) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("拒绝 Trig_ScanSegment：") << (errorMessage != nullptr ? *errorMessage : QString());
        return false;
    }

    const int segmentIndex = resolveScanSegmentIndex(commandBlock);  // 获取分段索引（32位合并）
    if (segmentIndex < 1 || segmentIndex > kMaxScanSegmentIndex) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("扫描段号无效：段号=%1，允许范围 1-%2")
                .arg(segmentIndex)
                .arg(kMaxScanSegmentIndex);
        }
        qWarning(LOG_FLOW).noquote() << QStringLiteral("拒绝 Trig_ScanSegment：段号无效")
                                     << QStringLiteral(" 段号=") << segmentIndex
                                     << QStringLiteral(" 允许最大=") << kMaxScanSegmentIndex;
        return false;
    }

    const int targetPathId = resolvePathIdForIncomingSegment(segmentIndex);
    const int segmentTotal = segmentTotalForPath(targetPathId);
    const int maxSegmentIndex = segmentTotal > 0 ? qMin(segmentTotal, kMaxScanSegmentIndex)
                                                 : kMaxScanSegmentIndex;
    if (segmentIndex > maxSegmentIndex) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("扫描段号无效：段号=%1，路径=%2 总段数=%3，允许范围 1-%3")
                .arg(segmentIndex)
                .arg(targetPathId)
                .arg(maxSegmentIndex);
        }
        qWarning(LOG_FLOW).noquote() << QStringLiteral("拒绝 Trig_ScanSegment：段号无效")
                                     << QStringLiteral(" 段号=") << segmentIndex
                                     << QStringLiteral(" 路径=") << targetPathId
                                     << QStringLiteral(" 总段数=") << segmentTotal
                                     << QStringLiteral(" 允许最大=") << maxSegmentIndex;
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(m_segmentCacheMutex);

        if (m_currentPathSegments.contains(segmentIndex) && !isCurrentPathSegmentSetComplete()) {
            if (errorMessage != nullptr) {
                *errorMessage = QStringLiteral(
                    "当前路径尚未扫满，拒绝重复段号：段号=%1，路径=%2")
                                      .arg(segmentIndex)
                                      .arg(m_currentPathId);
            }
            qWarning(LOG_FLOW).noquote()
                << QStringLiteral("拒绝 Trig_ScanSegment：路径内重复段号")
                << QStringLiteral(" 段号=") << segmentIndex
                << QStringLiteral(" 路径=") << m_currentPathId;
            return false;
        }

        if (m_currentPathSegments.contains(segmentIndex) && isCurrentPathSegmentSetComplete()) {
            const QVector<int> pathIds = enabledScanPathIds();
            if (pathIds.isEmpty() || pathIds.indexOf(targetPathId) < 0) {
                if (errorMessage != nullptr) {
                    *errorMessage = QStringLiteral("所有配置路径已扫描完成，无法再切换路径。");
                }
                return false;
            }
            if (targetPathId == m_currentPathId) {
                if (errorMessage != nullptr) {
                    *errorMessage = QStringLiteral("无下一路径可切换，段号=%1").arg(segmentIndex);
                }
                return false;
            }
        }

        if (m_pathSegmentCaptureResults.contains(targetPathId) &&
            m_pathSegmentCaptureResults[targetPathId].contains(segmentIndex)) {
            if (errorMessage != nullptr) {
                *errorMessage = QStringLiteral("路径 %1 段 %2 已缓存，拒绝重复扫描。")
                                    .arg(targetPathId)
                                    .arg(segmentIndex);
            }
            return false;
        }

        int cachedCount = 0;
        for (auto pathIt = m_pathSegmentCaptureResults.constBegin();
             pathIt != m_pathSegmentCaptureResults.constEnd();
             ++pathIt) {
            cachedCount += pathIt->size();
        }
        if (cachedCount >= kMaxPointCloudCacheSize) {
            if (errorMessage != nullptr) {
                *errorMessage = QStringLiteral("点云内存缓存已满：当前 %1 段，上限 %2")
                                    .arg(cachedCount)
                                    .arg(kMaxPointCloudCacheSize);
            }
            return false;
        }
    }

    return true;  // 验证通过
}

/**
 * @brief 完成扫描分段失败的处理
 * 
 * 统一的扫描分段失败出口，确保按照正确的顺序写入结果：
 * 1. 设置报警信息
 * 2. 写入空结果（0 图像数，0 点云帧数）
 * 3. 清除采集请求 ID
 * 4. 完成任务并标记为失败
 * 
 * 这样可以保证 PLC 不会读到旧数据或 inconsistent 的状态。
 * 
 * @param resultCode 结果码（5=设备未就绪，6=超时，7=处理失败，9=参数错误）
 * @param alarmLevel 报警级别（2=警告，3=严重错误）
 * @param alarmCode 报警代码
 * @param logMessage 日志消息（详细的技术描述）
 * @param alarmMessage 报警消息（显示给用户的简洁描述）
 */
void StateMachine::finishScanSegmentFailure(
    quint16 resultCode,
    quint16 alarmLevel,
    quint16 alarmCode,
    const QString& logMessage,
    const QString& alarmMessage)
{
    // 失败闭环也必须先写结果区，再写 Res，最后写 Ack=3，保证 PLC 不会读到旧数据。
    qWarning(LOG_FLOW).noquote()
        << QStringLiteral("Trig_ScanSegment 失败")
        << QStringLiteral(" 段号=") << m_activeTask.scanSegmentIndex
        << QStringLiteral(" Res=") << resultCode
        << QStringLiteral(" 原因=") << logMessage;
    setAlarm(alarmLevel, alarmCode, alarmMessage);              // 设置报警
    writeScanSegmentResult(m_activeTask.scanSegmentIndex, 0, 0); // 写入空结果
    
    // P0修复：严重错误时清理已缓存的扫描数据，防止内存泄漏
    if (resultCode >= 5) {
        qWarning(LOG_FLOW) << QStringLiteral("扫描失败，清空分段内存缓存，Res=") << resultCode;
        resetScanSegmentCache();
    }
    
    m_activeTask.captureRequestId = 0;                          // 清除采集请求 ID
    completeActiveTask(resultCode, protocol::AckState::Failed, false);  // 完成失败任务
}

}  // namespace scan_tracking::flow_control
