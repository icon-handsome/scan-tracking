#include "scan_tracking/mech_eye/point_cloud_io.h"

#include "scan_tracking/common/capture_cache_paths.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QImage>
#include <QLoggingCategory>
#include <QTextStream>

#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

Q_LOGGING_CATEGORY(LOG_POINT_CLOUD_IO, "mech_eye.point_cloud_io")

namespace scan_tracking::mech_eye {

namespace {

bool isFinitePoint(float x, float y, float z)
{
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

enum class PlyStorageFormat {
    Unknown = 0,
    Ascii,
    BinaryLittleEndian,
};

struct ParsedPlyHeader {
    PlyStorageFormat format = PlyStorageFormat::Unknown;
    int vertexCount = 0;
    bool hasNormals = false;
};

bool parsePlyHeader(QTextStream& stream, ParsedPlyHeader* header)
{
    if (header == nullptr) {
        return false;
    }

    *header = ParsedPlyHeader{};

    QString line = stream.readLine().trimmed();
    if (line != QStringLiteral("ply")) {
        return false;
    }

    bool inHeader = true;
    while (inHeader && !stream.atEnd()) {
        line = stream.readLine().trimmed();
        if (line.startsWith(QStringLiteral("format ascii"))) {
            header->format = PlyStorageFormat::Ascii;
        } else if (line.startsWith(QStringLiteral("format binary_little_endian"))) {
            header->format = PlyStorageFormat::BinaryLittleEndian;
        } else if (line.startsWith(QStringLiteral("element vertex"))) {
            header->vertexCount = line.section(QLatin1Char(' '), 2).toInt();
        } else if (line == QStringLiteral("property float nx")) {
            header->hasNormals = true;
        } else if (line == QStringLiteral("end_header")) {
            inHeader = false;
        }
    }

    return header->format != PlyStorageFormat::Unknown && header->vertexCount > 0;
}

bool assignLoadedPointCloud(
    PointCloudFrame* outFrame,
    std::shared_ptr<std::vector<float>> points,
    std::shared_ptr<std::vector<float>> normals,
    bool hasNormals)
{
    if (outFrame == nullptr || points == nullptr || points->empty()) {
        return false;
    }

    const int pointCount = static_cast<int>(points->size() / 3);
    outFrame->pointsXYZ = std::move(points);
    if (hasNormals && normals != nullptr && static_cast<int>(normals->size()) == pointCount * 3) {
        outFrame->normalsXYZ = std::move(normals);
    } else {
        outFrame->normalsXYZ.reset();
    }
    outFrame->pointCount = pointCount;
    outFrame->width = pointCount;
    outFrame->height = 1;
    return true;
}

bool loadAsciiPlyBody(QTextStream& stream, const ParsedPlyHeader& header, PointCloudFrame* outFrame)
{
    auto points = std::make_shared<std::vector<float>>();
    auto normals = std::make_shared<std::vector<float>>();
    points->reserve(static_cast<std::size_t>(header.vertexCount) * 3);
    if (header.hasNormals) {
        normals->reserve(static_cast<std::size_t>(header.vertexCount) * 3);
    }

    int loaded = 0;
    while (!stream.atEnd() && loaded < header.vertexCount) {
        const QString line = stream.readLine().trimmed();
        if (line.isEmpty()) {
            continue;
        }

        const QStringList tokens = line.split(QLatin1Char(' '), Qt::SkipEmptyParts);
        if (tokens.size() < 3) {
            continue;
        }

        const float x = tokens[0].toFloat();
        const float y = tokens[1].toFloat();
        const float z = tokens[2].toFloat();
        if (!isFinitePoint(x, y, z)) {
            continue;
        }

        points->push_back(x);
        points->push_back(y);
        points->push_back(z);

        if (header.hasNormals && tokens.size() >= 6) {
            normals->push_back(tokens[3].toFloat());
            normals->push_back(tokens[4].toFloat());
            normals->push_back(tokens[5].toFloat());
        } else if (header.hasNormals) {
            normals->push_back(0.0f);
            normals->push_back(0.0f);
            normals->push_back(1.0f);
        }

        ++loaded;
    }

    return assignLoadedPointCloud(outFrame, std::move(points), std::move(normals), header.hasNormals);
}

bool loadBinaryPlyBody(QFile& file, const ParsedPlyHeader& header, PointCloudFrame* outFrame)
{
    const int floatsPerVertex = header.hasNormals ? 6 : 3;
    const qint64 byteCount =
        static_cast<qint64>(header.vertexCount) * floatsPerVertex * static_cast<qint64>(sizeof(float));
    const QByteArray body = file.read(byteCount);
    if (body.size() != byteCount) {
        qWarning(LOG_POINT_CLOUD_IO) << QStringLiteral("loadPointCloudFrameFromPly：binary body 长度不足");
        return false;
    }

    const float* data = reinterpret_cast<const float*>(body.constData());
    auto points = std::make_shared<std::vector<float>>();
    auto normals = std::make_shared<std::vector<float>>();
    points->reserve(static_cast<std::size_t>(header.vertexCount) * 3);
    if (header.hasNormals) {
        normals->reserve(static_cast<std::size_t>(header.vertexCount) * 3);
    }

    for (int index = 0; index < header.vertexCount; ++index) {
        const int base = index * floatsPerVertex;
        const float x = data[base];
        const float y = data[base + 1];
        const float z = data[base + 2];
        if (!isFinitePoint(x, y, z)) {
            continue;
        }

        points->push_back(x);
        points->push_back(y);
        points->push_back(z);

        if (header.hasNormals) {
            normals->push_back(data[base + 3]);
            normals->push_back(data[base + 4]);
            normals->push_back(data[base + 5]);
        }
    }

    return assignLoadedPointCloud(outFrame, std::move(points), std::move(normals), header.hasNormals);
}

}  // namespace

QString defaultScanCacheDirectory()
{
    return scan_tracking::common::defaultCaptureCacheRoot();
}

QString buildSegmentPlyPath(
    const QString& configuredRoot,
    int segmentIndex,
    quint32 taskId,
    const QString& timestamp)
{
    const QString baseDir = scan_tracking::common::captureCacheMech3DDir(configuredRoot);
    if (baseDir.isEmpty()) {
        qWarning(LOG_POINT_CLOUD_IO).noquote() << "无法创建 mech_3d 缓存目录";
        return QString();
    }

    const QString ts =
        timestamp.trimmed().isEmpty() ? scan_tracking::common::buildCaptureTimestamp() : timestamp;
    const QString fileName =
        QStringLiteral("segment_%1_task%2_%3.ply").arg(segmentIndex).arg(taskId).arg(ts);
    return QDir(baseDir).absoluteFilePath(fileName);
}

QString buildComparisonPlyPath(
    const QString& configuredRoot,
    int segmentIndex,
    quint32 taskId,
    bool noiseRemovalNormal,
    const QString& timestamp)
{
    const QString baseDir = scan_tracking::common::captureCacheMech3DDir(configuredRoot);
    if (baseDir.isEmpty()) {
        qWarning(LOG_POINT_CLOUD_IO).noquote() << "无法创建 mech_3d 缓存目录";
        return QString();
    }

    const QString compareDir =
        QDir(baseDir).absoluteFilePath(QStringLiteral("compare"));
    const QString modeDir =
        QDir(compareDir).absoluteFilePath(noiseRemovalNormal
            ? QStringLiteral("noise_on")
            : QStringLiteral("noise_off"));
    if (!scan_tracking::common::ensureDirectoryExists(modeDir).isEmpty()) {
        const QString ts =
            timestamp.trimmed().isEmpty() ? scan_tracking::common::buildCaptureTimestamp() : timestamp;
        const QString fileName =
            QStringLiteral("segment_%1_task%2_%3_cmp.ply").arg(segmentIndex).arg(taskId).arg(ts);
        return QDir(modeDir).absoluteFilePath(fileName);
    }

    qWarning(LOG_POINT_CLOUD_IO).noquote() << "无法创建比较版点云目录:" << modeDir;
    return QString();
}

bool savePointCloudFrameToPly(const PointCloudFrame& frame, const QString& absolutePath)
{
    if (!frame.isValid() || absolutePath.trimmed().isEmpty()) {
        qWarning(LOG_POINT_CLOUD_IO) << QStringLiteral("savePointCloudFrameToPly：帧或路径无效");
        return false;
    }

    const auto& points = *frame.pointsXYZ;

    const int pointCount = frame.pointCount;
    const int availablePointCount = static_cast<int>(points.size() / 3);
    const int count = std::min(pointCount, availablePointCount);
    if (count <= 0) {
        qWarning(LOG_POINT_CLOUD_IO) << QStringLiteral("savePointCloudFrameToPly：无有效点");
        return false;
    }

    std::size_t validCount = 0;
    for (int index = 0; index < count; ++index) {
        const auto base = static_cast<std::size_t>(index * 3);
        if (isFinitePoint(points[base], points[base + 1], points[base + 2])) {
            ++validCount;
        }
    }

    if (validCount == 0) {
        qWarning(LOG_POINT_CLOUD_IO) << QStringLiteral("savePointCloudFrameToPly：全部为 NaN 点");
        return false;
    }

    std::vector<float> vertexBuffer;
    vertexBuffer.reserve(validCount * 3);
    for (int index = 0; index < count; ++index) {
        const auto base = static_cast<std::size_t>(index * 3);
        const float x = points[base];
        const float y = points[base + 1];
        const float z = points[base + 2];
        if (!isFinitePoint(x, y, z)) {
            continue;
        }

        vertexBuffer.push_back(x);
        vertexBuffer.push_back(y);
        vertexBuffer.push_back(z);
    }

    QFileInfo fileInfo(absolutePath);
    QDir().mkpath(fileInfo.absolutePath());

    std::ofstream ofs(absolutePath.toStdString(), std::ios::binary);
    if (!ofs.is_open()) {
        qWarning(LOG_POINT_CLOUD_IO).noquote()
            << QStringLiteral("savePointCloudFrameToPly：无法打开") << absolutePath;
        return false;
    }

    const std::string header =
        "ply\n"
        "format binary_little_endian 1.0\n"
        "element vertex " +
        std::to_string(validCount) +
        "\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "end_header\n";
    ofs.write(header.data(), static_cast<std::streamsize>(header.size()));
    if (!vertexBuffer.empty()) {
        ofs.write(reinterpret_cast<const char*>(vertexBuffer.data()),
                  static_cast<std::streamsize>(vertexBuffer.size() * sizeof(float)));
    }

    if (!ofs.good()) {
        qWarning(LOG_POINT_CLOUD_IO).noquote()
            << QStringLiteral("savePointCloudFrameToPly：写入失败") << absolutePath;
        return false;
    }

    ofs.close();

    qInfo(LOG_POINT_CLOUD_IO).noquote()
        << QStringLiteral("PLY 已保存：") << absolutePath
        << QStringLiteral(" format=binary_xyz validPoints=") << validCount
        << QStringLiteral("/") << count;

    return true;
}

bool loadPointCloudFrameFromPly(const QString& absolutePath, PointCloudFrame* outFrame)
{
    if (outFrame == nullptr || absolutePath.trimmed().isEmpty()) {
        return false;
    }

    QFile file(absolutePath);
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning(LOG_POINT_CLOUD_IO).noquote()
            << QStringLiteral("loadPointCloudFrameFromPly：无法打开") << absolutePath;
        return false;
    }

    ParsedPlyHeader header;
    QTextStream stream(&file);
    if (!parsePlyHeader(stream, &header)) {
        qWarning(LOG_POINT_CLOUD_IO) << QStringLiteral("loadPointCloudFrameFromPly：PLY 头无效");
        return false;
    }

    bool loaded = false;
    if (header.format == PlyStorageFormat::BinaryLittleEndian) {
        const qint64 bodyOffset = stream.pos();
        if (!file.seek(bodyOffset)) {
            qWarning(LOG_POINT_CLOUD_IO) << QStringLiteral("loadPointCloudFrameFromPly：无法定位 binary body");
            return false;
        }
        loaded = loadBinaryPlyBody(file, header, outFrame);
    } else if (header.format == PlyStorageFormat::Ascii) {
        loaded = loadAsciiPlyBody(stream, header, outFrame);
    } else {
        qWarning(LOG_POINT_CLOUD_IO) << QStringLiteral("loadPointCloudFrameFromPly：不支持的 PLY 格式");
        return false;
    }

    file.close();

    if (!loaded || !outFrame->isValid()) {
        qWarning(LOG_POINT_CLOUD_IO).noquote()
            << QStringLiteral("loadPointCloudFrameFromPly：无有效点") << absolutePath;
        return false;
    }

    qInfo(LOG_POINT_CLOUD_IO).noquote()
        << QStringLiteral("PLY 已加载：") << absolutePath
        << QStringLiteral(" pointCount=") << outFrame->pointCount
        << QStringLiteral(" hasNormals=") << outFrame->hasNormals()
        << QStringLiteral(" format=")
        << (header.format == PlyStorageFormat::BinaryLittleEndian ? QStringLiteral("binary")
                                                                    : QStringLiteral("ascii"));

    return true;
}

void releasePointCloudFrameBuffers(PointCloudFrame* frame)
{
    if (frame == nullptr) {
        return;
    }
    frame->pointsXYZ.reset();
    frame->normalsXYZ.reset();
}

QString buildSegmentMech2DPngPath(
    const QString& configuredRoot,
    int segmentIndex,
    quint32 taskId,
    const QString& timestamp)
{
    const QString baseDir = scan_tracking::common::captureCacheMech2DDir(configuredRoot);
    if (baseDir.isEmpty()) {
        qWarning(LOG_POINT_CLOUD_IO).noquote() << "无法创建 mech_2d 缓存目录";
        return QString();
    }

    const QString ts =
        timestamp.trimmed().isEmpty() ? scan_tracking::common::buildCaptureTimestamp() : timestamp;
    const QString fileName =
        QStringLiteral("segment_%1_task%2_%3.png").arg(segmentIndex).arg(taskId).arg(ts);
    return QDir(baseDir).absoluteFilePath(fileName);
}

bool saveGrayTextureFrameToPng(const GrayTextureFrame& frame, const QString& absolutePath)
{
    if (!frame.isValid() || absolutePath.trimmed().isEmpty()) {
        return false;
    }

    QImage image(frame.width, frame.height, QImage::Format_Grayscale8);
    if (image.isNull()) {
        return false;
    }

    for (int row = 0; row < frame.height; ++row) {
        auto* scanLine = image.scanLine(row);
        const auto offset = static_cast<std::size_t>(row * frame.width);
        for (int col = 0; col < frame.width; ++col) {
            scanLine[col] = (*frame.pixels)[offset + static_cast<std::size_t>(col)];
        }
    }

    if (!image.save(absolutePath, "PNG")) {
        qWarning(LOG_POINT_CLOUD_IO).noquote() << QStringLiteral("saveGrayTextureFrameToPng 失败：") << absolutePath;
        return false;
    }

    qInfo(LOG_POINT_CLOUD_IO).noquote()
        << QStringLiteral("Mech 2D PNG 已保存：") << absolutePath << frame.width << QStringLiteral("x") << frame.height;
    return true;
}

}  // namespace scan_tracking::mech_eye
