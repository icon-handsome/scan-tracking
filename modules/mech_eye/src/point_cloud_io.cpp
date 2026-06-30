/**
 * @file point_cloud_io.cpp
 * @brief 点云 PLY 与 2D PNG 的读写实现
 *
 * PLY 保存策略：过滤 NaN/Inf 后写入 binary_little_endian xyz。
 * PLY 加载策略：解析 header 后按 ASCII 或 binary 分支读取，跳过无效点。
 */
#include "scan_tracking/mech_eye/point_cloud_io.h"

#include "scan_tracking/common/capture_cache_paths.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QImage>
#include <QLoggingCategory>
#include <QRegularExpression>
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

/** @brief 判断三维坐标是否为有限值（非 NaN/Inf） */
bool isFinitePoint(float x, float y, float z)
{
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

/** @brief PLY 存储格式枚举，由 header 中的 format 行解析 */
enum class PlyStorageFormat {
    Unknown = 0,
    Ascii,
    BinaryLittleEndian,
};

/** @brief PLY 文件头解析结果 */
struct ParsedPlyHeader {
    PlyStorageFormat format = PlyStorageFormat::Unknown;
    int vertexCount = 0;
    bool hasNormals = false;
};

/**
 * @brief 从 QTextStream 当前位置逐行解析 PLY header，直至 end_header
 * @return 解析成功且 format 已知、vertexCount > 0 时返回 true
 */
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

/** @brief 将解析后的点/法向数组写入 outFrame，并设置 pointCount/width/height */
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

/** @brief 读取 ASCII PLY body：每行 x y z [nx ny nz]，跳过无效点 */
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

/** @brief 读取 binary_little_endian PLY body：每顶点 3 或 6 个 float（xyz 或 xyz+法向） */
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

/** @brief 委托 common 模块获取默认采集缓存根目录 */
QString defaultScanCacheDirectory()
{
    return scan_tracking::common::defaultCaptureCacheRoot();
}

/**
 * @brief 生成分段主点云 PLY 路径
 *
 * 目录：<root>/mech_3d/
 * 命名：segment_{segmentIndex}_task{taskId}_{timestamp}.ply
 * timestamp 与海康 2D 图共用，便于同段数据关联检索。
 */
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

/**
 * @brief 生成对比采集 PLY 路径（NoiseRemoval 开/关各一目录）
 *
 * noise_on  → 主流程 Normal 滤波点云
 * noise_off → comparisonCaptureEnabled 时的 Off 滤波点云
 */
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

/**
 * @brief 将 PointCloudFrame 写入 binary_little_endian PLY
 *
 * 步骤：
 * 1. 校验 frame 与路径
 * 2. 第一遍扫描统计有限值点数（header 中 vertex 数量）
 * 3. 第二遍拷贝有效点到连续 buffer
 * 4. 自动创建父目录，写入 header + float32 xyz body
 *
 * @note 不写入 normals；保存后的 pointCount 可能小于原始 frame（NaN 被剔除）
 */
bool savePointCloudFrameToPly(const PointCloudFrame& frame, const QString& absolutePath)
{
    if (!frame.isValid() || absolutePath.trimmed().isEmpty()) {
        qWarning(LOG_POINT_CLOUD_IO) << QStringLiteral("savePointCloudFrameToPly：帧或路径无效");
        return false;
    }

    const auto& points = *frame.pointsXYZ;

    // pointCount 与 buffer 长度取 min，防止元数据与数组不一致
    const int pointCount = frame.pointCount;
    const int availablePointCount = static_cast<int>(points.size() / 3);
    const int count = std::min(pointCount, availablePointCount);
    if (count <= 0) {
        qWarning(LOG_POINT_CLOUD_IO) << QStringLiteral("savePointCloudFrameToPly：无有效点");
        return false;
    }

    // 预扫描：PLY header 的 vertex 数量必须为实际写入的有效点数
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

    // 紧凑拷贝有效顶点，避免 PLY 文件中夹杂 NaN
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

    // 使用 std::ofstream 二进制写入 body；Qt QFile 亦可，此处与 PCL 生态习惯一致
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

/**
 * @brief 从 PLY 加载点云至 PointCloudFrame
 *
 * 支持 format ascii 与 binary_little_endian。
 * 加载后 width=pointCount、height=1（非 organized 格式）；
 * 若 header 含 nx 属性则填充 normalsXYZ。
 */
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
        // QTextStream 解析 header 后文件指针可能偏移，binary body 需 seek 到准确位置
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

bool convertTxtPointCloudToPly(const QString& txtPath, const QString& plyPath)
{
    if (txtPath.trimmed().isEmpty() || plyPath.trimmed().isEmpty()) {
        return false;
    }

    QFile txtFile(txtPath);
    if (!txtFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qWarning(LOG_POINT_CLOUD_IO).noquote()
            << QStringLiteral("convertTxtPointCloudToPly：无法打开") << txtPath;
        return false;
    }

    auto countValidPoints = [](QTextStream& stream) -> int {
        int validCount = 0;
        while (!stream.atEnd()) {
            const QString line = stream.readLine().trimmed();
            if (line.isEmpty()) {
                continue;
            }
            const QStringList tokens = line.split(QRegularExpression(QStringLiteral("\\s+")),
                                                  Qt::SkipEmptyParts);
            if (tokens.size() < 3) {
                continue;
            }
            const float x = tokens[0].toFloat();
            const float y = tokens[1].toFloat();
            const float z = tokens[2].toFloat();
            if (isFinitePoint(x, y, z)) {
                ++validCount;
            }
        }
        return validCount;
    };

    QTextStream countStream(&txtFile);
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    countStream.setEncoding(QStringConverter::Utf8);
#else
    countStream.setCodec("UTF-8");
#endif
    const int validCount = countValidPoints(countStream);
    if (validCount <= 0) {
        qWarning(LOG_POINT_CLOUD_IO).noquote()
            << QStringLiteral("convertTxtPointCloudToPly：无有效点") << txtPath;
        return false;
    }

    const QFileInfo plyInfo(plyPath);
    QDir().mkpath(plyInfo.absolutePath());

    QFile plyFile(plyPath);
    if (!plyFile.open(QIODevice::WriteOnly)) {
        qWarning(LOG_POINT_CLOUD_IO).noquote()
            << QStringLiteral("convertTxtPointCloudToPly：无法写入") << plyPath;
        return false;
    }

    const QByteArray header = QByteArrayLiteral("ply\n")
        + QByteArrayLiteral("format binary_little_endian 1.0\n")
        + QByteArrayLiteral("element vertex ")
        + QByteArray::number(validCount)
        + QByteArrayLiteral("\nproperty float x\nproperty float y\nproperty float z\nend_header\n");
    if (plyFile.write(header) != header.size()) {
        return false;
    }

    if (!txtFile.seek(0)) {
        return false;
    }

    QTextStream writeStream(&txtFile);
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    writeStream.setEncoding(QStringConverter::Utf8);
#else
    writeStream.setCodec("UTF-8");
#endif

    std::vector<float> vertexBuffer;
    vertexBuffer.reserve(4096);
    auto flushVertices = [&]() -> bool {
        if (vertexBuffer.empty()) {
            return true;
        }
        const qint64 bytes = static_cast<qint64>(vertexBuffer.size() * sizeof(float));
        if (plyFile.write(reinterpret_cast<const char*>(vertexBuffer.data()), bytes) != bytes) {
            return false;
        }
        vertexBuffer.clear();
        return true;
    };

    while (!writeStream.atEnd()) {
        const QString line = writeStream.readLine().trimmed();
        if (line.isEmpty()) {
            continue;
        }
        const QStringList tokens = line.split(QRegularExpression(QStringLiteral("\\s+")),
                                              Qt::SkipEmptyParts);
        if (tokens.size() < 3) {
            continue;
        }
        const float x = tokens[0].toFloat();
        const float y = tokens[1].toFloat();
        const float z = tokens[2].toFloat();
        if (!isFinitePoint(x, y, z)) {
            continue;
        }
        vertexBuffer.push_back(x);
        vertexBuffer.push_back(y);
        vertexBuffer.push_back(z);
        if (vertexBuffer.size() >= 4096 * 3 && !flushVertices()) {
            return false;
        }
    }

    if (!flushVertices()) {
        return false;
    }

    qInfo(LOG_POINT_CLOUD_IO).noquote()
        << QStringLiteral("TXT 已转换为 PLY：") << txtPath
        << QStringLiteral(" -> ") << plyPath
        << QStringLiteral(" pointCount=") << validCount;
    return true;
}

bool loadPointCloudFrameFromTxt(const QString& absolutePath, PointCloudFrame* outFrame)
{
    if (outFrame == nullptr || absolutePath.trimmed().isEmpty()) {
        return false;
    }

    QFile file(absolutePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qWarning(LOG_POINT_CLOUD_IO).noquote()
            << QStringLiteral("loadPointCloudFrameFromTxt：无法打开") << absolutePath;
        return false;
    }

    auto points = std::make_shared<std::vector<float>>();
    points->reserve(300000 * 3);

    QTextStream stream(&file);
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    stream.setEncoding(QStringConverter::Utf8);
#else
    stream.setCodec("UTF-8");
#endif

    while (!stream.atEnd()) {
        const QString line = stream.readLine().trimmed();
        if (line.isEmpty()) {
            continue;
        }

        const QStringList tokens = line.split(QRegularExpression(QStringLiteral("\\s+")),
                                              Qt::SkipEmptyParts);
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
    }

    file.close();

    if (!assignLoadedPointCloud(outFrame, std::move(points), nullptr, false)) {
        qWarning(LOG_POINT_CLOUD_IO).noquote()
            << QStringLiteral("loadPointCloudFrameFromTxt：无有效点") << absolutePath;
        return false;
    }

    qInfo(LOG_POINT_CLOUD_IO).noquote()
        << QStringLiteral("TXT 已加载：") << absolutePath
        << QStringLiteral(" pointCount=") << outFrame->pointCount;

    return true;
}

/** @brief 释放大数组 shared_ptr，元数据字段保留供日志/索引使用 */
void releasePointCloudFrameBuffers(PointCloudFrame* frame)
{
    if (frame == nullptr) {
        return;
    }
    frame->pointsXYZ.reset();
    frame->normalsXYZ.reset();
}

/** @brief 生成分段 Mech 2D 灰度图 PNG 路径（mech_2d 子目录，命名规则同 PLY） */
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

/**
 * @brief GrayTextureFrame → 8 位灰度 PNG
 *
 * 逐行拷贝 pixels 到 QImage::Format_Grayscale8 后调用 QImage::save。
 * 调用前需确保 frame.isValid() 且父目录可写。
 */
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
        // pixels 为行优先 uint8 数组，与 QImage scanLine 逐列对应
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
