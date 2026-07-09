#include "scan_tracking/common/pcd_xyz_loader.h"

#include <QFile>
#include <QFileInfo>
#include <QLoggingCategory>
#include <QRegularExpression>
#include <QTextStream>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

Q_LOGGING_CATEGORY(LOG_PCD_XYZ_LOADER, "common.pcd_xyz_loader")

namespace scan_tracking::common {

namespace {

constexpr int kMaxPcdPoints = 50'000'000;

enum class PcdDataFormat {
    Unknown = 0,
    Binary,
    Ascii,
    BinaryCompressed,
};

struct ParsedPcdHeader {
    QStringList fields;
    std::vector<int> sizes;
    std::vector<char> types;
    std::vector<int> counts;
    int width = 0;
    int height = 1;
    int points = 0;
    PcdDataFormat dataFormat = PcdDataFormat::Unknown;
    int bytesPerPoint = 0;
    int xOffsetBytes = -1;
    int yOffsetBytes = -1;
    int zOffsetBytes = -1;
};

bool isFinitePoint(float x, float y, float z)
{
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

int computeVertexLoadStride(int vertexCount, int maxPointCount)
{
    if (vertexCount <= 0 || maxPointCount <= 0 || vertexCount <= maxPointCount) {
        return 1;
    }
    return (vertexCount + maxPointCount - 1) / maxPointCount;
}

bool parseIntList(const QString& line, const QString& key, std::vector<int>* outValues)
{
    if (outValues == nullptr || !line.startsWith(key)) {
        return false;
    }
    outValues->clear();
    const QStringList tokens = line.mid(key.size()).trimmed().split(
        QRegularExpression(QStringLiteral("\\s+")), Qt::SkipEmptyParts);
    for (const QString& token : tokens) {
        bool ok = false;
        const int value = token.toInt(&ok);
        if (!ok) {
            return false;
        }
        outValues->push_back(value);
    }
    return !outValues->empty();
}

bool parseTypeList(const QString& line, const QString& key, std::vector<char>* outValues)
{
    if (outValues == nullptr || !line.startsWith(key)) {
        return false;
    }
    outValues->clear();
    const QStringList tokens = line.mid(key.size()).trimmed().split(
        QRegularExpression(QStringLiteral("\\s+")), Qt::SkipEmptyParts);
    for (const QString& token : tokens) {
        if (token.size() != 1) {
            return false;
        }
        outValues->push_back(token.at(0).toLatin1());
    }
    return !outValues->empty();
}

bool finalizePcdHeader(ParsedPcdHeader* header)
{
    if (header == nullptr) {
        return false;
    }

    const int fieldCount = header->fields.size();
    if (fieldCount <= 0
        || header->sizes.size() != fieldCount
        || header->types.size() != static_cast<std::size_t>(fieldCount)
        || header->counts.size() != fieldCount) {
        return false;
    }

    if (header->points <= 0) {
        if (header->width > 0 && header->height > 0) {
            header->points = header->width * header->height;
        } else {
            return false;
        }
    }

    if (header->points > kMaxPcdPoints) {
        qWarning(LOG_PCD_XYZ_LOADER).noquote()
            << QStringLiteral("PCD POINTS 超过上限：") << header->points;
        return false;
    }

    int offset = 0;
    for (int index = 0; index < fieldCount; ++index) {
        const int size = header->sizes[static_cast<std::size_t>(index)];
        const int count = header->counts[static_cast<std::size_t>(index)];
        if (size <= 0 || count <= 0) {
            return false;
        }

        const QString fieldName = header->fields.at(index).toLower();
        const char type = header->types[static_cast<std::size_t>(index)];
        if (fieldName == QStringLiteral("x") && type == 'F' && size == 4 && count == 1) {
            header->xOffsetBytes = offset;
        } else if (fieldName == QStringLiteral("y") && type == 'F' && size == 4 && count == 1) {
            header->yOffsetBytes = offset;
        } else if (fieldName == QStringLiteral("z") && type == 'F' && size == 4 && count == 1) {
            header->zOffsetBytes = offset;
        }

        offset += size * count;
    }

    header->bytesPerPoint = offset;
    return header->bytesPerPoint > 0
        && header->xOffsetBytes >= 0
        && header->yOffsetBytes >= 0
        && header->zOffsetBytes >= 0;
}

bool parsePcdHeaderLine(const QString& line, ParsedPcdHeader* header, bool* sawDataLine)
{
    if (header == nullptr || sawDataLine == nullptr) {
        return false;
    }

    if (line.isEmpty() || line.startsWith(QLatin1Char('#'))) {
        return true;
    }

    if (line.startsWith(QStringLiteral("FIELDS"))) {
        header->fields = line.mid(QStringLiteral("FIELDS").size())
                             .trimmed()
                             .split(QRegularExpression(QStringLiteral("\\s+")),
                                    Qt::SkipEmptyParts);
    } else if (line.startsWith(QStringLiteral("SIZE"))) {
        if (!parseIntList(line, QStringLiteral("SIZE"), &header->sizes)) {
            return false;
        }
    } else if (line.startsWith(QStringLiteral("TYPE"))) {
        if (!parseTypeList(line, QStringLiteral("TYPE"), &header->types)) {
            return false;
        }
    } else if (line.startsWith(QStringLiteral("COUNT"))) {
        if (!parseIntList(line, QStringLiteral("COUNT"), &header->counts)) {
            return false;
        }
    } else if (line.startsWith(QStringLiteral("WIDTH"))) {
        bool ok = false;
        header->width = line.mid(QStringLiteral("WIDTH").size()).trimmed().toInt(&ok);
        if (!ok) {
            return false;
        }
    } else if (line.startsWith(QStringLiteral("HEIGHT"))) {
        bool ok = false;
        header->height = line.mid(QStringLiteral("HEIGHT").size()).trimmed().toInt(&ok);
        if (!ok) {
            return false;
        }
    } else if (line.startsWith(QStringLiteral("POINTS"))) {
        bool ok = false;
        header->points = line.mid(QStringLiteral("POINTS").size()).trimmed().toInt(&ok);
        if (!ok) {
            return false;
        }
    } else if (line.startsWith(QStringLiteral("DATA"))) {
        const QString dataToken =
            line.mid(QStringLiteral("DATA").size()).trimmed().toLower();
        if (dataToken == QStringLiteral("binary")) {
            header->dataFormat = PcdDataFormat::Binary;
        } else if (dataToken == QStringLiteral("ascii")) {
            header->dataFormat = PcdDataFormat::Ascii;
        } else if (dataToken == QStringLiteral("binary_compressed")) {
            header->dataFormat = PcdDataFormat::BinaryCompressed;
        } else {
            return false;
        }
        *sawDataLine = true;
    }

    return true;
}

bool readPcdHeader(QFile& file, ParsedPcdHeader* header, qint64* bodyOffset)
{
    if (header == nullptr || bodyOffset == nullptr) {
        return false;
    }

    *header = ParsedPcdHeader{};
    *bodyOffset = -1;

    if (!file.seek(0)) {
        return false;
    }

    constexpr qint64 kMaxPcdHeaderBytes = 64 * 1024;
    qint64 consumedBytes = 0;
    bool sawDataLine = false;

    while (!file.atEnd()) {
        const QByteArray rawLine = file.readLine();
        if (rawLine.isEmpty()) {
            break;
        }

        consumedBytes += rawLine.size();
        if (consumedBytes > kMaxPcdHeaderBytes) {
            qWarning(LOG_PCD_XYZ_LOADER) << QStringLiteral("PCD header 过大，拒绝继续解析");
            return false;
        }

        const QString line = QString::fromLatin1(rawLine).trimmed();
        if (!parsePcdHeaderLine(line, header, &sawDataLine)) {
            return false;
        }
        if (sawDataLine) {
            *bodyOffset = file.pos();
            break;
        }
    }

    if (!sawDataLine || header->dataFormat == PcdDataFormat::Unknown || *bodyOffset < 0) {
        return false;
    }

    if (header->counts.empty() && !header->fields.isEmpty()) {
        header->counts.assign(header->fields.size(), 1);
    }

    return finalizePcdHeader(header);
}

float readFloatLE(const char* base, int offsetBytes)
{
    float value = 0.0f;
    std::memcpy(&value, base + offsetBytes, sizeof(float));
    return value;
}

bool appendFiniteXyz(float x, float y, float z, std::vector<float>* outXyz)
{
    if (!isFinitePoint(x, y, z)) {
        return false;
    }
    outXyz->push_back(x);
    outXyz->push_back(y);
    outXyz->push_back(z);
    return true;
}

bool loadBinaryPcdBody(QFile& file, const ParsedPcdHeader& header, std::vector<float>* outXyz, int maxPointCount)
{
    if (outXyz == nullptr || header.bytesPerPoint <= 0 || header.points <= 0) {
        return false;
    }

    const int stride = computeVertexLoadStride(header.points, maxPointCount);
    const int reserveCount =
        maxPointCount > 0 ? std::min(header.points, maxPointCount) : header.points;
    outXyz->clear();
    outXyz->reserve(static_cast<std::size_t>(reserveCount) * 3);

    std::vector<char> record(static_cast<std::size_t>(header.bytesPerPoint));
    int storedCount = 0;

    for (int index = 0; index < header.points; ++index) {
        if (file.read(record.data(), header.bytesPerPoint) != header.bytesPerPoint) {
            qWarning(LOG_PCD_XYZ_LOADER) << QStringLiteral("PCD binary body 读取中断 index=") << index;
            break;
        }

        if ((index % stride) != 0) {
            continue;
        }

        const float x = readFloatLE(record.data(), header.xOffsetBytes);
        const float y = readFloatLE(record.data(), header.yOffsetBytes);
        const float z = readFloatLE(record.data(), header.zOffsetBytes);
        if (!appendFiniteXyz(x, y, z, outXyz)) {
            continue;
        }

        ++storedCount;
        if (maxPointCount > 0 && storedCount >= maxPointCount) {
            break;
        }
    }

    return storedCount > 0;
}

int asciiTokenIndexForField(const ParsedPcdHeader& header, const QString& fieldName)
{
    int tokenIndex = 0;
    for (int fieldIndex = 0; fieldIndex < header.fields.size(); ++fieldIndex) {
        if (header.fields.at(fieldIndex).compare(fieldName, Qt::CaseInsensitive) == 0) {
            return tokenIndex;
        }
        tokenIndex += header.counts[static_cast<std::size_t>(fieldIndex)];
    }
    return -1;
}

bool loadAsciiPcdBodyWithFieldTokens(
    QTextStream& stream,
    const ParsedPcdHeader& header,
    std::vector<float>* outXyz,
    int maxPointCount)
{
    const int xToken = asciiTokenIndexForField(header, QStringLiteral("x"));
    const int yToken = asciiTokenIndexForField(header, QStringLiteral("y"));
    const int zToken = asciiTokenIndexForField(header, QStringLiteral("z"));
    if (xToken < 0 || yToken < 0 || zToken < 0) {
        return false;
    }

    const int stride = computeVertexLoadStride(header.points, maxPointCount);
    outXyz->clear();
    const int reserveCount =
        maxPointCount > 0 ? std::min(header.points, maxPointCount) : header.points;
    outXyz->reserve(static_cast<std::size_t>(reserveCount) * 3);

    int storedCount = 0;
    int parsedIndex = 0;

    while (!stream.atEnd() && parsedIndex < header.points) {
        const QString line = stream.readLine().trimmed();
        if (line.isEmpty()) {
            continue;
        }

        const QStringList tokens = line.split(QRegularExpression(QStringLiteral("\\s+")), Qt::SkipEmptyParts);
        const int requiredTokens = std::max({xToken, yToken, zToken}) + 1;
        if (tokens.size() < requiredTokens) {
            qWarning(LOG_PCD_XYZ_LOADER) << QStringLiteral("PCD ascii 行字段不足");
            return false;
        }

        if ((parsedIndex % stride) == 0) {
            bool okX = false;
            bool okY = false;
            bool okZ = false;
            const float x = tokens.at(xToken).toFloat(&okX);
            const float y = tokens.at(yToken).toFloat(&okY);
            const float z = tokens.at(zToken).toFloat(&okZ);
            if (okX && okY && okZ && appendFiniteXyz(x, y, z, outXyz)) {
                ++storedCount;
                if (maxPointCount > 0 && storedCount >= maxPointCount) {
                    break;
                }
            }
        }

        ++parsedIndex;
    }

    return storedCount > 0;
}

}  // namespace

bool loadPointCloudXyzFromPcd(
    const QString& absolutePath,
    std::vector<float>* outXyz,
    int maxPointCount)
{
    if (outXyz == nullptr || absolutePath.trimmed().isEmpty()) {
        return false;
    }

    QFile file(absolutePath);
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning(LOG_PCD_XYZ_LOADER).noquote()
            << QStringLiteral("无法打开 PCD：") << absolutePath;
        return false;
    }

    ParsedPcdHeader header;
    qint64 bodyOffset = -1;
    if (!readPcdHeader(file, &header, &bodyOffset)) {
        qWarning(LOG_PCD_XYZ_LOADER).noquote()
            << QStringLiteral("PCD 头解析失败：") << absolutePath;
        return false;
    }

    if (header.dataFormat == PcdDataFormat::BinaryCompressed) {
        qWarning(LOG_PCD_XYZ_LOADER).noquote()
            << QStringLiteral("暂不支持 binary_compressed PCD：") << absolutePath;
        return false;
    }

    bool loaded = false;
    if (header.dataFormat == PcdDataFormat::Binary) {
        if (!file.seek(bodyOffset)) {
            qWarning(LOG_PCD_XYZ_LOADER).noquote()
                << QStringLiteral("PCD binary body 定位失败：") << absolutePath;
            return false;
        }
        loaded = loadBinaryPcdBody(file, header, outXyz, maxPointCount);
    } else if (header.dataFormat == PcdDataFormat::Ascii) {
        if (!file.seek(bodyOffset)) {
            qWarning(LOG_PCD_XYZ_LOADER).noquote()
                << QStringLiteral("PCD ascii body 定位失败：") << absolutePath;
            return false;
        }
        QTextStream bodyStream(&file);
        bodyStream.setCodec("UTF-8");
        loaded = loadAsciiPcdBodyWithFieldTokens(bodyStream, header, outXyz, maxPointCount);
    }

    if (!loaded || outXyz->empty()) {
        qWarning(LOG_PCD_XYZ_LOADER).noquote()
            << QStringLiteral("PCD 无有效 xyz 点：") << absolutePath;
        return false;
    }

    qInfo(LOG_PCD_XYZ_LOADER).noquote()
        << QStringLiteral("PCD xyz 已加载：") << QFileInfo(absolutePath).fileName()
        << QStringLiteral(" pointCount=") << (outXyz->size() / 3)
        << QStringLiteral(" maxPointCount=") << maxPointCount;

    return true;
}

}  // namespace scan_tracking::common
