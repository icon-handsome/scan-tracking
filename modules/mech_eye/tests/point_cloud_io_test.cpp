#include "scan_tracking/common/capture_cache_paths.h"
#include "scan_tracking/common/pcd_xyz_loader.h"
#include "scan_tracking/mech_eye/point_cloud_io.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QList>
#include <QTemporaryDir>
#include <QTextStream>
#include <QtTest/QtTest>

#include <limits>

#ifndef SCAN_TRACKING_SOURCE_DIR
#define SCAN_TRACKING_SOURCE_DIR "."
#endif

using namespace scan_tracking::mech_eye;

class PointCloudIoTest : public QObject {
    Q_OBJECT

private slots:
    void roundTripSaveLoad();
    void roundTripSaveLoadPcd();
    void mergeSegmentsToPcd();
    void loadLegacyAsciiPly();
    void plyPathUsesMech3dSubdir();
    void loadThirdPartyPcdWithPaddingField();
    void loadThirdPartyPcdWithRgbField();
};

void PointCloudIoTest::roundTripSaveLoad()
{
    PointCloudFrame frame;
    frame.pointsXYZ = std::make_shared<std::vector<float>>();
    frame.normalsXYZ = std::make_shared<std::vector<float>>();
    frame.pointsXYZ->insert(frame.pointsXYZ->end(), {
        0.0f, 0.0f, 0.0f,
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
    });
    frame.normalsXYZ->insert(frame.normalsXYZ->end(), {
        0.0f, 0.0f, 1.0f,
        0.0f, 0.0f, 1.0f,
        0.0f, 0.0f, 1.0f,
    });
    frame.pointCount = 3;
    frame.width = 3;
    frame.height = 1;
    frame.frameId = 42;

    QTemporaryDir tempDir;
    QVERIFY(tempDir.isValid());

    const QString ts = QStringLiteral("20260525_120000_000");
    const QString plyPath = buildSegmentPlyPath(tempDir.path(), 1, 100u, ts);
    QVERIFY(plyPath.contains(QStringLiteral("mech_3d")));
    QVERIFY(!plyPath.isEmpty());
    QVERIFY(savePointCloudFrameToPly(frame, plyPath));
    QVERIFY(QFile::exists(plyPath));

    QFile savedFile(plyPath);
    QVERIFY(savedFile.open(QIODevice::ReadOnly | QIODevice::Text));
    QTextStream headerStream(&savedFile);
    QVERIFY(headerStream.readLine().trimmed() == QStringLiteral("ply"));
    QVERIFY(headerStream.readLine().trimmed() == QStringLiteral("format binary_little_endian 1.0"));
    QVERIFY(headerStream.readLine().trimmed() == QStringLiteral("element vertex 3"));
    QVERIFY(headerStream.readLine().trimmed() == QStringLiteral("property float x"));
    QVERIFY(headerStream.readLine().trimmed() == QStringLiteral("property float y"));
    QVERIFY(headerStream.readLine().trimmed() == QStringLiteral("property float z"));
    QVERIFY(headerStream.readLine().trimmed() == QStringLiteral("end_header"));
    savedFile.close();

    releasePointCloudFrameBuffers(&frame);
    QVERIFY(!frame.isValid());

    PointCloudFrame loaded;
    QVERIFY(loadPointCloudFrameFromPly(plyPath, &loaded));
    QCOMPARE(loaded.pointCount, 3);
    QVERIFY(loaded.isValid());
    QVERIFY(!loaded.hasNormals());
    QCOMPARE(loaded.pointsXYZ->size(), static_cast<std::size_t>(9));
}

void PointCloudIoTest::roundTripSaveLoadPcd()
{
    PointCloudFrame frame;
    frame.pointsXYZ = std::make_shared<std::vector<float>>();
    frame.pointsXYZ->insert(frame.pointsXYZ->end(), {
        0.0f, 0.0f, 0.0f,
        1.0f, 2.0f, 3.0f,
        std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f,
        4.0f, 5.0f, 6.0f,
    });
    frame.pointCount = 4;
    frame.width = 4;
    frame.height = 1;

    QTemporaryDir tempDir;
    QVERIFY(tempDir.isValid());

    const QString pcdPath = tempDir.filePath(QStringLiteral("roundtrip.pcd"));
    QVERIFY(savePointCloudFrameToPcd(frame, pcdPath));
    QVERIFY(QFile::exists(pcdPath));

    PointCloudFrame loaded;
    QVERIFY(loadPointCloudFrameFromPcd(pcdPath, &loaded));
    QCOMPARE(loaded.pointCount, 3);
    QCOMPARE(loaded.pointsXYZ->size(), static_cast<std::size_t>(9));
    QCOMPARE((*loaded.pointsXYZ)[6], 4.0f);
    QCOMPARE((*loaded.pointsXYZ)[7], 5.0f);
    QCOMPARE((*loaded.pointsXYZ)[8], 6.0f);
}

void PointCloudIoTest::mergeSegmentsToPcd()
{
    auto makeFrame = [](std::initializer_list<float> xyz) {
        PointCloudFrame frame;
        frame.pointsXYZ = std::make_shared<std::vector<float>>(xyz);
        frame.pointCount = static_cast<int>(xyz.size() / 3);
        frame.width = frame.pointCount;
        frame.height = 1;
        return frame;
    };

    QList<PointCloudFrame> segments;
    segments.push_back(makeFrame({0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f}));
    segments.push_back(makeFrame({2.0f, 0.0f, 0.0f, 3.0f, 0.0f, 0.0f}));

    QTemporaryDir tempDir;
    QVERIFY(tempDir.isValid());

    const QString pcdPath = tempDir.filePath(QStringLiteral("merged.pcd"));
    int mergedPointCount = 0;
    QVERIFY(mergePointCloudFramesToPcd(segments, pcdPath, &mergedPointCount));
    QCOMPARE(mergedPointCount, 4);

    PointCloudFrame loaded;
    QVERIFY(loadPointCloudFrameFromPcd(pcdPath, &loaded));
    QCOMPARE(loaded.pointCount, 4);
}

void PointCloudIoTest::loadLegacyAsciiPly()
{
    QTemporaryDir tempDir;
    QVERIFY(tempDir.isValid());

    const QString plyPath = tempDir.filePath(QStringLiteral("legacy_ascii.ply"));
    QFile file(plyPath);
    QVERIFY(file.open(QIODevice::WriteOnly | QIODevice::Text));
    QTextStream out(&file);
    out << "ply\n";
    out << "format ascii 1.0\n";
    out << "element vertex 2\n";
    out << "property float x\n";
    out << "property float y\n";
    out << "property float z\n";
    out << "property float nx\n";
    out << "property float ny\n";
    out << "property float nz\n";
    out << "end_header\n";
    out << "0 0 0 0 0 1\n";
    out << "1 2 3 0 0 1\n";
    file.close();

    PointCloudFrame loaded;
    QVERIFY(loadPointCloudFrameFromPly(plyPath, &loaded));
    QCOMPARE(loaded.pointCount, 2);
    QVERIFY(loaded.hasNormals());
    QCOMPARE((*loaded.pointsXYZ)[3], 1.0f);
    QCOMPARE((*loaded.pointsXYZ)[4], 2.0f);
    QCOMPARE((*loaded.pointsXYZ)[5], 3.0f);
}

void PointCloudIoTest::plyPathUsesMech3dSubdir()
{
    QTemporaryDir tempDir;
    QVERIFY(tempDir.isValid());

    const QString root = scan_tracking::common::captureCacheMech3DDir(tempDir.path());
    QVERIFY(root.endsWith(QStringLiteral("mech_3d")));
}

void PointCloudIoTest::loadThirdPartyPcdWithPaddingField()
{
    const QString pcdPath = QDir(QStringLiteral(SCAN_TRACKING_SOURCE_DIR)).filePath(
        QStringLiteral("third_party/Po_Kou_Ce_Liang/data/templates/type_0_template.pcd"));
    QVERIFY2(QFile::exists(pcdPath), qPrintable(QStringLiteral("missing: ") + pcdPath));

    PointCloudFrame loaded;
    QVERIFY(loadPointCloudFrameFromPcd(pcdPath, &loaded));
    QVERIFY(loaded.pointCount > 1000);
    QVERIFY(loaded.isValid());
    QCOMPARE(loaded.pointsXYZ->size(), static_cast<std::size_t>(loaded.pointCount * 3));
}

void PointCloudIoTest::loadThirdPartyPcdWithRgbField()
{
    const QString pcdPath = QDir(QStringLiteral(SCAN_TRACKING_SOURCE_DIR)).filePath(
        QStringLiteral("third_party/Thicknessmeasurement/input/inner_surface_sample.pcd"));
    QVERIFY2(QFile::exists(pcdPath), qPrintable(QStringLiteral("missing: ") + pcdPath));

    std::vector<float> xyz;
    QVERIFY(scan_tracking::common::loadPointCloudXyzFromPcd(pcdPath, &xyz, 0));
    QVERIFY(xyz.size() >= 9);
    QCOMPARE(xyz.size() % 3, static_cast<std::size_t>(0));
}

QTEST_MAIN(PointCloudIoTest)
#include "point_cloud_io_test.moc"
