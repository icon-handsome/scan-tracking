#include "scan_tracking/common/capture_cache_paths.h"
#include "scan_tracking/mech_eye/point_cloud_io.h"

#include <QCoreApplication>
#include <QFile>
#include <QTemporaryDir>
#include <QTextStream>
#include <QtTest/QtTest>

using namespace scan_tracking::mech_eye;

class PointCloudIoTest : public QObject {
    Q_OBJECT

private slots:
    void roundTripSaveLoad();
    void loadLegacyAsciiPly();
    void plyPathUsesMech3dSubdir();
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

QTEST_MAIN(PointCloudIoTest)
#include "point_cloud_io_test.moc"
