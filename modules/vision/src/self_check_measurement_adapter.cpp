#include "scan_tracking/vision/self_check_measurement_adapter.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <string>

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QFileInfo>

#include <opencv2/opencv.hpp>

#include "CodedMarkPointDetect.h"
#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/mech_eye/mech_eye_types.h"

namespace scan_tracking::vision::self_check {

namespace {

QString localPathFromEnv(const char* name)
{
    const char* value = std::getenv(name);
    if (value == nullptr || value[0] == '\0') {
        return {};
    }
    return QString::fromLocal8Bit(value);
}

QString defaultSelfCheckRootDirectory()
{
    return QCoreApplication::applicationDirPath() + QStringLiteral("/self_check");
}

struct IniSelfCheckConfig {
    float radiusMin = 30.0f;
    float radiusMax = 200.0f;
    float distErrMax = 1.0f;
    float interpolatedRadius = 5.0f;
    std::map<int, std::map<int, float>> id0Id1Dists;
};

std::string trimAscii(const std::string& text)
{
    const char* spaces = " \t\r\n";
    const std::string::size_type first = text.find_first_not_of(spaces);
    if (first == std::string::npos) {
        return {};
    }
    const std::string::size_type last = text.find_last_not_of(spaces);
    return text.substr(first, last - first + 1);
}

std::string stripIniComment(const std::string& line)
{
    bool inQuote = false;
    for (std::string::size_type i = 0; i < line.size(); ++i) {
        const char ch = line[i];
        if (ch == '"') {
            inQuote = !inQuote;
            continue;
        }
        if (!inQuote && (ch == ';' || ch == '#')) {
            return line.substr(0, i);
        }
        if (!inQuote && ch == '/' && i + 1 < line.size() && line[i + 1] == '/') {
            return line.substr(0, i);
        }
    }
    return line;
}

std::string toLowerAscii(std::string value)
{
    for (std::string::size_type i = 0; i < value.size(); ++i) {
        if (value[i] >= 'A' && value[i] <= 'Z') {
            value[i] = static_cast<char>(value[i] - 'A' + 'a');
        }
    }
    return value;
}

bool parseFloatValue(const std::string& value, float& output)
{
    char* end = nullptr;
    const float parsed = std::strtof(value.c_str(), &end);
    if (end == value.c_str()) {
        return false;
    }
    output = parsed;
    return true;
}

bool parseMarkerPair(const std::string& key, int& id0, int& id1)
{
    std::string normalized = key;
    std::replace(normalized.begin(), normalized.end(), '_', ',');
    std::replace(normalized.begin(), normalized.end(), '-', ',');

    const std::string::size_type comma = normalized.find(',');
    if (comma == std::string::npos) {
        return false;
    }

    const std::string first = trimAscii(normalized.substr(0, comma));
    const std::string second = trimAscii(normalized.substr(comma + 1));
    if (first.empty() || second.empty()) {
        return false;
    }

    id0 = std::atoi(first.c_str());
    id1 = std::atoi(second.c_str());
    return true;
}

bool loadSelfCheckIni(const QString& path, IniSelfCheckConfig& config, QString& error)
{
    std::ifstream input(path.toLocal8Bit().constData(), std::ios::binary);
    if (!input) {
        error = QStringLiteral("无法打开自检配置：") + path;
        return false;
    }

    std::string section;
    std::string line;
    int lineNo = 0;
    while (std::getline(input, line)) {
        ++lineNo;
        if (lineNo == 1 && line.size() >= 3U
            && static_cast<unsigned char>(line[0]) == 0xEF
            && static_cast<unsigned char>(line[1]) == 0xBB
            && static_cast<unsigned char>(line[2]) == 0xBF) {
            line.erase(0, 3);
        }

        line = trimAscii(stripIniComment(line));
        if (line.empty()) {
            continue;
        }

        if (line.front() == '[' && line.back() == ']') {
            section = toLowerAscii(trimAscii(line.substr(1, line.size() - 2)));
            continue;
        }

        const std::string::size_type equal = line.find('=');
        if (equal == std::string::npos) {
            error = QStringLiteral("自检配置第 %1 行格式错误").arg(lineNo);
            return false;
        }

        const std::string key = toLowerAscii(trimAscii(line.substr(0, equal)));
        const std::string value = trimAscii(line.substr(equal + 1));

        if (section == "detection") {
            float parsed = 0.0f;
            if (!parseFloatValue(value, parsed)) {
                error = QStringLiteral("自检配置第 %1 行浮点值无效").arg(lineNo);
                return false;
            }
            if (key == "radius_min") {
                config.radiusMin = parsed;
            } else if (key == "radius_max") {
                config.radiusMax = parsed;
            } else if (key == "dist_err_max") {
                config.distErrMax = parsed;
            } else if (key == "interpolated_radius") {
                config.interpolatedRadius = parsed;
            }
        } else if (section == "markerdistances") {
            int id0 = 0;
            int id1 = 0;
            float distance = 0.0f;
            if (!parseMarkerPair(key, id0, id1) || !parseFloatValue(value, distance)) {
                error = QStringLiteral("自检配置第 %1 行标记间距无效").arg(lineNo);
                return false;
            }
            config.id0Id1Dists[id0][id1] = distance;
        }
    }

    if (config.radiusMin <= 0.0f || config.radiusMax <= config.radiusMin) {
        error = QStringLiteral("自检配置半径范围无效");
        return false;
    }
    if (config.distErrMax < 0.0f || config.interpolatedRadius <= 0.0f) {
        error = QStringLiteral("自检配置误差或插值半径无效");
        return false;
    }
    if (config.id0Id1Dists.empty()) {
        error = QStringLiteral("自检配置未定义标记间距");
        return false;
    }
    return true;
}

cv::Mat grayTextureToCvMat(const scan_tracking::mech_eye::GrayTextureFrame& frame)
{
    cv::Mat image;
    if (!frame.isValid()) {
        return image;
    }

    image.create(frame.height, frame.width, CV_8UC1);
    const uint8_t* src = frame.pixels->data();
    for (int row = 0; row < frame.height; ++row) {
        uint8_t* dst = image.ptr<uint8_t>(row);
        const int offset = row * frame.width;
        for (int col = 0; col < frame.width; ++col) {
            dst[col] = src[static_cast<std::size_t>(offset + col)];
        }
    }
    return image;
}

bool isOrganizedPointCloud(const scan_tracking::mech_eye::PointCloudFrame& frame)
{
    return frame.isValid() && frame.width > 0 && frame.height > 0
        && frame.width * frame.height == frame.pointCount;
}

OrganizedPointCloud toOrganizedPointCloud(const scan_tracking::mech_eye::PointCloudFrame& frame)
{
    OrganizedPointCloud cloud;
    if (!isOrganizedPointCloud(frame) || frame.pointsXYZ == nullptr) {
        return cloud;
    }

    cloud.width = frame.width;
    cloud.height = frame.height;
    cloud.xyz = *frame.pointsXYZ;
    return cloud;
}

cv::Mat poseMatrixToRtMat(const PoseMatrix4x4& pose)
{
    cv::Mat rt(4, 4, CV_64F);
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            rt.at<double>(row, col) = static_cast<double>(
                pose.values[static_cast<std::size_t>(row * 4 + col)]);
        }
    }
    return rt;
}

QString validateCaptureBundle(const MultiCameraCaptureBundle& bundle, int segmentIndex)
{
    if (!bundle.mechEyeResult.success()) {
        return QStringLiteral("段 %1 Mech-Eye 采集失败：%2")
            .arg(segmentIndex)
            .arg(bundle.mechEyeResult.errorMessage);
    }
    if (bundle.mechEyeResult.mode != scan_tracking::mech_eye::CaptureMode::Capture2DAnd3D) {
        return QStringLiteral("段 %1 未使用 Capture2DAnd3D 模式").arg(segmentIndex);
    }
    if (!bundle.mechEyeResult.texture2D.isValid()) {
        return QStringLiteral("段 %1 缺少对齐灰度图").arg(segmentIndex);
    }
    if (!isOrganizedPointCloud(bundle.mechEyeResult.pointCloud)) {
        return QStringLiteral("段 %1 点云非组织化网格，无法插值").arg(segmentIndex);
    }
    if (!bundle.lbPoseResult.success || !bundle.lbPoseResult.poseMatrix.isValid()) {
        return QStringLiteral("段 %1 LB 位姿无效：%2")
            .arg(segmentIndex)
            .arg(bundle.lbPoseResult.message);
    }
    return {};
}

SelfCheckResult makeFailure(int algorithmCode, quint16 failWord0, const QString& message)
{
    SelfCheckResult result;
    result.ok = false;
    result.algorithmCode = algorithmCode;
    result.failWord0 = failWord0;
    result.message = message;
    return result;
}

quint16 mapAlgorithmCodeToFailWord0(int algorithmCode)
{
    switch (algorithmCode) {
    case 0:
        return 0;
    case 1999:
        return static_cast<quint16>(1u << 0);
    case 1998:
    case 1100:
    case 1200:
    case 1000:
        return static_cast<quint16>(1u << 2);
    default:
        if (algorithmCode >= 1300 && algorithmCode < 2000) {
            return static_cast<quint16>(1u << 2);
        }
        return static_cast<quint16>(1u << 3);
    }
}

}  // namespace

QString resolveSelfCheckConfigPath()
{
    const QString envPath = localPathFromEnv("SCAN_TRACKING_SELF_CHECK_CONFIG");
    if (!envPath.isEmpty()) {
        const QFileInfo envInfo(envPath);
        if (envInfo.isFile()) {
            return envInfo.absoluteFilePath();
        }
    }

    const auto* configMgr = scan_tracking::common::ConfigManager::instance();
    if (configMgr != nullptr) {
        const QString configured = configMgr->selfCheckConfig().configPath.trimmed();
        if (!configured.isEmpty()) {
            QFileInfo configuredInfo(configured);
            if (configuredInfo.isAbsolute() && configuredInfo.isFile()) {
                return configuredInfo.absoluteFilePath();
            }
            if (configMgr->configFilePath().isEmpty() == false) {
                const QDir configDir(QFileInfo(configMgr->configFilePath()).absolutePath());
                const QString relativePath = configDir.absoluteFilePath(configured);
                if (QFileInfo(relativePath).isFile()) {
                    return QFileInfo(relativePath).absoluteFilePath();
                }
            }
            const QString sourceRelative =
                QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(configured);
            if (QFileInfo(sourceRelative).isFile()) {
                return QFileInfo(sourceRelative).absoluteFilePath();
            }
        }
    }

    const QString defaultPath =
        defaultSelfCheckRootDirectory() + QStringLiteral("/self_check.ini");
    if (QFileInfo(defaultPath).isFile()) {
        return QFileInfo(defaultPath).absoluteFilePath();
    }

    const QString bundledPath = QCoreApplication::applicationDirPath()
        + QStringLiteral("/../third_party/Scanner_Self_Check/self_check.ini");
    return QFileInfo(bundledPath).absoluteFilePath();
}

SelfCheckResult runScannerSelfCheck(
    const MultiCameraCaptureBundle& frame0,
    const MultiCameraCaptureBundle& frame1)
{
    const QString frame0Error = validateCaptureBundle(frame0, 1);
    if (!frame0Error.isEmpty()) {
        return makeFailure(
            5,
            static_cast<quint16>(1u << 3),
            frame0Error);
    }
    const QString frame1Error = validateCaptureBundle(frame1, 2);
    if (!frame1Error.isEmpty()) {
        return makeFailure(
            5,
            static_cast<quint16>(1u << 3),
            frame1Error);
    }

    IniSelfCheckConfig iniConfig;
    QString iniError;
    const QString iniPath = resolveSelfCheckConfigPath();
    if (!loadSelfCheckIni(iniPath, iniConfig, iniError)) {
        return makeFailure(
            7,
            static_cast<quint16>(1u << 3),
            iniError);
    }

    CodedMark_SelfCheck checker;
    checker.radius_min = iniConfig.radiusMin;
    checker.radius_max = iniConfig.radiusMax;
    checker.dist_err_max = iniConfig.distErrMax;
    checker.interpolated_radius = iniConfig.interpolatedRadius;
    checker.id0_id1_dists = iniConfig.id0Id1Dists;

    checker.input_Img_set.push_back(grayTextureToCvMat(frame0.mechEyeResult.texture2D));
    checker.input_Img_set.push_back(grayTextureToCvMat(frame1.mechEyeResult.texture2D));
    checker.frame_cloud_set.push_back(toOrganizedPointCloud(frame0.mechEyeResult.pointCloud));
    checker.frame_cloud_set.push_back(toOrganizedPointCloud(frame1.mechEyeResult.pointCloud));
    checker.Rt_set.push_back(poseMatrixToRtMat(frame0.lbPoseResult.poseMatrix));
    checker.Rt_set.push_back(poseMatrixToRtMat(frame1.lbPoseResult.poseMatrix));

    const int algorithmCode = checker.Self_Check();

    SelfCheckResult result;
    result.algorithmCode = algorithmCode;
    result.ok = algorithmCode == 0;
    result.failWord0 = mapAlgorithmCodeToFailWord0(algorithmCode);
    result.failWord1 = 0;
    if (result.ok) {
        result.message = QStringLiteral("扫描仪自检合格");
    } else if (algorithmCode == 1999) {
        result.message = QStringLiteral("编码标记点间距超差，需重新标定");
    } else if (algorithmCode == 1998) {
        result.message = QStringLiteral("未检测到有效编码标记点");
    } else {
        result.message = QStringLiteral("扫描仪自检失败，算法码=%1").arg(algorithmCode);
    }
    return result;
}

}  // namespace scan_tracking::vision::self_check
