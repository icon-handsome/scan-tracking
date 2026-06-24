#pragma once

#include <QSettings>
#include <QString>
#include <QVector>
#include <QtCore/QtGlobal>
#include <array>
#include <mutex>
#include <vector>

#include "scan_tracking/common/station_profile.h"

namespace scan_tracking {
namespace common {

struct AppConfig {
    QString version;
    QString environment;
};

struct LoggerConfig {
    int level;
    // 保留字段：当前未实现自动清理/轮转，历史日志永久保留。
    int rotateDays;
};

struct ModbusConfig {
    QString host;
    int port;
    int unitId;
    int timeoutMs;
    int reconnectIntervalMs;
};

struct CameraConfig {
    QString defaultCamera;
    int scanTimeoutMs;
};

struct VisionCameraEndpointConfig {
    QString logicalName;
    QString cameraKey;
    QString ipAddress;
    QString serialNumber;
    // "exclusive"（默认，独占控制）或 "monitor"（只读监控，允许与 SCMVS 共存）
    QString accessMode = QStringLiteral("exclusive");
};

struct VisionConfig {
    QString mechEyeCameraKey;
    int mechCaptureTimeoutMs;
    int mechDepthRangeMin;
    int mechDepthRangeMax;
    /// 调试：同次额外采一帧（Noise=Off），与主流程（Noise=Normal）对比落盘
    bool mechPointCloudProcessingComparisonEnabled = false;
    int hikConnectTimeoutMs;
    int hikCaptureTimeoutMs;
    float hikExposureTimeUs = 50000.0f;  ///< 海康 A/B 双目曝光（微秒），连接时写入相机
    float hikGain = 0.0f;              ///< 海康 A/B 增益（dB）
    QString hikSdkRoot;
    VisionCameraEndpointConfig hikCameraA;
    VisionCameraEndpointConfig hikCameraB;
    VisionCameraEndpointConfig hikCameraC;
    bool hikCxpEnabled = false;
    int hikCxpCaptureTimeoutMs = 5000;
    float hikCxpExposureTimeUs = 50000.0f;
    float hikCxpGain = 0.0f;
    // CXP 触发模式：0=连续，1=软件触发，2=硬件触发(Line0)
    quint32 hikCxpTriggerMode = 1;
    QString hikCxpSmokeOutputDir;
    VisionCameraEndpointConfig hikCxpCameraA;
    VisionCameraEndpointConfig hikCxpCameraB;
    // 智能相机 C 的 TCP 服务端参数（IPC 作为服务端，相机主动连入）
    QString hikCameraCTcpListenIp;
    quint16 hikCameraCTcpListenPort;
    // 智能相机 C 的 FTP 落图目录（FileZilla Server 存图根目录）
    QString hikCameraCFtpDirectory;
};

struct FlowControlConfig {
    int pollIntervalMs;
    int heartbeatIntervalMs;
    int simulatedProcessingMs;
    /// 临时联调：跳过检测/位姿/点云后处理算法；Trig_ScanSegment 仍执行相机采集
    bool algorithmBypassEnabled = false;
    /// 第一条扫描路径联调：0=不暂停跑完全流程；N>0 时路径1第 N 点采集且 CXP 落盘完成后暂停，拒绝后续 Trig_ScanSegment
    int firstPathPauseAfterPoint = 0;
    /// @deprecated 分段点云/海康已改内存缓存；仅 LatencyTest 等调试落盘仍可读此路径
    QString scanCacheDirectory;
    /// @deprecated 不再用于分段 PLY 生命周期
    bool retainSegmentPly = true;
};

/// 每段扫描分组落盘：CXP 左右 2D 图（扁平 output/）+ 矩阵/meta（会话子目录）
struct SegmentCaptureExportConfig {
    bool enabled = false;
    QString outputRoot = QStringLiteral("output");
    /// false 时不落盘任何 PLY（段级 raw/stitched、Trig_Inspection 融合点云）；矩阵与 2D 仍按 enabled 落盘
    bool saveRawPointCloud = true;
};

/// Mech-Eye 点云 IPC 后处理（深度裁剪 / 离群 / 平滑 / 降采样）
struct PointCloudProcessingConfig {
    bool enabled = true;
    float depthMinMm = 100.0f;
    float depthMaxMm = 2000.0f;
    bool outlierRemovalEnabled = true;
    int outlierMeanK = 50;
    float outlierStddevMul = 1.0f;
    bool smoothingEnabled = true;
    float mlsSearchRadiusMm = 5.0f;
    int mlsPolynomialOrder = 2;
    bool downsampleEnabled = true;
    float voxelLeafSizeMm = 2.0f;
    int minPointsAfterProcessing = 1000;
};

struct TrackingConfig {
    int scanSegmentTotal = 3;  // 扫描段总数（从 config.ini 获取，PLC不下发）
};

/// 坡口工艺配方（HMI 下发或 config.ini 默认值）
struct BevelRecipe {
    bool active = false;
    int bevelType = 0;
    float angleDeg = 0.0f;
    float lengthMm = 0.0f;
    bool hasHole = false; ///< 是否有孔；未下发时默认 false（无孔）
};

/// 标准坡口型号（与 Po_Kou type_{n} 模板一一对应）
struct BevelRecipePreset {
    int bevelType = 0;
    QString name;
    float angleDeg = 0.0f;
    float lengthMm = 0.0f;
};

/// 两种量产型号：type0=45°/1mm，type1=30°/6mm
QVector<BevelRecipePreset> standardBevelRecipePresets();

/// 按 bevel_type 查标准型号；未知 type 返回空 recipe（active=false）
BevelRecipe bevelRecipePresetForType(int bevelType);

/// 坡口测量（Po_Kou）算法配置（[Bevel]）
struct BevelConfig {
    QString configPath = QStringLiteral("bevel/config.txt");
    QString templateDir = QStringLiteral("bevel/data/templates");
    float angleTolDeg = 2.0f;
    float lengthTolMm = 1.0f;
    BevelRecipe defaultRecipe;
};

/// 柱面/开孔测量（HeadMeasure）算法配置（[Hole]）
struct HoleConfig {
    QString configPath = QStringLiteral("hole/config/default.json");
    double icpRmsMaxMm = 5.0;
    double cylinderRmsMaxMm = 3.0;
};

/// 厚度测量算法配置（[Thickness]）
struct ThicknessConfig {
    QString configPath = QStringLiteral("thickness/config/thickness_config.json");
    double icpFitnessMax = 50.0;
};

/// 内表面测量算法配置（[InternalSurface]）：封头深度与容积
struct InternalSurfaceConfig {
    QString configPath = QStringLiteral("internal_surface/config/algorithm_config.json");
    int templateType = 1;
    double minDepthMm = 0.0;
    double minVolumeM3 = 0.0;
};

/// 扫描仪编码标记点距离自检（[SelfCheck]）
struct SelfCheckConfig {
    int totalPoints = 2;
    QString configPath = QStringLiteral("self_check/self_check.ini");
};

/// 综合检测算法类型（按 scan_paths 路径配置）
enum class InspectionType {
    Bevel,            ///< 坡口测量（Po_Kou）
    Hole,             ///< 柱面 / 开孔测量（HeadMeasure）
    Thickness,        ///< 厚度测量
    InternalSurface,  ///< 完整内表面测量
    CodeRead,         ///< 编号识别（海康 C / OCR，Trig_Inspection 按路径触发）
    Defect,           ///< 表面缺陷识别（海康 C，Trig_Inspection 按路径触发）
};

InspectionType inspectionTypeFromString(const QString& value);
QString inspectionTypeToString(InspectionType type);

/// HMI 显控 TCP 服务配置（[Hmi]）
struct OrbbecGeminiConfig {
    bool enabled = false;
    QString sdkRoot;
    QString serial;
    int deviceIndex = 0;
    int depthWidth = 640;
    int depthHeight = 480;
    int fps = 15;
    int captureTimeoutMs = 5000;
    int warmupFrameCount = 5;
    bool saveCaptureToDisk = true;
    QString captureCacheDir;
    bool enableColorStream = false;
    bool captureOnStart = true;
};

struct LivoxMid360Config {
    bool enabled = false;
    QString sdkRoot;
    /// Livox SDK2 JSON 配置，相对 sdkRoot 或绝对路径
    QString configFile;
    /// 空则连接第一台已发现设备
    QString serial;
    int discoveryTimeoutMs = 10000;
};

struct TfminiPlusConfig {
    bool enabled = false;
    QString portName;
    int baudRate = 115200;
    int collisionThresholdMm = 0;
    bool logFrames = false;
};

struct HmiConfig {
    bool enabled = true;       ///< 是否启动 HMI TCP 服务端
    quint16 tcpPort = 9900;    ///< 监听端口
    /// 是否允许显控发送 cmd.debug_trigger_inspection（用缓存点云跑坡口测量并推送，不写 PLC）
    bool allowDebugTriggerInspection = false;
    /// 全部启用路径扫描完成后是否推送预设 event.inspection.finished（临时演示）
    bool emitPresetInspectionOnPathsComplete = false;
};

struct LbPoseConfig {
    /// LB 算法权威配置（标定、GeoHash、Recon 等），对应 third_party/LB/track_config.ini
    QString trackConfigFile;
    /// 可选：覆盖 track_config.ini [Paths] template_points
    QString templateFile;
    /// 离线/回退：数据根目录与左右图 glob（在线 CXP 采集不使用）
    QString dataRoot;
    QString leftPattern;
    QString rightPattern;
    /// 新版 LB 查询角度容差（度）
    float angleToleranceDeg = 2.0f;
    /// 新版 LB 查询长度容差（mm）
    float lengthTolerance = 0.5f;
    /// 新版 LB 最低票数占比
    float minPercent = 0.5f;
};

/** LBN 位姿检测配置。生产环境请多工况标定，勿仅按单帧离线 success 放大容差。 */
struct LbnPoseConfig {
    bool enabled = true;
    /// TODO(marker): 转盘未装标记点联调时 true：跳过 LBN 检测，Rt 用 4×4 单位阵，T0' 不变。
    bool useIdentityRtWithoutMarkers = false;
    QString dataRoot;
    QString templateFile;
    float minDistance = 30.0f;   // mm，过小易纳入杂点三角形
    float maxDistance = 650.0f;
    float cosTolerance = 0.015f; // 过大易误匹配模板点
    float minPercent = 0.5f;     // 与 FastGeoHash::getResult 联动
    int cloudSearchRadiusPx = 20;
    int markerMinArea = 400;
    int markerMaxArea = 30000;
    int markerIntensityThreshold = 50;
    float markerDebscanDistPx = 300.0f;
};

/**
 * @brief 扫描点位配置
 *
 * 定义单个扫描点位的参数；转盘角度由 LBN/状态机运行时解算，不在配置中指定。
 */
struct ScanPointConfig {
    int pointIndex;           // 点位索引（从 1 开始）
    bool needRotation;        // 是否需要转动转盘（关键标志）
};

/**
 * @brief 扫描路径配置
 *
 * 定义一条完整的扫描路径，包含多个点位。
 */
struct ScanPathConfig {
    int pathId;               // 路径唯一标识符
    bool enabled;             // 是否启用此路径
    int totalPoints;          // 路径包含的点位总数
    InspectionType inspectionType = InspectionType::Bevel;  // 综合检测算法类型
    QString holeConfigPath;   // Hole JSON 配置（可选，缺省用 [Hole] configPath）
    QString thicknessConfigPath;  // 厚度 JSON 配置（可选，缺省用 [Thickness] configPath）
    int innerScanSegmentIndex = 0;  // 厚度：内表面段号（scan_paths 点位 pointIndex）
    int outerScanSegmentIndex = 0;  // 厚度：外表面段号
    std::vector<ScanPointConfig> points;  // 点位配置列表
};

/**
 * @brief 扫描路径总配置
 * 
 * 包含所有扫描路径配置、标定矩阵、执行策略等。
 * 从 scan_paths_config.json 文件加载。
 */
struct ScanPathsConfig {
    // 标定矩阵 T0（4x4 矩阵，行优先存储）
    std::array<float, 16> calibrationMatrixT0;
    
    // 所有扫描路径定义
    std::vector<ScanPathConfig> scanPaths;
    
    // 执行策略
    bool executeAllPaths;                // 是否执行所有启用的路径
    std::vector<int> selectedPathIds;    // 指定要执行的路径 ID 列表
    bool allowPathSkipOnError;           // 路径失败时是否跳过继续执行
    
    // 转盘配置
    bool turntableEnabled;               // 是否启用转盘控制
    
    // 配置文件元数据
    QString version;                     // 配置文件版本
    QString lastModified;                // 最后修改时间
};


class ConfigManager {
public:
    static void initialize();
    static void cleanup();
    static ConfigManager* instance();

    const AppConfig& appConfig() const;
    const LoggerConfig& loggerConfig() const;
    const ModbusConfig& modbusConfig() const;
    const CameraConfig& cameraConfig() const;
    const VisionConfig& visionConfig() const;
    const FlowControlConfig& flowControlConfig() const;
    const SegmentCaptureExportConfig& segmentCaptureExportConfig() const;
    const TrackingConfig& trackingConfig() const;
    const BevelConfig& bevelConfig() const;
    const HoleConfig& holeConfig() const;
    const ThicknessConfig& thicknessConfig() const;
    const InternalSurfaceConfig& internalSurfaceConfig() const;
    const SelfCheckConfig& selfCheckConfig() const;
    void setBevelRecipe(const BevelRecipe& recipe);
    BevelRecipe bevelRecipe() const;
    bool hasActiveBevelRecipe() const;
    InspectionType inspectionTypeForPath(int pathId) const;
    QString holeConfigPathForPath(int pathId) const;
    QString thicknessConfigPathForPath(int pathId) const;
    int innerScanSegmentIndexForPath(int pathId) const;
    int outerScanSegmentIndexForPath(int pathId) const;
    const OrbbecGeminiConfig& orbbecGeminiConfig() const;
    const LivoxMid360Config& livoxMid360Config() const;
    const TfminiPlusConfig& tfminiPlusConfig() const;
    QString configFilePath() const;
    const HmiConfig& hmiConfig() const;
    const LbPoseConfig& lbPoseConfig() const;
    const LbnPoseConfig& lbnPoseConfig() const;
    const PointCloudProcessingConfig& pointCloudProcessingConfig() const;
    const ScanPathsConfig& scanPathsConfig() const;  // 新增：获取扫描路径配置
    const StationProfile& stationProfile() const;

private:
    ConfigManager();
    ~ConfigManager();

    ConfigManager(const ConfigManager&) = delete;
    ConfigManager& operator=(const ConfigManager&) = delete;

    void load(const QString& filePath);
    void loadStationProfile(QSettings& settings, const QString& configFilePath);
    void writeDefaults(QSettings& settings);
    
    // 新增：加载扫描路径配置（JSON 格式）
    void loadScanPathsConfig(const QString& jsonFilePath);
    
    // 新增：验证扫描路径配置的合法性
    bool validateScanPathsConfig(QString* errorMessage = nullptr) const;

    static ConfigManager* s_instance;

    AppConfig m_appConfig;
    LoggerConfig m_loggerConfig;
    ModbusConfig m_modbusConfig;
CameraConfig m_cameraConfig;
    VisionConfig m_visionConfig;
    FlowControlConfig m_flowControlConfig;
    SegmentCaptureExportConfig m_segmentCaptureExportConfig;
    TrackingConfig m_trackingConfig;
    BevelConfig m_bevelConfig;
    HoleConfig m_holeConfig;
    ThicknessConfig m_thicknessConfig;
    InternalSurfaceConfig m_internalSurfaceConfig;
    SelfCheckConfig m_selfCheckConfig;
    mutable std::mutex m_bevelRecipeMutex;
    BevelRecipe m_runtimeBevelRecipe;
    bool m_runtimeRecipeSet = false;
    OrbbecGeminiConfig m_orbbecGeminiConfig;
    LivoxMid360Config m_livoxMid360Config;
    TfminiPlusConfig m_tfminiPlusConfig;
    QString m_configFilePath;
    HmiConfig m_hmiConfig;
    LbPoseConfig m_lbPoseConfig;
    LbnPoseConfig m_lbnPoseConfig;
    PointCloudProcessingConfig m_pointCloudProcessingConfig;
    StationProfile m_stationProfile;
    ScanPathsConfig m_scanPathsConfig;  // 新增：扫描路径配置
};

/// [SegmentCaptureExport] enabled=true 时输出 LB/LBN 矩阵、诊断明细及落盘日志
inline bool segmentCaptureExportEnabled()
{
    const ConfigManager* cm = ConfigManager::instance();
    return cm != nullptr && cm->segmentCaptureExportConfig().enabled;
}

}  // namespace common
}  // namespace scan_tracking
