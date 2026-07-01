#pragma once

// Po_Kou 坡口测量算法适配层。
//
// 在线/离线路径：PointCloudFrame 或 PLY/PCD 文件 → mech_eye 解析（梅卡 camera 头、中文路径）
// → 内存 float xyz 缓冲 → Po_Kou solveBevelFromXyzBuffer（PCL 仅在 po_kou 模块内分配/释放）。
// TXT 等文本点云仍直接传路径给 Po_Kou solveBevelFromPointCloudFile。
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <QtCore/QString>

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/mech_eye/mech_eye_types.h"

namespace scan_tracking::vision::bevel {

/// Po_Kou 求解选项（由工艺配方 BevelRecipe 与公差映射而来）
struct BevelSolveOptions {
    int forcedBevelType = -1;          ///< 强制坡口类型，-1 表示自动识别
    bool overrideStandard = false;     ///< 是否用下方范围覆盖 config.txt 标准值
    double standardAngleMinDeg = 0.0;  ///< 坡口角下限（度）
    double standardAngleMaxDeg = 0.0;  ///< 坡口角上限（度）
    double standardLengthMin = 0.0;    ///< 钝边长度下限（mm）
    double standardLengthMax = 0.0;    ///< 钝边长度上限（mm）
};

/// 坡口测量单次输出（与 TrackingService::InspectionMeasurement 字段对应）
struct BevelInspectionResult {
    bool invoked = false;   ///< 是否已进入算法（stub 构建可能为 false）
    bool ok = false;        ///< 算法是否成功（几何求解完成）；合格判定见 qualityCode
    int bevelType = -1;     ///< 坡口类型编号（V1.2 起取自工艺配方，非算法识别）
    float angleDeg = 0.0f;  ///< 实测坡口角（度）
    float lengthMm = 0.0f;  ///< 实测钝边长度（mm）
    float icpFitness = 0.0f;///< ICP 拟合度，越小越好
    int qualityCode = 10000;///< IPC 公差判定：0=合格，1=角度超差，2=长度超差，3=均超差；算法失败时 10000
    QString message;        ///< 失败原因或摘要
};

/// 将 Mech-Eye 点云帧转换为 PCL 点云（供测试与其它模块复用）。
pcl::PointCloud<pcl::PointXYZ>::Ptr toPclPointCloud(
    const scan_tracking::mech_eye::PointCloudFrame& frame);

/// 解析 config.ini / 环境变量 / exe 旁 bevel 目录，得到 config 绝对路径。
/// V1.2 单模板单 config：优先返回当前配方类型对应的 config_type{N}.txt，缺省回退 config.txt。
QString resolveBevelConfigPath();

/// 解析模板目录绝对路径（可为空，表示使用 config.txt 内相对路径）。
QString resolveBevelTemplateDir();

/// 由工艺配方与公差构造 Po_Kou 求解选项（供测试校验映射）。
BevelSolveOptions buildBevelSolveOptions(
    const scan_tracking::common::BevelRecipe& recipe,
    float angleTolDeg,
    float lengthTolMm);

/* 执行坡口测量（同步、阻塞；内部持有 PCL 全局锁）
 *
 * @param cloud         输入点云（通常为分段后处理结果）
 * @param recipe        工艺配方（坡口类型、标准角/长等）
 * @param angleTolDeg   角度公差（度）
 * @param lengthTolMm   长度公差（mm）
 * @return BevelInspectionResult
 */
BevelInspectionResult runBevelMeasurement(
    const scan_tracking::mech_eye::PointCloudFrame& cloud,
    const scan_tracking::common::BevelRecipe& recipe,
    float angleTolDeg,
    float lengthTolMm,
    int maxInputPointCount = 200000);

/// 从 PCD/PLY/TXT 点云文件执行坡口测量（PCL 加载与释放在 Po_Kou 模块内完成）。
BevelInspectionResult runBevelMeasurementFromPointCloudFile(
    const QString& cloudPath,
    const scan_tracking::common::BevelRecipe& recipe,
    float angleTolDeg,
    float lengthTolMm);

/* 使用 ConfigManager 当前激活配方与全局公差执行坡口测量 */
BevelInspectionResult runBevelMeasurement(
    const scan_tracking::mech_eye::PointCloudFrame& cloud);

}  // namespace scan_tracking::vision::bevel
