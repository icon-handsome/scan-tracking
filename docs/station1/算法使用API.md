# 坡口测量算法（Po_Kou_Ce_Liang）集成说明

本文档说明 `third_party/Po_Kou_Ce_Liang` 在 scan-tracking 项目中的用途、API 与主流程接入方式。

## 目录与构建

- 算法源码：`third_party/Po_Kou_Ce_Liang`
- CMake 目标：`po_kou_ce_liang`（别名 `Bevel::Measurement`）
- 开关：`SCAN_TRACKING_ENABLE_BEVEL_MEASUREMENT=ON`
- 依赖：PCL 1.12、OpenCV 3.4.3（`third_party/opencv-3.4.3-vc14_vc15`，LB/LBN/Po_Kou 共用）

## 主流程

```
PLC Trig_Inspection
  → StateMachine::loadMergedPointCloudForInspection()   // 路径级点云合并
  → TrackingService::inspectPointCloud()
  → bevel_measurement_adapter::runBevelMeasurement()
  → bevel::solveBevelFromRawCloud()
  → HMI event.inspection.finished / PLC NG 寄存器
```

## 适配层 API

头文件：`modules/vision/include/scan_tracking/vision/bevel_measurement_adapter.h`

```cpp
scan_tracking::vision::bevel::BevelInspectionResult runBevelMeasurement(
    const scan_tracking::mech_eye::PointCloudFrame& cloud);
```

输出映射到 `InspectionMeasurement`：

| 字段 | 含义 |
|------|------|
| `head_angle_tol` | 坡口角（deg） |
| `blunt_height_tol` | 钝边长度（mm） |
| `bevel_type` | 坡口类型 |
| `icp_fitness` | ICP 拟合度 |
| `quality_code` | 0=合格 |

## 配置

`config.ini`：

```ini
[Bevel]
configPath=bevel/config.txt
templateDir=bevel/data/templates
angleTolDeg=2.0
lengthTolMm=1.0
defaultBevelType=0
defaultAngleDeg=45.0
defaultLengthMm=1.0
```

### 两种标准坡口型号

| `bevel_type` | 坡口角 | 钝边长度 | 模板 |
|--------------|--------|----------|------|
| 0 | 45° | 1 mm | `type_0_*` |
| 1 | 30° | 6 mm | `type_1_*` |

坡口型号与有孔/无孔**独立**：通过 `has_hole`（bool）区分，**未传默认 `false`（无孔）**。

| `has_hole` | 含义 | Core 行为 |
|------------|------|-----------|
| `false` | 无孔 | 记录为无孔，**不调用**测孔径算法；`headMetrics.hole_opening_mm` 为 0 |
| `true` | 有孔 | 记录为有孔；测孔算法接入后将填充 `hole_opening_mm` |

`cmd.get_config` 的 `bevel.presets` 返回坡口型号表；`bevel.recipe` 含当前生效配方（含 `has_hole`）。

Qt 换型示例（45° 无孔，仅传型号时 `has_hole` 默认为 false）：

```json
{ "type": "cmd.set_bevel_recipe", "payload": { "bevel_type": 0 } }
```

30° 有孔：

```json
{ "type": "cmd.set_bevel_recipe", "payload": { "bevel_type": 1, "has_hole": true } }
```

也可显式下发（会覆盖标准值）：

```json
{ "type": "cmd.set_bevel_recipe", "payload": { "bevel_type": 0, "angle_deg": 45.0, "length": 1.0, "has_hole": false } }
```

环境变量：`SCAN_TRACKING_BEVEL_CONFIG_DIR`（指向含 `config.txt` 与 `data/` 的目录）。

运行时资源由构建后处理复制到 exe 旁 `bevel/` 目录。

## 算法 API（第三方）

```cpp
#include "BevelMeasurement.h"

bevel::BevelMeasurementResult solveBevelFromRawCloud(
    const bevel::CloudT::ConstPtr& rawCloud,
    const std::string& configPath,
    const std::string& templateDir = {});
```

## 测试

- `scan_tracking_bevel_smoke_test`：空云拒绝、PCL 转换
- `scan_tracking_tracking_smoke_test`：`inspectPointCloud` 链路

## 常见问题

1. **配置文件找不到**：确认 exe 旁存在 `bevel/config.txt`，或设置 `SCAN_TRACKING_BEVEL_CONFIG_DIR`。
2. **模板加载失败**：确认 `bevel/data/templates/type_0_*` 已部署。
3. **PCL/OpenCV DLL**：使用与 LB 相同的运行时部署（`scan_tracking_deploy_pcl_runtime` / `scan_tracking_deploy_opencv_runtime`）。

---

# LB 双目位姿算法（third_party/LB）集成说明

封头段（非转动点位）由 CXP 双目驱动 LB 算法，输出 `Rt_global`；IPC 将其作为 **T0** 直接变换梅卡点云（`p' = p × T0`），不再与 JSON `T0'` 及段位姿 `T` 做 `T0'×T` 组合。

## 目录与构建

- 算法源码：`third_party/LB`（`TR_Mark_*` + `AppConfig`）
- CMake 目标：`lb_pose_detection`（别名 `LbPoseDetection::lb_pose_detection`）
- 开关：`SCAN_TRACKING_ENABLE_LB_POSE_DETECTION`（默认 ON）
- 适配层：`modules/vision/src/lb_pose_detection_adapter.cpp` → `runLbPoseDetection()`

## 配置（算法参数以 track_config.ini 为准）

| 文件 | 职责 |
|------|------|
| `third_party/LB/track_config.ini` | **权威**：`[Recon]` 双目标定与重建约束、`[GeoHash]` 含 `scan_to_marker_RT`、`[Detector]`/`[Limits]` |
| `config.ini` `[LbPose]` | IPC 部署：`trackConfigFile` 指向上述 ini；`templateFile` 可覆盖模板路径 |

```ini
[LbPose]
trackConfigFile=third_party/LB/track_config.ini
templateFile=third_party/LB/data/template-3D-ALL-Shift-Cut-Cut.txt
```

算法侧更新标定或约束时，**只需替换 `track_config.ini`**（保持 UTF-8 BOM），无需再改 `config.ini` 中的矩阵字段。

## 主流程

- 转盘段（`needMechEye2D=true`）：仅 LBN，跳过 LB
- 封头段：CXP 双目 → `runLbPoseDetection()` → `bundle.lbPoseResult.poseMatrix`（Rt_global）
- 状态机：`applySegmentPoseStitching()` — LB 成功时 `T0 = Rt_global`；失败时回退 LBN 链 `T0'`

## HMI

`cmd.get_config` 的 `lbPose` 字段：`trackConfigFile`、`templateFile`、`dataRoot`（离线调试用）。

---

# 柱面/开孔测量算法（HeadMeasure / Hole）集成说明

本文档说明 `third_party/Hole` 在 scan-tracking 项目中的用途、API 与主流程接入方式。

## 目录与构建

- 算法源码：`third_party/Hole`
- CMake 目标：`head_measure`（别名 `Hole::Measurement`）
- 开关：`SCAN_TRACKING_ENABLE_HOLE_MEASUREMENT=ON`
- 依赖：PCL 1.12

## 主流程（与坡口并存）

```
PLC Trig_Inspection
  → StateMachine::loadMergedPointCloudForInspection(pathId)
  → TrackingService::inspectPointCloud(cloud, count, inspectionPathId)
  → [path inspectionType == hole]
      hole_measurement_adapter::runHoleMeasurement()
      → hm::MeasurePipeline::runWithScanCloud()
  → HMI event.inspection.finished / PLC NG 寄存器
```

路径分流由 `scan_paths_config.json` 每条路径的 `inspectionType` 决定：

| `inspectionType` | 算法 |
|------------------|------|
| `bevel`（默认） | Po_Kou 坡口测量 |
| `hole` | HeadMeasure 柱面/开孔测量 |

示例：

```json
{ "pathId": 1, "inspectionType": "bevel", ... }
{ "pathId": 2, "inspectionType": "hole", "holeConfigPath": "hole/config/path2.json", ... }
```

## 适配层 API

头文件：`modules/vision/include/scan_tracking/vision/hole_measurement_adapter.h`

```cpp
scan_tracking::vision::hole::HoleInspectionResult runHoleMeasurement(
    const scan_tracking::mech_eye::PointCloudFrame& cloud,
    int inspectionPathId = 0);
```

## headMetrics 映射

| Hole `MeasureResult` | HMI `headMetrics` |
|----------------------|-------------------|
| `innerDiameterMm` | `inner_diameter_mm` |
| `innerCircumferenceMm` | `inner_circumference_mm` |
| `roundnessToleranceMm` | `roundness_tol` |
| `straightSideSlopeDeg` | `straight_slope_tol` |
| `straightSideHeightMm` | `straight_height_mm` |
| `opening.centerToInnerWallDistanceMm` | `hole_opening_mm` |
| `opening.axisToHeadAxisAngleDeg` | `joint_fit_up_angle_deg` |

## 配置

`config.ini`：

```ini
[Hole]
configPath=hole/config/default.json
icpRmsMaxMm=5.0
cylinderRmsMaxMm=3.0
```

Hole 算法 JSON（`hole/config/default.json`）含模板点云路径、裁剪盒、开孔特征等；扫描点云由 IPC 内存注入，`input_frames` 留空即可。

模板 PCD 部署目录：`hole/template/`（与 JSON 中 `template_cloud` 相对路径对应）。

环境变量：`SCAN_TRACKING_HOLE_CONFIG_DIR`（指向含 `default.json` 的目录）。

运行时资源由构建后处理复制到 exe 旁 `hole/config/`。
