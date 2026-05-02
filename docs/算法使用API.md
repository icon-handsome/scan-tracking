# 蓝优算法使用说明 API

本文档面向 `scan-tracking` 项目的 Windows 集成场景，说明 `third_party/lanyou_first_detection` 中各检测类的用途、主要 API、当前项目内的最小接入方式，以及手动测试和常见问题。

## 1. 代码位置与当前集成方式

算法源码位于：
- `third_party/lanyou_first_detection`

当前项目中的测试适配层位于：
- `modules/vision/include/scan_tracking/vision/lanyou_detection_adapter.h`
- `modules/vision/src/lanyou_detection_adapter.cpp`
- `modules/vision/tests/lanyou_detection_adapter_smoke_test.cpp`

当前构建方式说明：
- 蓝优算法以源码方式参与编译。
- 默认编译为单一静态库 `lanyou_first_detection`。
- 目前只接入了测试专用适配层，尚未直接挂入正式业务状态机。
- 因此现阶段的目标是“先打通点云输入与算法调用链”，不是直接产出正式业务结果。

## 2. 依赖与编译要求

Windows 当前依赖：
- MSVC 2019
- CMake 3.21+
- Ninja
- Qt 5.15.2
- PCL 1.12.1
- OpenNI2 运行时

当前工程里与 PCL 相关的重要配置：
- `PCL_DIR=C:/Program Files/PCL 1.12.1/cmake`
- 运行测试程序时，需要把 Qt、PCL、OpenNI2 的 DLL 所在目录加入 `PATH`

注意：
- 蓝优算法是典型的 PCL/C++ 实现，不是 header-only 库。
- 不能只包含头文件使用，必须同时编译并链接其 `.cpp` 实现。
- 若后续需要封装为 DLL，可以在当前静态库方案稳定后再做。

## 3. 主要结果结构体

### 3.1 `FirstPoseDetectionParams`

定义位置：
- `third_party/lanyou_first_detection/include/utils/Params.h`

用途：
- 保存第一检测位的检测输出结果。
- 由 `FirstOutSurfaceDetection` 和 `FirstInlinerSurfaceDetection` 共同写入。

主要字段：
- `bbox_min_pt` / `bbox_max_pt`
  - 输入点云包围盒最小点、最大点。
- `cylinder_axis`
  - 封头圆柱轴方向向量。
- `cylinder_center`
  - 封头圆柱中心点。
- `inner_diameter`
  - 内径结果。
- `inner_diameter_tol`
  - 内径圆度/波动相关结果。
- `inner_circle_perimeter_tol`
  - 内圆周长相关结果。
- `head_depth_tol`
  - 封头深度相关结果。
- `hole_diameter_tol`
  - 开孔直径相关结果。
- `head_angle_tol`
  - 外表面坡口角结果。
- `blunt_height_tol`
  - 钝边高度结果。
- `straight_slope_tol`
  - 直边斜度结果。
- `straight_height_tol`
  - 直边高度结果。
- `inliner_error_log`
  - 第一检测位内表面流程错误信息。
- `outliner_error_log`
  - 第一检测位外表面流程错误信息。

说明：
- 这个结构体是第一检测位的统一结果载体。
- 同一轮检测中，内外表面算法一般写入同一个结果对象。

### 3.2 `SecondPoseDetectionParams`

定义位置：
- `third_party/lanyou_first_detection/include/utils/Params.h`

用途：
- 保存第二检测位的检测输出结果。

主要字段：
- `inner_diameter`
- `pipe_length`
- `inner_circle_perimeter_tol`
- `volume_tol`
- `A_welding_error_edge_tol`
- `A_welding_seam_edge_angle_tol`
- `A_welding_left_height_tol`
- `A_welding_undercut_tol`
- `B1_welding_error_edge_tol`
- `B1_welding_seam_edge_angle_tol`
- `B1_welding_left_height_tol`
- `B1_welding_undercut_tol`
- `B2_welding_error_edge_tol`
- `B2_welding_seam_edge_angle_tol`
- `B2_welding_left_height_tol`
- `B2_welding_undercut_tol`

说明：
- 第二检测位外表面结果主要面向焊缝相关特征。
- 当前项目里还没有把第二检测位接入到 smoke test。

## 4. 全局结果接口

定义位置：
- `third_party/lanyou_first_detection/include/utils/Params.h`

### `FirstPoseDetectionParams& GlobalFirstPoseParams()`

用途：
- 获取全局唯一的第一检测位结果对象。

说明：
- 蓝优第一检测位内部实现依赖这个全局结果对象。
- 不建议在多线程场景下直接并发复用。
- 如果后续业务需要并行跑多个工件，需要进一步做实例化隔离改造。

### `void ResetGlobalFirstPoseParams()`

用途：
- 在新一轮检测开始前清空上一轮第一检测位结果。

建议：
- 每次真正业务调用前先执行一次重置，避免脏数据遗留。

## 5. 第一检测位 API

### 5.1 `FirstOutSurfaceDetection`

头文件：
- `third_party/lanyou_first_detection/include/detection/first/FirstOutSurfaceDetection.h`

用途：
- 第一检测位外表面检测。
- 主要关注圆柱姿态、直边高度、直边斜度、坡口角等。

核心接口：

```cpp
class FirstOutSurfaceDetection {
public:
    FirstOutSurfaceDetection();
    bool Detect(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    const FirstPoseDetectionParams& GetParams() const;
    FirstPoseDetectionParams& GetParams();
};
```

调用说明：
- `Detect` 输入为 `pcl::PointCloud<pcl::PointXYZ>::Ptr&`。
- 参数是“可写引用”，因此调用时不要传 `const` 点云指针变量。
- 成功时返回 `true`，结果通过 `GetParams()` 获取。
- 失败时返回 `false`，具体原因一般记录在 `outliner_error_log`。

适用输入：
- 以封头外表面为主的点云。
- 点云质量过差、范围错误、配置文件缺失时，通常会失败。

### 5.2 `FirstInlinerSurfaceDetection`

头文件：
- `third_party/lanyou_first_detection/include/detection/first/FirstInlinerSurfaceDetection.h`

用途：
- 第一检测位内表面检测。
- 主要关注内径、开孔等特征。

核心接口：

```cpp
class FirstInlinerSurfaceDetection {
public:
    FirstInlinerSurfaceDetection();
    bool Detect(
        pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& hole_cloud);
    const FirstPoseDetectionParams& GetParams() const;
    FirstPoseDetectionParams& GetParams();
};
```

调用说明：
- `cloud` 为主体点云。
- `hole_cloud` 为孔区域或局部点云。
- 成功时返回 `true`，失败信息通常可从 `inliner_error_log` 获取。

## 6. 第二检测位 API

### 6.1 `SecondOutSurfaceDetection`

头文件：
- `third_party/lanyou_first_detection/include/detection/second/SecondOutSurfaceDetection.h`

用途：
- 第二检测位外表面检测。
- 主要涉及筒体圆柱拟合、焊缝错边、棱角度、余高、咬边等。

核心接口：

```cpp
class SecondOutSurfaceDetection {
public:
    SecondOutSurfaceDetection();
    bool Detect(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    bool Detect(
        pcl::PointCloud<pcl::PointXYZ>::Ptr& cylinder_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& seam_cloud);
    const SecondPoseDetectionParams& GetParams() const;
    SecondPoseDetectionParams& GetParams();
};
```

调用说明：
- 旧接口可直接传单份点云。
- 新接口可传圆柱点云和焊缝点云两份输入。
- 当前代码注释表明双输入接口里“当前仅使用圆柱点云”，后续使用前建议重新确认算法工程师最新约定。

### 6.2 `SecondInlinerSurfaceDetection`

头文件：
- `third_party/lanyou_first_detection/include/detection/second/SecondInlinerSurfaceDetection.h`

用途：
- 第二检测位内表面检测。

核心接口：

```cpp
class SecondInlinerSurfaceDetection {
public:
    SecondInlinerSurfaceDetection();
    bool Detect(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    const SecondPoseDetectionParams& GetParams() const;
    SecondPoseDetectionParams& GetParams();
};
```

说明：
- 当前头文件公开接口比较简单。
- 真正业务接入前，建议结合对应 `.cpp` 再核对其输入点云语义和输出字段含义。

## 7. 当前项目内的测试适配 API

头文件：
- `modules/vision/include/scan_tracking/vision/lanyou_detection_adapter.h`

用途：
- 把 `scan-tracking` 自己的点云结构转为 PCL 点云。
- 提供一个最小 smoke test 调用入口。

### 7.1 `toPclPointCloud(...)`

```cpp
pcl::PointCloud<pcl::PointXYZ>::Ptr toPclPointCloud(
    const scan_tracking::mech_eye::PointCloudFrame& frame);
```

作用：
- 将项目内 `PointCloudFrame` 转成 `pcl::PointCloud<pcl::PointXYZ>`。

行为说明：
- 输入点数量为 0 时，返回空点云对象。
- 若 `width * height == 点数`，保留有组织点云布局。
- 否则转成 `height = 1` 的无组织点云。
- 会基于坐标是否为有限值设置 `is_dense`。

### 7.2 `runFirstOutDetectionSmoke(...)`

```cpp
LanyouSmokeResult runFirstOutDetectionSmoke(
    const scan_tracking::mech_eye::PointCloudFrame& frame);
```

返回结构：

```cpp
struct LanyouSmokeResult {
    bool invoked = false;
    bool success = false;
    int inputPointCount = 0;
    QString message;
};
```

作用：
- 作为最小打通测试入口，调用 `FirstOutSurfaceDetection::Detect(...)`。

字段含义：
- `invoked`
  - 是否真的进入了算法调用。
- `success`
  - 算法本次是否返回成功。
- `inputPointCount`
  - 输入点数量。
- `message`
  - 调用结果或异常信息。

说明：
- 这个接口用于验证“接线是否打通”。
- 即使 `success=false`，只要 `invoked=true` 且程序未崩溃，也说明集成链路基本正常。

## 8. 最小调用示例

### 8.1 直接调用第一检测位外表面算法

```cpp
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "detection/first/FirstOutSurfaceDetection.h"
#include "utils/Params.h"

ResetGlobalFirstPoseParams();

auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(
    new pcl::PointCloud<pcl::PointXYZ>());

// TODO: 填充 cloud->points

FirstOutSurfaceDetection detector;
bool ok = detector.Detect(cloud);
const auto& params = detector.GetParams();

if (!ok) {
    // params.outliner_error_log 中通常会有失败原因
}
```

### 8.2 在 `scan-tracking` 中通过适配层调用

```cpp
#include "scan_tracking/vision/lanyou_detection_adapter.h"

scan_tracking::mech_eye::PointCloudFrame frame;
// TODO: 填充 frame.pointsXYZ / width / height

auto result = scan_tracking::vision::lanyou::runFirstOutDetectionSmoke(frame);

if (!result.invoked) {
    // 说明还没真正进入算法，比如输入为空
}

if (!result.success) {
    // 说明算法已调用，但检测本身没有成功
    // result.message 可用于打印日志
}
```

## 9. 手动 smoke test 运行方式

当前测试程序：
- `build/win-msvc2019-qtcore-ninja-debug/modules/vision/scan_tracking_lanyou_smoke_test.exe`

运行前建议先设置：

```powershell
cd D:\work\scan-tracking
$env:PATH="C:\Qt\5.15.2\msvc2019_64\bin;C:\Program Files\OpenNI2\Redist;C:\Program Files\PCL 1.12.1\bin;$env:PATH"
```

运行命令：

```powershell
.\build\win-msvc2019-qtcore-ninja-debug\modules\vision\scan_tracking_lanyou_smoke_test.exe
```

预期现象：
- 可能会看到算法内部的配置读取提示。
- 可能会看到 `PointsModel` 的日志输出。
- 最后一行出现：

```text
Lanyou adapter smoke tests passed
```

注意：
- 当前 smoke test 使用的是极小模拟点云。
- 因此“集成成功”的标准是：程序可运行、进入算法、测试断言通过。
- 并不是要求算法在这份模拟点云上产出有效业务特征。

## 10. 配置文件说明

当前算法目录下有配置文件：
- `third_party/lanyou_first_detection/config/first_config.cfg`
- `third_party/lanyou_first_detection/config/second_config.cfg`
- `third_party/lanyou_first_detection/config/config.cfg`

当前已知现象：
- 某些算法类构造后会尝试按相对路径读取配置。
- 如果当前工作目录不是算法目录附近，可能出现类似：

```text
cannot open ../config/first_config.cfg
cannot open config/first_config.cfg
```

这不一定导致程序崩溃，但可能导致算法退回默认参数或直接失败。

建议：
- 正式接入业务前，最好把配置路径改成明确可控的绝对路径或由主程序统一传入。
- 不建议长期依赖当前工作目录碰巧正确。

## 11. 当前集成边界

当前已经完成：
- Windows 下源码编译通过。
- PCL 依赖接通。
- 项目内点云结构到 PCL 的转换适配。
- 第一检测位外表面算法的 smoke test 调用打通。

当前尚未完成：
- 正式业务流程接入。
- 真实采集点云的效果验证。
- 第二检测位 smoke test。
- 配置文件路径标准化。
- 多线程/多任务并发隔离设计。

## 12. 常见问题

### 12.1 只包含头文件能不能用？

不能。

原因：
- 该算法不是 header-only。
- 主要实现都在 `.cpp` 中。
- 接口还直接依赖 PCL 类型，因此必须完成源码编译和链接。

### 12.2 以后现场能不能直接改？

可以。

因为目前集成的是完整源码，不是黑盒 DLL，所以以下问题都能现场修改：
- CMake/编译适配
- 配置文件路径
- 日志与异常处理
- 输入输出适配
- 部分算法外围逻辑

但如果涉及核心检测逻辑、阈值含义、几何判定依据，仍建议和原算法工程师确认。

### 12.3 为什么 smoke test 里算法可能返回失败，但测试仍然算通过？

因为 smoke test 的目标是验证：
- 数据类型能转过去
- 函数能调起来
- 工程不会崩
- 返回值和错误信息能收回来

它不是效果测试。

## 13. 后续建议

推荐按下面顺序继续推进：
1. 增加第二检测位的 smoke test。
2. 增加“读取真实点云文件并调用算法”的调试入口。
3. 统一配置文件路径管理。
4. 再考虑把算法挂到正式业务流程。

---

如后续接口有变化，请优先核对以下文件：
- `third_party/lanyou_first_detection/include/utils/Params.h`
- `third_party/lanyou_first_detection/include/detection/first/*.h`
- `third_party/lanyou_first_detection/include/detection/second/*.h`
- `modules/vision/include/scan_tracking/vision/lanyou_detection_adapter.h`
