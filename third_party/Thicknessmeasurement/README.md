# 厚度测量算法（V1.2）

基于 PCL 的厚度测量工程，已集成到 IPC 扫描跟踪项目。

## 工程内容

- `CMakeLists.txt`: 静态库 `thickness_measure`，可选 demo 可执行文件。
- `config/thickness_config.json`: 算法配置示例（V1.2 格式）。
- `src/Config.*`: JSON 配置读取。
- `src/ThicknessMeasurement.*`: 双模板 ICP、最近点搜索、切面投影和厚度计算。

## V1.2 相对 V1.1 的变化

- 内外表面各自独立模板点云与 ICP 配准。
- 支持两种厚度测量方法（`measurement.thickness_method`）：
  - `nearest_between_surfaces`（默认）
  - `tangent_plane_projection`
- 结果包含 `inner_icp_fitness_score`、`outer_icp_fitness_score`、`thickness_method`。

## 配置说明

`config/thickness_config.json` 中需要设置：

- `point_cloud.inner_template_cloud_path`: 内表面模板点云。
- `point_cloud.outer_template_cloud_path`: 外表面模板点云。
- `measurement.thickness_method`: 厚度计算方法。
- `preprocess`: 孤立点去除和体素降采样参数。
- `icp`: ICP 匹配参数。
- `template_cylinder`: 模板柱面轴线。
- `template_feature_points`: 两个特征点坐标。
- `output.result_path`: 输出结果 JSON 路径。

向后兼容：若配置中仍使用旧字段 `template_cloud_path`，将同时作为内外模板路径。

扫描点云路径不在 JSON 中配置：IPC 运行时由内存接口传入；独立 demo 通过命令行指定。

## IPC 集成接口

```cpp
bool MeasureThicknessFromClouds(
    const ThicknessConfig& config,
    const ThicknessPointCloudConstPtr& innerTemplateCloud,
    const ThicknessPointCloudConstPtr& outerTemplateCloud,
    const ThicknessPointCloudConstPtr& innerScanCloud,
    const ThicknessPointCloudConstPtr& outerScanCloud,
    ThicknessResult* result,
    std::string* error);
```

适配层：`modules/vision/src/thickness_measurement_adapter.cpp`

## 编译和运行

```bat
cmake --preset <your-preset>
cmake --build --preset <your-preset>
```

独立 demo（可选）：

```bat
cmake -DSCAN_TRACKING_BUILD_THICKNESS_DEMO=ON ...
thickness_measure_demo config\thickness_config.json input\inner_surface_sample.pcd input\outer_surface_sample.pcd
```
