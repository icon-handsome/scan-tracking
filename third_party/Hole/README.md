# 柱面和开孔测量（V1.2 标定）

这是用于封头内径、内周长、圆度公差、直边斜度、直边高度、开孔距离和开孔接头角度测量的 C++/PCL 工程。算法源码保持 V1.1 IPC 集成（含 `runWithScanCloud` 内存接口）；**V1.2 变更主要为标定参数与模板点云**。

## V1.2 配置要点

- 模板点云：`template/Zhu_Mian_Kai_Kong_Template_cut_trans.pcd`（变换后模板，需放入 `template/` 目录）
- `crop_boxes`：4 个包围盒（当前预处理裁剪代码默认关闭，参数预置供后续启用）
- 顶平面特征点、直边 B 点、开孔圆柱特征点：已按 V1.2 现场标定更新

IPC 运行时扫描点云由 `hole_measurement_adapter` 内存注入，`input_frames` 保持空数组即可。

## 构建

已提供 VS2013 工程：

```powershell
MSBuild.exe HeadMeasureVS2013.sln /p:Configuration=Release /p:Platform=x64
```

本机已使用 `C:\Program Files (x86)\MSBuild\12.0\Bin\amd64\MSBuild.exe` 编译通过，输出位于：

```text
build_vs2013\Release\head_measure.exe
```

也可使用支持 VS2013 生成器的旧版 CMake：

```powershell
cd "D:\1 自研\15 兰铀算法\测量算法\柱面和开孔测量"
cmake -G "Visual Studio 12 2013 Win64" -S . -B build
cmake --build build --config Release
```

注意：当前机器安装的新版 CMake 不再显示 `Visual Studio 12 2013` 生成器。若命令行生成失败，请使用支持 VS2013 的旧版 CMake，或在 VS2013 中手工创建项目并引用 `PCL_ROOT/include`、`PCL_ROOT/lib`、`PCL_ROOT/3rdParty/Boost` 等路径。

## 运行

```powershell
.\build\Release\head_measure.exe .\config\sample_config.json
```

## 输出

程序会打印所有拟合误差：

- `fit_error name=global_template_icp`
- `fit_error name=top_plane`
- `fit_error name=straight_side_cylinder`
- `fit_error name=axis_slice_x`
- `fit_error name=opening_local_icp`

最终输出 `measure_result`，包含尺寸集合。

## 标准说明

圆度公差参考 GB/T 7235-2004《产品几何量技术规范(GPS) 评定圆度误差的方法 半径变化量测量》。当前默认按最小区域圆思路计算同一截面最大半径与最小半径之差。
