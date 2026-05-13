# Scan Tracking

封头检测工位 IPC 侧控制台程序。

## 本机环境

- Windows + MSVC
- Qt 5.15.2: `C:\Qt\5.15.2\msvc2019_64`
- MSVC v142 / 19.29 x64: `C:\Program Files\Microsoft Visual Studio\18\Community\VC\Tools\MSVC\14.29.30133`
- CMake 3.21+
- Ninja
- Windows SDK
- PCL 1.12.0: `C:\Program Files\PCL 1.12.0`
- Eigen 3.4.0: `third_party\eigen-3.4.0`
- Mech-Eye SDK: `third_party\Mech-Eye SDK-2.5.4`
- 海康 MVS SDK: `third_party\MVS`
- OpenCV: `third_party\LB\opencv-3.4.3-vc14_vc15`

## 打开方式

1. 用 Visual Studio 打开仓库根目录 `D:\work\LY\IPC-192.168.110.173_track-main`
2. 选择 CMake 预设 `win-msvc2019-qtcore-ninja-debug`
3. 直接运行 `scan-tracking.exe`

项目默认启用严格工具链检查。请固定使用仓库内 CMake 预设：

- `win-msvc2019-qtcore-ninja-debug`
- `win-msvc2019-qtcore-ninja-release`

不要使用 Qt Creator 自动生成的旧 Kit、`out/` 缓存目录，或未带 preset 的手写 CMake 配置；选错 Qt、MSVC 版本或 x86 工具链时，CMake 会在配置阶段直接报错。

## 配置

配置文件位于仓库根目录：

- `config.ini`

启动时程序会优先读取这个文件。

## 构建

推荐用仓库内脚本：

```powershell
cmd /c tools\scan_tracking_dev.cmd configure-debug
cmd /c tools\scan_tracking_dev.cmd build-debug
```

## 运行

```powershell
cmd /c tools\scan_tracking_dev.cmd run-debug
```

## 日志

日志会输出到运行目录下的 `logs` 文件夹。

## 说明

- `LB` 算法已接入。
- 目前程序已可在本机启动。
- 如果未连接真实相机或 Modbus 服务，日志里会出现对应设备未发现或连接失败提示。
