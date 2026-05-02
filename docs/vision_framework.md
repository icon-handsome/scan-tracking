# 视觉集成框架说明

日期：2026-04-17

## 背景

结合 `docs/plan48.md` 的现状，当前项目已经具备：

- `MechEyeService`：异步触发梅卡曼德 3D 采集
- `CaptureResult`：向上层提供点云输出
- `StateMachine`：负责 PLC 握手与扫描流程
- `TrackingService`：目前还是算法占位实现

接下来要新增的现实需求是：

- 保留梅卡曼德扫描相机输出点云
- 增加两台海康相机输出 4x4 位姿矩阵
- 后续算法统一消费“1 份点云 + 2 份矩阵”

## 本次新增的框架

本次新增了 `modules/vision` 模块，暂不实现具体业务算法，也不直接绑定海康 SDK，只先把项目骨架搭起来。

### 模块职责

1. `vision_types.h`
- 定义统一数据契约
- 包括海康 4x4 矩阵结果
- 包括统一视觉采集请求
- 包括统一视觉采集结果包

2. `hik_camera_service.h/.cpp`
- 定义海康相机服务骨架
- 预留后续 MVS SDK 接入位置
- 当前只返回“未实现”的占位结果，不落具体 SDK 调用

3. `vision_pipeline_service.h/.cpp`
- 负责组织一次统一采集
- 一次请求包含：
  - 梅卡曼德点云
  - 海康 A 矩阵
  - 海康 B 矩阵
- 输出统一的 `MultiCameraCaptureBundle`

## 当前接入方式

程序启动时已经完成以下装配：

- 启动 `MechEyeService`
- 启动两个 `HikCameraService`
- 启动 `VisionPipelineService`
- 从 `config.ini` 的 `[Vision]` 段读取三台相机的配置骨架

## 当前配置项

已新增 `[Vision]` 配置段，默认键如下：

- `mechEyeCameraKey`
- `mechCaptureTimeoutMs`
- `hikConnectTimeoutMs`
- `hikCaptureTimeoutMs`
- `hikSdkRoot`
- `hikCameraAName`
- `hikCameraAKey`
- `hikCameraAIp`
- `hikCameraASerial`
- `hikCameraBName`
- `hikCameraBKey`
- `hikCameraBIp`
- `hikCameraBSerial`

默认 IP 已按现场情况预填：

- `hikCameraAKey = 192.168.10.12`
- `hikCameraBKey = 192.168.10.13`

## 未来接 SDK 的落点

后续如果接入海康 MVS SDK，建议只修改 `HikCameraService`：

1. `start()`
- 负责 SDK 初始化
- 发现设备
- 按 IP / SN / logicalName 建立连接

2. `requestPoseCapture()`
- 负责触发海康相机采集
- 获取 4x4 位姿矩阵
- 回填 `HikPoseCaptureResult.poseMatrix`

3. `VisionPipelineService`
- 无需理解 SDK 细节
- 继续只做三路结果的统一编排与聚合

## 给后续算法的直接输入

后续算法层可以直接消费：

- `scan_tracking::vision::MultiCameraCaptureBundle`

它已经统一包含：

- `mechEyeResult`
- `hikCameraAResult`
- `hikCameraBResult`

这样可以避免算法层再去分别了解：

- PLC 触发来源
- 梅卡 SDK 调用细节
- 海康 SDK 调用细节
- 多相机结果的关联关系

## 后续 TODO

- TODO(hik-pose): 在 `modules/vision/src/hik_camera_service.cpp` 的后台采集任务中接入真实取图、标定换算和 4x4 位姿矩阵生成。
- TODO(hik-reconnect): 增加海康相机掉线重连、异常回调和更细粒度错误码映射。
- TODO(vision-orchestrator): 在 `modules/vision/src/vision_pipeline_service.cpp` 中增加多相机统一时间戳、超时仲裁和同步策略。
- TODO(vision-bundle): 在 bundle 完成前增加矩阵有效性、坐标系一致性和外参版本校验。
- TODO(flow-integration): 后续把 `VisionPipelineService::requestCaptureBundle()` 接入 `StateMachine` 的真实扫描业务入口，替换当前分散的视觉占位调用。

## 本次已验证结果

- 已将本机 `C:\Program Files (x86)\MVS` 的最小开发集复制到：
  - `D:\work\scan-tracking\third_party\hik_mvs\include`
  - `D:\work\scan-tracking\third_party\hik_mvs\lib\win64`
  - `D:\work\scan-tracking\third_party\hik_mvs\runtime\Win64_x64`
  - `D:\work\scan-tracking\third_party\hik_mvs\docs`
- 已新增 `cmake/HikMvsSdk.cmake`，项目现在直接使用仓库内的海康 SDK 副本编译。
- 已在工控机上完成一次真实启动验证，启动日志显示：
  - 海康相机 A：`192.168.10.12`，序列号 `DA4444604`
  - 海康相机 B：`192.168.10.13`，序列号 `DA2738512`
- 当前 `HikCameraService` 已经具备：
  - SDK 初始化 / 反初始化
  - 设备枚举
  - 按 IP / SN / 名称匹配设备
  - 创建句柄并打开设备
  - 为 GigE 相机设置最佳包长
- 当前 `requestPoseCapture()` 仍返回单位矩阵占位结果，等待后续真实矩阵算法接入。
