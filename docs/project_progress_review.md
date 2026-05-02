# 项目开发进度梳理报告

## 1. 总体结论

当前整体进度约 **65%**。核心的 `QtCore + Modbus TCP + 状态机` 闭环已经搭起来，MechEye 异步采集也已完成骨架，`lanyou_first_detection` 的第一工位算法已经接入到 `TrackingService`。但真实联调还不完整，尤其是部分 PLC 触发器仍是占位实现，海康位姿矩阵还是占位值，统一视觉编排层也还没接入 `StateMachine` 主流程。

当前状态是：**可编译、可跑通部分链路，但功能不完整**。本周末联调属于 **有条件满足**，前提是联调范围先收敛到“Modbus 握手 + 扫描分段 + 第一工位检测烟雾测试”。如果要把海康位姿、统一视觉 bundle 和全部触发器一次性联完，当前还不够。

## 2. docs 文档梳理结果

| 文档 | 主要内容 | 涉及模块 | 结论 |
|---|---|---|---|
| `docs/plan48.md` | 规划了 MechEye 异步化、扫描分段真实接入、状态机改造路线，明确后续要把真实扫描流程替换掉模拟桩 | `modules/mech_eye`、`modules/flow_control`、`modules/tracking` | Step 1 基本完成，Step 2/3 还在推进中 |
| `docs/vision_framework.md` | 定义了 `HikCameraService`、`VisionPipelineService`、统一 `MultiCameraCaptureBundle`，目标是“1 份点云 + 2 份矩阵” | `modules/vision` | 框架骨架已建，真实矩阵和编排逻辑未完成 |
| `docs/算法使用API.md` | 说明蓝优第一工位算法的最小接入方式、全局参数对象、测试入口与配置文件 | `third_party/lanyou_first_detection`、`modules/vision`、`modules/tracking` | 第一工位链路已接入，但结果映射还偏临时 |
| `docs/封头检测工位PLC-IPC Modbus通信协议_v0.1.md` | 定义 PLC-IPC 寄存器、握手时序、超时/复位规则和联调顺序 | `modules/flow_control`、`modules/modbus` | 当前实现大体遵循握手顺序，但仍有未完成触发器 |
| `README.md` | 说明当前项目仍在模拟联调阶段，强调 Modbus 骨架、状态机闭环和固定占位值 | 全项目 | 与代码现状一致，真实 PLC 联机还没完成 |

## 3. 功能模块实现状态

| 模块 | 文档要求 | 当前代码状态 | 完成度 | 风险 |
|---|---|---|---|---|
| `modules/modbus` | 提供 Modbus TCP 读写、掉线重连、异步回调 | 已实现并被状态机使用 | 已完成 | 低 |
| `modules/flow_control::StateMachine` | 完成 PLC 握手、任务分发、超时、故障闭环 | 主链路已完成，但部分触发器仍是占位 | 基本完成 | 高 |
| `modules/mech_eye` | 异步采集点云，避免阻塞主线程 | 已完成，且采集结果带法向量 | 已完成 | 低 |
| `modules/tracking` | 综合点云检测、第一工位接入 | 已接入蓝优第一工位检测，但 NG 位和偏移量映射仍偏临时 | 部分完成 | 中 |
| `modules/vision` | 统一点云 + 海康矩阵编排 | 骨架有了，真实矩阵和统一调度未完成 | 部分完成 | 中-高 |
| `third_party/lanyou_first_detection` | 提供第一工位算法源码 | 第一工位基本可用，第二/第三工位仍雏形 | 部分完成 | 中 |

## 4. StateMachine 专项检查

| 检查项 | 结论 | 风险等级 | 建议 |
|---|---|---|---|
| 当前状态枚举是否完整 | `AppState` 只有 `Init/Ready/Scanning/Error`，足够支撑当前控制台程序，但没有暂停/恢复等细分状态 | 中 | 若周末只做联调可以先不扩展；后续可补 `Paused/Recovering` |
| 状态流转是否清晰 | `start -> Ready -> Scanning -> complete/fault -> Ready/Error` 主路径清楚 | 低 | 保持当前握手顺序，不要重写成单寄存器轮询 |
| 触发器覆盖是否完整 | `Trig_LoadGrasp / ScanSegment / Inspection / UnloadCalc / ResultReset` 有专用处理；`Trig_StationMaterialCheck / PoseCheck / SelfCheck / CodeRead` 仍走默认成功分支 | **高** | 这几个触发器必须补实现，或者至少改成明确 `NotImplemented/失败码`，否则 PLC 会被误导 |
| 异常、超时、故障是否有处理 | 已有超时定时器、Modbus 失败计数、相机致命错误、缓存清理和故障进入出口 | 低-中 | 逻辑较完整，联调时重点验证故障恢复和 PLC 清 Trig |
| 是否存在重复/不可达状态 | `protocol::IpcState::Paused`、`VisionPipelineState` 等状态定义了但主流程没用到；不是错误，但存在“定义有了、业务没接上” | 中 | 明确这些状态是否真的要参与联调，不要让 PLC 误判 |
| 是否存在切换后没做动作的问题 | `onModbusConnected / onModbusDisconnected / onProcessTimeout / onCaptureFinished` 基本都有动作 | 低 | 继续补齐未实现触发器的动作闭环 |
| 是否有多线程/异步竞争风险 | `m_isPollingPlc`、`m_activeTask.captureRequestId`、`completionAnnounced` 已做了基本防重入；但 `completeActiveTask()` 里的重试会在主线程里 `msleep`，有阻塞事件循环的风险 | 中 | 把重试策略改成异步重试或缩短阻塞窗口 |
| 是否与 `lanyou_first_detection` 有耦合 | 通过 `TrackingService::inspectSegments()` 间接接入，`Trig_Inspection` 会消费第一工位检测结果 | 低 | 这一段链路是通的，但结果映射仍需完善 |
| 是否有关键日志 | 有，状态切换、触发接收、超时、故障、采集回调、结果写回都有日志 | 低 | 联调时保留当前日志粒度，足够排查问题 |

对应代码：`modules/flow_control/src/state_machine.cpp`（尤其是 401-503、549-675、824-944、971-1160、1356-1420、1431-1630）和 `modules/flow_control/include/scan_tracking/flow_control/plc_protocol.h`。

## 5. lanyou_first_detection 专项检查

| 检查项 | 结论 | 风险等级 | 建议 |
|---|---|---|---|
| 模块入口在哪里 | 入口在 `modules/tracking/src/tracking_service.cpp` 的 `inspectSegments()`，最终调用 `runFirstStationDetection()` | 低 | 入口清晰，主流程已经接上 |
| 是否加入构建系统 | 已通过 `CMakeLists.txt` 的 `add_subdirectory(third_party/lanyou_first_detection)` 接入，并以 `Lanyou::FirstDetection` 链接 | 低 | 构建链路正常 |
| 是否被主流程调用 | 是，`StateMachine::executeInspectionTask()` 调用 `TrackingService::inspectSegments()`，再进入蓝优第一工位算法 | 低 | 这条链路可作为联调主线 |
| 是否与 `StateMachine` 正确衔接 | 衔接的是“综合检测”触发；但 `VisionPipelineService` 还没有进入 `StateMachine` 主流程 | 中 | 如果周末要联调统一视觉 bundle，需要再接一层 |
| 输入参数是否明确 | 需要外表面/内表面/内孔三份点云，来源是 `m_segmentCaptureResults` | 低 | 继续保持 `segmentIndex=1/2/3` 的配置一致性 |
| 输出结果是否被正确消费 | `TrackingService` 已消费 `FirstStationDetectionResult` 的成功/失败、参数和点数，但 NG 位、偏移量仍偏临时 | 中 | 需要把 `params` 映射到正式 PLC 结果表 |
| 异常返回是否被处理 | 有 try/catch 和空输入校验，但错误分类与结构化诊断还没完全落地 | 中 | 建议补成明确错误码和 NG 原因映射 |
| 是否有必要日志 | 有日志和 smoke test 输出，够用 | 低 | 保留当前日志，补充失败原因编号 |
| 是否有配置项 | 有 `first_config.cfg`、`second_config.cfg`、`config.cfg`，但测试 runner 里写死了本机路径 | 中 | 去掉硬编码路径，改为相对路径或环境变量默认值 |
| 是否存在硬编码路径/参数 | 有，`modules/vision/tests/lanyou_first_station_ply_runner.cpp:134` 写死了 `D:/work/.../config`，`config.cfg` 也写死了日志目录 | 中 | 联调前至少把测试入口改成可配置 |
| 是否有 TODO / FIXME / 占位实现 | 有，且集中在“错误映射、参数映射、结构化诊断” | 中 | 这些不会阻塞 smoke test，但会影响正式结果判定 |
| 是否会影响本周末联调 | 若只做第一工位 smoke/回归，不是阻塞；若要正式结果闭环，影响明显 | 中-高 | 先把“能跑”和“能判对”分开 |

对应代码：`modules/tracking/src/tracking_service.cpp`、`modules/vision/src/lanyou_first_station_adapter.cpp`、`third_party/lanyou_first_detection/*`。

## 6. TODO / FIXME / 待完成项清单

| 文件 | 行号 | 标记类型 | 内容 | 是否影响联调 | 建议处理方式 |
|---|---:|---|---|---|---|
| `modules/flow_control/src/state_machine.cpp` | 477-480 | TODO | 为 `Trig_StationMaterialCheck(20)/Trig_PoseCheck(21)/Trig_SelfCheck(25)/Trig_CodeRead(26)` 增加专用处理函数 | **是** | 周末前至少改成明确失败或快速返回未实现 |
| `modules/flow_control/src/state_machine.cpp` | 499-502 | TODO | `default` 分支不应长期返回默认成功码 | **是** | 改成明确失败码，避免误判成功 |
| `modules/flow_control/src/state_machine.cpp` | 515-517 | TODO | 接入外部抓取位姿提供模块 | 否（若本周不联调抓取） | 联调后处理 |
| `modules/flow_control/src/state_machine.cpp` | 530-531 | TODO | 接入外部卸料位姿计算模块 | 否（若本周不联调卸料） | 联调后处理 |
| `modules/flow_control/src/state_machine.cpp` | 1230-1232 | TODO | 用外部抓取位姿替换固定值 | 否 | 联调后处理 |
| `modules/flow_control/src/state_machine.cpp` | 1249-1251 | TODO | 用外部卸料位姿替换固定值 | 否 | 联调后处理 |
| `modules/tracking/src/tracking_service.cpp` | 161-188 | TODO | 第一工位 NG 位、偏移量、测量项映射仍是临时定义 | **是** | 按 PLC 缺陷原因表和工艺坐标系重映射 |
| `modules/vision/src/hik_camera_service.cpp` | 126 | TODO | 海康服务状态机粒度仍可继续拆分 | 否 | 联调后优化 |
| `modules/vision/src/hik_camera_service.cpp` | 178-178 | TODO | 真实采图、标定换算、4x4 位姿矩阵生成 | **是**（若要联调海康） | 先补真实矩阵，否则明确不纳入本周范围 |
| `modules/vision/src/vision_pipeline_service.cpp` | 112-120 | TODO | 统一时间戳、超时仲裁、多相机同步策略 | 中 | 联调前至少确认超时策略 |
| `modules/vision/src/vision_pipeline_service.cpp` | 183-183 | TODO | 矩阵有效性、坐标系一致性、统一错误码映射 | 中-高 | 先做基本校验，再谈完善 |
| `modules/vision/src/lanyou_first_station_adapter.cpp` | 51-52 | TODO | 映射 `outliner_error_log` 到业务 NG 原因 | 中 | 先补最少可读 NG 原因表 |
| `modules/vision/src/lanyou_first_station_adapter.cpp` | 62-63 | TODO | 映射 `inliner_error_log` 到业务 NG 原因 | 中 | 同上 |
| `modules/vision/src/lanyou_first_station_adapter.cpp` | 67-67 | TODO | 将参数映射到 `InspectionResult` 和稳定偏移量 | **是** | 这是正式结果闭环的关键 |
| `modules/vision/src/lanyou_first_station_adapter.cpp` | 71-76 | TODO | 结构化异常诊断和错误分类 | 中 | 增补错误码和日志字段 |
| `third_party/lanyou_first_detection/src/model/PointsModel.cpp` | 332 | TODO | Windows 集成需过滤 `inf/-inf` | 中 | 若大点云异常，优先处理 |
| `third_party/lanyou_first_detection/src/model/PointsModel.cpp` | 349 | TODO | Windows/PCL 1.12 下 OMP 法向估计栈溢出风险 | 中 | 如果现场点云大，优先规避 OMP 路径 |
| `third_party/lanyou_first_detection/src/model/PointsModel.cpp` | 354 | TODO | K 近邻替代以推进离线联调 | 中 | 保持现有可运行路径，联调后再优化 |
| `modules/vision/tests/lanyou_first_station_ply_runner.cpp` | 134 | 硬编码 | 测试 runner 写死 `D:/work/scan-tracking/third_party/lanyou_first_detection/config` | 中 | 改成环境变量默认值或相对路径 |
| `docs/vision_framework.md` | 116-120 | TODO | 视觉框架后续项：真实 pose、重连、统一编排、bundle 校验、flow 接入 | 中-高 | 作为联调后路线图，不要误以为已完成 |

## 7. 联调前风险清单

| 风险项 | 严重程度 | 影响 | 建议处理 |
|---|---|---|---|
| 未实现触发器被默认写成功 | 高 | PLC 会误以为 `StationMaterialCheck / PoseCheck / SelfCheck / CodeRead` 已通过 | 先改成明确失败或禁止触发 |
| 统一视觉 bundle 未接入主流程 | 高 | 海康相机和统一矩阵联调无入口 | 明确本周范围，或把 bundle 接入 `StateMachine` |
| 海康位姿仍是单位矩阵占位 | 高 | 若 PLC/算法依赖位姿，结果不可信 | 要么补真实矩阵，要么不纳入本周联调 |
| 第一工位 NG 位/偏移量仍是临时映射 | 中-高 | 检测结果可能与 PLC 期望不一致 | 按协议表做正式映射 |
| `completeActiveTask()` 的重试会阻塞主线程 | 中 | 异常场景下会拖慢心跳和轮询 | 改成异步重试或缩短阻塞时间 |
| 测试 runner 路径写死 | 中 | 换机器后 smoke 跑不起来 | 改成可配置路径 |
| 运行配置未在目标机上确认 | 高 | 会直接连错 Modbus 或相机 | 先核对 `config.ini` |
| 联调范围未收敛 | 高 | 容易把“能跑”和“能判对”混在一起 | 先锁定联调清单 |

## 8. 下一步开发计划

### P0：联调前必须完成

1. 把 `Trig_StationMaterialCheck`、`Trig_PoseCheck`、`Trig_SelfCheck`、`Trig_CodeRead` 从默认成功分支里拆出来，至少改成明确失败码或未实现码，避免 PLC 误判。
2. 明确周末联调范围：如果要联海康位姿和统一视觉 bundle，就必须把 `VisionPipelineService` 或至少 `HikCameraService` 的真实输出接入；如果不联，就不要把它们算进成功标准。
3. 把 `TrackingService` 里的第一工位结果映射改成正式 PLC 结果表，至少补齐 NG 原因位和偏移量定义。
4. 在远端目标机上确认 `build/.../app/config.ini`，重点核对 Modbus host/port/unitId、相机 key、Tracking 段号映射。

### P1：联调前建议完成

1. 去掉 `modules/vision/tests/lanyou_first_station_ply_runner.cpp` 里的硬编码路径。
2. 给蓝优算法补最小化失败码/日志编号，方便现场直接定位“是点云问题、算法问题还是配置问题”。
3. 补一轮最小 smoke：`Trig_ScanSegment -> Trig_Inspection -> Trig_ResultReset`，确保回路不丢数据、不卡死。
4. 如果海康本周不联调，把相关入口在 README 或运行说明里明确标记为“未纳入本轮联调”。

### P2：联调后优化

1. 完成统一视觉 bundle 的时间戳仲裁、矩阵校验和错误码统一。
2. 把第一/二/三工位算法的结果结构统一成稳定接口。
3. 逐步把 `StateMachine` 里剩余的模拟结果替换成真实抓取/卸料/检测结果。
4. 继续收敛日志格式，方便后续做自动化回归。

## 9. 联调准备清单

- 编译环境是否确认：远端 Windows 工控机的 `CMake / Qt / MSVC / Ninja / Mech-Eye SDK / Hik MVS SDK` 是否都能找到。
- 运行配置是否确认：`build/.../app/config.ini` 是否存在，内容是否和现场一致。
- 设备 IP / 串口 / Modbus 参数是否确认：PLC、模拟器、相机 IP、Unit ID、端口、超时都要核对。
- `StateMachine` 状态流转是否已验证：至少要验证 `Trig -> Ack=1 -> Res -> Ack=2 -> 清 Trig -> Ack=0`。
- `lanyou_first_detection` 是否已接入并验证：第一工位 smoke 和 `Trig_Inspection` 路径要跑通。
- 配置文件是否准备好：`first_config.cfg`、`second_config.cfg`、`config.cfg`、`config.ini` 都要确认。
- 日志是否足够排查问题：`flow_control / modbus / mech_eye / vision / tracking` 的关键日志要保留。
- 异常断开/重连逻辑是否可用：Modbus 断开、相机忙、采集超时都要测一次。
- 测试用例或手动测试步骤是否准备好：至少准备 `ResultReset / ScanSegment / Inspection` 的手工步骤。
- 回滚方案是否准备好：一旦新版链路不稳定，要能切回纯 Modbus 模拟联调。

## 10. 建议结论

本周末**可以联调，但必须收敛范围**。建议把目标定义为：先打通 Modbus 握手、扫描分段、第一工位检测烟雾测试，再视时间决定是否加海康位姿和统一视觉 bundle。

联调前最优先处理的 3 件事是：
1. 处理 `StateMachine` 里那 4 个还在默认成功的触发器。
2. 确认并冻结远端 `config.ini` 和 PLC/相机参数。
3. 把第一工位结果映射改成正式的 PLC 结果定义，至少别再用临时 NG 位。

可以暂时不处理的内容：第二/第三工位算法、统一视觉 bundle 的完整坐标一致性校验、`HikCameraService` 的更细状态拆分、测试 runner 的通用化重构。这些不影响“先把联调跑起来”，但会影响后续规模化落地。
## 启动固化记录（2026-05-01）

- 已验证主程序可启动并进入事件循环。
- 本次稳定启动点为 SCAN_TRACKING_STARTUP_STAGE=0，仅初始化 Modbus。
- 运行日志确认程序可连接到本机 PLC 模拟器，随后因模拟器侧关闭连接而进入重连流程。
- 当前联调脚本已保留，后续可在此基础上继续把 stage=5 逐步恢复。
