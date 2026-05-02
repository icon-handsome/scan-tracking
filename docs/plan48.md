# 4月8日开发计划

## 1. 背景

本轮迭代的目标是基于 6.11 时序，实现真实的分段扫描与拼接检测流程，从而替换掉目前 \modules/flow_control/src/state_machine.cpp\ 中的模拟桩逻辑。

主要接管的核心业务触发指令包括：
- \Trig_ScanSegment\ （分段扫描触发）
- \Trig_Inspection\ （综合解算评判触发）

需要真实接入并联动的模块：
- \modules/mech_eye\：调用 3D 相机 SDK 真实执行扫描获取点云数据
- \modules/tracking\：负责后续的点云拼接与综合缺陷评判计算（前期允许先用 Mock 模拟骨架替代真实的复杂算法）

## 2. 架构约束与底线

- **严禁阻塞主线程**：\QModbusTcpClient\ 的 \50~100ms\ 轮询和心跳保活必须时刻保持畅通，坚决杜绝任何耗时卡顿。
- **纯内存缓冲存放数据**：采用无数据库（No DB）方案，完全在内存中临时缓存分段扫描的点云等大载荷执行数据。
- **安全的内存释放机制**：每次综合检测解算完成后，或者一旦收到 \Trig_ResultReset\ (流程重置) 指令，必须安全、彻底地清空并释放点云缓冲内存，防范内存泄露。
- **精准的错误映射与降级机制**：务必精准捕获相机断连、SDK 获取超时或计算异常等情况，并规范地将其映射回写到相应的 \Res_xxx\、\Device_Fault_Word0\ 或 \NG_Reason_Word0\ 状态寄存器中。
- **严守 Modbus 四步握手原则**：所有真实业务介入后仍然必须严格听命于 \Trig -> Ack=1 -> 回填 Res 与结果数据 -> Ack=2 -> 外部清空 Trig=0 -> Ack=0\ 这个不可撼动的闭环交互法则。

## 3. 核心执行步骤

### 步骤 1：搭建相机后台异步工作流
重构并扩展 \modules/mech_eye\，彻底将其打造为一个线程安全的独立组件。使其能在后台异步稳定执行真实的 3D 相机采集操作，并通过 Qt 信号（Signals）将收集到的点云数据异步抛出给外部业务机。

#### 步骤 1 子计划
1. **主线程外观模式(代理) + 后台工作线程架构重构**：将 \modules/mech_eye\ 分层隔离。所有直面相机 SDK 的繁重交互调用，必须尽数收拢至一个专职对应的 \QThread\ (后台工作线程) 内部运行；主线程只充当门面（Facade），负责向下派发交互指令流并向上接收最终结果数据。
2. **统一定义交互数据类型**：专门设计如 \CaptureRequest\ (采集请求参数集)、\CaptureResult\ (采集后输出的点云及标定结果)、\CameraStatus\ (设备联机在线状态) 与 \CaptureError\ (故障定性明细) 这些强类型的结构体作为标准的内外通信载体。
3. **升级 MechEyeService 为全异步服务**：将其从原来的“同步查询辅助工具类”升维成名副其实的“异步服务控制器”。通过 Qt 信号槽机制，对外暴露异步初始化、异步拍照触发动作、状态心跳反馈以及设备错误自动警报响应接口。
4. **全生命周期管控入驻后台**：悉数交由工作子线程掌握主导相机的：搜索寻机、网络建联、采图成像、异常抓取捕获、强制断开及最终资源的释放毁灭等动作；坚决封堵主线程越界直接调用 SDK 原生接口的后门。
5. **前置编织线程级安全保护网**：提前在代码中布控健壮的应对机制：如请求指令的序列号追溯排查、抵御反复秒击引发重入的限流抗抖防卫、当析构指令降临时能优雅断开并退出的防崩手段、相机物理断网剥落后的重连恢复尝试，以及捕获任何致命异常抛出时的底层清场兜底逻辑。
6. **为流控中心预留对接锚点(钩子)**：这一步先不去强行推翻剥动当下的全局 \
low_control\ 状态机链轨；但务必在所改建的相机服务出入交互口上，精设能够无缝契合后续“步骤 2”将进行数据接驳拼装工作时的外延 Callback/Signal 回调钩子。
7. **执行步骤 1 的全栈贯通保活验证**：必须确认实施后的重构代码不仅能通过 CMake 顺利编译；而且必须在接线实测多轮异步触发获取数据的过程里，绝对不能牵扯绊住挂滞当前 Qt 的主事件消息循环队列，以及底层的 Modbus 轮询监控线程，心跳保活必须丝毫不减顺畅。

### 步骤 2：改造 Trig_ScanSegment 分段扫描流控业务
在 \state_machine.cpp\ 当中，将现有的 \Trig_ScanSegment\ 响应分支剔除原有的模拟打桩逻辑，正式接驳由“步骤 1”完工所吐出的异步相机信号。将异步拿到返回的具体点云图元根据切分传入的 \ScanSegmentIndex\（段号）施以有序的分段缓冲暂存。死锁直至确实验明收到解开异步回调包裹响应之时，才能再向 PLC 提报发写 \Res_ScanSegment\ 结果码合并推挂出 \Ack_ScanSegment=2\。

#### 步骤 2 子计划

### 步骤 3：构建 Trig_Inspection 整体检测计算与清场流
当触发流控状态捕捉到了 \Trig_Inspection\ (总体检测下达令) 后，立即将此前一直暂存囤压在内存池里的全套“分段点云集”投递抽推至计算枢纽 \modules/tracking\ 里开展合并运算（前期阶段允许暂用虚置的 Mock 验算返回骨架做中转演练过渡）。索要到最终的综合检勘判定与评星定级报告之后，要把各分项疵点原因代码分流悉数誊注进 \NG_Reason_Word0\ 以及最终回填 \Res_Inspection\ 之内。核录定案结束后紧抓收场闭卷清理任务：立即执行指令销毁、防堵内存决堤，彻底抹杀掉已被运算利用过的这批冗大断残点云内存缓冲池。

#### 步骤 3 子计划

## 4. 当前阶段性结论

当务之急是要**全力优先打通且夯实步骤 1**。
因为这至关重要、承上启下的异步底层结构大改重构，是日后真正保证有底气能够不卡顿地允许 \Trig_ScanSegment\ 相机图采大任务与 \Trig_Inspection\ 重型检测任务在纯后台非阻塞大生态闭路集成下运转的无价基石。

## Step1.1 修改后的现状态

### 完成项

- 已将 `modules/mech_eye` 收敛为一致的“主线程 Facade + 后台 Worker 线程”结构。
- `MechEyeService` 负责主线程异步代理，`MechEyeWorker` 负责在独立线程中执行 SDK 调用。
- 已补齐跨线程类型注册，避免 queued connection 传参失败。
- 已补齐默认相机配置读取，优先使用 `config.ini` 中的 `defaultCamera`。
- 已补齐采集超时参数下传，SDK 采集接口会使用请求中的 `timeoutMs`。
- 已补齐 3D 与 2D+3D 的点云回填逻辑，`CaptureResult.pointCloud` 可真正带回点云数据。
- `StateMachine` 仍保持现有模拟业务逻辑，仅监听 `MechEye` 状态和致命错误。

### 变更文件

- `modules/mech_eye/include/scan_tracking/mech_eye/mech_eye_types.h`
- `modules/mech_eye/include/scan_tracking/mech_eye/mech_eye_service.h`
- `modules/mech_eye/include/scan_tracking/mech_eye/mech_eye_worker.h`
- `modules/mech_eye/src/mech_eye_service.cpp`
- `modules/mech_eye/src/mech_eye_worker.cpp`
- `modules/flow_control/src/state_machine.cpp`

### 验证结果

- 2026-04-10 验证了 Windows IPC 环境，构建 Debug 模式下的 Ninja 编译成功，解决了 MSVC 标准库和 Qt 跨线程类型引用的错误。
- 运行 run-debug 验证了 MechEyeService 主线程无阻塞，独立的 MechEyeWorker 会在后台独立跑通“正在搜索相机 -> 正在连接相机 -> 获取超时”全流程。
- 并且证实了 Modbus TCP 的前台心跳没有受到影响流转顺畅，证明相机的采集阻塞耗时操作已经被完美底层隔离。
- Step 1.1 完美收尾，能够承接后续开发流程。

### 验证结果???

- Step2 和 Step3 尚未开始，相关的触发任务仍然使用模拟流程。
- 下一步需要将真实相机回调信号串入业务，正式迎接 PLC 的状态机器交互和握手。
- 当前点云仅保留 XYZ 数据，如果算法还需要法向等数据，后续再扩展。

### 验证结果?

- 已经可以正式进入 Step2，将实际的逻辑接入 Trig_ScanSegment 的四步真实握手流程。
- 需要改造 state_machine.cpp，监听相关接口，根据传入的 ScanSegmentIndex 暂存分段图元缓存数据。
- 准备启动步骤 2 并写完基于内存挂载大点云的流控系统。


## 5. 最新 PLC 寄存器地址与流程备忘 (与PLC苗工确认版)


最新流控响应流程需按照以下寄存器地址时序来改造 `state_machine.cpp`：


1. **下料区与下料坐标解算 (第一步任务触发)**：写绝对地址 `40025 = 1`。IPC 在安全判断通过后响应计算，写入 `40145 - 40158`。


2. **上料抓取点解算触发**：待第一步通过且设备就位后，触发 `40020 = 1`。IPC 解算后写入 `40116 - 40129`。


3. **单段扫描执行 (重复 5-6 次)**：触发指令 `40023 = 1`。*(注意：不再判断扫描工位有料判断)*。IPC 扫完段并留存缓存后，握手并写入结果至 `40135 - 40137`。


4. **综合扫描检测 (Trig_Inspection)**：所有单阶段分段扫描完后总触发。IPC 汇集点云进行验算(暂Mock)，结果写入 `40140 - 40142`。


5. **下料执行确认**：因为第一步已做解算，是否再次校验待定，此步暂空。

## 6. 2026-04-17 最新进展补充

### 本轮新增目标

- 在保留 `modules/mech_eye` 真实点云采集链路的前提下，为两台海康相机预留正式接入框架。
- 新框架需统一输出：
  - 梅卡曼德点云
  - 海康相机 A 的 4x4 矩阵
  - 海康相机 B 的 4x4 矩阵
- 继续遵守“严禁阻塞主线程”的底线，不允许破坏现有 Modbus 轮询和 Qt 事件循环。

### 本轮已完成项

- 已新增 `modules/vision` 模块，作为视觉集成层。
- 已定义统一数据契约 `MultiCameraCaptureBundle`，用于向后续算法层统一交付：
  - `mechEyeResult`
  - `hikCameraAResult`
  - `hikCameraBResult`
- 已新增 `HikCameraService`，完成海康 MVS SDK 的最小可用接入：
  - SDK 初始化 / 反初始化
  - 枚举设备
  - 按 IP / SN / 名称匹配设备
  - 创建设备句柄并打开设备
  - GigE 最佳包长设置
- 已新增 `VisionPipelineService`，负责统一编排“一次梅卡点云 + 两次海康矩阵采集”的聚合骨架。
- 已将本机 `C:\\Program Files (x86)\\MVS` 的最小开发集复制进项目内：
  - `third_party/hik_mvs/include`
  - `third_party/hik_mvs/lib/win64`
  - `third_party/hik_mvs/runtime/Win64_x64`
  - `third_party/hik_mvs/docs`
- 已新增 `cmake/HikMvsSdk.cmake`，项目现在可直接使用仓库内的海康 SDK 副本完成编译与运行时部署。
- 已补齐 `[Vision]` 配置项，默认预填两台相机地址：
  - `192.168.10.12`
  - `192.168.10.13`

### 关于非阻塞要求的落实情况

- `MechEyeService` 仍然保持“主线程 Facade + 后台 Worker/QThread”的异步结构，未回退。
- `HikCameraService` 当前已经将“设备枚举 / 匹配 / 打开”和“矩阵采集入口”放入后台线程执行。
- 主线程当前仅负责：
  - 发起请求
  - 接收 Qt 信号
  - 维持状态机与 PLC 轮询
- 因此到目前为止，视觉侧新增代码仍符合“不阻塞主线程”的架构要求。

### 实机验证结果

- 已在工控机上完成真实编译验证，`build-debug` 成功。
- 已在工控机上完成真实启动验证，启动日志确认：
  - 海康相机 A 已连接：`192.168.10.12`，序列号 `DA4444604`
  - 海康相机 B 已连接：`192.168.10.13`，序列号 `DA2738512`
  - 梅卡曼德相机已连接：`RUM3525CA510C099`

### 当前边界

- 海康 MVS SDK 已经“接通到设备层”，但业务层仍未生成真实 4x4 矩阵。
- 当前 `requestPoseCapture()` 返回的是单位矩阵占位结果，仅用于打通服务骨架与后续接口。
- `VisionPipelineService` 目前只负责聚合三路结果，不负责最终算法判断。
- `StateMachine` 暂未接入 `VisionPipelineService::requestCaptureBundle()` 的真实流程入口。

### 下一步建议

1. 在 `HikCameraService::requestPoseCapture()` 的后台任务中接入真实采图、标定换算和 4x4 位姿矩阵生成。
2. 在 `VisionPipelineService` 中补统一时间戳、超时仲裁、矩阵有效性和坐标系一致性校验。
3. 将 `VisionPipelineService::requestCaptureBundle()` 正式接入 `Trig_ScanSegment` 或新的统一扫描业务入口。

### 本轮收尾补充

- 已再次检查视觉链路的非阻塞要求：
  - `MechEyeService` 继续保持 `Facade + Worker/QThread` 的异步模型。
  - `HikCameraService` 已明确采用后台线程执行连接与采集请求，避免阻塞主线程、PLC 轮询和 Qt 事件循环。
- 已更新 `.gitignore`，明确忽略 `third_party/`，避免将本地复制进仓库的海康 SDK 运行时和库文件纳入版本管理。
- 已在以下位置补齐后续开发 TODO，方便后续算法开发时快速续接：
  - `docs/vision_framework.md`
  - `modules/vision/src/hik_camera_service.cpp`
  - `modules/vision/src/vision_pipeline_service.cpp`
- 当前版本定位已经明确：
  - 框架、SDK 接线、配置、编译和相机连通性已经打通。
  - 真实矩阵算法、时序治理和业务接入保留到下一阶段实现。
