# 封头检测工位 IPC-Qt 显控通信协议

**版本**：v1.1（2026-06-19 增补 `cmd.set_unload_area_config` 与 `status.plc` 下料区封头字段）  
**适用范围**：第一工位核心控制程序（Windows）与 Qt 显控界面（麒麟 OS）的 TCP/IP 通信。

---

## 1. 通信基础约定

### 1.1 网络架构
- **TCP Server**：核心控制程序（Core）。
- **TCP Client**：Qt 显控程序（Qt）。单客户端连接，不支持多个显控端同时连接。
- **异常处理**：
  - 心跳机制：双方周期发送心跳包（建议 2000ms），超过 6 秒未收到心跳判断为断连。
  - 断连行为：Qt 断连时 Core 不自动暂停，保持继续运行。

### 1.2 报文格式
TCP 是流式协议，为解决粘包和半包问题，采用长度前缀的帧格式：
```text
[4 字节大端长度头] + [JSON 正文 UTF-8]
```
- **长度头**：`uint32_t`，大端网络字节序（Big-Endian），表示后续 JSON 正文的字节数。
- **正文**：不包含大点云和 2D 图像原始数据，仅传输数据摘要。

### 1.3 JSON 基本结构
所有通信的 JSON 正文统一格式如下：
```json
{
  "version": "1.0",
  "msgId": "uuid-或自增序列号",
  "type": "消息类型",
  "timestamp": 1710000000000,
  "payload": {}
}
```
- `version`：协议版本，固定 `"1.0"`。
- `msgId`：请求-响应匹配的唯一标识。
- `type`：消息类型，见后续定义。
- `timestamp`：发送时的 Unix 毫秒时间戳。
- `payload`：实际数据，依 `type` 变化。

### 1.4 消息模型
1. **request**：Qt 主动发起请求或发送控制命令。
2. **response**：Core 对 request 的应答，`msgId` 必须与请求一致。
3. **event**：Core 主动上报状态、结果、报警、日志等，Core 自行生成 `msgId`。

---

## 2. 状态上报与事件 (Event)

由 Core 主动发给 Qt，频率根据字段要求不同。

### 2.1 心跳
- `type`: `heartbeat.ping` (发送方), `heartbeat.pong` (接收方响应)
- `payload`: `{}`

### 2.2 系统运行状态 (`status.system`)
- **频率**：状态变更上报，或 500ms 周期。
- **payload 字段**：
  - `ipcState` (int): 0=未初始化, 1=初始化中, 2=就绪, 3=忙碌, 4=暂停, 5=故障
  - `appState` (string): "Init", "Ready", "Scanning", "Error"
  - `stage` (int): 当前工艺阶段 0~10
  - `alarmLevel` (int): 0=无, 1=提示, 2=黄警, 3=红警
  - `alarmCode`, `warnCode` (int)
  - `ipcReady` (int): 0/1
  - `progress` (int): 0~100

### 2.3 PLC 状态 (`status.plc`)
- **频率**：500ms 周期。
- **payload 字段**：
  - `plcHeartbeat` (int)
  - `plcSystemState` (int): 0=待机, 1=自动, 2=暂停, 3=报警停机, 4=手动
  - `workMode` (int): 0=空闲, 1=上料, 2=扫描, 3=检测, 4=下料
  - `flowEnable` (int): 0/1
  - `safetyWord` (int): 位域字典
  - `taskId` (int), `productType` (int), `recipeId` (int)
  - `scanSegmentIndex` (int), `scanSegmentTotal` (int)
  - `robotStatusWord` (int): 埃斯顿机械臂状态字（PLC 转发 Robot Modbus 40004，位定义见 IPC-PLC 协议 §8.1.1）
  - `telescopicRodStatus` (int): 伸缩杆状态，0=待机, 1=运行, 2=故障（PLC 40041）
  - `rollerSetFreqHz` (int): 滚轮设定频率 Hz（PLC 40042）
  - `rollerRunFreqHz` (int): 滚轮运行频率 Hz（PLC 40043）
  - `electromagnetStatus` (int): 电磁吸盘状态，0=退磁, 1=充磁, 2=报警（PLC 40044）
  - `estopButtonStatus` (int): 急停按钮，0=断开(未按下), 1=按下（PLC 40045）
  - `loadVisionStatusCode` (int): 上料视觉状态码（PLC 40046），1100=坐标发送完成，1102=拍照计算完成
  - `unloadNgAreaFull` (int): NG 区满料，0=未满料, 1=满料（PLC 40047）
  - `unloadOkAreaFull` (int): OK 区满料，0=未满料, 1=满料（PLC 40048）
  - `plcUnloadNgAreaCount` (int): PLC 上报 NG 区封头数量（PLC 40049，单位：个）
  - `plcUnloadOkAreaCount` (int): PLC 上报 OK 区封头数量（PLC 40050，单位：个）
  - `modbusConnected` (bool)
  - `unloadAreaMaxStackCount` (int): 下料区封头最大叠加个数（IPC 缓存，来源显控 `cmd.set_unload_area_config`）
  - `unloadAreaOkCount` (int): 下料 OK 区封头数量（**显控经 IPC 写入 40177 的配置/计数**，非 PLC 40050）
  - `unloadAreaNgCount` (int): 下料 NG 区封头数量（**显控经 IPC 写入 40178 的配置/计数**，非 PLC 40049）
  - `unloadAreaAutoClear` (bool): 自动清零使能，true=PLC 在整垛取走后自动清零对应计数

> **辅机字段**：第一、第二工位均推送；无 PLC 数据时缺省为 0。`telescopicRodStatus` 或 `electromagnetStatus` 变为 **2** 时，Core 向显控推送 `event.alarm`（`level=2`，`code` 920/921，见 §2.8）。  
> **下料区计数区分**：`plcUnloadOkAreaCount` / `plcUnloadNgAreaCount` 为 **PLC 实时上报**（40049/40050）；`unloadAreaOkCount` / `unloadAreaNgCount` 为 **显控经 `cmd.set_unload_area_config` 写入 IPC 后回显**（40177/40178），两者来源不同，UI 勿混用。

### 2.4 相机与设备状态 (`status.camera` / `status.device`)
- **频率**：
  - Core 每 **500ms** 轮询一次是否需推送，但 **仅当 JSON payload 与上次下发不一致时才真正发送**（变更去重，稳态下不会每 500ms 刷一条）。
  - 相机/流水线/梅卡等 **连接态或 `state` 变化** 时立即尝试推送，同样受 payload 去重约束（海康仅 `connected` 变化才触发 `status.camera` 内容变化，采图过程中的文字状态不会刷屏）。
  - 新客户端接入时 **强制全量** 各推送一次（含 `status.camera`）。
- **显控侧预期**：稳态监视下 `status.camera` 很少出现；连接瞬间可能连收数条（`mechEye`/`hikA`/`hikB`/`hikC`/`pipeline` 分别就绪时）；不应出现无变化的周期性刷屏。
- **camera payload**:
  - `mechEye`: `{ state(int), model, sn, connected(bool) }`
  - `hikA`, `hikB`, `hikC`: `{ roleName, connected(bool) }`（`hikC` 的 `connected`：MVS SDK 已连接 **或** 智能相机 TCP 已接入）
  - `pipeline`: `{ state(int) }`
- **device payload**:
  - `onlineWord0`, `faultWord0` (int) 位域字典（`online` 与 `fault` 按位对齐，便于显控做设备条）

**`status.device` 位定义（word0，低位 Bit0）**

| Bit | `onlineWord0` | `faultWord0` |
|-----|---------------|--------------|
| 0 | IPC Core 进程在线 | IPC 故障（`ipcState=Fault` / `appState=Error` / 红警 `alarmLevel≥3`） |
| 1 | HMI 客户端已连接 | （保留） |
| 2 | Mech-Eye 可用（非 Idle/Error） | Mech-Eye `Error` |
| 3 | 视觉流水线 `Ready` | 视觉流水线 `Error` |
| 4 | CXP 双目 A/B 或海康 C 任一台已连接 | CXP A/B 与海康 C 均未连接（服务已配置时） |
| 5 | Tracking 模块已加载 | （保留） |
| 6 | Modbus 已连接 | Modbus 未连接 |
| 7 | （保留） | 黄警 `alarmLevel≥2` |

### 2.5 流程事件 (`event.scan.*` / `event.task.*`)
- `event.scan.started`: `{ segmentIndex, taskId }`
- `event.scan.finished`: `{ segmentIndex, resultCode, imageCount, cloudFrameCount }`
- `event.image.captured`: `{ requestId, cameraKey, pointCount, width, height, elapsedMs, errorCode }` (不传原图)
- `event.bundle.captured`: `{ segmentIndex, taskId, mechOk, hikAOk, hikBOk, lbOk }`

### 2.6 第一工位检测结果 (`event.inspection.finished`)
- **产生时机**：
  - **正式**：PLC `Trig_Inspection` → `TrackingService::inspectPointCloud`（Po_Kou 坡口测量）→ `HmiTcpServer::publishInspectionResult`
  - **调试**：显控 `cmd.debug_trigger_inspection`（须 `config.ini [Hmi] allowDebugTriggerInspection=true`）
- **推送策略**：`resultCode=1`（OK）与 `resultCode=2`（NG）**均推送**；显控须处理失败场景。
- **连接初始化**：显控 TCP 接入成功后，Core **一次性**推送 `event.inspection.finished`，`resultCode=0`，`headMetrics` 及测量字段均为 **0**，`message="等待检测"`，供 UI 绑定初始化；**非真实检测结果**，正式检测完成后会再次推送覆盖。
- **临时演示（path1~5 扫完）**：`config.ini [Hmi] emitPresetInspectionOnPathsComplete=true` 时，全部 `selectedPathIds` 启用路径扫描完成后 Core 推送 **预设** `event.inspection.finished`（`resultCode=1`，`headMetrics` 含 12 项现场演示值，非算法实测）；每轮扫描仅一次，`Trig_ResultReset` 后可再次触发。
- **payload 字段**（Po_Kou 坡口测量）：
  - `resultCode` (int): **0=尚未检测/连接占位**，1=OK, 2=NG
  - `ngReasonWord0`, `ngReasonWord1` (int)
  - `measureItemCount` (int)
  - `sourcePointCount` (int): 合并后输入点云点数
  - `head_angle_tol` (float): 坡口角（deg）
  - `blunt_height_tol` (float): 钝边长度（mm）
  - `bevel_type` (int): 坡口类型
  - `icp_fitness` (float): ICP 拟合度
  - `quality_code` (int): 0=在标准范围内
  - `expected_bevel_type`, `expected_angle_deg`, `expected_length` (float/int): 当前生效工艺配方
  - `has_hole` (bool): 当前工件是否有孔；`false`=无孔（不测孔径），`true`=有孔（将来调用测孔算法）
  - `angle_tol_deg`, `length_tol_mm` (float): 合格判定公差（来自 config.ini）
  - `headMetrics` (object): 显控 12 项指标统一显示包，**始终包含下列 12 键**；显控 UI 建议直接绑定此对象
    - `inner_diameter_mm` (float): 封头内径（mm），**暂为 0**
    - `roundness_tol` (float): 封头圆度公差，**暂为 0**
    - `straight_slope_tol` (float): 封头直边斜度，**暂为 0**
    - `head_depth_mm` (float): 封头深度（mm），**暂为 0**
    - `straight_height_tol` (float): 封头直边高度，**暂为 0**
    - `bevel_angle_deg` (float): 封头坡口角度（deg），Po_Kou 实测，与 `head_angle_tol` 一致
    - `blunt_height_mm` (float): 封头钝边高度（mm），Po_Kou 实测，与 `blunt_height_tol` 一致
    - `inner_circumference_mm` (float): 封头内周长（mm），**暂为 0**
    - `hole_opening_mm` (float): 封头开孔（mm）；**无孔时固定 0**；有孔时待测孔算法接入后填实测值
    - `joint_fit_up_angle_deg` (float): 封头接头组对角度（deg），**暂为 0**
    - `thickness_mm` (float): 封头测厚（mm），**暂为 0**
    - `head_volume_m3` (float): 封头容积（m³），**暂为 0**
  - `message` (string): 概要描述

### 2.7 其他检测校验完成 (`event.xxx.finished`)
- `event.pose_check.finished`: `{ success, resultCode, poseDeviationMm, rt[16], message }`
- `event.load_grasp.finished`: `{ resultCode, x, y, z, rx, ry, rz }`
- `event.unload_calc.finished`: `{ resultCode, x, y, z, rx, ry, rz }`
- `event.self_check.finished`: `{ resultCode, failWord0 }`
- `event.code_read.finished`: `{ resultCode, codeValue(string) }`
- `event.result_reset.finished`: `{ resultCode }`

### 2.8 报警与日志
- `event.alarm`: `{ level, code, message, timestamp }` (单向发送，无需回执)
- **辅机 PLC 报警 code（920 段，与 Modbus 900 段、相机 910 段区分）**：
  - `920`：`telescopicRodStatus` 变为 2（伸缩杆故障），`level=2`
  - `921`：`electromagnetStatus` 变为 2（电磁吸盘报警），`level=2`
  - 边沿触发：仅在由非 2 变为 2 时推送一次；新客户端接入后同步缓存，不重复推送当前故障态
- `event.log`: `{ severity, category, message, file, line, timestamp }`

---

## 3. Qt 控制命令 (Request / Response)

Qt 发送 request（附带不重复的 `msgId`），Core 执行后返回对应 `msgId` 的 response，response `payload` 基础结构包含：`{ "success": true/false, "message": "描述" }`，部分命令会附加额外字段。

| type | request payload | response payload 附加字段 | 说明 |
|---|---|---|---|
| `cmd.start` | `{}` | - | 启动状态机 |
| `cmd.stop` | `{}` | - | 停止状态机 |
| `cmd.reset` | `{}` | - | 重置状态 |
| `cmd.clear_alarm` | `{}` | - | 清除当前报警记录 |
| `cmd.get_status` | `{}` | `system`, `plc`, `camera`, `device` 全量状态对象 | 主动拉取全量状态 |
| `cmd.get_config` | `{ "section": "..." }` | 指定 section 或全量 JSON；`bevel.recipe` 含当前配方 | 获取 Core 侧配置 |
| `cmd.set_bevel_recipe` | `{ "bevel_type": 0, "has_hole": false }` 或含 `angle_deg`/`length` | 回显 `bevel_type`、`angle_deg`、`length`、`has_hole`、`angleTolDeg`、`lengthTolMm` | 设置坡口配方。**标准型号**：type0=45°/1mm，type1=30°/6mm；`has_hole` 可选，**未传默认 false（无孔）** |
| `cmd.trigger_scan` | `{ "segmentIndex": 1, "taskId": 123 }` | - | 触发单段扫描 |
| `cmd.trigger_inspection` | `{ "taskId": 123 }` | - | 触发综合检测（**Core 拒绝**，须 PLC） |
| `cmd.debug_trigger_inspection` | `{}` | `resultCode`, `cachedSegmentIndices`, `cachedSegmentCount`, `inspectionMessage`, `multiPathNote` | **调试**：用内存缓存点云跑蓝友并推 `event.inspection.finished`；须 `allowDebugTriggerInspection=true`；不写 PLC |
| `cmd.trigger_self_check` | `{}` | - | 显控触发自检：IPC 置 `IPC_CurrentStage=SelfCheck` 通知 PLC；PLC 调度两点 `Trig_ScanSegment` 后下发 `Trig_SelfCheck` 执行算法；完成后推送 `event.self_check.finished` |
| `cmd.trigger_pose_check` | `{}` | - | 触发位姿校验 (当前为占位) |
| `cmd.trigger_code_read` | `{}` | - | 触发条码读取 (当前为占位) |
| `cmd.trigger_result_reset`| `{}` | - | 触发结果缓存清空 |
| `cmd.capture_mech_eye` | `{ "cameraKey":"...", "mode":0, "timeoutMs":5000 }` | `"requestId": 123` | 单相机独立采图 |
| `cmd.capture_bundle` | `{ "segmentIndex":1, "taskId": 123 }` | `"requestId": 123` | 触发多相机集成采集 |
| `cmd.refresh_camera` | `{}` | - | 刷新相机连接状态 |
| `cmd.modbus_connect` | `{}` | - | 重连 PLC Modbus |
| `cmd.modbus_disconnect` | `{}` | - | 断开 PLC Modbus |
| `cmd.set_unload_area_config` | 见下表 | 回显四个字段及 `plcWritten` | 设置下料区封头叠加上限、OK/NG 区数量、自动清零使能；**IPC 收到后写入 PLC 40176~40179** |

**`cmd.set_unload_area_config` request payload（字段均可选，但至少提供一个）**

| 字段 | 类型 | 范围 | 说明 |
|------|------|------|------|
| `maxStackCount` | int | 0~255 | 下料区封头最大叠加个数 |
| `okCount` | int | 0~9999 | 下料 OK 区当前封头数量 |
| `ngCount` | int | 0~9999 | 下料 NG 区当前封头数量 |
| `autoClear` | bool | - | 自动清零使能：`true`=启用（PLC 在整垛取走后自动清零对应区计数），`false`=关闭 |

**response payload 附加字段**：`maxStackCount`、`okCount`、`ngCount`、`autoClear`（bool）、`plcWritten`（bool，是否已成功写入 Modbus）。

> **数据流**：显控维护 UI 计数 → 经 TCP 下发 IPC → IPC 写入 PLC 结果区 `40176~40179`（无 Trig 握手，PLC 周期读取）。IPC 重启或结果区清零后，显控应重新下发当前配置。

> **备注**：不需要支持 `cmd.set_config` （热修改配置）。Qt 不直连 PLC；下料区封头计数经 `cmd.set_unload_area_config` 由 IPC 转发至 Modbus 40176~40179。第二、第三工位参数暂不加入协议。  
> **`cmd.trigger_*` 与 `cmd.debug_trigger_inspection` 区别**：`cmd.trigger_self_check` 由显控发起，IPC 通过 Modbus 状态字通知 PLC 进入自检流程（不占用 scan_paths 生产路径）；PLC 完成两点扫描并下发 `Trig_SelfCheck` 后 IPC 执行算法。其余 `cmd.trigger_*` 一律拒绝（防撞机）；`cmd.debug_trigger_inspection` 为配置门控的演示/联调入口，默认关闭。

---

## 4. 示例

### 4.1 触发检测与响应
**Qt 请求：**
```json
{
  "version": "1.0",
  "msgId": "req-101",
  "type": "cmd.trigger_inspection",
  "timestamp": 1710000000000,
  "payload": {
    "taskId": 123
  }
}
```
**Core 响应：**
```json
{
  "version": "1.0",
  "msgId": "req-101",
  "type": "cmd.trigger_inspection",
  "timestamp": 1710000000050,
  "payload": {
    "success": true,
    "message": "Inspection triggered."
  }
}
```

### 4.3 设置下料区封头配置

**Qt 请求：**
```json
{
  "version": "1.0",
  "msgId": "req-210",
  "type": "cmd.set_unload_area_config",
  "timestamp": 1710000000000,
  "payload": {
    "maxStackCount": 8,
    "okCount": 3,
    "ngCount": 1,
    "autoClear": true
  }
}
```

**Core 响应：**
```json
{
  "version": "1.0",
  "msgId": "req-210",
  "type": "cmd.set_unload_area_config",
  "timestamp": 1710000000050,
  "payload": {
    "success": true,
    "message": "下料区封头配置已转发 PLC",
    "maxStackCount": 8,
    "okCount": 3,
    "ngCount": 1,
    "autoClear": true,
    "plcWritten": true
  }
}
```

### 4.2 检测完成事件上报
**Core 上报：**
```json
{
  "version": "1.0",
  "msgId": "evt-200",
  "type": "event.inspection.finished",
  "timestamp": 1710000005000,
  "payload": {
    "resultCode": 1,
    "ngReasonWord0": 0,
    "ngReasonWord1": 0,
    "measureItemCount": 2,
    "sourcePointCount": 125000,
    "head_angle_tol": 45.2,
    "blunt_height_tol": 1.05,
    "bevel_type": 0,
    "icp_fitness": 0.98,
    "quality_code": 0,
    "headMetrics": {
      "inner_diameter_mm": 0,
      "roundness_tol": 0,
      "straight_slope_tol": 0,
      "head_depth_mm": 0,
      "straight_height_tol": 0,
      "bevel_angle_deg": 45.2,
      "blunt_height_mm": 1.05,
      "inner_circumference_mm": 0,
      "hole_opening_mm": 0,
      "joint_fit_up_angle_deg": 0,
      "thickness_mm": 0,
      "head_volume_m3": 0
    },
    "expected_bevel_type": 0,
    "expected_angle_deg": 45,
    "expected_length": 1,
    "has_hole": false,
    "angle_tol_deg": 2,
    "length_tol_mm": 1,
    "message": "坡口测量通过：angle=45.200 deg, length=1.050 mm, bevelType=0, icpFitness=0.980000。"
  }
}
```
