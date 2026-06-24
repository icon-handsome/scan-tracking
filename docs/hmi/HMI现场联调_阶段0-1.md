# HMI 现场联调 — 阶段 0 + 1 + 3.1 + 调试检测

**目标**：麒麟 Qt 显控（TCP Client）能稳定连接本仓库 IPC Core（TCP Server），解析 `status.*`、`event.*` 及检测测量字段；可选验证 `cmd.debug_trigger_inspection` 演示链路。

> **本仓库仅 Server**。总览见 **[HMI开发交接说明.md](./HMI开发交接说明.md)**。

---

## 1. Core 侧（本仓库）

- HMI TCP 监听 **`config.ini [Hmi] tcpPort`**（默认 **9900**）。
- 客户端连接后：`core.hello` + 四类 `status.*` 全量推送。
- 周期：**500ms** 状态轮询（**payload 不变则不下发**，稳态不会刷 `status.camera`）、**2s** 心跳 `heartbeat.ping`。
- Core `[TCPIP]`：默认仅在实际推送 `status.*`（payload 变化）时打印 `→ TX status.xxx` 摘要；需心跳/每帧 trace 时将 `hmi_protocol.h` 的 `kHmiTcpVerboseTrace` 改为 `true`。
- 业务事件由状态机/相机/Modbus 触发上报。

---

## 2. 阶段 0 验收（传输 + 握手）

| 步骤 | 操作 | 预期 |
|------|------|------|
| 1 | 运行 Core | 日志：`HMI TCP 服务端已在端口 <tcpPort> 启动` |
| 2 | Qt 显控连接 Core | `QTcpSocket` 连 `<Core_IP>:<tcpPort>` |
| 3 | 帧格式 | **4 字节大端长度 + JSON**，禁止裸 JSON |
| 4 | 握手 | 收到 `core.hello`；发 `hmi.hello` 收 success |
| 5 | 心跳 | 收到 `heartbeat.ping` 后回 `heartbeat.pong`（`msgId` 与 ping 一致） |
| 6 | 拉状态 | 发 `cmd.get_status` 或等待周期 `status.*` |

帧发送可参考 Server 侧 `hmi_protocol.cpp` 的 `serializeFrame` / `buildEnvelope`。

---

## 3. 阶段 1 验收（status / event）

Qt 显控应能解析并展示：

- `status.system` — ipcState / appState / alarmLevel / progress
- `status.plc` — modbusConnected / workMode / scanSegmentIndex
- `status.camera` — mechEye / hikA / hikB / hikC
- `status.device` — onlineWord0 / faultWord0（位定义见协议 §2.4）

梅卡/海康 **连接或断开** 时（不含采图过程）另收 `event.alarm`：`level=1` 已连接 / `level=2` 已断开，`code` 910–913，文案含「已连接」「已断开」。新客户端接入时只同步状态，不刷历史 alarm。

触发 PLC 流程后应收到：

- `event.scan.started` / `event.scan.finished`
- `event.bundle.captured`
- `event.inspection.finished`
- `event.alarm`（异常时）

---

## 4. Qt 显控集成（独立仓库）

1. 在麒麟 Qt 工程实现 TCP Client，按 [`封头检测工位_TCP_IP显控通信协议_v1.0.md`](../protocols/封头检测工位_TCP_IP显控通信协议_v1.0.md) 解帧。
2. 可按 `type` 分发：`status.*`、`event.*`、`cmd.*` 响应。
3. 连接成功后建议先发 `hmi.hello`，再 `cmd.get_status`。
4. **不要**在 UI 调用 `cmd.trigger_scan` / `cmd.trigger_inspection` 等（Core 会返回失败）。
5. 演示检测页可使用 `cmd.debug_trigger_inspection`（须 Core `allowDebugTriggerInspection=true`）。

协议常量与组包可参考本仓库（仅作引用，不编入 Core 工程）：

- `modules/hmi_server/include/scan_tracking/hmi_server/hmi_protocol.h`
- `modules/hmi_server/src/hmi_protocol.cpp`

---

## 5. 常见故障

| 现象 | 原因 | 处理 |
|------|------|------|
| 立刻断连 | 未加 4 字节长度头 | 对齐 `serializeFrame` |
| 只有心跳无 status | 未解析 `type` | 处理 `status.system` 等 |
| modbusConnected 一直 false | PLC 未连 | 先确认 Modbus；status 仍会推 |
| `status.camera` 刷屏 | 旧版 Core 在海康 `stateChanged` 时误强制重发 | 升级 Core；显控侧仍建议按 `type`+payload 去重显示 |
| 长时间无 `status.camera` | 相机状态未变（正常） | 发 `cmd.get_status` 或拔插相机验证变更推送 |
| 发调试检测命令被拒绝 | `allowDebugTriggerInspection=false` | 改 `config.ini` 为 true 并重启 Core |
| 有命令响应但无检测事件 | 显控未订阅 `event.inspection.finished` | 成功/失败均会推送；NG 也要展示 |
| 调试检测 NG「缺少必需分段」 | 缓存未凑齐外/内/孔三段 | 对照 `[Tracking]` 段号与 `cachedSegmentIndices`；确认已按序完成对应 `Trig_ScanSegment` |
| 检测成功但无 run_* 落盘 | 位姿拼接未执行或目录不可写 | 看 Core 日志 `persistLastPoseStitchArtifactToDisk`；确认 `output/` 可写 |

---

## 6. 阶段 3.1 验收（检测测量字段）

**路径 A — PLC 正式流程**：`Trig_Inspection` 成功后推送。  
**路径 B — 调试命令**：发 `cmd.debug_trigger_inspection`（不写 PLC，不清缓存）。

`event.inspection.finished` 应含：

`head_angle_tol`、`straight_height_tol`、`straight_slope_tol`、`inner_diameter`、`blunt_height_tol`、`inner_diameter_tol`、`hole_diameter_tol`、`head_depth_tol` 及偏移量字段；失败时 `resultCode=2`，日志字段见 `outlinerErrorLog` / `inlinerErrorLog`。

数据链路（Core 内）：

```text
FirstPoseDetectionParams → InspectionMeasurement → TrackingService::deliverInspectionResult
  → HmiTcpServer::publishInspectionResult → event.inspection.finished
```

---

## 7. Qt 解析示例（检测完成）

```cpp
void onJsonEnvelope(const QJsonObject& envelope) {
    const QString type = envelope.value(QStringLiteral("type")).toString();
    if (type != QStringLiteral("event.inspection.finished")) {
        return;
    }
    const QJsonObject payload = envelope.value(QStringLiteral("payload")).toObject();
    const int resultCode = payload.value(QStringLiteral("resultCode")).toInt();
    const QJsonObject headMetrics = payload.value(QStringLiteral("headMetrics")).toObject();
    const double bevelAngleDeg = headMetrics.value(QStringLiteral("bevel_angle_deg")).toDouble();
    // resultCode==2 时也要更新 UI，展示 message / 错误日志
}
```

---

## 8. 调试命令验收（可选，客户演示）

### 8.1 配置

```ini
[Hmi]
enabled=true
tcpPort=9900
allowDebugTriggerInspection=true
```

修改后**重启 Core**。

### 8.2 步骤

| 步骤 | 操作 | 预期 |
|------|------|------|
| 1 | 显控连 Core | 收到 `core.hello` |
| 2 | 完成若干 `Trig_ScanSegment`（至少凑齐 `[Tracking]` 外/内/孔段） | Core 日志 `cacheSize` 增加 |
| 3 | 发 `cmd.debug_trigger_inspection` | 收到命令响应 + `event.inspection.finished` |
| 4 | Core 日志 | `[TCPIP] 蓝友检测结果已推送 event.inspection.finished` |

### 8.3 请求示例

```json
{
  "version": "1.0",
  "msgId": "dbg-001",
  "type": "cmd.debug_trigger_inspection",
  "timestamp": 0,
  "payload": {}
}
```

响应 `payload` 附加字段：`cachedSegmentIndices`、`cachedSegmentCount`、`inspectionMessage`、`multiPathNote`（说明多路径融合尚未实现）。

### 8.4 日常开发

切换其他功能时保持 `allowDebugTriggerInspection=false` 即可，不影响正式 PLC 流程与 HMI 状态推送。
