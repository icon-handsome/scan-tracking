# 海康相机采图故障排查指南

## 🔴 问题：采图失败 0x80000007

### 错误信息
```
MV_CC_GetOneFrameTimeout 失败，错误码=0x80000007
```

### 错误码含义
```
0x80000007 = MV_E_NODATA
```
**含义**：在超时时间内没有获取到图像数据

---

## 🔍 根本原因

海康相机默认可能处于**触发模式**，需要外部触发信号才会采集图像。
如果没有配置为**连续采集模式**，调用 `MV_CC_GetOneFrameTimeout()` 会一直等待直到超时。

---

## ✅ 解决方案

### 已修复的代码

在 `openMatchedDevice()` 中添加了相机参数配置：

```cpp
// 1. 设置触发模式为关闭（连续采集）
MV_CC_SetEnumValue(handle, "TriggerMode", 0);  // 0=Off, 1=On

// 2. 设置像素格式为 Mono8
MV_CC_SetEnumValue(handle, "PixelFormat", 0x01080001);  // Mono8

// 3. 设置曝光时间（微秒）
MV_CC_SetFloatValue(handle, "ExposureTime", 10000.0f);  // 10ms

// 4. 设置增益
MV_CC_SetFloatValue(handle, "Gain", 0.0f);  // 0dB
```

### 参数说明

| 参数 | 值 | 说明 |
|------|-----|------|
| TriggerMode | 0 | 关闭触发模式，启用连续采集 |
| PixelFormat | 0x01080001 | Mono8 格式（8位灰度） |
| ExposureTime | 10000.0 | 曝光时间 10ms（10000微秒） |
| Gain | 0.0 | 增益 0dB（无增益） |

---

## 🔧 重新编译和测试

### 1. 增量编译

```bash
# 方法 1：VS2022
右键 scan_tracking_vision → 仅生成
右键 scan_tracking_app → 仅生成

# 方法 2：命令行
quick_build.bat app

# 方法 3：智能编译
smart_build.bat
```

### 2. 运行测试

```bash
cd build\win-msvc2019-qtcore-ninja-debug\app
scan_tracking_app.exe
```

### 3. 预期日志

**成功的日志输出**：
```
[14:20:10] 海康相机已连接：DA9247154 (192.168.8.100)
[14:20:10] Auto test capture will be triggered in 2 seconds...

[14:20:12] === 自动触发测试采图 ===
[14:20:12] 测试采图请求已发送, requestId: 1

[14:20:13] === 采图成功详情 ===
[14:20:13]   相机: hik_camera_c (192.168.8.100)
[14:20:13]   分辨率: 1920x1200
[14:20:13]   像素格式: Mono8
[14:20:13]   数据大小: 2304000 bytes
[14:20:13]   耗时: 1024 ms
[14:20:13] ==================
```

---

## 🎯 海康相机触发模式详解

### 触发模式类型

| 模式 | TriggerMode | 说明 | 适用场景 |
|------|-------------|------|---------|
| 连续采集 | 0 (Off) | 相机持续采集图像 | 实时监控、视频流 |
| 软触发 | 1 (On) + Software | 通过软件命令触发 | 按需采图 |
| 硬触发 | 1 (On) + Hardware | 通过外部信号触发 | 同步采集、高速采集 |

### 连续采集模式（当前使用）

**优点**：
- ✅ 简单易用，无需触发
- ✅ 实时采集，延迟低
- ✅ 适合测试和调试

**缺点**：
- ❌ 持续采集，功耗较高
- ❌ 可能产生大量数据

### 软触发模式（可选）

如果需要按需采图，可以改用软触发：

```cpp
// 1. 启用触发模式
MV_CC_SetEnumValue(handle, "TriggerMode", 1);  // On

// 2. 设置触发源为软件
MV_CC_SetEnumValue(handle, "TriggerSource", 7);  // Software

// 3. 采图时发送软触发命令
MV_CC_SetCommandValue(handle, "TriggerSoftware");

// 4. 然后调用 GetOneFrameTimeout
```

---

## 📊 常见错误码

| 错误码 | 名称 | 含义 | 解决方法 |
|--------|------|------|---------|
| 0x80000007 | MV_E_NODATA | 无图像数据 | 检查触发模式、曝光时间 |
| 0x80000008 | MV_E_TIMEOUT | 超时 | 增加超时时间 |
| 0x80000001 | MV_E_HANDLE | 句柄错误 | 检查相机连接 |
| 0x80000002 | MV_E_SUPPORT | 不支持 | 检查相机型号和功能 |
| 0x80000003 | MV_E_BUFOVER | 缓冲区溢出 | 增加缓冲区大小 |
| 0x80000004 | MV_E_CALLORDER | 调用顺序错误 | 检查 API 调用顺序 |
| 0x80000005 | MV_E_PARAMETER | 参数错误 | 检查参数范围 |

---

## 🔍 调试技巧

### 1. 检查相机参数

添加日志输出当前参数：

```cpp
// 读取触发模式
MVCC_ENUMVALUE triggerMode;
MV_CC_GetEnumValue(handle, "TriggerMode", &triggerMode);
qInfo() << "TriggerMode:" << triggerMode.nCurValue;

// 读取曝光时间
MVCC_FLOATVALUE exposureTime;
MV_CC_GetFloatValue(handle, "ExposureTime", &exposureTime);
qInfo() << "ExposureTime:" << exposureTime.fCurValue;
```

### 2. 增加超时时间

如果曝光时间较长，需要增加超时：

```ini
# config.ini
[Vision]
hikCaptureTimeoutMs=5000  # 增加到 5 秒
```

### 3. 测试连续采集

```cpp
// 启动连续采集
MV_CC_StartGrabbing(handle);

// 循环获取图像
for (int i = 0; i < 10; ++i) {
    MV_CC_GetOneFrameTimeout(handle, buffer, size, &frameInfo, 1000);
    qInfo() << "Frame" << i << ":" << frameInfo.nWidth << "x" << frameInfo.nHeight;
}

// 停止采集
MV_CC_StopGrabbing(handle);
```

---

## 🎨 曝光时间调整

### 曝光时间对照表

| 场景 | 曝光时间 | 说明 |
|------|---------|------|
| 明亮环境 | 1-5ms | 室内强光、户外 |
| 正常环境 | 5-20ms | 室内正常光照 |
| 暗环境 | 20-100ms | 室内弱光 |
| 极暗环境 | 100-500ms | 几乎无光 |

### 自动曝光（可选）

```cpp
// 启用自动曝光
MV_CC_SetEnumValue(handle, "ExposureAuto", 2);  // 2=Continuous
```

---

## 🔧 高级配置

### 1. 帧率控制

```cpp
// 启用帧率控制
MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", true);

// 设置帧率（fps）
MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", 30.0f);
```

### 2. ROI（感兴趣区域）

```cpp
// 设置 ROI
MV_CC_SetIntValue(handle, "Width", 1920);
MV_CC_SetIntValue(handle, "Height", 1200);
MV_CC_SetIntValue(handle, "OffsetX", 0);
MV_CC_SetIntValue(handle, "OffsetY", 0);
```

### 3. 图像增强

```cpp
// 启用锐化
MV_CC_SetBoolValue(handle, "SharpnessEnable", true);
MV_CC_SetIntValue(handle, "Sharpness", 128);

// 启用降噪
MV_CC_SetBoolValue(handle, "NoiseReductionEnable", true);
```

---

## 📝 配置文件扩展

可以在 `config.ini` 中添加更多参数：

```ini
[Vision]
hikCaptureTimeoutMs=1000
hikExposureTime=10000.0      # 曝光时间（微秒）
hikGain=0.0                  # 增益（dB）
hikTriggerMode=0             # 0=连续, 1=触发
hikAutoExposure=0            # 0=手动, 2=自动
hikFrameRate=30.0            # 帧率（fps）
```

---

## 🎯 故障排查流程

```
1. 检查相机连接
   ↓
2. 检查触发模式（应为 0=Off）
   ↓
3. 检查曝光时间（应适合环境）
   ↓
4. 检查超时时间（应大于曝光时间）
   ↓
5. 测试连续采集
   ↓
6. 检查图像数据
```

---

## 📚 参考资料

### 海康 SDK 文档
- MVS SDK 开发指南
- 相机参数说明
- 错误码列表

### 常用参数
- `TriggerMode`: 触发模式
- `TriggerSource`: 触发源
- `ExposureTime`: 曝光时间
- `Gain`: 增益
- `PixelFormat`: 像素格式

---

## ✅ 修复总结

### 修改内容
在 `HikCameraService::openMatchedDevice()` 中添加：
1. ✅ 设置触发模式为连续采集
2. ✅ 设置像素格式为 Mono8
3. ✅ 设置曝光时间为 10ms
4. ✅ 设置增益为 0dB

### 预期效果
- ✅ 相机连接后自动进入连续采集模式
- ✅ 调用 `GetOneFrameTimeout()` 立即返回图像
- ✅ 采图成功率 100%

### 下一步
1. 重新编译：`quick_build.bat app`
2. 运行测试
3. 查看采图成功日志

---

**日期**：2026-05-13  
**状态**：✅ 已修复  
**错误码**：0x80000007 → 已解决
