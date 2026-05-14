========================================
海康智能相机 FTP 图像监控功能说明
========================================

## 功能概述

已实现完整的 FTP 图像监控功能，用于检测智能相机上传到 FTP 服务器的图像文件。

## 实现的文件

1. **头文件**
   - modules/vision/include/scan_tracking/vision/hik_smart_camera_ftp_monitor.h
   
2. **实现文件**
   - modules/vision/src/hik_smart_camera_ftp_monitor.cpp
   
3. **集成文件**
   - modules/vision/include/scan_tracking/vision/hik_camera_c_controller.h (已更新)
   - modules/vision/src/hik_camera_c_controller.cpp (已更新)
   - modules/vision/CMakeLists.txt (已更新)
   - app/src/console_runtime.cpp (已更新)

## 核心功能

### 1. 目录监控
- 使用 QFileSystemWatcher 监控 FTP 上传目录
- 监控目录：D:\HikCameraFTP
- 自动检测新上传的图像文件

### 2. 文件类型识别
根据文件名自动识别拍照类型：
- **SurfaceDefect** (表面缺陷) - 文件名包含: surface, defect, 表面, 缺陷
- **WeldDefect** (焊缝缺陷) - 文件名包含: weld, 焊缝, 焊接
- **NumberRecognition** (编号识别) - 文件名包含: number, ocr, 编号, 识别

### 3. 文件完整性检查
- 检测文件大小是否稳定（默认1秒）
- 确保文件传输完成后才处理
- 避免处理正在上传的文件

### 4. 支持的图像格式
- jpg, jpeg, png, bmp, tif, tiff

## 工作流程

```
1. 程序启动 → 初始化 FTP 监控器
2. 监控 D:\HikCameraFTP 目录
3. 检测到新文件 → 发射 newImageDetected 信号
4. 等待文件传输完成（大小稳定）
5. 文件就绪 → 发射 imageReady 信号
6. 触发后续处理（算法识别等）
```

## 配置参数

可通过以下方法调整参数：

```cpp
m_ftpMonitor->setFileCheckInterval(500);    // 文件检查间隔（毫秒）
m_ftpMonitor->setFileStableTime(1000);      // 文件稳定时间（毫秒）
m_ftpMonitor->setSupportedExtensions(...);  // 支持的文件扩展名
```

## 信号说明

### HikSmartCameraFtpMonitor 信号

1. **monitorStarted(QString directory)**
   - 监控器启动成功

2. **newImageDetected(ImageFileInfo imageInfo)**
   - 检测到新图像文件（可能还在上传中）

3. **imageReady(ImageFileInfo imageInfo)**
   - 图像文件传输完成，可以处理

4. **error(QString errorMessage)**
   - 监控器错误

### HikCameraCController 信号

1. **imageReceived(CaptureType type, QString filePath, qint64 fileSize)**
   - FTP 图像接收完成，可供应用层处理

## ImageFileInfo 结构

```cpp
struct ImageFileInfo {
    QString filePath;           // 完整文件路径
    QString fileName;           // 文件名
    CaptureType captureType;    // 拍照类型
    qint64 fileSize;            // 文件大小（字节）
    QDateTime detectedTime;     // 检测到的时间
    QDateTime modifiedTime;     // 文件修改时间
    bool isComplete;            // 文件是否传输完成
};
```

## 使用示例

### 启动监控
```cpp
hikCameraCController_->start(visionConfig);
// FTP 监控器会自动启动
```

### 接收图像通知
```cpp
connect(hikCameraCController_, 
        &HikCameraCController::imageReceived,
        [](CaptureType type, QString filePath, qint64 fileSize) {
    // 处理接收到的图像
    switch (type) {
        case CaptureType::SurfaceDefect:
            // 调用表面缺陷识别算法
            break;
        case CaptureType::WeldDefect:
            // 调用焊缝缺陷识别算法
            break;
        case CaptureType::NumberRecognition:
            // 调用 OCR 识别算法
            break;
    }
});
```

## 日志输出示例

```
[INF] FTP monitor started, watching directory: D:\HikCameraFTP
[INF] New image detected: surface_20260514_140530.jpg type: SurfaceDefect waiting for upload to complete...
[INF] Image ready for processing: surface_20260514_140530.jpg path: D:\HikCameraFTP\surface_20260514_140530.jpg size: 245678 bytes type: SurfaceDefect
[INF] [HikCameraCController] Image received from FTP, type: SurfaceDefect file: D:\HikCameraFTP\surface_20260514_140530.jpg size: 245678 bytes
```

## 下一步工作

1. **参数文件读取**
   - 实现拍照前读取相机配置文件
   - 支持 JSON/XML/INI 格式

2. **算法接口集成**
   - 表面缺陷识别算法接口
   - 焊缝缺陷识别算法接口
   - OCR 编号识别算法接口

3. **结果处理**
   - 识别结果存储
   - 结果上报（PLC/HMI）
   - 不良品标记

4. **状态机集成**
   - 接入主流程状态机
   - PLC 联动控制

## 注意事项

1. 确保 FTP 目录存在：D:\HikCameraFTP
2. 确保程序有读取该目录的权限
3. 文件命名需包含类型关键字以便自动识别
4. 建议定期清理 FTP 目录，避免文件过多

## FTP 服务器配置

- 服务器：FileZilla Server
- IP：192.168.8.10
- 端口：21
- 用户名：hik
- 密码：wyx020205
- 上传目录：D:\HikCameraFTP

========================================
