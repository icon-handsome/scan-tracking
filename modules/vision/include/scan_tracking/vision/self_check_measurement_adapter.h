#pragma once

// 扫描仪编码标记点距离自检适配层（Scanner_Self_CheckV1.2）。
//
// 将 Mech-Eye 2D+3D 组合采集与 LB 位姿转换为 CodedMark_SelfCheck 输入，
// 供 Trig_SelfCheck 在 PLC 完成自检两点扫描后调用。

#include <QtCore/QString>

#include "scan_tracking/vision/vision_types.h"

namespace scan_tracking::vision::self_check {

struct SelfCheckResult {
    bool ok = false;
    int algorithmCode = 0;
    quint16 failWord0 = 0;
    quint16 failWord1 = 0;
    QString message;
};

/// 解析 self_check.ini 绝对路径（config.ini / 环境变量 / exe 旁 self_check/）
QString resolveSelfCheckConfigPath();

/// 对两帧组合采集结果执行扫描仪自检（阻塞、同步）
SelfCheckResult runScannerSelfCheck(
    const MultiCameraCaptureBundle& frame0,
    const MultiCameraCaptureBundle& frame1);

}  // namespace scan_tracking::vision::self_check
