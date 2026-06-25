#pragma once

/**
 * @file mech_eye_nano_smoke_test.h
 * @brief Mech-Eye Nano 烟雾测试入口
 *
 * 独立于 MechEyeService 的同步验证程序，直接在主线程调用 SDK：
 * 发现 → 连接首台相机 → capture2D → 保存 PNG → 断开。
 * 用于现场快速确认网络、SDK 安装与相机固件是否正常。
 */

namespace scan_tracking {
namespace mech_eye {

class MechEyeNanoSmokeTest {
public:
    /** @brief 执行完整烟雾测试，结果输出至 stdout */
    static void run();
};

}  // namespace mech_eye
}  // namespace scan_tracking