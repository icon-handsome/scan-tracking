/**
 * @file mech_eye_nano_smoke_test.cpp
 * @brief Mech-Eye Nano 烟雾测试：同步 SDK 调用，不依赖 MechEyeService 线程模型
 *
 * 与生产路径的区别：
 * - 直接在调用线程阻塞执行 discover/connect/capture（无 QThread）
 * - 仅验证 2D 采集与 PNG 落盘，不涉及 3D 点云与 StateMachine
 * - 输出目录：<cwd>/output/captures/
 *
 * 典型用途：SDK 安装验证、相机网络连通性、固件版本确认。
 */
#include "scan_tracking/mech_eye/mech_eye_nano_smoke_test.h"

#include <exception>
#include <vector>

#include <QtCore/QTextStream>
#include <QtCore/QString>
#include <QtCore/QDir>
#include <QtCore/QDateTime>
#include <QtGui/QImage>

#include "area_scan_3d_camera/Camera.h"
#include "area_scan_3d_camera/Frame2D.h"

namespace scan_tracking {
namespace mech_eye {

void MechEyeNanoSmokeTest::run()
{
    QTextStream stream(stdout);
    stream << "\n--- [ Mech-Eye Nano Smoke Test Started ] ---\n";

    try {
        // ----------------------------------------------------------------
        // 步骤 1：局域网发现所有 Mech-Eye 相机并打印列表
        // ----------------------------------------------------------------
        stream << "1. Discovering cameras...\n";
        std::vector<mmind::eye::CameraInfo> camera_list = mmind::eye::Camera::discoverCameras();
        if (camera_list.empty()) {
            stream << "   [ERROR] No cameras found.\n--- [ Smoke Test Finished ] ---\n\n";
            stream.flush();
            return;
        }

        stream << "   Found " << static_cast<qulonglong>(camera_list.size()) << " camera(s).\n";
        for (size_t i = 0; i < camera_list.size(); ++i) {
            stream << "   [" << i << "] Model: " << QString::fromStdString(camera_list[i].model)
                   << ", SN: " << QString::fromStdString(camera_list[i].serialNumber)
                   << ", IP: " << QString::fromStdString(camera_list[i].ipAddress) << "\n";
        }

        // ----------------------------------------------------------------
        // 步骤 2：连接列表中第一台相机（不做 cameraKey 筛选）
        // ----------------------------------------------------------------
        stream << "\n2. Connecting to the first camera ("
               << QString::fromStdString(camera_list[0].serialNumber) << ")...\n";
        mmind::eye::Camera camera;
        auto status = camera.connect(camera_list[0]);
        if (!status.isOK()) {
            stream << "   [ERROR] Failed to connect: "
                   << QString::fromStdString(status.errorDescription) << "\n";
            stream << "--- [ Smoke Test Finished ] ---\n\n";
            stream.flush();
            return;
        }
        stream << "   [SUCCESS] Connected.\n";

        // ----------------------------------------------------------------
        // 步骤 3：触发 2D 采集并保存至 output/captures/
        // ----------------------------------------------------------------
        stream << "\n3. Triggering 2D capture...\n";
        mmind::eye::Frame2D frame_2d;
        status = camera.capture2D(frame_2d);
        if (!status.isOK()) {
            stream << "   [ERROR] Capture 2D failed: "
                   << QString::fromStdString(status.errorDescription) << "\n";
        } else {
            stream << "   [SUCCESS] Captured 2D image!\n";

            const QString out_dir_path = QDir::currentPath() + QStringLiteral("/output/captures");
            QDir out_dir;
            if (!out_dir.mkpath(out_dir_path)) {
                stream << "   [ERROR] Failed to create directory: " << out_dir_path << "\n";
            } else {
                // 文件名带毫秒时间戳，避免连续运行覆盖
                const QString file_name =
                    QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss_zzz"))
                    + QStringLiteral("_2DImage.png");
                const QString save_path = out_dir_path + QStringLiteral("/") + file_name;

                QImage img;
                // 根据相机 2D 传感器类型选择灰度或 RGB 包装方式
                if (frame_2d.colorType() == mmind::eye::ColorTypeOf2DCamera::Monochrome) {
                    mmind::eye::GrayScale2DImage grayImage = frame_2d.getGrayScaleImage();
                    uint8_t* data = reinterpret_cast<uint8_t*>(grayImage.data());
                    const int width = static_cast<int>(grayImage.width());
                    const int height = static_cast<int>(grayImage.height());
                    if (data && width > 0 && height > 0) {
                        // bytesPerLine = width（每像素 1 字节）
                        img = QImage(data, width, height, width, QImage::Format_Grayscale8);
                    }
                } else if (frame_2d.colorType() == mmind::eye::ColorTypeOf2DCamera::Color) {
                    mmind::eye::Color2DImage colorImage = frame_2d.getColorImage();
                    uint8_t* data = reinterpret_cast<uint8_t*>(colorImage.data());
                    const int width = static_cast<int>(colorImage.width());
                    const int height = static_cast<int>(colorImage.height());
                    if (data && width > 0 && height > 0) {
                        // RGB888：bytesPerLine = width * 3
                        img = QImage(data, width, height, width * 3, QImage::Format_RGB888);
                    }
                }

                if (!img.isNull()) {
                    if (img.save(save_path)) {
                        stream << "   [SUCCESS] Saved 2D image to: "
                               << QDir::toNativeSeparators(save_path) << "\n";
                    } else {
                        stream << "   [ERROR] Failed to save 2D image to: "
                               << QDir::toNativeSeparators(save_path) << "\n";
                    }
                } else {
                    stream << "   [ERROR] Failed to convert 2D image data to QImage.\n";
                }
            }
        }

        // ----------------------------------------------------------------
        // 步骤 4：断开连接并释放 SDK 资源
        // ----------------------------------------------------------------
        stream << "\n4. Disconnecting...\n";
        camera.disconnect();
        stream << "   [SUCCESS] Disconnected.\n";

    } catch (const std::exception& e) {
        stream << "   [EXCEPTION] Standard exception: " << QString::fromLocal8Bit(e.what()) << "\n";
    } catch (...) {
        stream << "   [EXCEPTION] Unknown exception occurred.\n";
    }

    stream << "--- [ Mech-Eye Nano Smoke Test Finished ] ---\n\n";
    stream.flush();
}

}  // namespace mech_eye
}  // namespace scan_tracking
