# Mech-Eye 3D 相机模块 (scan_tracking_mech_eye)
#
# 组件：
#   MechEyeService   — 主线程门面，管理 worker 线程与采集请求
#   MechEyeWorker    — SDK 发现/连接/采图（独立 QThread）
#   point_cloud_io   — 分段 PLY/PNG 落盘与加载
#   point_cloud_processor — PCL 后处理与 T0'×T 坐标变换
#   MechEyeNanoSmokeTest — 独立烟雾测试（不依赖 Service 线程）
#
scan_tracking_require_mech_eye_sdk()

find_package(PCL 1.12 REQUIRED COMPONENTS common filters surface)

add_library(scan_tracking_mech_eye STATIC
    src/mech_eye_service.cpp
    src/mech_eye_worker.cpp
    src/point_cloud_io.cpp
    src/point_cloud_processor.cpp
    src/mech_eye_nano_smoke_test.cpp
    include/scan_tracking/mech_eye/mech_eye_types.h
    include/scan_tracking/mech_eye/mech_eye_service.h
    include/scan_tracking/mech_eye/mech_eye_worker.h
    include/scan_tracking/mech_eye/point_cloud_io.h
    include/scan_tracking/mech_eye/point_cloud_processor.h
)

# 启用 AUTOMOC：MechEyeService 和 MechEyeWorker 都使用了 Q_OBJECT 宏，
# 需要 moc 处理生成元对象代码。仅在 target 级别开启，不污染全局设置。
set_target_properties(scan_tracking_mech_eye PROPERTIES
    AUTOMOC ON
    FOLDER "modules"
)

target_include_directories(scan_tracking_mech_eye
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(scan_tracking_mech_eye
    PUBLIC
        Qt5::Core
        Qt5::Gui
        scan_tracking_common
    PRIVATE
        MechEyeSdk::MechEyeApi
        ${PCL_LIBRARIES}
)

target_include_directories(scan_tracking_mech_eye
    PRIVATE
        ${PCL_INCLUDE_DIRS}
)

target_compile_definitions(scan_tracking_mech_eye
    PRIVATE
        ${PCL_DEFINITIONS}
)

target_compile_features(scan_tracking_mech_eye PUBLIC cxx_std_17)

if(BUILD_TESTING)
    find_package(Qt5 REQUIRED COMPONENTS Test)

    add_executable(scan_tracking_point_cloud_io_test
        tests/point_cloud_io_test.cpp
    )

    target_link_libraries(scan_tracking_point_cloud_io_test
        PRIVATE
            Qt5::Core
            Qt5::Test
            scan_tracking_mech_eye
            scan_tracking_common
    )

    add_test(NAME scan_tracking_point_cloud_io_test
        COMMAND scan_tracking_point_cloud_io_test
    )

    set_target_properties(scan_tracking_point_cloud_io_test PROPERTIES
        FOLDER "tests"
    )

    add_executable(scan_tracking_point_cloud_processor_test
        tests/point_cloud_processor_test.cpp
    )

    target_link_libraries(scan_tracking_point_cloud_processor_test
        PRIVATE
            Qt5::Core
            Qt5::Test
            scan_tracking_mech_eye
            scan_tracking_common
    )

    add_test(NAME scan_tracking_point_cloud_processor_test
        COMMAND scan_tracking_point_cloud_processor_test
    )

    set_target_properties(scan_tracking_point_cloud_processor_test PROPERTIES
        FOLDER "tests"
    )
endif()
