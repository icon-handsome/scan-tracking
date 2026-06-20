set(SCAN_TRACKING_VISION_SOURCES
    src/hik_mvs_sdk_runtime.cpp
    src/hik_cxp_mono_io.cpp
    src/hik_cxp_camera_service.cpp
    src/hik_mono_io.cpp
    src/hik_camera_service.cpp
    src/hik_camera_c_controller.cpp
    src/hik_smart_camera_tcp_server.cpp
    src/hik_smart_camera_ftp_monitor.cpp
    src/vision_pipeline_service.cpp
    include/scan_tracking/vision/vision_types.h
    include/scan_tracking/vision/hik_mvs_sdk_runtime.h
    include/scan_tracking/vision/hik_cxp_mono_io.h
    include/scan_tracking/vision/hik_cxp_camera_service.h
    include/scan_tracking/vision/hik_mono_io.h
    include/scan_tracking/vision/hik_camera_service.h
    include/scan_tracking/vision/hik_camera_c_controller.h
    include/scan_tracking/vision/hik_smart_camera_tcp_server.h
    include/scan_tracking/vision/hik_smart_camera_ftp_monitor.h
    include/scan_tracking/vision/vision_pipeline_service.h
)

set(SCAN_TRACKING_VISION_PRIVATE_LIBS
    HikMvsSdk::MvCameraControl
)

if(SCAN_TRACKING_ENABLE_LBN_POSE_DETECTION)
    list(APPEND SCAN_TRACKING_VISION_SOURCES
        src/lbn_pose_detection_adapter.cpp
        include/scan_tracking/vision/lbn_pose_detection_adapter.h
    )
    list(APPEND SCAN_TRACKING_VISION_PRIVATE_LIBS
        LbnPose::lbn_pose
    )
endif()

if(SCAN_TRACKING_ENABLE_LB_POSE_DETECTION)
    list(APPEND SCAN_TRACKING_VISION_SOURCES
        src/lb_pose_detection_adapter.cpp
        include/scan_tracking/vision/lb_pose_detection_adapter.h
    )
    list(APPEND SCAN_TRACKING_VISION_PRIVATE_LIBS
        LbPoseDetection::lb_pose_detection
    )
else()
    list(APPEND SCAN_TRACKING_VISION_SOURCES
        src/lb_pose_detection_adapter_stub.cpp
        include/scan_tracking/vision/lb_pose_detection_adapter.h
    )
endif()

add_library(scan_tracking_vision STATIC ${SCAN_TRACKING_VISION_SOURCES})

set_target_properties(scan_tracking_vision PROPERTIES
    AUTOMOC ON
    FOLDER "modules"
)

target_include_directories(scan_tracking_vision
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(SCAN_TRACKING_ENABLE_LBN_POSE_DETECTION)
    target_include_directories(scan_tracking_vision
        PRIVATE
            ${CMAKE_SOURCE_DIR}/third_party/LBN
    )
endif()

target_link_libraries(scan_tracking_vision
    PUBLIC
        Qt5::Core
        Qt5::Network
        scan_tracking_common
        scan_tracking_mech_eye
    PRIVATE
        ${SCAN_TRACKING_VISION_PRIVATE_LIBS}
)

target_compile_features(scan_tracking_vision PUBLIC cxx_std_17)

if(BUILD_TESTING)
    find_package(Qt5 REQUIRED COMPONENTS Test)

    add_executable(scan_tracking_hik_mono_io_test
        tests/hik_mono_io_test.cpp
    )

    target_link_libraries(scan_tracking_hik_mono_io_test
        PRIVATE
            Qt5::Core
            Qt5::Test
            scan_tracking_vision
            scan_tracking_common
    )

    add_test(NAME scan_tracking_hik_mono_io_test
        COMMAND scan_tracking_hik_mono_io_test
    )

    set_target_properties(scan_tracking_hik_mono_io_test PROPERTIES
        FOLDER "tests"
    )
endif()

add_executable(scan_tracking_cxp_smoke
    tests/cxp_smoke_runner.cpp
)

target_link_libraries(scan_tracking_cxp_smoke
    PRIVATE
        Qt5::Core
        scan_tracking_vision
        scan_tracking_common
)

set_target_properties(scan_tracking_cxp_smoke PROPERTIES
    AUTOMOC ON
    FOLDER "tests"
)

scan_tracking_deploy_hik_mvs_runtime(scan_tracking_cxp_smoke)
scan_tracking_deploy_mech_eye_runtime(scan_tracking_cxp_smoke)
scan_tracking_deploy_opencv_runtime(scan_tracking_cxp_smoke)

if(SCAN_TRACKING_ENABLE_BEVEL_MEASUREMENT)
    add_library(scan_tracking_bevel_adapter STATIC
        src/bevel_measurement_adapter.cpp
        include/scan_tracking/vision/bevel_measurement_adapter.h
    )

    target_include_directories(scan_tracking_bevel_adapter
        PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/include
        PRIVATE
            ${CMAKE_SOURCE_DIR}/third_party/Po_Kou_Ce_Liang/include
    )

    target_link_libraries(scan_tracking_bevel_adapter
        PUBLIC
            Qt5::Core
            scan_tracking_common
            scan_tracking_mech_eye
            Bevel::Measurement
    )

    target_compile_features(scan_tracking_bevel_adapter PUBLIC cxx_std_17)

    set_target_properties(scan_tracking_bevel_adapter PROPERTIES
        FOLDER "tests"
    )

    add_executable(scan_tracking_bevel_smoke_test
        tests/bevel_measurement_adapter_smoke_test.cpp
    )

    target_link_libraries(scan_tracking_bevel_smoke_test
        PRIVATE
            Qt5::Core
            scan_tracking_bevel_adapter
    )

    set_target_properties(scan_tracking_bevel_smoke_test PROPERTIES
        FOLDER "tests"
    )

    scan_tracking_deploy_bevel_runtime(scan_tracking_bevel_smoke_test)
    scan_tracking_deploy_pcl_runtime(scan_tracking_bevel_smoke_test)
    scan_tracking_deploy_opencv_runtime(scan_tracking_bevel_smoke_test)
endif()

if(SCAN_TRACKING_ENABLE_THICKNESS_MEASUREMENT)
    add_library(scan_tracking_thickness_adapter STATIC
        src/thickness_measurement_adapter.cpp
        include/scan_tracking/vision/thickness_measurement_adapter.h
    )

    target_include_directories(scan_tracking_thickness_adapter
        PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/include
        PRIVATE
            ${CMAKE_SOURCE_DIR}/third_party/Thicknessmeasurement/src
    )

    target_link_libraries(scan_tracking_thickness_adapter
        PUBLIC
            Qt5::Core
            scan_tracking_common
            scan_tracking_mech_eye
            Thickness::Measurement
    )

    target_compile_features(scan_tracking_thickness_adapter PUBLIC cxx_std_17)

    set_target_properties(scan_tracking_thickness_adapter PROPERTIES
        FOLDER "modules"
    )
endif()

if(SCAN_TRACKING_ENABLE_HOLE_MEASUREMENT)
    add_library(scan_tracking_hole_adapter STATIC
        src/hole_measurement_adapter.cpp
        include/scan_tracking/vision/hole_measurement_adapter.h
    )

    target_include_directories(scan_tracking_hole_adapter
        PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/include
        PRIVATE
            ${CMAKE_SOURCE_DIR}/third_party/Hole/include
    )

    target_link_libraries(scan_tracking_hole_adapter
        PUBLIC
            Qt5::Core
            scan_tracking_common
            scan_tracking_mech_eye
            Hole::Measurement
    )

    target_compile_features(scan_tracking_hole_adapter PUBLIC cxx_std_17)

    set_target_properties(scan_tracking_hole_adapter PROPERTIES
        FOLDER "modules"
    )
endif()

if(SCAN_TRACKING_ENABLE_INTERNAL_SURFACE_MEASUREMENT)
    add_library(scan_tracking_internal_surface_adapter STATIC
        src/internal_surface_measurement_adapter.cpp
        include/scan_tracking/vision/internal_surface_measurement_adapter.h
    )

    target_include_directories(scan_tracking_internal_surface_adapter
        PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/include
        PRIVATE
            ${CMAKE_SOURCE_DIR}/third_party/InternalSurfaceMeasurement/include
    )

    target_link_libraries(scan_tracking_internal_surface_adapter
        PUBLIC
            Qt5::Core
            scan_tracking_common
            scan_tracking_mech_eye
            InternalSurface::Measurement
    )

    target_compile_features(scan_tracking_internal_surface_adapter PUBLIC cxx_std_17)

    set_target_properties(scan_tracking_internal_surface_adapter PROPERTIES
        FOLDER "modules"
    )
endif()

if(SCAN_TRACKING_ENABLE_SCANNER_SELF_CHECK)
    add_library(scan_tracking_self_check_adapter STATIC
        src/self_check_measurement_adapter.cpp
        include/scan_tracking/vision/self_check_measurement_adapter.h
    )

    target_include_directories(scan_tracking_self_check_adapter
        PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/include
        PRIVATE
            ${CMAKE_SOURCE_DIR}/third_party/Scanner_Self_Check
            ${SCAN_TRACKING_OPENCV_INCLUDE_DIR}
    )

    target_link_libraries(scan_tracking_self_check_adapter
        PUBLIC
            Qt5::Core
            scan_tracking_common
            scan_tracking_mech_eye
            ScannerSelfCheck::core
    )

    target_compile_features(scan_tracking_self_check_adapter PUBLIC cxx_std_17)

    set_target_properties(scan_tracking_self_check_adapter PROPERTIES
        FOLDER "modules"
    )
endif()

if(SCAN_TRACKING_ENABLE_LBN_POSE_DETECTION)
    add_executable(scan_tracking_lbn_offline_runner
        tests/lbn_offline_runner.cpp
    )

    target_include_directories(scan_tracking_lbn_offline_runner
        PRIVATE
            ${CMAKE_SOURCE_DIR}/third_party/LBN
            ${SCAN_TRACKING_OPENCV_INCLUDE_DIR}
    )

    target_link_libraries(scan_tracking_lbn_offline_runner
        PRIVATE
            Qt5::Core
            scan_tracking_vision
            LbnPose::lbn_pose
    )

    if(MSVC)
        target_link_options(scan_tracking_lbn_offline_runner PRIVATE /STACK:268435456)
    endif()

    set_target_properties(scan_tracking_lbn_offline_runner PROPERTIES
        FOLDER "tests"
    )
endif()
