add_library(scan_tracking_vision STATIC
    src/hik_camera_service.cpp
    src/hik_camera_c_controller.cpp
    src/lb_pose_detection_adapter.cpp
    src/vision_pipeline_service.cpp
    include/scan_tracking/vision/vision_types.h
    include/scan_tracking/vision/hik_camera_service.h
    include/scan_tracking/vision/hik_camera_c_controller.h
    include/scan_tracking/vision/lb_pose_detection_adapter.h
    include/scan_tracking/vision/vision_pipeline_service.h
)

set_target_properties(scan_tracking_vision PROPERTIES
    AUTOMOC ON
    FOLDER "modules"
)

target_include_directories(scan_tracking_vision
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(scan_tracking_vision
    PUBLIC
        Qt5::Core
        scan_tracking_common
        scan_tracking_mech_eye
    PRIVATE
        HikMvsSdk::MvCameraControl
        LbPoseDetection::lb_pose_detection
)

target_compile_features(scan_tracking_vision PUBLIC cxx_std_17)

if(SCAN_TRACKING_ENABLE_LANYOU_DETECTION)
    add_library(scan_tracking_lanyou_test_adapter STATIC
        src/lanyou_detection_adapter.cpp
        src/lanyou_first_station_adapter.cpp
        include/scan_tracking/vision/lanyou_detection_adapter.h
        include/scan_tracking/vision/lanyou_issue_codes.h
        include/scan_tracking/vision/lanyou_first_station_adapter.h
    )

    target_include_directories(scan_tracking_lanyou_test_adapter
        PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

    target_link_libraries(scan_tracking_lanyou_test_adapter
        PUBLIC
            Qt5::Core
            scan_tracking_mech_eye
            Lanyou::FirstDetection
    )

    target_compile_features(scan_tracking_lanyou_test_adapter PUBLIC cxx_std_17)

    set_target_properties(scan_tracking_lanyou_test_adapter PROPERTIES
        FOLDER "tests"
    )

    add_executable(scan_tracking_lanyou_smoke_test
        tests/lanyou_detection_adapter_smoke_test.cpp
    )

    target_link_libraries(scan_tracking_lanyou_smoke_test
        PRIVATE
            Qt5::Core
            scan_tracking_lanyou_test_adapter
    )

    set_target_properties(scan_tracking_lanyou_smoke_test PROPERTIES
        FOLDER "tests"
    )

    add_executable(scan_tracking_lanyou_first_station_adapter_smoke_test
        tests/lanyou_first_station_adapter_smoke_test.cpp
    )

    target_link_libraries(scan_tracking_lanyou_first_station_adapter_smoke_test
        PRIVATE
            Qt5::Core
            scan_tracking_lanyou_test_adapter
    )

    set_target_properties(scan_tracking_lanyou_first_station_adapter_smoke_test PROPERTIES
        FOLDER "tests"
    )

    add_executable(scan_tracking_lanyou_first_station_ply_runner
        tests/lanyou_first_station_ply_runner.cpp
    )

    target_link_libraries(scan_tracking_lanyou_first_station_ply_runner
        PRIVATE
            Lanyou::FirstDetection
    )

    if(MSVC)
        target_link_options(scan_tracking_lanyou_first_station_ply_runner PRIVATE /STACK:268435456)
    endif()

    set_target_properties(scan_tracking_lanyou_first_station_ply_runner PROPERTIES
        FOLDER "tests"
    )
endif()

