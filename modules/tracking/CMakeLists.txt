add_library(scan_tracking_tracking STATIC
    src/tracking_service.cpp
    src/lb_pose_check.cpp
)

target_include_directories(scan_tracking_tracking
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(scan_tracking_tracking PUBLIC cxx_std_17)

target_link_libraries(scan_tracking_tracking
    PUBLIC
        Qt5::Core
        scan_tracking_common
        scan_tracking_mech_eye
    PRIVATE
        LbPoseDetection::lb_pose_detection
        scan_tracking_lanyou_test_adapter
)

if(SCAN_TRACKING_ENABLE_LANYOU_DETECTION)
    target_link_libraries(scan_tracking_tracking
        PRIVATE
            scan_tracking_lanyou_test_adapter
    )

    add_executable(scan_tracking_tracking_smoke_test
        tests/tracking_service_smoke_test.cpp
    )

    target_link_libraries(scan_tracking_tracking_smoke_test
        PRIVATE
            Qt5::Core
            scan_tracking_tracking
    )

    set_target_properties(scan_tracking_tracking_smoke_test PROPERTIES
        FOLDER "tests"
    )
endif()

set_target_properties(scan_tracking_tracking PROPERTIES
    FOLDER "modules"
)
