set(scan_tracking_app_sources
    src/main.cpp
    src/console_runtime.cpp
    include/scan_tracking/app/console_runtime.h
)

add_executable(scan_tracking ${scan_tracking_app_sources})

target_include_directories(scan_tracking
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/common/include
        ${CMAKE_SOURCE_DIR}/modules/mech_eye/include
        ${CMAKE_SOURCE_DIR}/modules/orbbec_gemini/include
        ${CMAKE_SOURCE_DIR}/modules/livox_mid360/include
        ${CMAKE_SOURCE_DIR}/modules/tfmini_plus/include
        ${CMAKE_SOURCE_DIR}/modules/vision/include
        ${CMAKE_SOURCE_DIR}/modules/tracking/include
        ${CMAKE_SOURCE_DIR}/modules/modbus/include
        ${CMAKE_SOURCE_DIR}/modules/flow_control/include
        ${CMAKE_SOURCE_DIR}/modules/hmi_server/include
)

target_link_libraries(scan_tracking
    PRIVATE
        Qt5::Core
        Qt5::Network
        scan_tracking_common
        Qt5::Gui
        scan_tracking_mech_eye
        scan_tracking_orbbec_gemini
        scan_tracking_livox_mid360
        scan_tracking_tfmini_plus
        scan_tracking_vision
        scan_tracking_tracking
        scan_tracking_modbus
        scan_tracking_flow_control
        scan_tracking_hmi_server
)

target_compile_features(scan_tracking PRIVATE cxx_std_17)

set_target_properties(scan_tracking PROPERTIES
    OUTPUT_NAME "scan-tracking"
    FOLDER "applications"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}"
)

if(EXISTS "${CMAKE_PREFIX_PATH}/bin/windeployqt.exe")
    add_custom_command(TARGET scan_tracking POST_BUILD
        COMMAND "${CMAKE_PREFIX_PATH}/bin/windeployqt.exe"
            --dir "$<TARGET_FILE_DIR:scan_tracking>"
            --no-compiler-runtime
            --no-translations
            "$<TARGET_FILE:scan_tracking>"
        COMMENT "Deploying Qt runtime"
    )
endif()

scan_tracking_deploy_mech_eye_runtime(scan_tracking)
scan_tracking_deploy_orbbec_runtime(scan_tracking)
scan_tracking_deploy_livox_runtime(scan_tracking)
scan_tracking_deploy_hik_mvs_runtime(scan_tracking)
scan_tracking_deploy_pcl_runtime(scan_tracking)
scan_tracking_deploy_opencv_runtime(scan_tracking)
scan_tracking_deploy_vtk_runtime(scan_tracking)
scan_tracking_deploy_openni2_runtime(scan_tracking)
if(SCAN_TRACKING_ENABLE_BEVEL_MEASUREMENT)
    scan_tracking_deploy_bevel_runtime(scan_tracking)
endif()
if(SCAN_TRACKING_ENABLE_HOLE_MEASUREMENT)
    scan_tracking_deploy_hole_runtime(scan_tracking)
endif()
if(SCAN_TRACKING_ENABLE_THICKNESS_MEASUREMENT)
    scan_tracking_deploy_thickness_runtime(scan_tracking)
endif()
if(SCAN_TRACKING_ENABLE_INTERNAL_SURFACE_MEASUREMENT)
    scan_tracking_deploy_internal_surface_runtime(scan_tracking)
endif()

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(scan_tracking)
endif()
