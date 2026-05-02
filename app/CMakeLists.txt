set(scan_tracking_app_sources
    src/main.cpp
    src/console_runtime.cpp
    include/scan_tracking/app/console_runtime.h
)

add_executable(scan_tracking ${scan_tracking_app_sources})

target_include_directories(scan_tracking
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(scan_tracking
    PRIVATE
        Qt5::Core
        scan_tracking_common
        Qt5::Gui
        scan_tracking_mech_eye
        scan_tracking_vision
        scan_tracking_tracking
        scan_tracking_modbus
        scan_tracking_flow_control
)

target_compile_features(scan_tracking PRIVATE cxx_std_17)

set_target_properties(scan_tracking PROPERTIES
    OUTPUT_NAME "scan-tracking"
    FOLDER "applications"
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
scan_tracking_deploy_hik_mvs_runtime(scan_tracking)
scan_tracking_deploy_pcl_runtime(scan_tracking)
scan_tracking_deploy_opencv_runtime(scan_tracking)
scan_tracking_deploy_vtk_runtime(scan_tracking)
scan_tracking_deploy_openni2_runtime(scan_tracking)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(scan_tracking)
endif()
