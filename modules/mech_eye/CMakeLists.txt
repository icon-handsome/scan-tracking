scan_tracking_require_mech_eye_sdk()

add_library(scan_tracking_mech_eye STATIC
    src/mech_eye_service.cpp
    src/mech_eye_worker.cpp
    src/mech_eye_nano_smoke_test.cpp
    include/scan_tracking/mech_eye/mech_eye_types.h
    include/scan_tracking/mech_eye/mech_eye_service.h
    include/scan_tracking/mech_eye/mech_eye_worker.h
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
)

target_compile_features(scan_tracking_mech_eye PUBLIC cxx_std_17)
