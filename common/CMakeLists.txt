find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Core REQUIRED)

add_library(scan_tracking_common STATIC
    src/application_info.cpp
    src/logger.cpp
    src/config_manager.cpp
)

target_include_directories(scan_tracking_common
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(scan_tracking_common
    PRIVATE
        Qt${QT_VERSION_MAJOR}::Core
)

target_compile_features(scan_tracking_common PUBLIC cxx_std_17)

target_compile_definitions(scan_tracking_common
    PUBLIC
        SCAN_TRACKING_VERSION="${PROJECT_VERSION}"
)

set_target_properties(scan_tracking_common PROPERTIES
    FOLDER "libraries"
)
