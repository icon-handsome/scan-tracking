cmake_minimum_required(VERSION 3.21)

project(ScanTracking
    VERSION 0.1.0
    DESCRIPTION "Scanning and tracking QtCore console application"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    add_compile_options(/utf-8)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(ProjectOptions)
include(MechEyeSdk)
include(HikMvsSdk)
include(PclRuntime)

option(SCAN_TRACKING_ENABLE_LANYOU_DETECTION "Build Lanyou PCL-based detection algorithms" ON)
option(SCAN_TRACKING_ENABLE_LB_POSE_DETECTION "Build the legacy LB pose detection module" ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core Gui Network)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Gui Network)

if(SCAN_TRACKING_ENABLE_LB_POSE_DETECTION)
    add_subdirectory(third_party/LB)
endif()

if(SCAN_TRACKING_ENABLE_LANYOU_DETECTION)
    add_subdirectory(third_party/lanyou_first_detection)
endif()

add_subdirectory(common)
add_subdirectory(modules)
add_subdirectory(app)
