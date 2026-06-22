cmake_minimum_required(VERSION 3.21)

project(LbPoseDetection LANGUAGES CXX)

find_package(OpenCV REQUIRED
    PATHS "${SCAN_TRACKING_OPENCV_LIB_DIR}"
    NO_DEFAULT_PATH
)

set(LB_LIBRARY_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/TR_Mark_3D_Recon.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/TR_Mark_Track.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/AppConfig.cpp"
)

set(LB_LIBRARY_HEADERS
    "${CMAKE_CURRENT_SOURCE_DIR}/TR_Mark_3D_Recon.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/TR_Mark_Track.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/AppConfig.h"
)

set(LB_DATA_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/template_for_scanner_ori.txt"
    "${CMAKE_CURRENT_SOURCE_DIR}/data/template-3D-ALL-Shift-Cut-Cut.txt"
    "${CMAKE_CURRENT_SOURCE_DIR}/track_config.ini"
)

add_library(lb_pose_detection STATIC ${LB_LIBRARY_SOURCES})
add_library(LbPoseDetection::lb_pose_detection ALIAS lb_pose_detection)

target_sources(lb_pose_detection
    PRIVATE
        ${LB_LIBRARY_HEADERS}
        ${LB_DATA_FILES}
)

set_source_files_properties(${LB_DATA_FILES} PROPERTIES HEADER_FILE_ONLY TRUE)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "third_party/LB" FILES
    ${LB_LIBRARY_SOURCES}
    ${LB_LIBRARY_HEADERS}
    ${LB_DATA_FILES}
)

target_include_directories(lb_pose_detection
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}"
        "${SCAN_TRACKING_OPENCV_INCLUDE_DIR}"
)

target_link_libraries(lb_pose_detection
    PUBLIC
        ${OpenCV_LIBS}
)

target_compile_features(lb_pose_detection PUBLIC cxx_std_17)

if(MSVC)
    # Legacy vendor headers use out-of-class qualified member declarations.
    target_compile_options(lb_pose_detection PRIVATE /utf-8 /bigobj /permissive)
endif()

set_target_properties(lb_pose_detection PROPERTIES
    FOLDER "third_party"
    AUTOMOC OFF
    AUTOUIC OFF
    AUTORCC OFF
)
