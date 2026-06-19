cmake_minimum_required(VERSION 3.21)

project(HeadCylinderOpeningMeasure LANGUAGES CXX)

find_package(PCL 1.12 REQUIRED COMPONENTS
    common
    io
    filters
    registration
    sample_consensus
)

set(HOLE_SOURCES
    src/Config.cpp
    src/MeasurePipeline.cpp
    src/Geometry.cpp
)

set(HOLE_HEADERS
    include/HeadMeasure/Config.h
    include/HeadMeasure/Geometry.h
    include/HeadMeasure/MeasurePipeline.h
    include/HeadMeasure/Types.h
)

set(HOLE_CONFIG_FILES
    config/default.json
    config/path2.json
    config/sample_config.json
)

set(HOLE_TEMPLATE_FILES
    template/Zhu_Mian_Kai_Kong_Template_cut_trans.pcd
)

add_library(head_measure STATIC
    ${HOLE_SOURCES}
)

add_library(Hole::Measurement ALIAS head_measure)

target_sources(head_measure
    PRIVATE
        ${HOLE_HEADERS}
        ${HOLE_CONFIG_FILES}
        ${HOLE_TEMPLATE_FILES}
)

set_source_files_properties(${HOLE_CONFIG_FILES} PROPERTIES HEADER_FILE_ONLY TRUE)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "third_party/Hole" FILES
    ${HOLE_SOURCES}
    ${HOLE_HEADERS}
    ${HOLE_CONFIG_FILES}
    ${HOLE_TEMPLATE_FILES}
)

target_include_directories(head_measure
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        ${PCL_INCLUDE_DIRS}
)

target_link_libraries(head_measure
    PUBLIC
        ${PCL_LIBRARIES}
)

target_compile_features(head_measure PUBLIC cxx_std_17)

target_compile_definitions(head_measure
    PUBLIC
        ${PCL_DEFINITIONS}
)

if(MSVC)
    target_compile_options(head_measure PRIVATE /utf-8 /W4)
    target_compile_definitions(head_measure PRIVATE _CRT_SECURE_NO_WARNINGS _SCL_SECURE_NO_WARNINGS)
else()
    target_compile_options(head_measure PRIVATE -Wall -Wextra -pedantic)
endif()

set_target_properties(head_measure PROPERTIES
    FOLDER "third_party"
)

option(SCAN_TRACKING_BUILD_HOLE_DEMO "Build standalone head_measure demo executable" OFF)

if(SCAN_TRACKING_BUILD_HOLE_DEMO)
    add_executable(head_measure_demo src/main.cpp)
    target_link_libraries(head_measure_demo PRIVATE head_measure)
    set_target_properties(head_measure_demo PROPERTIES FOLDER "third_party")
endif()
