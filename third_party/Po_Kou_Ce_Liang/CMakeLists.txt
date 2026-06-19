cmake_minimum_required(VERSION 3.21)

project(PoKouCeLiang LANGUAGES CXX)

# V1.2 起算法仅依赖 PCL + Eigen，不再使用 OpenCV（投影/SVM 已移除）。

find_package(PCL 1.12 REQUIRED COMPONENTS
    common
    io
    filters
    kdtree
    search
    registration
    sample_consensus
)

add_library(po_kou_ce_liang STATIC
    src/BevelMeasurement.cpp
)

add_library(Bevel::Measurement ALIAS po_kou_ce_liang)

target_include_directories(po_kou_ce_liang
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        ${PCL_INCLUDE_DIRS}
)

target_link_libraries(po_kou_ce_liang
    PUBLIC
        ${PCL_LIBRARIES}
)

target_compile_features(po_kou_ce_liang PUBLIC cxx_std_17)

target_compile_definitions(po_kou_ce_liang
    PUBLIC
        ${PCL_DEFINITIONS}
)

if(MSVC)
    target_compile_options(po_kou_ce_liang PRIVATE /utf-8)
endif()

set_target_properties(po_kou_ce_liang PROPERTIES
    FOLDER "third_party"
)
