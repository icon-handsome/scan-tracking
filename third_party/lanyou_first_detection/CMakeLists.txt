cmake_minimum_required(VERSION 3.21)

project(LanyouFirstDetection VERSION 1.0 LANGUAGES CXX)

option(LANYOU_FIRST_DETECTION_BUILD_DEMO "Build the standalone PLY demo executable" OFF)
option(LANYOU_FIRST_DETECTION_BUILD_SHARED "Build lanyou_first_detection as a shared library" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    add_compile_options(/utf-8 /bigobj)
endif()

find_package(PCL 1.12 REQUIRED COMPONENTS
    common
    features
    filters
    io
    kdtree
    sample_consensus
    search
    segmentation
    surface
)

set(LANYOU_EIGEN_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/../eigen-3.4.0"
    CACHE PATH
    "Path to the bundled Eigen headers"
)

file(GLOB_RECURSE LANYOU_DETECTION_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/detection/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/log_manager/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/model/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/utils/*.cpp"
)

set(LANYOU_LIBRARY_TYPE STATIC)
if(LANYOU_FIRST_DETECTION_BUILD_SHARED)
    set(LANYOU_LIBRARY_TYPE SHARED)
endif()

add_library(lanyou_first_detection ${LANYOU_LIBRARY_TYPE}
    ${LANYOU_DETECTION_SOURCES}
)

add_library(Lanyou::FirstDetection ALIAS lanyou_first_detection)

target_include_directories(lanyou_first_detection
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        "${LANYOU_EIGEN_ROOT}"
        ${PCL_INCLUDE_DIRS}
)

target_link_libraries(lanyou_first_detection
    PUBLIC
        ${PCL_LIBRARIES}
)

target_compile_definitions(lanyou_first_detection
    PUBLIC
        ${PCL_DEFINITIONS}
)

target_compile_features(lanyou_first_detection PUBLIC cxx_std_17)

set_target_properties(lanyou_first_detection PROPERTIES
    FOLDER "third_party"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin"
)

if(LANYOU_FIRST_DETECTION_BUILD_DEMO)
    add_executable(lanyou_first_detection_demo
        "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp"
    )

    target_link_libraries(lanyou_first_detection_demo
        PRIVATE
            lanyou_first_detection
    )

    set_target_properties(lanyou_first_detection_demo PROPERTIES
        FOLDER "third_party"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin"
    )
endif()
