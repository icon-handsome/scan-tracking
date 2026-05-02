set(CMAKE_AUTOMOC ON)
add_library(scan_tracking_flow_control STATIC
    include/scan_tracking/flow_control/state_machine.h
    src/state_machine.cpp
)

target_include_directories(scan_tracking_flow_control
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(scan_tracking_flow_control
    PUBLIC
        Qt5::Core
        scan_tracking_common
        scan_tracking_modbus
        scan_tracking_mech_eye
        scan_tracking_vision
        scan_tracking_tracking
)
