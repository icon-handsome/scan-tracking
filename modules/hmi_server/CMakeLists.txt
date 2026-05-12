set(scan_tracking_hmi_server_sources
    src/hmi_protocol.cpp
    src/hmi_session.cpp
    src/hmi_tcp_server.cpp
    include/scan_tracking/hmi_server/hmi_protocol.h
    include/scan_tracking/hmi_server/hmi_session.h
    include/scan_tracking/hmi_server/hmi_tcp_server.h
)

add_library(scan_tracking_hmi_server STATIC ${scan_tracking_hmi_server_sources})

target_include_directories(scan_tracking_hmi_server
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(scan_tracking_hmi_server
    PUBLIC
        Qt5::Core
        Qt5::Network
    PRIVATE
        scan_tracking_common
        scan_tracking_flow_control
        scan_tracking_modbus
        scan_tracking_mech_eye
        scan_tracking_vision
        scan_tracking_tracking
)

target_compile_features(scan_tracking_hmi_server PUBLIC cxx_std_17)

set_target_properties(scan_tracking_hmi_server PROPERTIES
    FOLDER "modules"
)
