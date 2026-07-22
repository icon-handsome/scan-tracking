set(CMAKE_AUTOMOC ON)
add_library(scan_tracking_flow_control STATIC
    include/scan_tracking/flow_control/checkpoint_store.h
    include/scan_tracking/flow_control/state_machine.h
    include/scan_tracking/flow_control/station_trigger_policy.h
    include/scan_tracking/flow_control/task_handler_context.h
    include/scan_tracking/flow_control/task_handler_registry.h
    include/scan_tracking/flow_control/handlers/itask_handler.h
    include/scan_tracking/flow_control/handlers/code_read_handler.h
    include/scan_tracking/flow_control/handlers/inspection_handler.h
    include/scan_tracking/flow_control/handlers/load_grasp_handler.h
    include/scan_tracking/flow_control/handlers/pose_check_handler.h
    include/scan_tracking/flow_control/handlers/result_reset_handler.h
    include/scan_tracking/flow_control/handlers/scan_segment_handler.h
    include/scan_tracking/flow_control/handlers/self_check_handler.h
    include/scan_tracking/flow_control/handlers/station_material_check_handler.h
    include/scan_tracking/flow_control/handlers/unload_calc_handler.h
    src/checkpoint_store.cpp
    src/station_trigger_policy.cpp
    src/state_machine.cpp
    src/handlers/code_read_handler.cpp
    src/handlers/inspection_handler.cpp
    src/handlers/load_grasp_handler.cpp
    src/handlers/pose_check_handler.cpp
    src/handlers/result_reset_handler.cpp
    src/handlers/scan_segment_handler.cpp
    src/handlers/self_check_handler.cpp
    src/handlers/station_material_check_handler.cpp
    src/handlers/task_handler_registry.cpp
    src/handlers/unload_calc_handler.cpp
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

if(SCAN_TRACKING_ENABLE_SCANNER_SELF_CHECK)
    target_link_libraries(scan_tracking_flow_control
        PRIVATE
            scan_tracking_self_check_adapter
    )
endif()
