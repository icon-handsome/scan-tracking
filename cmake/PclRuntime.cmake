include_guard(GLOBAL)

set(
    SCAN_TRACKING_PCL_RUNTIME_DIR
    "C:/Program Files/PCL 1.12.0/bin"
    CACHE PATH
    "Path to the local PCL runtime DLL directory"
)

if(NOT DEFINED SCAN_TRACKING_OPENCV_RUNTIME_DIR OR "${SCAN_TRACKING_OPENCV_RUNTIME_DIR}" STREQUAL "")
    include(OpenCvBundle)
endif()

set(
    SCAN_TRACKING_VTK_RUNTIME_DIR
    "C:/Program Files/PCL 1.12.0/3rdParty/VTK/bin"
    CACHE PATH
    "Path to the local VTK runtime DLL directory"
)

set(
    SCAN_TRACKING_OPENNI2_RUNTIME_DIR
    "C:/Program Files/OpenNI2/Redist"
    CACHE PATH
    "Path to the local OpenNI2 runtime DLL directory"
)

function(scan_tracking_deploy_pcl_runtime target_name)
    if(NOT EXISTS "${SCAN_TRACKING_PCL_RUNTIME_DIR}")
        message(WARNING "PCL runtime directory not found: ${SCAN_TRACKING_PCL_RUNTIME_DIR}")
        return()
    endif()

    add_custom_command(TARGET ${target_name} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:${target_name}>"
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${SCAN_TRACKING_PCL_RUNTIME_DIR}"
            "$<TARGET_FILE_DIR:${target_name}>"
        COMMENT "Deploying PCL runtime"
    )
endfunction()

function(scan_tracking_deploy_opencv_runtime target_name)
    if(NOT EXISTS "${SCAN_TRACKING_OPENCV_RUNTIME_DIR}")
        message(WARNING "OpenCV runtime directory not found: ${SCAN_TRACKING_OPENCV_RUNTIME_DIR}")
        return()
    endif()

    add_custom_command(TARGET ${target_name} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:${target_name}>"
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${SCAN_TRACKING_OPENCV_RUNTIME_DIR}"
            "$<TARGET_FILE_DIR:${target_name}>"
        COMMENT "Deploying OpenCV runtime"
    )
endfunction()

function(scan_tracking_deploy_vtk_runtime target_name)
    if(NOT EXISTS "${SCAN_TRACKING_VTK_RUNTIME_DIR}")
        message(WARNING "VTK runtime directory not found: ${SCAN_TRACKING_VTK_RUNTIME_DIR}")
        return()
    endif()

    add_custom_command(TARGET ${target_name} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:${target_name}>"
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${SCAN_TRACKING_VTK_RUNTIME_DIR}"
            "$<TARGET_FILE_DIR:${target_name}>"
        COMMENT "Deploying VTK runtime"
    )
endfunction()

function(scan_tracking_deploy_hole_runtime target_name)
    set(_hole_source_root "${CMAKE_SOURCE_DIR}/third_party/Hole")
    if(NOT EXISTS "${_hole_source_root}/config/default.json")
        message(WARNING "Hole runtime source not found: ${_hole_source_root}")
        return()
    endif()

    add_custom_command(TARGET ${target_name} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:${target_name}>/hole/config"
        COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:${target_name}>/hole/template"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${_hole_source_root}/config/default.json"
            "$<TARGET_FILE_DIR:${target_name}>/hole/config/default.json"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${_hole_source_root}/config/path2.json"
            "$<TARGET_FILE_DIR:${target_name}>/hole/config/path2.json"
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${_hole_source_root}/template"
            "$<TARGET_FILE_DIR:${target_name}>/hole/template"
        COMMENT "Deploying Hole measurement runtime config"
    )
endfunction()

function(scan_tracking_deploy_thickness_runtime target_name)
    set(_thickness_source_root "${CMAKE_SOURCE_DIR}/third_party/Thicknessmeasurement")
    if(NOT EXISTS "${_thickness_source_root}/config/thickness_config.json")
        message(WARNING "Thickness runtime source not found: ${_thickness_source_root}")
        return()
    endif()

    add_custom_command(TARGET ${target_name} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:${target_name}>/thickness/config"
        COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:${target_name}>/thickness/input"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${_thickness_source_root}/config/thickness_config.json"
            "$<TARGET_FILE_DIR:${target_name}>/thickness/config/thickness_config.json"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${_thickness_source_root}/config/path3.json"
            "$<TARGET_FILE_DIR:${target_name}>/thickness/config/path3.json"
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${_thickness_source_root}/input"
            "$<TARGET_FILE_DIR:${target_name}>/thickness/input"
        COMMENT "Deploying thickness measurement runtime assets"
    )
endfunction()

function(scan_tracking_deploy_internal_surface_runtime target_name)
    set(_internal_surface_source_root "${CMAKE_SOURCE_DIR}/third_party/InternalSurfaceMeasurement")
    if(NOT EXISTS "${_internal_surface_source_root}/config/algorithm_config.json")
        message(WARNING "Internal surface runtime source not found: ${_internal_surface_source_root}")
        return()
    endif()

    add_custom_command(TARGET ${target_name} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:${target_name}>/internal_surface/config"
        COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:${target_name}>/internal_surface/templates"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${_internal_surface_source_root}/config/algorithm_config.json"
            "$<TARGET_FILE_DIR:${target_name}>/internal_surface/config/algorithm_config.json"
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${_internal_surface_source_root}/templates"
            "$<TARGET_FILE_DIR:${target_name}>/internal_surface/templates"
        COMMENT "Deploying internal surface measurement runtime assets"
    )
endfunction()

function(scan_tracking_deploy_bevel_runtime target_name)
    set(_bevel_source_root "${CMAKE_SOURCE_DIR}/third_party/Po_Kou_Ce_Liang")
    if(NOT EXISTS "${_bevel_source_root}/config.txt")
        message(WARNING "Bevel runtime source not found: ${_bevel_source_root}")
        return()
    endif()

    add_custom_command(TARGET ${target_name} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:${target_name}>/bevel"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${_bevel_source_root}/config.txt"
            "$<TARGET_FILE_DIR:${target_name}>/bevel/config.txt"
        # V1.2 按坡口类型选 config：config_type{N}.txt（缺省回退 config.txt）
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${_bevel_source_root}/config_type0.txt"
            "$<TARGET_FILE_DIR:${target_name}>/bevel/config_type0.txt"
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${_bevel_source_root}/data"
            "$<TARGET_FILE_DIR:${target_name}>/bevel/data"
        COMMENT "Deploying Po_Kou bevel measurement runtime assets"
    )
endfunction()

function(scan_tracking_deploy_openni2_runtime target_name)
    if(NOT EXISTS "${SCAN_TRACKING_OPENNI2_RUNTIME_DIR}")
        message(WARNING "OpenNI2 runtime directory not found: ${SCAN_TRACKING_OPENNI2_RUNTIME_DIR}")
        return()
    endif()

    add_custom_command(TARGET ${target_name} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:${target_name}>"
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${SCAN_TRACKING_OPENNI2_RUNTIME_DIR}"
            "$<TARGET_FILE_DIR:${target_name}>"
        COMMENT "Deploying OpenNI2 runtime"
    )
endfunction()
