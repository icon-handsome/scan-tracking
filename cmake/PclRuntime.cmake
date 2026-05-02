include_guard(GLOBAL)

set(
    SCAN_TRACKING_PCL_RUNTIME_DIR
    "C:/Program Files/PCL 1.12.0/bin"
    CACHE PATH
    "Path to the local PCL runtime DLL directory"
)

set(
    SCAN_TRACKING_OPENCV_RUNTIME_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/LB/opencv-3.4.3-vc14_vc15/opencv/build/x64/vc15/bin"
    CACHE PATH
    "Path to the local OpenCV runtime DLL directory"
)

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
