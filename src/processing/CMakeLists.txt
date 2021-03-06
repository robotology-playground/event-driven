option(ENABLE_vFramer "Build visualisation module" ON)
option(ENABLE_vPreProcess "Build simple pre-processing module" ON)
option(ENABLE_vCircle "Build circle detector" OFF)
option(ENABLE_vCluster "Build cluster tracker" OFF)
option(ENABLE_vFlow "Build optical flow module" OFF)
option(ENABLE_vSkinInterface "Build basic skin pre-processing" OFF)
option(ENABLE_vCorner "Build corner detection module" OFF)
option(ENABLE_DualCamTransform "Build Frame->ATIS geometric transform" OFF)
option(ENABLE_vHexviewer "Enable a hexadecimal raw-event viewer" ON)
option(ENABLE_esim "Build a video to events simulator" ON)

if(OpenCV_FOUND)
    if(ENABLE_vFramer)
        add_subdirectory(vFramer)
    endif()

    if(ENABLE_vPreProcess)
        add_subdirectory(vPreProcess)
    endif()

    if(ENABLE_esim)
        add_subdirectory(esim-yarp)
    endif()

else()
    message("Warning: OpenCV not found. Skipping vFramer, vPreProcess")
endif()

if(ENABLE_vCircle)
    if(VLIB_DEPRECATED)
        add_subdirectory(vCircle)
    else()
        message("vCircle requires VLIB_DEPRECATED")
    endif()
endif()

if(ENABLE_vCluster)
    add_subdirectory(vCluster)
endif()

if(ENABLE_vFlow)
    if(VLIB_DEPRECATED)
        add_subdirectory(vFlow)
    else()
        message("vFlow requires VLIB_DEPRECATED")
    endif()
endif()

if(ENABLE_vSkinInterface)
    add_subdirectory(vSkinInterface)
endif()

if(ENABLE_vCorner)
    if(VLIB_DEPRECATED)
        add_subdirectory(vCorner)
    else()
        message("vCorner requires VLIB_DEPRECATED")
    endif()
endif()

if(ENABLE_DualCamTransform)
    add_subdirectory(DualCamTransform)
endif()

if(ENABLE_vHexviewer)
    add_subdirectory(vHexviewer)
endif()


