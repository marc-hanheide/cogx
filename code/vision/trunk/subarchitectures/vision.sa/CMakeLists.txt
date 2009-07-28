project (VISIONSA)

include_directories(/usr/local/include/opencv/)

include_directories(${COGX_ROOT}/tools/math/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/math/src/c++/math)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/utils)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/components)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/stereo/src/c++/components)
include_directories(${COGX_ROOT}/tools/hardware/stereo/src/c++/autogen)

# auto generated code
include_directories(src/c++/vision/autogen)

add_subdirectory(src/c++/vision/autogen)

#add_subdirectory (src/c++/vision/components/VisualWMVisualisation)
#add_subdirectory (src/c++/vision/components/ObjectDetectorFERNS)
#add_subdirectory (src/c++/vision/components/DummyDriver)
add_subdirectory (src/c++/vision/components/VideoViewer)
if(BUILD_HAL_STEREO)
add_subdirectory (src/c++/vision/components/StereoViewer)
endif(BUILD_HAL_STEREO)
add_subdirectory (src/c++/vision/components/ObjectTracker)
add_subdirectory (src/c++/vision/components/ObjectTrackerDriver)
add_subdirectory (src/c++/vision/components/ObjectTrackerTest)
add_subdirectory (src/c++/vision/components/VisionDriver)

add_subdirectory (src/c++/vision/components/Reconstruction)
add_subdirectory (src/c++/vision/components/ObjectDetector)
add_subdirectory (src/c++/vision/components/ObjectDetectorDriver)
if(BUILD_HAL_STEREO)
add_subdirectory (src/c++/vision/components/PlanePopOut)
add_subdirectory (src/c++/vision/components/SOIFilter)
endif(BUILD_HAL_STEREO)

#add_subdirectory (src/c++/vision/components/ObjectRecognizer)
#add_subdirectory (src/python/ObjectRecognizer)


