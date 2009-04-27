project (VISIONSA)

# use cmake files packaged with CogX and CAST
set(CMAKE_MODULE_PATH "${CAST_ROOT}/cmake;${COGX_ROOT}/cmake")
include(${CAST_ROOT}/cmake/FindCAST.cmake)
include(${CAST_ROOT}/cmake/UseCAST.cmake)

include_directories(${COGX_ROOT}/tools/math/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/math/src/c++/math)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/utils)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/components)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/autogen)

# auto generated code
include_directories(src/c++/vision/autogen)

add_subdirectory(src/c++/vision/autogen)

#add_subdirectory (src/c++/vision/components/VisualWMVisualisation)
#add_subdirectory (src/c++/vision/components/ObjectDetectorFERNS)
add_subdirectory (src/c++/vision/components/ObjectTracker)
add_subdirectory (src/c++/vision/components/ObjectTrackerDriver)
add_subdirectory (src/c++/vision/components/VideoViewer)
add_subdirectory (src/c++/vision/components/DummyDriver)

