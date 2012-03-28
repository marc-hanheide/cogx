project(vision.VideoViewer)

FIND_PACKAGE( OpenCV REQUIRED )

include_directories(${COGX_ROOT}/tools/hardware/pointCloud/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/laser/src/c++/autogen)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})

ENABLE_COGX_VISUALIZATION(TRUE OPENCV)

# Set the sources.
SET(SOURCES VideoViewer.cpp)
include_directories(.)

add_cast_component(VideoViewer ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} 
   Video VideoUtils
   ${VISUALIZATION_LIBRARIES}
   ${OPENCV_LIBRARIES}
   )

