project(VideoGrabber)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

FIND_PACKAGE( OpenCV REQUIRED )
include_directories(${OPENCV_INCLUDE_DIRS})

# Set the sources.
SET(SOURCES
   CoreStructs.cpp
   Grabber.cpp
   VideoGrabber.cpp
   )

ENABLE_COGX_VISUALIZATION(TRUE OPENCV)

option(OPT_VIDEOGRABBER_POINTCLOUD "Video Grabber supports 3D point-cloud grabbing." ON)
if(OPT_VIDEOGRABBER_POINTCLOUD)
   include_directories(
      ${COGX_ROOT}/tools/hardware/src/cpp/autogen
      ${COGX_ROOT}/tools/hardware/src/cpp/components
      )
   SET(SOURCES ${SOURCES}
      PcGrabber.cpp
      )
   add_definitions(-DFEAT_VIDEOGRABBER_POINTCLOUD)
   set(EXTRALIBS PointCloud PointCloudClient)
endif()

add_cast_component(VideoGrabber ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} 
   Video VideoUtils
   ${VISUALIZATION_LIBRARIES}
   ${OPENCV_LIBRARIES}
   ${EXTRALIBS}
   )

