FIND_PACKAGE( OpenCV REQUIRED )

include_directories(.)
include_directories(/opt/local/include)
link_directories (/opt/local/lib)

# Set the sources.
SET(SOURCES VideoServer.cpp)
SET(HEADERS VideoServer.h)

add_library(VideoServer SHARED ${SOURCES})
target_link_libraries(VideoServer CASTCore CDL CASTArchitecture)
target_link_libraries(VideoServer Video VideoUtils)

INSTALL(TARGETS VideoServer LIBRARY DESTINATION lib)


option(BUILD_HAL_VIDEO_IMG_SEQ "Build image sequence video server" YES)
option(BUILD_HAL_VIDEO_IMG_SEQ_UI "Build UI for image sequence server (requires Visualization SA)" NO)
if(BUILD_HAL_VIDEO_IMG_SEQ)
   if(BUILD_HAL_VIDEO_IMG_SEQ_UI)
      # -DFEAT_VISUALIZATION is already defined by v11n.cmake, but we re-define it here
      add_definitions(-DFEAT_VISUALIZATION) # Valid for this directory and below
      include_directories(${VISUALIZATION_INCLUDE_DIRS})
   else(BUILD_HAL_VIDEO_IMG_SEQ_UI)
      remove_definitions(-DFEAT_VISUALIZATION)
   endif(BUILD_HAL_VIDEO_IMG_SEQ_UI)

   add_cast_component(OpenCvImgSeqServer OpenCvImgSeqServer.cpp)
   link_cast_component(${CAST_COMPONENT_NAME} Video VideoServer VideoUtils)
   link_cast_component(${CAST_COMPONENT_NAME} ${OPENCV_LDFLAGS})

   if(BUILD_HAL_VIDEO_IMG_SEQ_UI)
      link_cast_component(${CAST_COMPONENT_NAME} ${VISUALIZATION_LIBRARIES})
   endif(BUILD_HAL_VIDEO_IMG_SEQ_UI)
endif(BUILD_HAL_VIDEO_IMG_SEQ)

option(BUILD_HAL_VIDEO_LIVE "Build live video server" YES)
if(BUILD_HAL_VIDEO_LIVE)
   add_cast_component(OpenCvLiveServer OpenCvLiveServer.cpp)
   link_cast_component(${CAST_COMPONENT_NAME} Video VideoServer VideoUtils)
   link_cast_component(${CAST_COMPONENT_NAME} ${OPENCV_LDFLAGS})
endif(BUILD_HAL_VIDEO_LIVE)

option(BUILD_HAL_VIDEO_POINTGREY "Build Point Grey video server" NO)
if(BUILD_HAL_VIDEO_POINTGREY)
   add_cast_component(PointGreyServer PointGreyServer.cpp)
   link_cast_component(${CAST_COMPONENT_NAME} Video VideoServer VideoUtils)
   link_cast_component(${CAST_COMPONENT_NAME} flycapture)
endif(BUILD_HAL_VIDEO_POINTGREY)

