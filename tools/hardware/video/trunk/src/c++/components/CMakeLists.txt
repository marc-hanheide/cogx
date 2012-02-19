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
   add_cast_component(PointGreyServerUSB PointGreyServerUSB.cpp)
   link_cast_component(${CAST_COMPONENT_NAME} Video VideoServer VideoUtils)
   link_cast_component(${CAST_COMPONENT_NAME} flycapture)
endif(BUILD_HAL_VIDEO_POINTGREY)

option(BUILD_HAL_VIDEO_PLAYER "Build Player video server" NO)
if(BUILD_HAL_VIDEO_PLAYER)
   find_package(Player REQUIRED)
   include_directories(${PLAYER_INCLUDE})
   add_cast_component(PlayerVideoServer PlayerVideoServer.cpp)
   link_cast_component(${CAST_COMPONENT_NAME} Video VideoServer VideoUtils)
   link_cast_component(${CAST_COMPONENT_NAME} ${PLAYER_LIBRARIES})
endif(BUILD_HAL_VIDEO_PLAYER)

option(BUILD_HAL_VIDEO_PROSILICA "Build Prosilica video server" NO)
if(BUILD_HAL_VIDEO_PROSILICA)
include_directories(./prosilica/inc-pc)

if (UNIX AND NOT WIN32)
    if (CMAKE_SIZEOF_VOID_P MATCHES "8")
	ADD_DEFINITIONS(-D_LINUX -D_x64)
    else (CMAKE_SIZEOF_VOID_P MATCHES "8")
	ADD_DEFINITIONS(-D_LINUX -D_x86)
    endif (CMAKE_SIZEOF_VOID_P MATCHES "8")
endif (UNIX AND NOT WIN32)

add_cast_component(ProsilicaServer ProsilicaServer.cpp)
link_cast_component(${CAST_COMPONENT_NAME} Video VideoServer VideoUtils PvAPI)
endif(BUILD_HAL_VIDEO_PROSILICA)
