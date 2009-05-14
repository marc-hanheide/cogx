# Start feature learning project.
PROJECT(VideoServer)


#if darwin, then don't include v4l device
IF (${CMAKE_SYSTEM_NAME} STREQUAL "Darwin")
SET(SOURCES VideoServer.cpp ImgSeqDevice.cpp OpenCvDevice.cpp
    VideoSrv.cpp) 
SET(HEADERS VideoServer.h VideoDevice.h ImgSeqDevice.h OpenCvDevice.h
    VideoSrv.hpp)
ELSE (${CMAKE_SYSTEM_NAME} STREQUAL "Darwin")
SET(SOURCES VideoServer.cpp ImgSeqDevice.cpp V4LDevice.cpp
    OpenCvDevice.cpp VideoSrv.cpp)
SET(HEADERS VideoServer.h VideoDevice.h ImgSeqDevice.h V4LDevice.h
    OpenCvDevice.h VideoSrv.hpp)
ENDIF (${CMAKE_SYSTEM_NAME} STREQUAL "Darwin")


ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OMNIORB4_LIBRARIES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} FrameworkInterface FrameworkIDL FrameworkCore)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} CASTCore CASTArchitecture CASTIDL)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} VisionIDL Vision VisionUtils)
#TARGET_LINK_LIBRARIES(${PROJECT_NAME} VisionIDL Vision VisionCommon VisionUtils)
#TARGET_LINK_LIBRARIES(${PROJECT_NAME} raw1394 dc1394_control)

# Use pkg-config to find opencv libraries...
INCLUDE(UsePkgConfig)
PKGCONFIG("opencv" OCV_INCLUDE_DIR OCV_LINK_DIR OCV_LINK_FLAGS OCV_CFLAGS)
#message(STATUS "inc: -${OCV_INCLUDE_DIR}")
#message(STATUS "link flags: -${OCV_LINK_FLAGS}")
#message(STATUS "cflags: -${OCV_CFLAGS}")
#message(STATUS "link dir: -${OCV_LINK_DIR}")
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OCV_LINK_FLAGS})

INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
INSTALL(FILES ${HEADERS} DESTINATION include/vision/components/VideoServer)
