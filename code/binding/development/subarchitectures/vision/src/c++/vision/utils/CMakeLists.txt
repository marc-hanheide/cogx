PROJECT(VisionUtils)

# Set the sources.
SET(SOURCES 
VisionUtils.cpp 
VideoClientProcess.cpp 
CamPars.cpp 
Socket.cpp
XDRStream.cpp 
ConnHub.cpp 
Connection.cpp 
VideoClt.cpp
ImageFrameStream.cpp  
VisionProxy.cpp 
SceneObjectWriter.cpp
)

SET(HEADERS 
VisionUtils.h 
VideoClientProcess.h 
CamPars.h 
Socket.hpp
XDRStream.hpp 
ConnHub.hpp 
Connection.hpp 
VideoClt.hpp
ImageFrameStream.hpp 
VisionProxy.h 
SceneObjectWriter.hpp
)

FIND_PATH(SVS_DIR libsvs.a)

SET(IMLIB_LIB /usr/X11R6/lib)
LINK_DIRECTORIES(${IMLIB_LIB})

# fix the libgl linking error 
IF (${CMAKE_SYSTEM_NAME} STREQUAL "Darwin")
  IF (${CMAKE_SYSTEM_VERSION} STREQUAL "9.2.0")
     SET (CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-dylib_file,/System/Library/Frameworks/OpenGL.framework/Versions/A/Libraries/libGL.dylib:/System/Library/Frameworks/OpenGL.framework/Versions/A/Libraries/libGL.dylib")
  ENDIF (${CMAKE_SYSTEM_VERSION} STREQUAL "9.2.0")
ENDIF (${CMAKE_SYSTEM_NAME} STREQUAL "Darwin")


# Add the Vision library to the build targets.
ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OMNIORB4_LIBRARIES})

# Add libraries to link against.
TARGET_LINK_LIBRARIES(${PROJECT_NAME} FrameworkInterface FrameworkIDL FrameworkCore)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} CASTCore CASTArchitecture CASTIDL)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} Vision)

INCLUDE(UsePkgConfig)
PKGCONFIG("opencv" OCV_INCLUDE_DIR OCV_LINK_DIR OCV_LINK_FLAGS OCV_CFLAGS)
#message(STATUS "inc: -${OCV_INCLUDE_DIR}")
#message(STATUS "link flags: -${OCV_LINK_FLAGS}")
#message(STATUS "cflags: -${OCV_CFLAGS}")
#message(STATUS "link dir: -${OCV_LINK_DIR}")
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OCV_LINK_FLAGS})

INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
INSTALL(FILES ${HEADERS} DESTINATION include/vision/utils)


