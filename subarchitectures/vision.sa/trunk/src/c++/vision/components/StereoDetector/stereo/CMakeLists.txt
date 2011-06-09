PROJECT(SDStereo)

include_directories(.)
#include_directories(../Gestalts3D)

# Set the sources.
SET(HEADERS
  StereoBase.h
#	StereoCamera.hh
  StereoClosures.h
  StereoCore.h
  StereoCorners.h
  StereoCubes.h
  StereoEllipses.h
  StereoFlaps.h
  StereoFlapsAri.h
  StereoLines.h
  StereoLJunctions.h
  StereoRectangles.h
  StereoTypes.h
)

SET(SOURCES 
	StereoBase.cpp
#	StereoCamera.cc
  StereoClosures.cpp
  StereoCore.cpp
  StereoCorners.cpp
  StereoCubes.cpp
  StereoEllipses.cpp
  StereoFlaps.cpp
  StereoFlapsAri.cpp
  StereoLines.cpp
  StereoLJunctions.cpp
  StereoRectangles.cpp
  StereoTypes.cpp
)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
#target_link_libraries(${PROJECT_NAME} SDGestalts3D)
INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

