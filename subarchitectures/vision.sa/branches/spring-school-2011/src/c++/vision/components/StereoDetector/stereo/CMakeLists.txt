PROJECT(SDStereo)

include_directories(.)

# Set the sources.
SET(HEADERS
	StereoBase.h
	StereoCamera.hh
	StereoClosures.h
	StereoCore.hh
	StereoCubes.h
	StereoEllipses.h
	StereoFlaps.h
	StereoFlapsAri.h
	StereoLJunctions.h
	StereoRectangles.h
)

SET(SOURCES 
	StereoBase.cpp
	StereoCamera.cc
	StereoClosures.cpp
	StereoCore.cc
	StereoCubes.cpp
	StereoEllipses.cpp
	StereoFlaps.cpp
	StereoFlapsAri.cpp
	StereoLJunctions.cpp
	StereoRectangles.cpp
)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})
INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

