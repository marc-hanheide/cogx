PROJECT(SDStereo)

include_directories(.)

# Set the sources.
SET(HEADERS
	StereoCore.hh
	StereoCamera.hh
	StereoBase.h
	StereoEllipses.h
	StereoClosures.h
	StereoRectangles.h
	StereoFlaps.h
)

SET(SOURCES 
	StereoCore.cc
	StereoCamera.cc
	StereoBase.cpp
	StereoEllipses.cpp
	StereoClosures.cpp
	StereoRectangles.cpp
	StereoFlaps.cpp
)


ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})
INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

