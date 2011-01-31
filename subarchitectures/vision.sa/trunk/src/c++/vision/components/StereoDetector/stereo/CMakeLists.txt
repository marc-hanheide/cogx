PROJECT(SDStereo)

include_directories(.)

# Set the sources.
SET(HEADERS
	Closure3D.h
	Corner3D.h
	Flap3D.h
	Gestalt3D.h
	LJunction3D.h
	Rectangle3D.h
	StereoBase.h
	StereoCamera.hh
	StereoClosures.h
	StereoCore.hh
	StereoCorners.h
	StereoCubes.h
	StereoEllipses.h
	StereoFlaps.h
	StereoFlapsAri.h
	StereoLJunctions.h
	StereoRectangles.h
	StereoTypes.h
)

SET(SOURCES 
	Closure3D.cpp
	Corner3D.cpp
	Flap3D.cpp
	Gestalt3D.cpp
	LJunction3D.cpp
	Rectangle3D.cpp
	StereoBase.cpp
	StereoCamera.cc
	StereoClosures.cpp
	StereoCore.cc
	StereoCorners.cpp
	StereoCubes.cpp
	StereoEllipses.cpp
	StereoFlaps.cpp
	StereoFlapsAri.cpp
	StereoLJunctions.cpp
	StereoRectangles.cpp
	StereoTypes.cpp
)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})
INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

