PROJECT(SDStereo)

include_directories(.)

# Set the sources.
SET(HEADERS
	../math/Vector2.hh
	Closure3D.h
	Corner3D.h
	Ellipse3D.h
	Flap3D.h
	Gestalt3D.h
	Line3D.h
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
	StereoLines.h
	StereoLJunctions.h
	StereoRectangles.h
	StereoTypes.h
)

SET(SOURCES 
	Closure3D.cpp
	Corner3D.cpp
	Ellipse3D.cpp
	Flap3D.cpp
	Gestalt3D.cpp
	Line3D.cpp
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
	StereoLines.cpp
	StereoLJunctions.cpp
	StereoRectangles.cpp
	StereoTypes.cpp
)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})
INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

