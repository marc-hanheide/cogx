include_directories(.)

include_directories(matas)
include_directories(vs3)

add_subdirectory(matas)
add_subdirectory(vs3)

# Set the sources.
SET(HEADERS
	StereoFlapDetector.h
)

SET(SOURCES 
	StereoFlapDetector.cpp
	StereoCore.cc
	StereoCamera.cc
)

add_cast_component(StereoFlapDetector ${SOURCES})

link_cast_component(${CAST_COMPONENT_NAME} SFSrc SFMatas)
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils)

