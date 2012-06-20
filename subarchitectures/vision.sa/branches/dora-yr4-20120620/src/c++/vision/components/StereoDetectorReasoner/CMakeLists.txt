include_directories(.)

include_directories(math)

# Set the sources.
SET(HEADERS
	Object.h
	Plane.h
	StereoDetectorReasoner.h
)

SET(SOURCES
	Object.cpp
	Plane.cpp
	StereoDetectorReasoner.cpp
)

add_cast_component(StereoDetectorReasoner ${SOURCES})

link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils)
link_cast_component(${CAST_COMPONENT_NAME} Stereo StereoClient)

