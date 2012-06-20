include_directories(.)

# Set the sources.
SET(HEADERS
	Soi2Roi.h
)

SET(SOURCES 
	Soi2Roi.cpp
)

add_cast_component(Soi2Roi ${SOURCES})

link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils)
link_cast_component(${CAST_COMPONENT_NAME} Stereo StereoClient)

