include_directories(.)

include_directories(matas)
include_directories(math)
include_directories(utils)
include_directories(vs3)
include_directories(stereo)
include_directories(reasoner)

add_subdirectory(matas)
add_subdirectory(math)
add_subdirectory(utils)
add_subdirectory(vs3)
add_subdirectory(stereo)
add_subdirectory(reasoner)


# Set the sources.
SET(HEADERS
	StereoDetector.h
)

SET(SOURCES 
	StereoDetector.cpp
)

add_cast_component(StereoDetector ${SOURCES})

link_cast_component(${CAST_COMPONENT_NAME} SDUtils SDStereo SDMatas SDvs3 SDReasoner)
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils)
link_cast_component(${CAST_COMPONENT_NAME} Stereo StereoClient)



