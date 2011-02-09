include_directories(.)

add_definitions(-DHAVE_CAST)

include_directories(matas)
include_directories(math)
include_directories(utils)
include_directories(vs3)
include_directories(stereo)
include_directories(reasoner)
include_directories(CEdge)

add_subdirectory(CEdge)
add_subdirectory(matas)
add_subdirectory(utils)
add_subdirectory(vs3)
add_subdirectory(stereo)
add_subdirectory(reasoner)

# Set the sources.
SET(HEADERS
	./math/Vector2.hh
	StereoDetector.h
)

SET(SOURCES 
	StereoDetector.cpp
)

add_cast_component(StereoDetector ${SOURCES} ${HEADERS})

link_cast_component(${CAST_COMPONENT_NAME} SDUtils SDStereo SDMatas SDvs3 SDReasoner SDCEdge)
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils)
link_cast_component(${CAST_COMPONENT_NAME} Stereo StereoClient)



