include_directories(.)

add_definitions(-DHAVE_CAST)

include_directories(TomGine)
include_directories(matas)
include_directories(math)
include_directories(utils)
include_directories(vs3)
include_directories(stereo)
include_directories(reasoner)
include_directories(CEdge)
include_directories(objRepresentation)
include_directories(TomGineWraper)

add_subdirectory(TomGine)
add_subdirectory(CEdge)
add_subdirectory(matas)
add_subdirectory(utils)
add_subdirectory(vs3)
add_subdirectory(stereo)
# add_subdirectory(reasoner)
add_subdirectory(objRepresentation)
add_subdirectory(TomGineWraper)

# Set the sources.
SET(HEADERS
	StereoDetector.h
)

SET(SOURCES 
	StereoDetector.cpp
)

add_cast_component(StereoDetector ${SOURCES} ${HEADERS})

link_cast_component(${CAST_COMPONENT_NAME} SDUtils SDObjRepresentation SDStereo SDMatas SDvs3 SDCEdge)
link_cast_component(${CAST_COMPONENT_NAME} freetype ftgl SDGLWindow SDTomGine SDTomGineWraper)
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils)
link_cast_component(${CAST_COMPONENT_NAME} Stereo StereoClient)



