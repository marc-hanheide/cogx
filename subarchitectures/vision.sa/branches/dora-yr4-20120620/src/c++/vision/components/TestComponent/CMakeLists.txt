include_directories(.)

#include_directories(matas)
#include_directories(utils)
#include_directories(vs3)
#include_directories(windowUtils)
#include_directories(stereo)

#add_subdirectory(matas)
#add_subdirectory(utils)
#add_subdirectory(vs3)
#add_subdirectory(windowUtils)
#add_subdirectory(stereo)

# Set the sources.
SET(HEADERS
	TestComponent.h
)

SET(SOURCES 
	TestComponent.cpp
)

add_cast_component(TestComponent ${SOURCES})

#link_cast_component(${CAST_COMPONENT_NAME} SDUtils SDStereo SDMatas SDvs3)
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils)
link_cast_component(${CAST_COMPONENT_NAME} Stereo StereoClient)

