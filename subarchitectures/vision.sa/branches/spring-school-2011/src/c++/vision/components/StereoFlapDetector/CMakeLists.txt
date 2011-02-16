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

#link_cast_component(${CAST_COMPONENT_NAME} glut)
#link_cast_component(${CAST_COMPONENT_NAME} GLU)
#link_cast_component(${CAST_COMPONENT_NAME} SDL)
#link_cast_component(${CAST_COMPONENT_NAME} SDLmain)
#link_cast_component(${CAST_COMPONENT_NAME} Imlib)
#link_cast_component(${CAST_COMPONENT_NAME} rt)
#link_cast_component(${CAST_COMPONENT_NAME} m)
#link_cast_component(${CAST_COMPONENT_NAME} dl)
#link_cast_component(${CAST_COMPONENT_NAME} pthread)
#link_cast_component(${CAST_COMPONENT_NAME} highgui)
#link_cast_component(${CAST_COMPONENT_NAME} cxcore)
#link_cast_component(${CAST_COMPONENT_NAME} cv)
#link_cast_component(${CAST_COMPONENT_NAME} qt-mt)


link_cast_component(${CAST_COMPONENT_NAME} SFSrc SFMatas)
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils)

