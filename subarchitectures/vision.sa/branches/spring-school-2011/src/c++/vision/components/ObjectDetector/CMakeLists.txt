include_directories(.)

include_directories(mxCvTools)
include_directories(matas)
include_directories(src)

add_subdirectory(mxCvTools)
add_subdirectory(matas)
add_subdirectory(src)

# Set the sources.
SET(HEADERS
#	ObjectDetector.h
#	VisionUtils3.h
#	Vs3Interface.h
)

SET(SOURCES 
	ObjectDetector.cpp
	VisionUtils.cpp
	Vs3Interface.cpp
)

add_cast_component(ObjectDetector ${SOURCES})

link_cast_component(${CAST_COMPONENT_NAME} glut)
link_cast_component(${CAST_COMPONENT_NAME} GLU)
link_cast_component(${CAST_COMPONENT_NAME} SDL)
link_cast_component(${CAST_COMPONENT_NAME} SDLmain)
link_cast_component(${CAST_COMPONENT_NAME} rt)
link_cast_component(${CAST_COMPONENT_NAME} m)
link_cast_component(${CAST_COMPONENT_NAME} dl)
link_cast_component(${CAST_COMPONENT_NAME} pthread)
link_cast_component(${CAST_COMPONENT_NAME} ${OPENCV_LDFLAGS})


link_cast_component(${CAST_COMPONENT_NAME} Vs3Src Vs3Matas Vs3MxCvTools)
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils)

