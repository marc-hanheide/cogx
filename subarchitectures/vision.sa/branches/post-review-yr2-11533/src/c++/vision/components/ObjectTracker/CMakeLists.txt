project(ObjectTracker)

include_directories(.)
include_directories(Tracker/include)
include_directories(/usr/include/opencv)
include_directories(/usr/include/SDL)

add_subdirectory(Tracker)

link_directories(/usr/local/lib)

# Set the sources
SET(SOURCES ObjectTracker.cpp)

add_cast_component(ObjectTracker ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} Tracker)
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils)
link_cast_component(${CAST_COMPONENT_NAME} ${OpenCV_LIBRARIES} opencv_core)
link_cast_component(${CAST_COMPONENT_NAME} SDL GLU glut)
