# needed to allow visualization to use IplImages 
add_definitions(-DFEAT_VISUALIZATION_OPENCV)

FIND_PACKAGE( OpenCV REQUIRED )

include_directories(.)
include_directories(../..)
include_directories(${VISUALIZATION_INCLUDE_DIRS})
link_directories(${VISUALIZATION_LIBRARY_DIRS})

add_cast_component(ObjectRecognizer3D3 ObjectRecognizer3D.cpp)
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils ${VISUALIZATION_LIBRARIES})
link_cast_component(${CAST_COMPONENT_NAME} v4rCModelRecogniser v4rPCore)
link_cast_component(${CAST_COMPONENT_NAME} ${OPENCV_LDFLAGS})

