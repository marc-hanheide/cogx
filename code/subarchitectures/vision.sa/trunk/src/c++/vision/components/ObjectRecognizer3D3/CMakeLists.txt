# needed to allow visualization to use IplImages 
add_definitions(-DFEAT_VISUALIZATION_OPENCV)

FIND_PACKAGE( OpenCV REQUIRED )
FIND_PACKAGE( V4R REQUIRED )

include_directories(.)
include_directories(../..)
include_directories(${VISUALIZATION_INCLUDE_DIRS})

add_cast_component(ObjectRecognizer3D3 ObjectRecognizer3D.cpp)
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils ${VISUALIZATION_LIBRARIES})
link_cast_component(${CAST_COMPONENT_NAME} CModelRecogniser PGeometry PMath)
link_cast_component(${CAST_COMPONENT_NAME} ${OPENCV_LDFLAGS})

