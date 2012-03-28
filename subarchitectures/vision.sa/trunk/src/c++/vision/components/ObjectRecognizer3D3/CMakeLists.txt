project(vision.ObjectRecognizer3D3)

FIND_PACKAGE( OpenCV REQUIRED )

include_directories(.)
include_directories(../..)

ENABLE_COGX_VISUALIZATION(TRUE OPENCV)

add_cast_component(ObjectRecognizer3D3 ObjectRecognizer3D.cpp)
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils ${VISUALIZATION_LIBRARIES})
link_cast_component(${CAST_COMPONENT_NAME} v4rCModelRecogniser v4rPCore)
link_cast_component(${CAST_COMPONENT_NAME} ${OPENCV_LDFLAGS})

