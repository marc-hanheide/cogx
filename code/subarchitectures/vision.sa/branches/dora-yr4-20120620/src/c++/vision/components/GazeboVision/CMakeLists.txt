include_directories(.)

include_directories(${COGX_ROOT}/tools/math/src/c++/math)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/autogen)

Find_package(Player)
FIND_PACKAGE( OpenCV REQUIRED )

ENABLE_COGX_VISUALIZATION(TRUE OPENCV)

add_cast_component(GazeboVision GazeboVision.cpp)
link_cast_component(${CAST_COMPONENT_NAME} VisionData ${PLAYER_LDFLAGS} ${OPENCV_LDFLAGS})

