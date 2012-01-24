
add_definitions(-DFEAT_VISUALIZATION_OPENCV)

include_directories(${VISUALIZATION_INCLUDE_DIRS})

include_directories(${COGX_ROOT}/tools/math/src/c++/math)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/autogen)

Find_package(Player)
FIND_PACKAGE( OpenCV REQUIRED )
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})

add_cast_component(GazeboTurntable GazeboTurntable.cpp)
link_cast_component(${CAST_COMPONENT_NAME} ${VISUALIZATION_LIBRARIES} ${PLAYER_LDFLAGS} ${OPENCV_LDFLAGS}) 
