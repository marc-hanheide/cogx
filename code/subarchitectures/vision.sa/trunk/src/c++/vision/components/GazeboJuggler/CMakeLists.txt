include_directories(.)
include_directories(${VISUALIZATION_INCLUDE_DIRS})

include_directories(${COGX_ROOT}/tools/math/src/c++/math)
#include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/autogen)

Find_package(Player)
# FIND_PACKAGE( OpenCV REQUIRED )

add_cast_component(GazeboJuggler GazeboJuggler.cpp)
link_cast_component(${CAST_COMPONENT_NAME}
   ${PLAYER_LDFLAGS}
   # ${OPENCV_LDFLAGS}
   ${VISUALIZATION_LIBRARIES}
   )

