project (GazeboJuggler)

include_directories(.)

include_directories(${COGX_ROOT}/tools/math/src/c++/math)
#include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/autogen)

Find_package(Player)
# FIND_PACKAGE( OpenCV REQUIRED )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

ENABLE_COGX_VISUALIZATION(TRUE)

add_cast_component(GazeboJuggler GazeboJuggler.cpp)
link_cast_component(${CAST_COMPONENT_NAME}
   ${PLAYER_LDFLAGS}
   # ${OPENCV_LDFLAGS}
   ${VISUALIZATION_LIBRARIES}
   )

add_custom_target (${CAST_COMPONENT_NAME}.Resources
   DEPENDS res/juggler_gen.inc)
add_custom_command (
   OUTPUT res/juggler_gen.inc
   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/res
   COMMENT "make ${CAST_COMPONENT_NAME}.Resources"
   COMMAND make
)
add_dependencies(${CAST_COMPONENT_NAME} ${CAST_COMPONENT_NAME}.Resources)
