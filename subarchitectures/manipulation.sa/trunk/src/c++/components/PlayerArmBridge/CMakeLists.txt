
include_directories(${COGX_ROOT}/tools/math/src/c++/math)
#include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/utils)
#include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/components)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/autogen)
#include_directories(${COGX_ROOT}/tools/hardware/stereo/src/c++/components)
#include_directories(${COGX_ROOT}/tools/hardware/stereo/src/c++/autogen)

Find_package(Player)

ENABLE_COGX_VISUALIZATION(TRUE)

add_cast_component(PlayerArmBridge PlayerArmBridge.cpp)
target_link_libraries(PlayerArmBridge ManipulationData ${PLAYER_LDFLAGS} ${VISUALIZATION_LIBRARIES} ${OPENCV_LDFLAGS})
