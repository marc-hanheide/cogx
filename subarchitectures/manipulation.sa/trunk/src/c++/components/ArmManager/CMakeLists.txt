
include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/autogen)
#include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/utils)
#include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/components)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/autogen)
#include_directories(${COGX_ROOT}/tools/hardware/stereo/src/c++/components)
#include_directories(${COGX_ROOT}/tools/hardware/stereo/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/pointCloud/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/laser/src/c++/autogen)
#include_directories(${COGX_ROOT}/subarchitectures/manipulation.sa/src/c++/autogen/execution)

Find_package(Player)

ENABLE_COGX_VISUALIZATION(TRUE)

add_cast_component(ArmManager ArmManager.cpp)
target_link_libraries(ArmManager ManipulationData ManipulationExeData VisionData ${PLAYER_LDFLAGS} ${VISUALIZATION_LIBRARIES} ${OPENCV_LDFLAGS})
