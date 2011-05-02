
include_directories(${VISUALIZATION_INCLUDE_DIRS})

include_directories(${COGX_ROOT}/subarchitectures/spatial.sa/src/c++/autogen)
include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/autogen)

include_directories(${COGX_ROOT}/tools/math/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/math/src/c++/math)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/utils)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/components)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/pointCloud/src/c++/components)
include_directories(${COGX_ROOT}/tools/hardware/pointCloud/src/c++/autogen)


add_cast_component(RobotPoseCalculator RobotPoseCalculator.cpp)
target_link_libraries(RobotPoseCalculator ManipulationData VisionData NavData ${PLAYER_LDFLAGS} ${VISUALIZATION_LIBRARIES} ${OPENCV_LDFLAGS})
