
include_directories(${COGX_ROOT}/tools/hardware/laser/src/c++/components/)
include_directories(${COGX_ROOT}/tools/hardware/laser/src/c++/autogen/)
include_directories(.)

add_cast_component(PeopleDetector PeopleDetector.cpp PeopleDetector.hpp)
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils ${OPENCV_LDFLAGS})
link_cast_component(${CAST_COMPONENT_NAME} Laser LaserClientUtils)
