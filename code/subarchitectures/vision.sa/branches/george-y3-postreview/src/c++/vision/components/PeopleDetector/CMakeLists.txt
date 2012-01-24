
include_directories(${COGX_ROOT}/tools/hardware/laser/src/c++/components/)
include_directories(${COGX_ROOT}/tools/hardware/laser/src/c++/autogen/)
include_directories(${COGX_ROOT}/tools/hardware/pointCloud/src/c++/components/)
include_directories(${COGX_ROOT}/tools/hardware/pointCloud/src/c++/autogen/)
include_directories(.)
include_directories(${VISUALIZATION_INCLUDE_DIRS})


add_cast_component(PeopleDetector PeopleDetector.cpp PeopleDetector.hpp)
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils ${OPENCV_LDFLAGS})
link_cast_component(${CAST_COMPONENT_NAME} Laser LaserClientUtils)

# needed to allow visualization to use IplImages 
add_definitions(-DFEAT_VISUALIZATION_OPENCV)

add_cast_component(PCPeopleDetector PCPeopleDetector.cpp PCPeopleDetector.hpp)
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils ${OPENCV_LDFLAGS})
link_cast_component(${CAST_COMPONENT_NAME} PointCloudClient)
link_cast_component(${CAST_COMPONENT_NAME} ${VISUALIZATION_LIBRARIES})
