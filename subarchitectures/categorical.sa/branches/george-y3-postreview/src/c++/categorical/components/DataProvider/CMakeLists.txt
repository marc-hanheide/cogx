include_directories(.)
include_directories(${COGX_ROOT}/tools/hardware/laser/src/c++/components/)
include_directories(${COGX_ROOT}/tools/hardware/laser/src/c++/autogen/)
include_directories(${COGX_ROOT}/tools/hardware/robotbase/src/c++/components/)
include_directories(${COGX_ROOT}/tools/hardware/robotbase/src/c++/autogen/)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/utils)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/components)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/autogen)

add_cast_component(CategoricalDataProvider DataProvider.cpp LaserCorrector.cpp)
link_cast_component(${CAST_COMPONENT_NAME} CategoricalData CategoricalShared)
link_cast_component(${CAST_COMPONENT_NAME} Laser Scan2dReceiver Robotbase OdometryReceiver)
link_cast_component(${CAST_COMPONENT_NAME} VisionData Video VideoUtils)
link_cast_component(${CAST_COMPONENT_NAME} SpatialData)


