include_directories(${COGX_ROOT}/tools/hardware/ptz/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/laser/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/pointCloud/src/c++/autogen)

ENABLE_COGX_VISUALIZATION(TRUE OPENCV)

# Set the sources.
SET(SOURCES CheckSystemCalibration.cpp)
include_directories(.)

add_cast_component(CheckSystemCalibration ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} 
   VisionData Video VideoUtils PTZ
   ${VISUALIZATION_LIBRARIES}
   ${OPENCV_LIBRARIES}
   )

