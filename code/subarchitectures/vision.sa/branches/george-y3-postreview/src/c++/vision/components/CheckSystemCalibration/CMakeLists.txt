include_directories(${COGX_ROOT}/tools/hardware/ptz/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/laser/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/pointCloud/src/c++/autogen)

# Set the sources.
SET(SOURCES CheckSystemCalibration.cpp)
include_directories(
   .
   ${VISUALIZATION_INCLUDE_DIRS}
   )
add_definitions(-DFEAT_VISUALIZATION_OPENCV)

add_cast_component(CheckSystemCalibration ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} 
   VisionData Video VideoUtils PTZ
   ${VISUALIZATION_LIBRARIES}
   ${OPENCV_LIBRARIES}
   )

