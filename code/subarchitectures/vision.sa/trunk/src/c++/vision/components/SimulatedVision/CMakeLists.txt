include_directories(.)

ENABLE_COGX_VISUALIZATION(TRUE)

# Set the sources.
SET(SOURCES SimulatedVision.cpp)

add_cast_component(SimulatedVision ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME}
   VisionData
   ${VISUALIZATION_LIBRARIES}
   # CogxIceAutogen
   )

