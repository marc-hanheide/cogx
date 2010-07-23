include_directories(.)

#include_directories(${Boost_INCLUDE_DIR})
#link_directories(${Boost_LIBRARY_DIRS})

include_directories(${VISUALIZATION_INCLUDE_DIRS})

# Set the sources.
SET(SOURCES SimulatedVision.cpp)

add_cast_component(SimulatedVision ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME}
   VisionData
   ${VISUALIZATION_LIBRARIES}
   # CogxIceAutogen DisplayClient
   )

