include_directories(.)

include_directories(${VISUALIZATION_INCLUDE_DIRS})

# Set the sources.
SET(SOURCES Scene2D.cpp)

add_cast_component(VirtualScene2D ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME}
   VisionData
   ${VISUALIZATION_LIBRARIES}
   plotter
   )

