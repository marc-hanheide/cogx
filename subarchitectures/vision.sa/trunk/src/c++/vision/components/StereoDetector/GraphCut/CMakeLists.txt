PROJECT(SDGraphCut)

include_directories(.)

find_package(PCL 1.0 REQUIRED COMPONENTS)
include_directories(${PCL_INCLUDE_DIRS})

# Set the sources.
SET(HEADERS
  GraphCut.h
  Graph.h
)

SET(SOURCES 
  GraphCut.cpp
  Graph.cpp
  Edge.h
  disjoint-set.h
)

add_cast_component(${PROJECT_NAME} ${SOURCES} ${HEADERS})
link_cast_component(${CAST_COMPONENT_NAME} ${PCL_LIBRARIES})

