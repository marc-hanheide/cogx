PROJECT(SDLearner)

include_directories(.)

find_package(PCL 1.0 REQUIRED COMPONENTS)
include_directories(${PCL_INCLUDE_DIRS})

# Set the sources.
SET(HEADERS
  Learner.h
  LearnPropBase.h
)

SET(SOURCES 
  Learner.cpp
  LearnPropBase.cpp
)

add_cast_component(${PROJECT_NAME} ${SOURCES} ${HEADERS})
link_cast_component(${CAST_COMPONENT_NAME} ${PCL_LIBRARIES})

