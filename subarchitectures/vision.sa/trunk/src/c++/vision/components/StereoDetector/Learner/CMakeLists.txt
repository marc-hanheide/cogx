PROJECT(SDLearner)

include_directories(.)

find_package(PCL 1.0 REQUIRED COMPONENTS)
include_directories(${PCL_INCLUDE_DIRS})

# Set the sources.
SET(HEADERS
  CalculateRelations.h
  Learner.h
  LearnPropBase.h
  SVMFileCreator.h
)

SET(SOURCES 
  CalculateRelations.cpp
  Learner.cpp
  LearnPropBase.cpp
  SVMFileCreator.cpp
)

add_cast_component(${PROJECT_NAME} ${SOURCES} ${HEADERS})
link_cast_component(${CAST_COMPONENT_NAME} ${PCL_LIBRARIES})

