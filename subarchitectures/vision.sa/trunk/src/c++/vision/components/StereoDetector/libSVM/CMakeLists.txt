PROJECT(SDlibSVM)

include_directories(.)

# Set the sources.
SET(HEADERS
  svm.h
  SVMPredictor.h
)

SET(SOURCES 
  svm.cpp
  SVMPredictor.cpp
)

add_cast_component(${PROJECT_NAME} ${SOURCES} ${HEADERS})
link_cast_component(${CAST_COMPONENT_NAME})

