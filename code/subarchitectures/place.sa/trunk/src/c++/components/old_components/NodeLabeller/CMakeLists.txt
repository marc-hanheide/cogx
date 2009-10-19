PROJECT(PlaceNodeLabeller)

# Set the sources.
SET(SOURCES NodeLabeller.cpp OutputsCache.cpp ConfidenceEstimator.cpp)
SET(HEADERS NodeLabeller.h OutputsCache.h ConfidenceEstimator.h)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OMNIORB4_LIBRARIES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} FrameworkInterface FrameworkIDL FrameworkCore)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} CASTCore CASTArchitecture CASTIDL)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PlaceIDL PlaceShared)


INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
INSTALL(FILES ${HEADERS} DESTINATION include/place/components/NodeLabeller)

