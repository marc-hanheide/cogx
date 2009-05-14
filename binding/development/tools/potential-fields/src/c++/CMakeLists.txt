PROJECT(PotentialFields)

SET(SOURCES ProximityMap.cpp ProjectiveMap.cpp)

SET(HEADERS myVector.h potentialField.h scene.h ProximityMap.h
ProjectiveMap.h)

# Add the library.
ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})


INSTALL(FILES ${HEADERS} DESTINATION include/pf)
INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

# the executable is useful for testing things
#ADD_EXECUTABLE(pftest  main.cpp)
#TARGET_LINK_LIBRARIES(pftest PotentialFields)
##TARGET_LINK_LIBRARIES(pftest glut)

#INSTALL(TARGETS pftest RUNTIME DESTINATION bin)
