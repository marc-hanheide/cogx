project(laserextractor)

# Set the sources.
set(SOURCES main.cpp)

add_executable(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME} SequentialAdaBoost)
target_link_libraries(${PROJECT_NAME} PlaceShared)

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)

