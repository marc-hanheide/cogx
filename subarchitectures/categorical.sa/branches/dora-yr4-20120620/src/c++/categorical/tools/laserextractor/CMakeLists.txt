project(laserextractor)

# Set the sources.
set(SOURCES main.cpp)

add_executable(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME} SequentialAdaBoost)
target_link_libraries(${PROJECT_NAME} CategoricalShared)

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)

