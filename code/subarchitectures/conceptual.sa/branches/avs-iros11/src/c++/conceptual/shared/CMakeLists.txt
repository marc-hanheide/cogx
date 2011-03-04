project(ConceptualShared)

set(SOURCES VariableNameGenerator.cc)

add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_link_libraries(${PROJECT_NAME} CDL ConceptualData)
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
