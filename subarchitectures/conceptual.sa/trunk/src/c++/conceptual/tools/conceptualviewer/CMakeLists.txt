project(conceptualviewer)

# Set the sources.
set(SOURCES main.cpp)

add_executable(${PROJECT_NAME} ${SOURCES})

set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH ${ROCS_DIR}/lib)
link_cast_component(${PROJECT_NAME} dai)

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)

