option(BUILD_SA_CONCEPTUAL_VIEWER "Build Conceptual.SA Viewer" YES)

if(BUILD_SA_CONCEPTUAL_VIEWER)
project(conceptualviewer)

# Set the sources.
set(SOURCES main.cpp)

add_executable(${PROJECT_NAME} ${SOURCES})

set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH ${ROCS_DIR}/lib)
link_cast_component(${PROJECT_NAME} dai)

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)
endif(BUILD_SA_CONCEPTUAL_VIEWER)

