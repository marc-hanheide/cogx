project(conceptualviewer)

include_directories(${ROCS_SRC_INCLUDES})

# Set the sources.
set(SOURCES main.cpp)

include_directories(${COGX_ROOT}/tools/rocs/thirdparty/libdai/include)

add_executable(${PROJECT_NAME} ${SOURCES})

set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH ${ROCS_DIR}/lib)
link_cast_component(${PROJECT_NAME} dai)

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin)

