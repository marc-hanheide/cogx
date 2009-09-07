include(Slice2Py)

cast_slice2Py(GEN_FILE Planner.ice ${CAST_SLICE_DIR} ${CAST_SLICE_DIR}/cast/slice ${CMAKE_CURRENT_SOURCE_DIR}/../../../binder/src/slice )
cast_slice2Py(GEN_FILE2 ../binder/src/slice/BinderEssentials.ice ${CAST_SLICE_DIR}/cast/slice ${CAST_SLICE_DIR} )

add_custom_target(python ALL DEPENDS ${GEN_FILE} ${GEN_FILE2})

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/standalone/config.auto.in" "${CMAKE_CURRENT_SOURCE_DIR}/standalone/config.auto")

install(CODE "exec_program(\"${CMAKE_COMMAND}\" ARGS -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_INSTALL_PREFIX}/python)")
