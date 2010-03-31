include(Slice2Py)

cast_slice2Py(GEN_FILE P+lanner.ice ${CAST_SLICE_DIR} ${CAST_SLICE_DIR}/cast/slice ${COGX_ROOT}/subarchitectures/binder/src/slice)
cast_slice2Py(GEN_FILE2 ../binder/src/slice/beliefmodels.ice ${CAST_SLICE_DIR}/cast/slice ${CAST_SLICE_DIR} )

add_custom_target(python ALL DEPENDS ${GEN_FILE} ${GEN_FILE2})

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/standalone/config.auto.in" "${CMAKE_CURRENT_SOURCE_DIR}/standalone/config.auto")

#file(GLOB_RECURSE moduleFiles *.py)
#file(GLOB_RECURSE moduleFilesRel RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.py)

 install(DIRECTORY ./ DESTINATION ${CMAKE_INSTALL_PREFIX}/python
          PATTERN ".svn" EXCLUDE
          PERMISSIONS OWNER_WRITE OWNER_READ
                      GROUP_WRITE GROUP_READ)

install(CODE "exec_program(\"${CMAKE_COMMAND}\" ARGS -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_INSTALL_PREFIX}/planner)")



#install(CODE "exec_program(\"${CMAKE_COMMAND}\" ARGS -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_INSTALL_PREFIX}/python)")
#install(CODE "file(MAKE_DIRECTORY  ${CMAKE_INSTALL_PREFIX}/python)")
#install(CODE "execute_process(COMMAND cp -R --parents . ${CMAKE_INSTALL_PREFIX}/python WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})")

