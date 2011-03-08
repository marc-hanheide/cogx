configure_file("${CMAKE_CURRENT_SOURCE_DIR}/standalone/config.auto.in" "${CMAKE_CURRENT_SOURCE_DIR}/standalone/config.auto")

if(PLANNER_STANDALONE)
  return()
endif()

include(Slice2Py)

cast_slice2Py(GEN_FILE Planner.ice ${CAST_SLICE_DIR} ${CAST_SLICE_DIR}/cast/slice ${COGX_ROOT}/subarchitectures/binder/src/slice
  ${COGX_ROOT}/tools/beliefs/apis/beliefs/src/slice
  ${COGX_ROOT}/tools/beliefs/apis/beliefs+cast/src/slice )
cast_slice2Py(GEN_FILE2 ../../tools/beliefs/apis/beliefs/src/slice/beliefs.ice ${CAST_SLICE_DIR}/cast/slice ${CAST_SLICE_DIR} )
cast_slice2Py(GEN_FILE3 ../../tools/beliefs/apis/beliefs+cast/src/slice/beliefs_cast.ice ${CAST_SLICE_DIR}/cast/slice ${CAST_SLICE_DIR} )
cast_slice2Py(GEN_FILE4 ../../tools/cogxutils/src/slice/beliefs_cogx.ice ${CAST_SLICE_DIR}/cast/slice ${CAST_SLICE_DIR} )
cast_slice2Py(GEN_SPATIAL_PROB_ICE ../../subarchitectures/spatial.sa/src/slice/SpatialProbabilities.ice  ${CAST_SLICE_DIR}/cast/slice ${CAST_SLICE_DIR} 
              ${COGX_ROOT}/subarchitectures/spatial.sa/src/slice)
set(GENERATED ${GEN_FILE} ${GEN_FILE2} ${GEN_FILE3} ${GEN_FILE4} ${GEN_SPATIAL_PROB_ICE})

if(BUILD_SA_COMA)
cast_slice2Py(GEN_COMA_ICE ../../subarchitectures/coma.sa/src/slice/ComaData.ice  ${CAST_SLICE_DIR}/cast/slice ${CAST_SLICE_DIR} 
              ${COGX_ROOT}/subarchitectures/spatial.sa/src/slice)
list(APPEND GENERATED ${GEN_COMA_ICE})
endif(BUILD_SA_COMA)
if(BUILD_SA_CONCEPTUAL)
cast_slice2Py(GEN_SPATIAL_DATA_ICE ../../subarchitectures/spatial.sa/src/slice/SpatialData.ice  ${CAST_SLICE_DIR}/cast/slice ${CAST_SLICE_DIR} 
              ${COGX_ROOT}/subarchitectures/spatial.sa/src/slice  ${COGX_ROOT}/tools/math/src/slice/)
cast_slice2Py(GEN_CONC_ICE ../../subarchitectures/conceptual.sa/src/slice/ConceptualData.ice  ${CAST_SLICE_DIR}/cast/slice ${CAST_SLICE_DIR} 
              ${COGX_ROOT}/subarchitectures/spatial.sa/src/slice ${COGX_ROOT}/tools/math/src/slice/)
list(APPEND GENERATED ${GEN_CONC_ICE} ${GEN_SPATIAL_DATA_ICE})
endif(BUILD_SA_CONCEPTUAL)

add_custom_target(python ALL DEPENDS ${GENERATED})

#file(GLOB_RECURSE moduleFiles *.py)
#file(GLOB_RECURSE moduleFilesRel RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.py)

 install(DIRECTORY ./ DESTINATION ${CMAKE_INSTALL_PREFIX}/python
          FILE_PERMISSIONS OWNER_WRITE OWNER_READ
                           GROUP_WRITE GROUP_READ
          PATTERN ".svn" EXCLUDE
          PATTERN "mapsim" EXCLUDE
)

install(CODE "exec_program(\"${CMAKE_COMMAND}\" ARGS -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_INSTALL_PREFIX}/planner)")



#install(CODE "exec_program(\"${CMAKE_COMMAND}\" ARGS -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_INSTALL_PREFIX}/python)")
#install(CODE "file(MAKE_DIRECTORY  ${CMAKE_INSTALL_PREFIX}/python)")
#install(CODE "execute_process(COMMAND cp -R --parents . ${CMAKE_INSTALL_PREFIX}/python WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})")

