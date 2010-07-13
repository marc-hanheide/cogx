
include(Slice2Py)

cast_slice2py(SLICE_GEN01
   ../slice/DisplayServer.ice # The strange path is a hack to get arround cast_slice2py "smart" paths
   # include dirs
   ${CAST_SLICE_DIR}/cast/slice
   ${COGX_ROOT}/tools/math/src/slice
   ${COGX_ROOT}/tools/hardware/video/src/slice
   /usr/share/slice
   /usr/local/share
   )

add_custom_target(Slice2Py_Generated ALL DEPENDS ${SLICE_GEN01})

#install($(){SLICE_GEN01}
install(
   DIRECTORY ./ DESTINATION ${CMAKE_INSTALL_PREFIX}/python/${PYTHON_SLICEGEN_DIR}
   PATTERN ".svn" EXCLUDE
   PATTERN "CMakeLists.txt" EXCLUDE
   PERMISSIONS OWNER_WRITE OWNER_READ GROUP_WRITE GROUP_READ
   )

