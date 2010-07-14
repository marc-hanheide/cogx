
project (PythonMath)
include(UseCAST)
include(Slice2Py)

cast_slice2py(SLICE_GEN01
   ../../slice/Math.ice
   # include dirs
   ${CAST_SLICE_DIR}/cast/slice
   ${COGX_ROOT}/tools/math/src/slice
   ${COGX_ROOT}/tools/hardware/video/src/slice
   /usr/share/slice
   /usr/local/share
   )

add_custom_target(Math_SliceGenerated ALL DEPENDS ${SLICE_GEN01})

install(
   DIRECTORY ./ DESTINATION ${PYTHON_INSTALL_SLICEGEN}
   PATTERN ".svn" EXCLUDE
   PATTERN "CMakeLists.txt" EXCLUDE
   PERMISSIONS OWNER_WRITE OWNER_READ GROUP_WRITE GROUP_READ
   )

