
include(UseCAST)
include(Slice2Py)

set(V_SLICE_INCLUDE_DIRS
   ${CAST_SLICE_DIR}
   ${CAST_SLICE_DIR}/cast/slice
   ${CAST_SLICE_DIR}
   ${COGX_ROOT}/tools/math/src/slice
   ${COGX_ROOT}/tools/hardware/video/src/slice
   ${COGX_ROOT}/tools/hardware/laser/src/slice
   ${COGX_ROOT}/tools/hardware/pointCloud/src/slice
   /usr/share/slice
   /usr/local/share
   )

cast_slice2py(SLICE_GEN01 src/slice/VisionData.ice ${V_SLICE_INCLUDE_DIRS})
cast_slice2py(SLICE_GEN02 src/slice/ObjectRecognizerSrv.ice ${V_SLICE_INCLUDE_DIRS})

add_custom_target(VisionSa_SliceGenerated ALL DEPENDS ${SLICE_GEN01} ${SLICE_GEN02})

install(
   DIRECTORY ./ DESTINATION ${PYTHON_INSTALL_SLICEGEN}
   PATTERN ".svn" EXCLUDE
   PATTERN "CMakeLists.txt" EXCLUDE
   PERMISSIONS OWNER_WRITE OWNER_READ GROUP_WRITE GROUP_READ
   )

