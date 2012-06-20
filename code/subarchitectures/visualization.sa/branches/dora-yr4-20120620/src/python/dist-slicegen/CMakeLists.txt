
include(Slice2Py)

set(V_SLICE_INCLUDE_DIRS
   ${CAST_SLICE_DIR}
   ${ICE_SLICE_DIR}

   ${CAST_SLICE_DIR}/cast/slice
   ${COGX_ROOT}/tools/math/src/slice
   ${COGX_ROOT}/tools/hardware/video/src/slice
   /usr/share/slice
   /usr/share/Ice/slice
   /usr/local/share
   /opt/local/share
   /opt/local/share/slice
   /opt/local/share/slice/Ice/
   )

cast_slice2py(SLICE_GEN01 ../slice/DisplayServer.ice ${V_SLICE_INCLUDE_DIRS})

add_custom_target(Visualization_SliceGenerated ALL DEPENDS ${SLICE_GEN01})

install(
   DIRECTORY . DESTINATION  ${PYTHON_INSTALL_SLICEGEN}
   PATTERN ".svn" EXCLUDE
   PATTERN "CMakeLists.txt" EXCLUDE
   PERMISSIONS OWNER_WRITE OWNER_READ GROUP_WRITE GROUP_READ
   )

