include(Slice2Cpp)

include_directories(.)

#message("SLD:" ${CAST_SLICE_DIR} )
# expects to find .ice file under PROJECT/src/slice e.g. laser/src/slice
# 
# latter args are include paths to serach
cast_slice2cpp(GEN_CPP GEN_HPP DisplayServer.ice ${CAST_SLICE_DIR}
    ${COGX_ROOT}/tools/math/src/slice
    ${COGX_ROOT}/tools/hardware/video/src/slice
    /usr/share/slice
    /usr/share/Ice/slice
    )

#    ${COGX_ROOT}/tools/hardware/stereo/src/slice)

#cast_slice2cpp(GEN_CPP_ML GEN_HPP_ML MatlabData.ice ${CAST_SLICE_DIR}
#    ${COGX_ROOT}/tools/math/src/slice
#    ${COGX_ROOT}/tools/hardware/video/src/slice)
##    ${COGX_ROOT}/tools/hardware/stereo/src/slice)

#cast_slice2cpp(GEN_CPP_OR GEN_HPP_OR ObjectRecognizerSrv.ice ${CAST_SLICE_DIR}
#    ${COGX_ROOT}/tools/math/src/slice
#    ${COGX_ROOT}/tools/hardware/video/src/slice)
##    ${COGX_ROOT}/tools/hardware/stereo/src/slice)

add_library(VisualizationData SHARED ${GEN_CPP} ${GEN_CPP_ML} ${GEN_CPP_OR})
target_link_libraries(VisualizationData CDL)
target_link_libraries(VisualizationData Video Math)
install(TARGETS VisualizationData LIBRARY DESTINATION lib)


