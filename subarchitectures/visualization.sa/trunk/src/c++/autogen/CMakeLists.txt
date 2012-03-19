include(Slice2Cpp)

include_directories(.)

#message("SLD:" ${CAST_SLICE_DIR} )
# expects to find .ice file under PROJECT/src/slice e.g. laser/src/slice
# 
# latter args are include paths to serach
cast_slice2cpp(GEN_CPP GEN_HPP DisplayServer.ice ${CAST_SLICE_DIR}
    ${ICE_SLICE_DIR}
    ${ICE_HOME}/share/Ice/slice
    ${ICE_HOME}/share/slice
    /opt/local/share/Ice/slice
    /opt/local/share/slice
    /opt/local/share
    )


add_library(VisualizationData SHARED ${GEN_CPP} ${GEN_CPP_ML} ${GEN_CPP_OR})
target_link_libraries(VisualizationData CDL)
install(TARGETS VisualizationData LIBRARY DESTINATION lib)


