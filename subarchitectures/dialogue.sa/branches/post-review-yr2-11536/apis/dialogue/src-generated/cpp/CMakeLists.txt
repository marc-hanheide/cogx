include(Slice2Cpp)

#message("SLD:" ${CAST_SLICE_DIR} )
# expects to find .ice file under PROJECT/src/slice e.g. laser/src/slice
# 
# latter args are include paths to serach

cast_slice2cpp(GEN_CPP_LF GEN_HPP_LF lf.ice ${CAST_SLICE_DIR})

cast_slice2cpp(GEN_CPP GEN_HPP dialogue.ice ${CAST_SLICE_DIR}
    ${COGX_ROOT}/tools/beliefs/apis/beliefs/src/slice
    ${ICE_SLICE_DIR}
    ${ICE_HOME}/share/Ice/slice
    ${ICE_HOME}/share/slice
    /opt/local/share/Ice/slice
    /opt/local/share/slice
    /opt/local/share
    )

include_directories(.)
include_directories(
   ${COGX_ROOT}/tools/beliefs/apis/beliefs/src/cpp/autogen
   )
link_directories(
   ${COGX_ROOT}/output/lib
   )

add_library(DialogueData SHARED ${GEN_CPP} ${GEN_CPP_LF})
target_link_libraries(DialogueData CDL)
target_link_libraries(DialogueData Beliefs)
install(TARGETS DialogueData LIBRARY DESTINATION lib)


