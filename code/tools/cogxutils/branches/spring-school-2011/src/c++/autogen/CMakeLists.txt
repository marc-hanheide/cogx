include(Slice2Cpp)

include_directories(.)
include_directories(${COGX_ROOT}/tools/beliefs/apis/beliefs+cast/src/cpp/autogen)
include_directories(${COGX_ROOT}/tools/beliefs/apis/beliefs/src/cpp/autogen)



# expects to find .ice file under PROJECT/src/slice e.g. laser/src/slice
# 
# latter args are include paths to serach
cast_slice2cpp(GEN_CPP GEN_HPP beliefs_cogx.ice ${CAST_SLICE_DIR}
  ${COGX_ROOT}/tools/beliefs/apis/beliefs/src/slice
  ${COGX_ROOT}/tools/beliefs/apis/beliefs+cast/src/slice )

add_library(BeliefsCogX SHARED ${GEN_CPP})
target_link_libraries(BeliefsCogX CDL Beliefs BeliefsCAST)
install(TARGETS BeliefsCogX LIBRARY DESTINATION lib)

