include(Slice2Cpp)

include_directories(.)
include_directories(${COGX_ROOT}/tools/beliefs/apis/beliefs/src/cpp/autogen)



# expects to find .ice file under PROJECT/src/slice e.g. laser/src/slice
# 
# latter args are include paths to serach
cast_slice2cpp(GEN_CPP GEN_HPP beliefs_cast.ice ${COGX_ROOT}/tools/beliefs/apis/beliefs/src/slice ${CAST_SLICE_DIR})

add_library(BeliefsCAST SHARED ${GEN_CPP})
target_link_libraries(BeliefsCAST CDL Beliefs)
install(TARGETS BeliefsCAST LIBRARY DESTINATION lib)

