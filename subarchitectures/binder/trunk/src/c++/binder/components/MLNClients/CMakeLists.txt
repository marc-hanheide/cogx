include_directories(.)
include_directories(../..)
include_directories(../../abstract)
include_directories(${COGX_ROOT}/tools/beliefs/apis/beliefs/src/cpp/autogen)
include_directories(${COGX_ROOT}/tools/beliefs/apis/beliefs+cast/src/cpp/autogen)
include_directories(${COGX_ROOT}/subarchitectures/binder.sa/src/c++/binder/autogen)
include_directories(${COGX_ROOT}/tools/cogxutils/src/c++/autogen)
include_directories(${COGX_ROOT}/subarchitectures/dialogue.sa/apis/dialogue/src/c++/autogen)

# MLNBeliefEvdFilter
add_cast_component(MLNBeliefEvdFilter MLNBeliefEvdFilter.cpp)
add_cast_component(MLNRefResolutionClient MLNRefResolutionClient.cpp)
# MLNBeliefEvdFilter
link_cast_component(MLNBeliefEvdFilter BeliefsCAST Binder BeliefsCogX)
link_cast_component(MLNRefResolutionClient BeliefsCAST Binder BeliefsCogX RefSlice)



