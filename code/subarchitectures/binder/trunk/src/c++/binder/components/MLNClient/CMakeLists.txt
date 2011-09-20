include_directories(.)
include_directories(../..)
include_directories(../../abstract)
include_directories(${COGX_ROOT}/tools/beliefs/apis/beliefs/src/cpp/autogen)
include_directories(${COGX_ROOT}/tools/beliefs/apis/beliefs+cast/src/cpp/autogen)
include_directories(${COGX_ROOT}/subarchitectures/binder.sa/src/c++/binder/autogen)
include_directories(${COGX_ROOT}/tools/cogxutils/src/c++/autogen)

# Set the sources.
SET(SOURCES MLNClient.cpp)

add_cast_component(MLNClient ${SOURCES})

link_cast_component(${CAST_COMPONENT_NAME}  BeliefsCAST Binder BeliefsCogX)



