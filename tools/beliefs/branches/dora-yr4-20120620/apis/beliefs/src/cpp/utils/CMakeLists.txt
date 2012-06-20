include_directories(.)
#include_directories(${COGX_ROOT}/tools/beliefs/apis/beliefs/src/cpp/autogen)
include_directories(../autogen)

set(sources Belief.cpp)

set(headers Belief.hpp)

add_library(BeliefUtils SHARED ${sources} ${headers})

target_link_libraries(BeliefUtils Beliefs)

install(TARGETS BeliefUtils LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
