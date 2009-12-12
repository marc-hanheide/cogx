# Author: Charles Gretton (charles.gretton@gmail.com)
#
    
include_directories(.)
include(Slice2Cpp)

include_directories(./include/)

include_directories(${ROOT_OF_PLANNING}/include/)

# ICE header for SLICE-based planning objects should be here...
# I got this idea from some CMake... files I found in the
# Michael-B sections of the planning SVN entries. 
    
add_definitions(-std=c++0x -mno-align-stringops -minline-all-stringops -march=native)

add_definitions(-pedantic -vg -Wall -Wextra)

set(CMAKE_VERBOSE_MAKEFILE true)


cast_slice2cpp(GEN_CPP GEN_HPP testHarnessCASTRPC.ice)
add_library(testHarnessCASTRPC SHARED ${GEN_CPP})
install(TARGETS testHarnessCASTRPC LIBRARY DESTINATION lib ARCHIVE DESTINATION lib) 

    
# add_definitions(-DPLANNER_FACTORY_DESIGNATION="Factory that produces planners")
# add_definitions(-DCLASSICAL_PLANNER_DESIGNATION="Classical planner")
# add_definitions(-DDEBUG_LEVEL=400)

add_definitions(-DDEBUG_LEVEL=2000)
    
add_definitions(-DRPC_WARNINGS=0)

add_cast_component(test_harness__CAST_RPC_client test_harness__CAST_RPC_client.cc)

link_cast_component(${CAST_COMPONENT_NAME} testHarnessCASTRPC)

add_cast_component(test_harness__CAST_RPC_client_clone1 test_harness__CAST_RPC_client_clone1.cc)

link_cast_component(${CAST_COMPONENT_NAME} testHarnessCASTRPC)

add_cast_component(test_harness__CAST_RPC_client_clone2 test_harness__CAST_RPC_client_clone2.cc)

link_cast_component(${CAST_COMPONENT_NAME} testHarnessCASTRPC)

add_cast_component(test_harness__CAST_RPC_server test_harness__CAST_RPC_server.cc)

link_cast_component(${CAST_COMPONENT_NAME} testHarnessCASTRPC)

add_cast_component(test_harness__CAST_RPC_server_clone1 test_harness__CAST_RPC_server_clone1.cc)

link_cast_component(${CAST_COMPONENT_NAME} testHarnessCASTRPC)
