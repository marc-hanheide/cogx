# Author: Charles Gretton (charles.gretton@gmail.com)
#
# (this is based on \file{subarchitectures/PCogX/src/c++-0x/CMakeLists.txt})
    
include_directories(.)
    
include_directories(./include/)

include_directories(${ROOT_OF_PLANNING}/include/)

# ICE header for SLICE-based planning objects should be here...
# I got this idea from some CMake... files I found in the
# Michael-B sections of the planning SVN entries. 
include_directories(${ROOT_OF_PLANNING}/subarchitectures/PCogX/src/c++-0x/)
    
add_definitions(-std=c++0x -mno-align-stringops -minline-all-stringops -march=native)

add_definitions(-pedantic -Wall -Wextra)

set(CMAKE_VERBOSE_MAKEFILE true)

add_definitions(-DPLANNER_FACTORY_DESIGNATION"Factory that produces planners.")
add_definitions(-DPLANNER_CLIENT_DESIGNATION"A client of the planner; used for testing.")
add_definitions(-DDEBUG_LEVEL=199)

add_cast_component(planner_client planner_client.cc)

link_cast_component(${CAST_COMPONENT_NAME} PCogX)
