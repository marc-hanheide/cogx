# Author: Charles Gretton (charles.gretton@gmail.com)
#
# (this is based on \file{subarchitectures/PCogX/src/c++-0x/CMakeLists.txt})


include_directories(.)
    
include_directories(./include/)

link_directories(${ROOT_OF_PLANNING}/ayPlan/)
include_directories(${ROOT_OF_PLANNING}/include/)
include_directories(${ROOT_OF_PLANNING}/ayPlan/)

add_definitions(-std=c++0x -mno-align-stringops -minline-all-stringops -march=native)

add_definitions(-DPLANNER_FACTORY_DESIGNATION="Factory that produces planners")
add_definitions(-DCLASSICAL_PLANNER_DESIGNATION="Classical planner")
# add_definitions(-DDEBUG_LEVEL=500)

add_definitions(-pedantic -Wall -Wextra)

set(CMAKE_VERBOSE_MAKEFILE true)

########################################################
# BEGIN :: SLICE related CMAKE stuff          :: BEGIN #
########################################################
include(Slice2Cpp)

cast_slice2cpp(GEN_CPP GEN_HPP PCogX.ice)
add_library(PCogX SHARED ${GEN_CPP})
install(TARGETS PCogX LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

########################################################
# END :: SLICE related CMAKE stuff              :: END #
########################################################
    
set(AYPLAN_LIBS ayPlan)
link_libraries(${AYPLAN_LIBS})
    
set(ZLIB_LIBS z)
link_libraries(${ZLIB_LIBS})

add_cast_component(planner_factory planner_factory.cc)

link_cast_component(${CAST_COMPONENT_NAME} PCogX)

add_cast_component(classical_planner classical_planner.cc)

link_cast_component(${CAST_COMPONENT_NAME} PCogX)













    
# We all remember Harvey Atkins -- prior to 2001 he was doing
# the voice for Crag for "The Ripping Friends". Kricfalusi
# didn't want Harvey playing Crag because of his age
# -- birthed in 1942. What a crazy world we live in gramps...
# I haven't checked my facts, but I believe Mark Dailey ended up
# with the role.


    
# include_directories(.)
# include(Slice2Cpp)
    
# include_directories(./include/)

# include_directories(${ROOT_OF_PLANNING}/include/)

# # ICE header for SLICE-based planning objects should be here...
# # I got this idea from some CMake... files I found in the
# # Michael-B sections of the planning SVN entries. 
# include_directories(${ROOT_OF_PLANNING}/subarchitectures/test_harness__PCogX/src/c++-0x/)
    
# add_definitions(-std=c++0x -mno-align-stringops -minline-all-stringops -march=native)

# add_definitions(-pedantic -Wall -Wextra)

# set(CMAKE_VERBOSE_MAKEFILE true)


# cast_slice2cpp(GEN_CPP GEN_HPP PCogX.ice)
# add_library(PCogX SHARED ${GEN_CPP})
# install(TARGETS PCogX LIBRARY DESTINATION lib ARCHIVE DESTINATION lib) 

    
# # add_definitions(-DPLANNER_FACTORY_DESIGNATION="Factory that produces planners")
# # add_definitions(-DCLASSICAL_PLANNER_DESIGNATION="Classical planner")
# # add_definitions(-DDEBUG_LEVEL=400)

# add_definitions(-DPLANNER_FACTORY_DESIGNATION="Factory that produces planners")
# add_definitions(-DPLANNER_CLIENT_DESIGNATION="A client of the planner we use for testing") 
# # add_definitions(-DDEBUG_LEVEL=400)

# add_cast_component(planner_client planner_client.cc)

# link_cast_component(${CAST_COMPONENT_NAME} PCogX)
