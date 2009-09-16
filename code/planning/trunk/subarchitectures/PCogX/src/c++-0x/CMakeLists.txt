# Author: Charles Gretton (charles.gretton@gmail.com)
#  
# An espontaneo jumps into the fold, later declaring
# in an anguishing pang of languishing attention:
# "Give me autotools, and take away this pain doc..."


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
