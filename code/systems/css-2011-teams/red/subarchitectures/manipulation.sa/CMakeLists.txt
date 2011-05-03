project (Manipulation)

# use cmake files packaged with CAST
include(UseCAST)

include_directories(${COGX_ROOT}/tools/math/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/math/src/c++/math)

include_directories(src/c++)
include_directories(src/c++/autogen)

# add a subdirectory containing another CMakeLists.txt file
add_subdirectory(src/c++/autogen)

option(BUILD_SA_MANIPULATION_PLAYERARMBRIDGE "build bridge component between Golem and player" NO)
if(BUILD_SA_MANIPULATION_PLAYERARMBRIDGE)
  add_subdirectory (src/c++/components/PlayerArmBridge)
endif(BUILD_SA_MANIPULATION_PLAYERARMBRIDGE)

option(BUILD_SA_MANIPULATION_GAZEBO_TURNTABLE "build gazebo turn table for learning of simulated objects" NO)
if(BUILD_SA_MANIPULATION_GAZEBO_TURNTABLE)
  add_subdirectory (src/c++/components/GazeboTurntable)
endif(BUILD_SA_MANIPULATION_GAZEBO_TURNTABLE)

option(BUILD_TEAM_RED "build team red manipulation planner and tools" YES)
if(BUILD_TEAM_RED)
  add_subdirectory (src/c++/components/ManipulationPlanner)
endif(BUILD_TEAM_RED)

option(BUILD_SS_MANIPULATION "build spring school manipulation system" YES)
if(BUILD_SS_MANIPULATION)
  add_subdirectory (src/c++/components/SSManipulation)
endif(BUILD_SS_MANIPULATION)

