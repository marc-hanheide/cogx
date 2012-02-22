project (Manipulation)

# use cmake files packaged with CAST
include(UseCAST)

include_directories(${COGX_ROOT}/tools/math/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/math/src/c++/math)

include_directories(src/c++)
include_directories(src/c++/autogen)

# add a subdirectory containing another CMakeLists.txt file
add_subdirectory(src/c++/autogen)
if (DO_SLICE_ONLY)
   return()
endif()

option(BUILD_SA_MANIPULATION_PLAYERARMBRIDGE "build bridge component between Golem and player" NO)
if(BUILD_SA_MANIPULATION_PLAYERARMBRIDGE)
  add_subdirectory (src/c++/components/PlayerArmBridge)
endif(BUILD_SA_MANIPULATION_PLAYERARMBRIDGE)

option(BUILD_SA_MANIPULATION_GAZEBO_TURNTABLE "build gazebo turn table for learning of simulated objects" NO)
if(BUILD_SA_MANIPULATION_GAZEBO_TURNTABLE)
  add_subdirectory (src/c++/components/GazeboTurntable)
endif(BUILD_SA_MANIPULATION_GAZEBO_TURNTABLE)

option(BUILD_SA_MANIPULATION_ARMMANAGER "build arm manager component" NO)
if(BUILD_SA_MANIPULATION_ARMMANAGER)
  add_subdirectory (src/c++/autogen/execution)
  add_subdirectory (src/c++/components/ArmManager)
endif(BUILD_SA_MANIPULATION_ARMMANAGER)

option(BUILD_SA_MANIPULATION_POINTINGTEST "build pointing test component" NO)
if(BUILD_SA_MANIPULATION_POINTINGTEST)
  add_subdirectory (src/c++/components/PointingTest)
endif(BUILD_SA_MANIPULATION_POINTINGTEST)
