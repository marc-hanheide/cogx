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

