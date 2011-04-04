cmake_minimum_required (VERSION 2.6)
project (BINDER)

option(BUILD_SA_BINDING_MLNENGINE "build MLN engine" NO)
if (BUILD_SA_BINDING_MLNENGINE)
add_subdirectory (src/c++/binder/components/MLNEngine)
endif (BUILD_SA_BINDING_MLNENGINE)

add_subdirectory(src/c++/binder/autogen)

#a safer way of finding your own stuff
#set(CMAKE_MODULE_PATH  /Users/plison/svn.cogx/binder-dev/tools/cast/cmake/ ${CMAKE_MODULE_PATH})

#include(Slice2Cpp)

# auto generated code
#add_subdirectory (src/c++/autogen)
#add_subdirectory (src/c++)

