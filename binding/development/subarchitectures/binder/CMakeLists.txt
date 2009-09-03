
cmake_minimum_required (VERSION 2.6)
project (BINDER)

#include(/Users/plison/svn.cogx/binder-dev/tools/cast/cmake/CASTDeps.cmake)
include(/Users/plison/svn.cogx/binder-dev/tools/cast/cmake/Slice2Cpp.cmake)

# auto generated code
add_subdirectory (src/c++/autogen)
add_subdirectory (src/c++)

