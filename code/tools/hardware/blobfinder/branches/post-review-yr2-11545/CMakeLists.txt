cmake_minimum_required (VERSION 2.6)
project (BlobFinder)

include_directories(${COGX_ROOT}/tools/math/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/math/src/c++/math)

# auto generated code
add_subdirectory (src/c++/autogen)

add_subdirectory (src/c++/components)

