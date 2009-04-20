cmake_minimum_required (VERSION 2.6)
project (LASER)

# use cmake files packaged with CogX and CAST
set(CMAKE_MODULE_PATH "${CAST_ROOT}/cmake;${COGX_ROOT}/cmake")
include(${CAST_ROOT}/cmake/UseCAST.cmake)


include_directories(${COGX_ROOT}/tools/d-lib/src/c++/d)

# make a "use" cmake include too
include_directories(${CURE_INSTALLROOT}/include/cure)

# auto generated code
add_subdirectory (src/c++/autogen)

add_subdirectory (src/c++/components)

