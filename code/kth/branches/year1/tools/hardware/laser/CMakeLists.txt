cmake_minimum_required (VERSION 2.6)
project (LASER)

include_directories(${COGX_ROOT}/tools/d-lib/src/c++/d)

# make a "use" cmake include too
include_directories(${CURE_INSTALLROOT}/include/cure)

# auto generated code
add_subdirectory (src/c++/autogen)

add_subdirectory (src/c++/components)

