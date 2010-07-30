cmake_minimum_required (VERSION 2.6)
project (NAVSA)

include_directories(${COGX_ROOT}/tools/d-lib/src/c++/d)
include_directories(${COGX_ROOT}/tools/math/src/c++/math)
include_directories(${COGX_ROOT}/tools/math/src/c++/autogen)
# make a "use" cmake include too
include_directories(${CURE_INSTALLROOT}/include/cure)

# auto generated code
add_subdirectory (src/c++/autogen)

		 

add_subdirectory (src/c++/components)

