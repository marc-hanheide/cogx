cmake_minimum_required (VERSION 2.6)
project (HWTESTS)

include_directories(${COGX_ROOT}/tools/d-lib/src/c++/d)

include_directories(${COGX_ROOT}/tools/hardware/laser/src/c++/components/)
include_directories(${COGX_ROOT}/tools/hardware/laser/src/c++/autogen/)
include_directories(${COGX_ROOT}/tools/hardware/robotbase/src/c++/components/)
include_directories(${COGX_ROOT}/tools/hardware/robotbase/src/c++/autogen/)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/components/)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/autogen/)

# make a "use" cmake include too
include_directories(${CURE_INSTALLROOT}/include/cure)

add_subdirectory (src/c++/components)

