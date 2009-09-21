project (Video)

include_directories(${COGX_ROOT}/tools/math/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/math/src/c++/math)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/utils)

# auto generated code
add_subdirectory(src/c++/autogen)

add_subdirectory(src/c++/utils)
add_subdirectory(src/c++/components)
add_subdirectory(src/c++/test)

