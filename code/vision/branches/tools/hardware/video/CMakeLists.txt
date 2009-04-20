project (Video)

# use cmake files packaged with CogX and CAST
set(CMAKE_MODULE_PATH "${CAST_ROOT}/cmake;${COGX_ROOT}/cmake")
include(${CAST_ROOT}/cmake/FindCAST.cmake)
include(${CAST_ROOT}/cmake/UseCAST.cmake)

include_directories(${COGX_ROOT}/tools/math/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/math/src/c++/math)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/utils)

include_directories(${CAST_ROOT}/include/cast/cdl)

# auto generated code
add_subdirectory(src/c++/autogen)

add_subdirectory(src/c++/utils)
add_subdirectory(src/c++/components)
add_subdirectory(src/c++/test)

