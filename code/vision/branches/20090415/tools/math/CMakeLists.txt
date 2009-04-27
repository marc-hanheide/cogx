project(MATH)

# use cmake files packaged with CAST
set(CMAKE_MODULE_PATH "${CAST_ROOT}/cmake;${COGX_ROOT}/cmake")

include(${CAST_ROOT}/cmake/CASTDeps.cmake)
include(${CAST_ROOT}/cmake/CASTBuild.cmake)

include_directories(${COGX_ROOT}/tools/math/src/c++/autogen)

# auto generated code
add_subdirectory(src/c++/autogen)
add_subdirectory(src/c++/math)

