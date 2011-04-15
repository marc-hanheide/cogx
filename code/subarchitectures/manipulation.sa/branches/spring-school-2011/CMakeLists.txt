project (Manipulation)

# use cmake files packaged with CAST
include(UseCAST)

include_directories(src/c++)
include_directories(src/c++/autogen)

# add a subdirectory containing another CMakeLists.txt file
add_subdirectory(src/c++/autogen)

include_directories(${COGX_ROOT}/tools/math/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/math/src/c++/math)
