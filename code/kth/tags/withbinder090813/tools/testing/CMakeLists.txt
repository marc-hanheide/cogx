project(MULTISATEST)

# use cmake files packaged with CAST

include(CASTDeps)
include(CASTBuild)

include_directories(${COGX_ROOT}/tools/testing/src/c++/autogen)

# auto generated code
add_subdirectory(src/c++/autogen)

add_subdirectory(src/c++/)


