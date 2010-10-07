project(MATH)

# use cmake files packaged with CAST

include(CASTDeps)
include(CASTBuild)

include_directories(${COGX_ROOT}/tools/math/src/c++/autogen)

# auto generated code
add_subdirectory(src/c++/autogen)

add_subdirectory(src/c++/math)

if(BUILD_PYTHON_COMPONENTS)
   add_subdirectory(src/python/slicegen)
endif(BUILD_PYTHON_COMPONENTS)
