project(MATLAB)

# use cmake files packaged with CAST

include(CASTDeps)
include(CASTBuild)

include_directories(${COGX_ROOT}/tools/matlab/src/c++/autogen)

find_package(Matlab)

# auto generated code
add_subdirectory(src/c++/autogen)

#if (MATLAB_FOUND)
#	add_subdirectory(src/c++/mlhelper)
#endif (MATLAB_FOUND)

