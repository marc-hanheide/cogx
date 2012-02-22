project (VisualisationSa)

include(UseCAST)

include_directories(${COGX_ROOT}/tools/math/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/math/src/c++/math)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/utils)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/components)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/stereo/src/c++/components)
include_directories(${COGX_ROOT}/tools/hardware/stereo/src/c++/autogen)

# auto generated code
include_directories(src/c++/autogen)
include_directories(src/c++/core)

add_subdirectory(src/c++/autogen)
if (DO_SLICE_ONLY)
   return()
endif()

if(BUILD_PYTHON_COMPONENTS)
   add_subdirectory(src/python)
endif(BUILD_PYTHON_COMPONENTS)

option(BUILD_SA_V11N_AUTOGEN_ONLY "ONLY build autogen parts of V11N" OFF)
if(BUILD_SA_V11N_AUTOGEN_ONLY)
   return()
endif()

add_subdirectory(src/c++/core)
add_subdirectory(src/c++/clients)

