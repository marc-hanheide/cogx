cmake_minimum_required (VERSION 2.6)


set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
set(CMAKE_INSTALL_PREFIX /usr/local CACHE STRING "Prefix prepended to install directories")

project (CAST)




# get current dir
get_filename_component(CAST_ROOT . ABSOLUTE)
get_filename_component(COGX_ROOT . ABSOLUTE)

# use cmake files packaged with CAST
set(CMAKE_MODULE_PATH ${CAST_ROOT}/cmake)

include(${CAST_ROOT}/cmake/CASTDeps.cmake)
include(${CAST_ROOT}/cmake/CASTBuild.cmake)
include(${CAST_ROOT}/cmake/FindCAST.cmake)


# build and install cast
add_subdirectory (src/c++)

# supporting material
add_subdirectory (cmake)
add_subdirectory (config)
add_subdirectory (bin)


include_directories(${CAST_INCLUDE_DIR}/cast/core)
include_directories(${CAST_INCLUDE_DIR}/cast/cdl)
include_directories(${CAST_INCLUDE_DIR}/cast/architecture)
include_directories(${CAST_INCLUDE_DIR}/cast/server)

# vision

add_subdirectory (tools/math)

option(BUILD_HAL_VIDEO "Build hardware video" YES)
if(BUILD_HAL_VIDEO)
add_subdirectory (tools/hardware/video)
endif(BUILD_HAL_VIDEO)

option(BUILD_SA_VISION "Build vision subarchitecure" YES)
if(BUILD_SA_VISION)
add_subdirectory (subarchitectures/vision.sa)
endif(BUILD_SA_VISION)
