cmake_minimum_required (VERSION 2.6)


set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel.")
set(CMAKE_INSTALL_PREFIX /usr/local CACHE STRING "Prefix prepended to install directories")

project (CAST)




# get current dir
get_filename_component(CAST_ROOT . ABSOLUTE)

# use cmake files packaged with CAST
set(CMAKE_MODULE_PATH ${CAST_ROOT}/cmake)

include(${CAST_ROOT}/cmake/CASTDeps.cmake)
include(${CAST_ROOT}/cmake/CASTBuild.cmake)


# build and install cast
add_subdirectory (src/c++)

# supporting material
add_subdirectory (cmake)
add_subdirectory (config)
add_subdirectory (bin)


