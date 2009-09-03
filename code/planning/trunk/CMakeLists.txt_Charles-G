cmake_minimum_required (VERSION 2.6)

project (PCogX)

get_filename_component(PCogX_ROOT . ABSOLUTE)

# YOU EDIT HERE
set(ROOT_OF_PLANNING /data/private/grettonc/CogX/SVN/cogx/code/planning/trunk/)

# YOU EDIT HERE
set(ROOT_OF_RANDOM_CAST_CMAKE_STUFF /data/private/grettonc/share/cast/)

# set the output directory.
set(OUTPUT ${PCogX_ROOT}/output CACHE PATH "Path prefix for installation")
set(CMAKE_INSTALL_PREFIX "${OUTPUT}" CACHE INTERNAL "Prefix prepended to install directories" FORCE)

# use cmake files packaged with CAST as well, add your install prefix here too
set(CMAKE_MODULE_PATH ${ROOT_OF_RANDOM_CAST_CMAKE_STUFF}/cmake/ ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH ${ROOT_OF_RANDOM_CAST_CMAKE_STUFF}/cmake/ ${CMAKE_MODULE_PATH})

find_package(CAST REQUIRED)
include(UseCAST)
include(CASTBuild)

add_definitions(-Wall)

# Include your subarchitecture projects in the following way
# add_subdirectory (subarchitectures/hello-world)

add_subdirectory (subarchitectures/PCogX)
    
add_subdirectory (subarchitectures/test_harness__PCogX)

# add_subdirectory (subarchitectures/dt-planning.sa)
