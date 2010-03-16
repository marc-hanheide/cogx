cmake_minimum_required (VERSION 2.6)
project (CogX)

get_filename_component(COGX_ROOT . ABSOLUTE)

# set the output directory.
set(OUTPUT ${COGX_ROOT}/output CACHE PATH "Path prefix for installation")
set(CMAKE_INSTALL_PREFIX "${OUTPUT}" CACHE INTERNAL "Prefix prepended to install directories" FORCE)

#add_subdirectory(tools/python)

# use cmake files packaged with CAST as well
set(CMAKE_MODULE_PATH /usr/local/share/cast/cmake ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH /opt/local/share/cast/cmake ${CMAKE_MODULE_PATH})

find_package(CAST REQUIRED)
include(UseCAST)
include(CASTBuild)
include("${COGX_ROOT}/tools/alchemy/alchemy.cmake")

# use cmake files packaged with CAST as well
set(CMAKE_MODULE_PATH "${COGX_ROOT}/cmake;${CAST_CMAKE_DIR}" ${CMAKE_MODULE_PATH})

add_definitions(-Wall)

# UNCOMMENT THE FOLLOWING FOR THE NAVIGATION SUBARCHITECTURE

#add_subdirectory (tools/d-lib)

option(BUILD_SA_BINDING "Build binder subarchitecture" ON)
if(BUILD_SA_BINDING)
add_subdirectory (subarchitectures/binder)
endif(BUILD_SA_BINDING)
      
option(BUILD_ALCHEMY "Build Alchemy package" ON)
if(BUILD_ALCHEMY)
	BUILD_ALCHEMY_MACRO()
endif(BUILD_ALCHEMY)

option(DO_ANT "automatically compile the ant stuff" NO)
if(DO_ANT)
ADD_CUSTOM_TARGET( ant_build ALL )
	ADD_CUSTOM_COMMAND(
		TARGET  ant_build
		COMMAND ant WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
     	)
endif(DO_ANT)
