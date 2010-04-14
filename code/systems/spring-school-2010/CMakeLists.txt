cmake_minimum_required (VERSION 2.6)
project (CogX)

get_filename_component(COGX_ROOT . ABSOLUTE)

# set the output directory.
set(OUTPUT ${COGX_ROOT}/output CACHE PATH "Path prefix for installation")
set(CMAKE_INSTALL_PREFIX "${OUTPUT}" CACHE INTERNAL "Prefix prepended to install directories" FORCE)


# use cmake files packaged with CAST as well
set(CMAKE_MODULE_PATH /usr/local/share/cast/cmake ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH /opt/local/share/cast/cmake ${CMAKE_MODULE_PATH})

find_package(CAST REQUIRED)
include(UseCAST)
include(CASTBuild)


#Adding alchemy to the compilation
#include("${COGX_ROOT}/tools/alchemy/alchemy.cmake")


# use cmake files packaged with CAST as well
set(CMAKE_MODULE_PATH "${COGX_ROOT}/cmake;${CAST_CMAKE_DIR}" ${CMAKE_MODULE_PATH})

add_definitions(-Wall)

# UNCOMMENT THE FOLLOWING FOR THE NAVIGATION SUBARCHITECTURE

add_subdirectory (tools/d-lib)

# option(BUILD_SA_BINDING "Build binder subarchitecure" YES)
# if(BUILD_SA_BINDING)
# add_subdirectory (subarchitectures/binder)
# endif(BUILD_SA_BINDING)

option(BUILD_BELIEFMODELS "Build beliefmodels" YES)
if(BUILD_BELIEFMODELS)
add_subdirectory (tools/beliefs)
endif(BUILD_BELIEFMODELS)

option(BUILD_ALCHEMY "Build Alchemy package" ON)
if(BUILD_ALCHEMY)
  add_subdirectory (tools/alchemy)
#	BUILD_ALCHEMY_MACRO()
endif(BUILD_ALCHEMY)


option(BUILD_SA_MOTIVATION "Build motivation subarchitecure" NO)
if(BUILD_SA_MOTIVATION)
add_subdirectory (subarchitectures/motivation.sa)
endif(BUILD_SA_MOTIVATION)


option(BUILD_SA_NAV "Build navigation subarchitecure" YES)
if(BUILD_SA_NAV)
add_subdirectory (subarchitectures/nav.sa)
endif(BUILD_SA_NAV)


option(BUILD_HAL_ROBOT "Build hardware robotbase" YES)
if(BUILD_HAL_ROBOT)
add_subdirectory (tools/hardware/robotbase)
endif(BUILD_HAL_ROBOT)

option(BUILD_HAL_LASER "Build hardware laser" YES)
if(BUILD_HAL_LASER)
add_subdirectory (tools/hardware/laser)
endif(BUILD_HAL_LASER)


option(BUILD_HAL_TTS "Build hardware tts" NO)
if(BUILD_HAL_TTS)
add_subdirectory (tools/hardware/tts)
endif(BUILD_HAL_TTS)

option(BUILD_HAL_PTZ "Build hardware ptz" YES)
if(BUILD_HAL_PTZ)
add_subdirectory (tools/hardware/ptz)
endif(BUILD_HAL_PTZ)

option(BUILD_HAL_BLOBFINDER "Build hardware blobfinder" YES)
if(BUILD_HAL_BLOBFINDER)
add_subdirectory (tools/hardware/blobfinder)
endif(BUILD_HAL_BLOBFINDER)

option(BUILD_HAL_UTILS "Build hardware utils" YES)
if(BUILD_HAL_UTILS)
add_subdirectory (tools/hardware/utils)
endif(BUILD_HAL_UTILS)

option(BUILD_HAL_TESTS "Build hardware tests" NO)
if(BUILD_HAL_TESTS)
add_subdirectory (tools/hardware/tests)
endif(BUILD_HAL_TESTS)


# UNCOMMENT THE FOLLOWING FOR THE VISION SUBARCHITECTURE


add_subdirectory (tools/math)

option(BUILD_HAL_VIDEO "Build hardware video" YES)
if(BUILD_HAL_VIDEO)
add_subdirectory (tools/hardware/video)
endif(BUILD_HAL_VIDEO)

option(BUILD_HAL_STEREO "Build hardware stereo" NO)
if(BUILD_HAL_STEREO)
add_subdirectory (tools/hardware/stereo)
endif(BUILD_HAL_STEREO)

option(BUILD_SA_VISION "Build vision subarchitecure" YES)
if(BUILD_SA_VISION)
add_subdirectory (subarchitectures/vision.sa)
endif(BUILD_SA_VISION)


# Spatial
option(BUILD_SA_SPATIAL "Build spatial subarchitecure" YES)
if(BUILD_SA_SPATIAL)
    add_subdirectory (subarchitectures/spatial.sa)
endif(BUILD_SA_SPATIAL)

option(BUILD_MULTI_TESTER "Build multi-sa tester" NO)
if(BUILD_MULTI_TESTER)
    add_subdirectory (tools/testing)
endif(BUILD_MULTI_TESTER)

# Planner
option(BUILD_SA_PLANNER "Build planner subarchitecure" NO)
if(BUILD_SA_PLANNER)
    add_subdirectory (subarchitectures/planner.sa)
endif(BUILD_SA_PLANNER)

# Fake motivation
option(BUILD_SA_FAKE_MOTIVATION "Build fake motivation subarchitecure" YES)
if(BUILD_SA_FAKE_MOTIVATION)
    add_subdirectory (subarchitectures/fakemotivation.sa)
endif(BUILD_SA_FAKE_MOTIVATION)

add_subdirectory(tools/scripts)

option(DO_ANT "automatically compile the ant stuff" YES)
if(DO_ANT)
ADD_CUSTOM_TARGET( ant_build ALL )
	ADD_CUSTOM_COMMAND(
		TARGET  ant_build
		COMMAND ant -Dcastslice.dir=${CAST_SLICE_DIR} -Dcast.jar=${CAST_INSTALL_ROOT}/share/java/cast.jar -Dice.jar=${ICE_HOME}/share/java/Ice.jar WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
     	)
endif(DO_ANT)


