cmake_minimum_required (VERSION 2.6)

# ------------ <CCache> ------------
if(USE_CCACHE)
	set(CMAKE_CXX_COMPILER "ccache")
	set(CMAKE_CXX_COMPILER_ARG1 "g++")
	set(CMAKE_C_COMPILER "ccache")
	set(CMAKE_C_COMPILER_ARG1 "gcc")
endif(USE_CCACHE)
# ------------ </CCache> ------------

project (CogX)

#message(${CMAKE_CXX_COMPILER})
get_filename_component(COGX_ROOT . ABSOLUTE)

# set the output directory.
set(OUTPUT ${COGX_ROOT}/output CACHE PATH "Path prefix for installation")
set(CMAKE_INSTALL_PREFIX "${OUTPUT}" CACHE INTERNAL "Prefix prepended to install directories" FORCE)


# use cmake files packaged with CAST as well
set(CMAKE_MODULE_PATH /usr/share/cast/cmake ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH /usr/local/share/cast/cmake ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH /opt/local/share/cast/cmake ${CMAKE_MODULE_PATH})
set(QT_QTOPENGL_INCLUDE_DIR  /usr/include/qt4/QtOpenGL)

find_package(CAST REQUIRED)
include(UseCAST)
include(CASTBuild)
include(cmake/PythonBuild.cmake)

# use cmake files packaged with CAST as well
set(CMAKE_MODULE_PATH "${COGX_ROOT}/cmake;${CAST_CMAKE_DIR}" ${CMAKE_MODULE_PATH})

add_definitions(-Wall)

include(subarchitectures/visualization.sa/cmake/v11n.cmake)

add_subdirectory (tools/d-lib)



option(BUILD_DORA "Build dora specific code" YES)
if(BUILD_DORA)
add_subdirectory (scenarios/dora)
endif(BUILD_DORA)


option(BUILD_SCENARIO_SHARED "Build shared scenario code" YES)
if(BUILD_SCENARIO_SHARED)
add_subdirectory (scenarios/shared)
endif(BUILD_SCENARIO_SHARED)


option(BUILD_BELIEFS "Build beliefs" YES)
if(BUILD_BELIEFS)
add_subdirectory (tools/beliefs/apis/beliefs)
add_subdirectory (tools/beliefs/apis/beliefs+cast)
add_subdirectory (tools/cogxutils/)
endif(BUILD_BELIEFS)

option(BUILD_SA_BINDING "Build binder subarchitecure" YES)
if(BUILD_SA_BINDING)
add_subdirectory (subarchitectures/binder.sa)
endif(BUILD_SA_BINDING)

option(BUILD_SA_MOTIVATION "Build motivation subarchitecure" YES)
if(BUILD_SA_MOTIVATION)
add_subdirectory (subarchitectures/motivation.sa)
endif(BUILD_SA_MOTIVATION)


option(BUILD_SA_EXECUTION "Build execution subarchitecure" YES)
if(BUILD_SA_EXECUTION)
add_subdirectory (subarchitectures/execution.sa)
endif(BUILD_SA_EXECUTION)



option(BUILD_SA_COMA "Build sa_coma specific code" YES)
if(BUILD_SA_COMA)
add_subdirectory (subarchitectures/coma.sa)
endif(BUILD_SA_COMA)


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


option(BUILD_SA_VISION "Build vision subarchitecure" YES)
if(BUILD_SA_VISION)
add_subdirectory (subarchitectures/vision.sa)
endif(BUILD_SA_VISION)

# Spatial
option(BUILD_SA_SPATIAL "Build spatial subarchitecure" YES)
if(BUILD_SA_SPATIAL)
    add_subdirectory (subarchitectures/spatial.sa)
endif(BUILD_SA_SPATIAL)


# Planner
option(BUILD_SA_PLANNER "Build planner subarchitecure" YES)
if(BUILD_SA_PLANNER)
    add_subdirectory (subarchitectures/planner.sa)
endif(BUILD_SA_PLANNER)


add_subdirectory(tools/scripts)

# Find ICE_SLICE_DIR by searching for Ice/Identity.ice
#message(" ice slice: " ${ICE_SLICE_DIR})
find_file(SLICE_IDENTITY Ice/Identity.ice
   PATHS /usr/share/slice /usr/share/Ice/slice)
if(SLICE_IDENTITY)
   string(REGEX REPLACE "/Ice/Identity\\.ice$" "" ICE_SLICE_DIR ${SLICE_IDENTITY})
endif(SLICE_IDENTITY)
#message(" ice slice: " ${ICE_SLICE_DIR})

configure_file(${CMAKE_SOURCE_DIR}/cmake/build.properties.in ${CMAKE_SOURCE_DIR}/build.properties)

option(DO_ANT "automatically compile the ant stuff" YES)
if(DO_ANT)
ADD_CUSTOM_TARGET( ant_build ALL )
	ADD_CUSTOM_COMMAND(
		TARGET  ant_build
		COMMAND ant -Dcastslice.dir=${CAST_SLICE_DIR} -Dcast.jar=${CAST_INSTALL_ROOT}/share/java/cast.jar -Dice.jar=${ICE_HOME}/share/java/Ice.jar WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
     	)
endif(DO_ANT)

