cmake_minimum_required (VERSION 2.6)
project (CogX)
	
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

# use cmake files packaged with CAST as well
set(CMAKE_MODULE_PATH "${COGX_ROOT}/cmake;${CAST_CMAKE_DIR}" ${CMAKE_MODULE_PATH})

add_definitions(-Wall)

add_subdirectory (tools/d-lib)

include(cmake/PythonBuild.cmake) 

option(BUILD_BELIEFS "Build beliefs" YES)
if(BUILD_BELIEFS)
add_subdirectory (tools/beliefs/apis/beliefs)
add_subdirectory (tools/beliefs/apis/beliefs+cast)
add_subdirectory (tools/cogxutils/)
endif(BUILD_BELIEFS)

option(BUILD_SA_VIZ "Build visualization subarchitecure" NO)
if(BUILD_SA_VIZ)
include(subarchitectures/visualization.sa/cmake/v11n.cmake) 
endif(BUILD_SA_VIZ)

option(BUILD_SA_BINDING "Build binder subarchitecure" NO)
if(BUILD_SA_BINDING)
add_subdirectory (subarchitectures/binder)
endif(BUILD_SA_BINDING)

option(BUILD_SA_MOTIVATION "Build motivation subarchitecure" NO)
if(BUILD_SA_MOTIVATION)
add_subdirectory (subarchitectures/motivation.sa)
endif(BUILD_SA_MOTIVATION)

option(BUILD_SA_DIALOGUE "Build dialogue subarchitecure" NO)
if(BUILD_SA_DIALOGUE)
add_subdirectory (subarchitectures/dialogue.sa)
endif(BUILD_SA_DIALOGUE)


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

option(BUILD_SA_VISION "Build vision subarchitecure" NO)
if(BUILD_SA_VISION)
add_subdirectory (subarchitectures/vision.sa)
endif(BUILD_SA_VISION)


# Spatial
option(BUILD_SA_SPATIAL "Build spatial subarchitecure" NO)
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
option(BUILD_SA_FAKE_MOTIVATION "Build fake motivation subarchitecure" NO)
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


configure_file(${CMAKE_SOURCE_DIR}/cmake/build.properties.in ${CMAKE_SOURCE_DIR}/build.properties)