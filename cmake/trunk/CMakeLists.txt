cmake_minimum_required (VERSION 2.6)

#
# If the user specifies -DCMAKE_BUILD_TYPE on the command line, take their definition
# and dump it in the cache along with proper documentation, otherwise set CMAKE_BUILD_TYPE
# to Debug prior to calling PROJECT()
#
IF(DEFINED CMAKE_BUILD_TYPE)
   SET(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." FORCE)
ELSE()
   SET(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." FORCE)
ENDIF()

# ------------ <CCache> ------------
if(USE_CCACHE)
   set(CMAKE_CXX_COMPILER "ccache")
   set(CMAKE_CXX_COMPILER_ARG1 "g++")
   set(CMAKE_C_COMPILER "ccache")
   set(CMAKE_C_COMPILER_ARG1 "gcc")
endif(USE_CCACHE)
# ------------ </CCache> ------------

project (CogX)

# (mmarko, 2012-02-22)
option(DO_SLICE_ONLY "Generate and compile slice libs, but nothing else." NO)

# (mmarko, 2012-01-03)
# The clean-all target is used for cleaning stuff in components that are built
# by custom build scripts (eg. a Makefile or ant).
#
# Example with a Makefile that creates binaries in the source directory:
#    add_custom_target (MyMakeProject ALL
#       WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
#       COMMAND make
#       )
#    add_custom_target (clean-MyMakeProject
#       WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
#       COMMAND make clean
#       )
#    if(TARGET clean-all)
#       add_dependencies(clean-all DEPENDS clean-MyMakeProject)
#    endif()
#
# After cmake, the following can be done:
#   make clean-all  -  make all targets that clean-all depends on
#   make clean-MyMakeProject  -  clean only this target
#   make clean clean-all - clean everything
#
# See also clean-AntBuild near the end of this file. 
#
# NOTE: CAST Control will pick up all clean-* targets when executing Make Clean from the menu.
add_custom_target (clean-all)

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


if(EXISTS ${COGX_ROOT}/subarchitectures/visualization.sa)
   include(subarchitectures/visualization.sa/cmake/v11n.cmake)
endif()

if(EXISTS ${COGX_ROOT}/tools/castutils)
   add_subdirectory (tools/castutils)
endif()

if(EXISTS ${COGX_ROOT}/tools/d-lib)
   add_subdirectory (tools/d-lib)
endif()



# ------------ <ROCS> ------------
if (EXISTS ${COGX_ROOT}/tools/rocs)
   option(BUILD_ROCS "Build the internal version of the ROCS library." YES)
   if(BUILD_ROCS AND NOT DO_SLICE_ONLY)
      # We don't want the ROCS build system to define its own build options. We will set them as variables.
      # mmarko: This will also prevent ROCS from setting various CMAKE_ variables
      set(ROCS_DONT_DEFINE_OPTIONS yes)
      # Build type used for rocs
      set(ROCS_BUILD_TYPE "Release")
      # Now, set the build options of ROCS
      set(ROCS_BUILD_MODULE_CORE yes)
      set(ROCS_BUILD_MODULE_ML yes )
      set(ROCS_BUILD_MODULE_CONCEPT yes )
      set(ROCS_BUILD_THIRDPARTY_LIBDAI yes)
      set(ROCS_BUILD_GPL yes)
      # Set the directory where ROCS should be installed
      set(ROCS_DIR ${OUTPUT}/rocs)
      # Finally, add ROCS
      add_subdirectory(tools/rocs)
      get_directory_property(ROCS_SRC_INCLUDES DIRECTORY tools/rocs INCLUDE_DIRECTORIES)
      message(STATUS ${ROCS_SRC_INCLUDES})
   endif(BUILD_ROCS)
endif()
# ------------ </ROCS> ------------


if (EXISTS ${COGX_ROOT}/scenarios/dora)
   option(BUILD_DORA "Build dora specific code" NO)
   if(BUILD_DORA)
      add_subdirectory (scenarios/dora)
   endif(BUILD_DORA)
endif()

if (EXISTS ${COGX_ROOT}/scenarios/george)
   option(BUILD_GEORGE "Build george specific code" NO)
   if(BUILD_GEORGE)
      add_subdirectory (scenarios/george)
   endif(BUILD_GEORGE)
endif()

if (EXISTS ${COGX_ROOT}/scenarios/shared)
   option(BUILD_SCENARIO_SHARED "Build shared scenario code" NO)
   if(BUILD_SCENARIO_SHARED)
      add_subdirectory (scenarios/shared)
   endif(BUILD_SCENARIO_SHARED)
endif()


if (EXISTS ${COGX_ROOT}/tools/beliefs)
   option(BUILD_BELIEFS "Build beliefs" YES)
   if(BUILD_BELIEFS)
      add_subdirectory (tools/beliefs/apis/beliefs)
      add_subdirectory (tools/beliefs/apis/beliefs+cast)
      add_subdirectory (tools/cogxutils/)
   endif(BUILD_BELIEFS)
endif()

if (EXISTS ${COGX_ROOT}/subarchitectures/binder.sa)
   option(BUILD_SA_BINDING "Build binder subarchitecure" NO)
   if(BUILD_SA_BINDING)
      add_subdirectory (subarchitectures/binder.sa)
   endif(BUILD_SA_BINDING)
endif()

if (EXISTS ${COGX_ROOT}/subarchitectures/motivation.sa)
   option(BUILD_SA_MOTIVATION "Build motivation subarchitecure" NO)
   if(BUILD_SA_MOTIVATION)
      add_subdirectory (subarchitectures/motivation.sa)
   endif(BUILD_SA_MOTIVATION)
endif()

if (EXISTS ${COGX_ROOT}/subarchitectures/execution.sa)
   option(BUILD_SA_EXECUTION "Build execution subarchitecure" NO)
   if(BUILD_SA_EXECUTION)
      add_subdirectory (subarchitectures/execution.sa)
   endif(BUILD_SA_EXECUTION)
endif()

if (EXISTS ${COGX_ROOT}/subarchitectures/manipulation.sa)
   option(BUILD_SA_MANIPULATION "Build manipulation subarchitecure" NO)
   if(BUILD_SA_MANIPULATION)
      if (DO_SLICE_ONLY AND NOT EXISTS "/usr/local/share/golem/slice/TinyIce.ice")
         message(" *** TinyIce.ice from Golem not found (required by manipulation.sa/java)")
         message(" *** SKIPPING manipulation.sa")
         set(BUILD_SA_MANIPULATION OFF)
      else()
         add_subdirectory (subarchitectures/manipulation.sa)
      endif()
   endif(BUILD_SA_MANIPULATION)
endif()

if (EXISTS ${COGX_ROOT}/subarchitectures/dialogue.sa)
   option(BUILD_SA_DIALOGUE "Build dialogue subarchitecure" NO)
   if(BUILD_SA_DIALOGUE)
      add_subdirectory (subarchitectures/dialogue.sa)
   endif(BUILD_SA_DIALOGUE)
endif()

if (EXISTS ${COGX_ROOT}/subarchitectures/coma.sa)
   option(BUILD_SA_COMA "Build sa_coma specific code" NO)
   if(BUILD_SA_COMA)
      add_subdirectory (subarchitectures/coma.sa)
   endif(BUILD_SA_COMA)
endif()

if (EXISTS ${COGX_ROOT}/subarchitectures/categorical.sa)
   option(BUILD_SA_CATEGORICAL "Build categorical subarchitecure" NO)
   if(BUILD_SA_CATEGORICAL)
      add_subdirectory (subarchitectures/categorical.sa)
   endif(BUILD_SA_CATEGORICAL)
endif()

if (EXISTS ${COGX_ROOT}/subarchitectures/conceptual.sa)
   option(BUILD_SA_CONCEPTUAL "Build conceptual subarchitecure" NO)
   if(BUILD_SA_CONCEPTUAL)
      add_subdirectory (subarchitectures/conceptual.sa)
   endif(BUILD_SA_CONCEPTUAL)
endif()

if (EXISTS ${COGX_ROOT}/subarchitectures/default.sa)
   option(BUILD_SA_DEFAULT "Build default subarchitecure" NO)
   if(BUILD_SA_DEFAULT)
      add_subdirectory (subarchitectures/default.sa)
   endif(BUILD_SA_DEFAULT)
endif()


if (EXISTS ${COGX_ROOT}/tools/abducer)
   option(BUILD_ABDUCER "Build abducer" YES)
   if (BUILD_ABDUCER)
      add_subdirectory (tools/abducer)
   endif(BUILD_ABDUCER)
endif()


if (EXISTS ${COGX_ROOT}/tools/hardware/robotbase)
   option(BUILD_HAL_ROBOT "Build hardware robotbase" YES)
   if(BUILD_HAL_ROBOT)
      add_subdirectory (tools/hardware/robotbase)
   endif(BUILD_HAL_ROBOT)
endif()

if (EXISTS ${COGX_ROOT}/tools/hardware/laser)
   option(BUILD_HAL_LASER "Build hardware laser" YES)
   if(BUILD_HAL_LASER)
      add_subdirectory (tools/hardware/laser)
   endif(BUILD_HAL_LASER)
endif()

if (EXISTS ${COGX_ROOT}/tools/hardware/tts)
   option(BUILD_HAL_TTS "Build hardware tts" NO)
   if(BUILD_HAL_TTS)
      add_subdirectory (tools/hardware/tts)
   endif(BUILD_HAL_TTS)
endif()

if (EXISTS ${COGX_ROOT}/tools/hardware/ptz)
   option(BUILD_HAL_PTZ "Build hardware ptz" YES)
   if(BUILD_HAL_PTZ)
      add_subdirectory (tools/hardware/ptz)
   endif(BUILD_HAL_PTZ)
endif()

if (EXISTS ${COGX_ROOT}/tools/hardware/blobfinder)
   option(BUILD_HAL_BLOBFINDER "Build hardware blobfinder" YES)
   if(BUILD_HAL_BLOBFINDER)
      add_subdirectory (tools/hardware/blobfinder)
   endif(BUILD_HAL_BLOBFINDER)
endif()

if (EXISTS ${COGX_ROOT}/tools/hardware/utils)
   option(BUILD_HAL_UTILS "Build hardware utils" YES)
   if(BUILD_HAL_UTILS)
      add_subdirectory (tools/hardware/utils)
   endif(BUILD_HAL_UTILS)
endif()

if (EXISTS ${COGX_ROOT}/tools/hardware/tests)
   option(BUILD_HAL_TESTS "Build hardware tests" NO)
   if(BUILD_HAL_TESTS)
      add_subdirectory (tools/hardware/tests)
   endif(BUILD_HAL_TESTS)
endif()

if (EXISTS ${COGX_ROOT}/tools/math)
   # UNCOMMENT THE FOLLOWING FOR THE VISION SUBARCHITECTURE
   add_subdirectory (tools/math)
endif()

if (EXISTS ${COGX_ROOT}/tools/hardware/video)
   option(BUILD_HAL_VIDEO "Build hardware video" YES)
   if(BUILD_HAL_VIDEO)
      add_subdirectory (tools/hardware/video)
   endif(BUILD_HAL_VIDEO)
endif()

#option(BUILD_HAL_STEREO "Build hardware stereo" NO)
#if(BUILD_HAL_STEREO)
#add_subdirectory (tools/hardware/stereo)
#endif(BUILD_HAL_STEREO)

if (EXISTS ${COGX_ROOT}/tools/hardware/pointCloud)
   option(BUILD_HAL_POINT_CLOUD "Build point cloud server/client" NO)
   option(HAVE_PCL_LIBRARY "have pcl library installed." NO)
   if(BUILD_HAL_POINT_CLOUD)
      add_subdirectory (tools/hardware/pointCloud)
   endif(BUILD_HAL_POINT_CLOUD)
endif()

if (EXISTS ${COGX_ROOT}/tools/hardware/kinect)
   option(BUILD_HAL_KINECT "Build hardware kinect" NO)
   if(BUILD_HAL_KINECT)
      add_subdirectory (tools/hardware/kinect)
   endif(BUILD_HAL_KINECT)
endif()

if (EXISTS ${COGX_ROOT}/subarchitectures/vision.sa)
   option(BUILD_SA_VISION "Build vision subarchitecure" NO)
   if(BUILD_SA_VISION)
      add_subdirectory (subarchitectures/vision.sa)
   endif(BUILD_SA_VISION)
endif()

# Spatial
if (EXISTS ${COGX_ROOT}/subarchitectures/spatial.sa)
   option(BUILD_SA_SPATIAL "Build spatial subarchitecure" NO)
   if(BUILD_SA_SPATIAL)
      add_subdirectory (subarchitectures/spatial.sa)
   endif(BUILD_SA_SPATIAL)
endif()

if (EXISTS ${COGX_ROOT}/tools/testing)
   option(BUILD_MULTI_TESTER "Build multi-sa tester" NO)
   if(BUILD_MULTI_TESTER)
      add_subdirectory (tools/testing)
   endif(BUILD_MULTI_TESTER)
endif()

# Planner
if (EXISTS ${COGX_ROOT}/subarchitectures/planner.sa)
   option(BUILD_SA_PLANNER "Build planner subarchitecure" NO)
   if(BUILD_SA_PLANNER)
      add_subdirectory (subarchitectures/planner.sa)
   endif(BUILD_SA_PLANNER)
endif()

# Fake motivation
if (EXISTS ${COGX_ROOT}/subarchitectures/fakemotivation.sa)
   option(BUILD_SA_FAKE_MOTIVATION "Build fake motivation subarchitecure" NO)
   if(BUILD_SA_FAKE_MOTIVATION)
      add_subdirectory (subarchitectures/fakemotivation.sa)
   endif(BUILD_SA_FAKE_MOTIVATION)
endif()

if (EXISTS ${COGX_ROOT}/tools/syscalb)
   option(BUILD_CALIB "Build system calibration tools" YES)
   if(BUILD_CALIB)
      add_subdirectory (tools/syscalb)
   endif(BUILD_CALIB)
endif()

if (EXISTS ${COGX_ROOT}/tools/gripper)
   option(BUILD_GRIPPER "Build simple player gripper tool" NO)
   if(BUILD_GRIPPER)
      add_subdirectory (tools/gripper)
   endif(BUILD_GRIPPER)
endif()

if (EXISTS ${COGX_ROOT}/tools/scripts)
   add_subdirectory(tools/scripts)
endif()

# Find ICE_SLICE_DIR by searching for Ice/Identity.ice
#message(" ice slice: " ${ICE_SLICE_DIR})
find_file(SLICE_IDENTITY Ice/Identity.ice
   PATHS /usr/share/slice /usr/share/Ice/slice)
if(SLICE_IDENTITY)
   string(REGEX REPLACE "/Ice/Identity\\.ice$" "" ICE_SLICE_DIR ${SLICE_IDENTITY})
endif(SLICE_IDENTITY)
#message(" ice slice: " ${ICE_SLICE_DIR})

if (EXISTS ${COGX_ROOT}/cmake)
   configure_file(${CMAKE_SOURCE_DIR}/cmake/build.properties.in ${CMAKE_SOURCE_DIR}/build.properties)
endif()

option(DO_ANT "automatically compile the ant stuff" YES)
if(DO_ANT)
   set(ANT_TARGET all)
   if(DO_SLICE_ONLY)
      set(ANT_TARGET compile-slice)
   endif()
   ADD_CUSTOM_TARGET( ant_build ALL )
   ADD_CUSTOM_COMMAND(
      TARGET  ant_build
      COMMAND ant ${ANT_TARGET}
	 -Dcastslice.dir=${CAST_SLICE_DIR}
	 -Dcast.jar=${CAST_INSTALL_ROOT}/share/java/cast.jar
	 -Dice.jar=${ICE_HOME}/share/java/Ice.jar
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
      )
   
   # (2011-12-23,mmarko):
   #   This target (clean-*) will be discovered and executed by CAST Control
   add_custom_target (clean-AntBuild
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
      COMMAND ant clean
      )
   if(TARGET clean-all)
      add_dependencies(clean-all DEPENDS clean-AntBuild)
   endif()
endif(DO_ANT)

#option(DO_V4R_MAKE "automatically compile the v4r library (external make)" NO)
#if(DO_V4R_MAKE)
#   #add_custom_target( v4r_build ALL )
#   #add_custom_command(
#   #   TARGET  v4r_build
#   #   COMMAND bash cmake/makev4r.sh
#   #   WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
#   #   )
#   MESSAGE(STATUS "V4R is no longer part of CogX build. Use GAR installer.")
#endif()

MESSAGE(STATUS " * Build type: " ${CMAKE_BUILD_TYPE})
MESSAGE(STATUS " * Do Ant: " ${DO_ANT})
MESSAGE(STATUS " * Do slice only: " ${DO_SLICE_ONLY})
#MESSAGE(STATUS " * Do make V4R: " ${DO_V4R_MAKE})
MESSAGE(STATUS " * Use ccache: " ${USE_CCACHE})
