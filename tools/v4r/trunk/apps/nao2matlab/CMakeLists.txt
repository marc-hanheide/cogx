SET(APPS_NAO2MATLAB 1 CACHE BOOL "Add description here." )
IF(APPS_NAO2MATLAB)

cmake_minimum_required ( VERSION 2.6 )
project ( nao )


IF ( "x${AL_DIR}x" STREQUAL "xx"  )
  SET( AL_DIR "$ENV{AL_DIR}" )
  MESSAGE( STATUS "AL_DIR is ${AL_DIR}")
  IF (NOT EXISTS ${AL_DIR}  )
        MESSAGE( FATAL_ERROR "AL_DIR does not exist")
  ENDIF(NOT EXISTS ${AL_DIR}  )
ELSE ("x${AL_DIR}x" STREQUAL "xx" )
  MESSAGE( FATAL_ERROR "Set AL_DIR in your .profile file")
ENDIF("x${AL_DIR}x" STREQUAL "xx" )

set(NAO_DIR ${CMAKE_CURRENT_SOURCE_DIR})
SET (EXECUTABLE_OUTPUT_PATH "${NAO_DIR}/bin")
SET (LIBRARY_OUTPUT_PATH "${NAO_DIR}/lib")

set(Boost_USE_MULTITHREADED ON)
FIND_PACKAGE (Boost 1.38.0 REQUIRED COMPONENTS date_time thread program_options serialization regex filesystem signals)
set (BOOST_LIBRARIES ${Boost_LIBRARIES})

set ( OPENCV_LIBRARIES  cv cxcore highgui )

FIND_PACKAGE(PythonLibs 2.6 REQUIRED)

SET (CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "-g")

set ( AL_LIBRARIES
    ${AL_DIR}/lib/libalcommon.so
    ${AL_DIR}/lib/libalvalue.so
    ${AL_DIR}/lib/libippc.so
    ${AL_DIR}/lib/liblibcore.so
    ${AL_DIR}/lib/libfindippc.so
    ${AL_DIR}/lib/liblibsoap.so
    ${AL_DIR}/lib/libtinyxml.so 
)

set ( AL_INCLUDES
    ${AL_DIR}/modules/proxies
    ${AL_DIR}/include/python2.6
    ${AL_DIR}/include
    ${AL_DIR}/include/alcore
    ${AL_DIR}/include/alcommon
    ${AL_DIR}/include/libthread
    ${AL_DIR}/include/alvalue
    ${AL_DIR}/include/alaudio
    ${AL_DIR}/include/albonjourdiscovery
    ${AL_DIR}/include/alfactory
    ${AL_DIR}/include/allauncher
    ${AL_DIR}/include/almath
    ${AL_DIR}/include/alproxies
    ${AL_DIR}/include/alsa
    ${AL_DIR}/include/alstreamer
    ${AL_DIR}/include/altools
    ${AL_DIR}/include/alvision
    ${AL_DIR}/include/tinyxml
)
add_subdirectory ( src )

ENDIF(APPS_NAO2MATLAB)
