cmake_minimum_required ( VERSION 2.6 )
PROJECT(v4r)

include(v4r.cmake)      # V4R_DIR 
include(opencv.cmake)   # OPENCV_LIBRARIES 
include(tomgine.cmake)	# TomGine definitions & includes

set(Boost_USE_MULTITHREADED ON)
FIND_PACKAGE (Boost 1.38.0 REQUIRED COMPONENTS date_time thread program_options serialization regex filesystem signals)
set (BOOST_LIBRARIES ${Boost_LIBRARIES})

SET(LIBRARY_OUTPUT_PATH "${V4R_DIR}/lib")
SET(EXECUTABLE_OUTPUT_PATH "${V4R_DIR}/bin")

add_definitions(-DDEBUG)
add_definitions(-DBOOST_FILESYSTEM_VERSION=2)

include_directories(${V4R_DIR})

add_subdirectory(apps)
add_subdirectory(ak)
add_subdirectory(v4r)
add_subdirectory(external)
