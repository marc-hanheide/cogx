
FIND_LIBRARY(CPPUNIT cppunit)
FIND_PATH(CPPUNIT_HEADERS cppunit/TestRunner.h)
IF ( CPPUNIT AND CPPUNIT_HEADERS)
   #MESSAGE("-- Looking for Cppunit - found")
ELSE ( CPPUNIT AND CPPUNIT_HEADERS )
   MESSAGE( FATAL_ERROR "-- Looking for Cppunit - not found")
ENDIF ( CPPUNIT AND CPPUNIT_HEADERS )


set(SOURCES 
   test_ptrvector.cpp
   ../ptrvector.cpp
   )
include_directories(${CPPUNIT_HEADERS})
add_executable(TestVisualizationCore ${SOURCES})
target_link_libraries(TestVisualizationCore ${CPPUNIT})


# Based on VideoViewer

find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})

set(SOURCES TestComponent.cpp)

find_package(LibPlot)
set(HAS_LIBPLOT ${LIBPLOT_FOUND})
if(HAS_LIBPLOT)
   add_definitions(-DHAS_LIBPLOT)
   set(LIBPLOT_LIBS plotter)
endif(HAS_LIBPLOT)

add_definitions(-DFEAT_VISUALIZATION)
include_directories( .
   ${VISUALIZATION_INCLUDE_DIRS}
   )
include_directories(${COGX_ROOT}/tools/math/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/math/src/c++/math)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/utils)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/components)
include_directories(${COGX_ROOT}/tools/hardware/video/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/stereo/src/c++/components)
include_directories(${COGX_ROOT}/tools/hardware/stereo/src/c++/autogen)
include_directories(${COGX_ROOT}/subarchitectures/vision.sa/src/c++/vision/autogen)
include_directories(${COGX_ROOT}/tools/hardware/pointCloud/src/c++/autogen)
include_directories(${COGX_ROOT}/tools/hardware/laser/src/c++/autogen)

add_cast_component(VisualizationTest ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} 
   DisplayServer DisplayClient VisualizationData
   Video VideoUtils
   ${OPENCV_LDFLAGS}
   ${LIBPLOT_LIBS}
   )

