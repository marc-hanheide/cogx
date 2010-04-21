
FIND_LIBRARY(CPPUNIT cppunit)
FIND_PATH(CPPUNIT_HEADERS cppunit/TestRunner.h)
IF ( CPPUNIT AND CPPUNIT_HEADERS)
   MESSAGE("-- Looking for Cppunit - found")
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
set(SOURCES TestComponent.cpp)
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

add_cast_component(VisualizationTest ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} 
   VisionData Video VideoUtils
   DisplayClient VisualizationData
   ${OPENCV_LDFLAGS}
   )

