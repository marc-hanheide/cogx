

FIND_LIBRARY(CPPUNIT cppunit)
FIND_PATH(CPPUNIT_HEADERS cppunit/TestRunner.h)
IF ( CPPUNIT AND CPPUNIT_HEADERS)
 MESSAGE("-- Looking for Cppunit - found")
ELSE ( CPPUNIT AND CPPUNIT_HEADERS )
 MESSAGE( FATAL_ERROR "-- Looking for Cppunit - not found")
ENDIF ( CPPUNIT AND CPPUNIT_HEADERS )

include_directories(${CPPUNIT_HEADERS})

set(SOURCES 
   test_ptrvector.cpp
   ../ptrvector.cpp
   )
add_executable(TestVisualizationCore ${SOURCES})
target_link_libraries(TestVisualizationCore ${CPPUNIT})
