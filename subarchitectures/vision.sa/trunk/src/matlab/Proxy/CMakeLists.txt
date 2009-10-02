
PROJECT(VisualLearnerProxy)
INCLUDE_DIRECTORIES(${VisualLearnerCtf_BINARY_DIR}/build)
LINK_DIRECTORIES( ${VisualLearnerCtf_BINARY_DIR}/build )
SET(SOURCES 
   initlib.cpp  
   MatlabHelper.cpp
   RecogniserP.cpp
   )
SET(HEADERS initlib.h RecogniserP.h)
add_library(VisualLearnerProxy SHARED ${SOURCES})
target_link_libraries(VisualLearnerProxy VisionData)
target_link_libraries(VisualLearnerProxy VisualLearnerCtf)

# Use pkg-config to find opencv libraries...
include(UsePkgConfig)
PKGCONFIG("opencv" OCV_INCLUDE_DIR OCV_LINK_DIR OCV_LINK_FLAGS OCV_CFLAGS)
include_directories(${OCV_INCLUDE_DIR})
target_link_libraries(VisualLearnerProxy ${OCV_LINK_FLAGS})

install(TARGETS VisualLearnerProxy LIBRARY DESTINATION lib)

