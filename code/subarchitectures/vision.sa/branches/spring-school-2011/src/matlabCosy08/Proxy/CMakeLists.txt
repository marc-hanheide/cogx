
PROJECT(VisualLearnerProxy)
INCLUDE_DIRECTORIES(${VisualLearnerCtf_BINARY_DIR}/build)
LINK_DIRECTORIES( ${VisualLearnerCtf_BINARY_DIR}/build )
# LINK_DIRECTORIES( ${MATLAB_DIR}/bin/glnxa64 )
SET(SOURCES 
   initlib.cpp  
   MatlabHelper.cpp
   VisualLearnerProxy.cpp
   )
SET(HEADERS initlib.h RecogniserP.h)
add_library(VisualLearnerProxy SHARED ${SOURCES})
target_link_libraries(VisualLearnerProxy VisionData)
target_link_libraries(VisualLearnerProxy VisualLearnerCtf)

find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})
target_link_libraries(VisualLearnerProxy ${OPENCV_LDFLAGS})

install(TARGETS VisualLearnerProxy LIBRARY DESTINATION lib)

