PROJECT(ObjectMClassDetector)
SET(SOURCES
  ObjectMClassDetector.cpp parameters.cpp hog_features.cpp SW_MultHOG.cpp   
  libAnnotation/annorect.cpp libAnnotation/annotation.cpp libAnnotation/annotationlist.cpp libAnnotation/xmlhelpers.cpp 
  libChiSquare/chisquare.cc libChiSquare/gamma.cc 
  libCmd/BoolOption.C libCmd/CmdOption.C libCmd/CmdParser.C libCmd/EnumOption.C libCmd/StringArgument.C libCmd/StringOption.C 
  libFeatures/featurevector.cc
  libHOG/HOG.cpp libHOG/HOGparams.cpp 
  libInifile/inifile.cpp
  libPPProcess/nonmaxsuppression.cpp 
  libSVMdense/binary_format.cpp libSVMdense/svm_common.cpp libSVMdense/svm_hideo.cpp libSVMdense/svm_learn.cpp libSVMdense/svm.cpp
)
SET(HEADERS
  ObjectMClassDetector.h
)


ADD_DEFINITIONS(-DDENSE -O3)
ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OMNIORB4_LIBRARIES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} VisionIDL)
#TARGET_LINK_LIBRARIES(${PROJECT_NAME} VisionCommon Vision VisionUtils VisionUtils TUDVision)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} Vision VisionUtils)
#TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${TUDVISION_DIR}/libTUDVision.a Qt3Support QtGui QtCore)
TARGET_LINK_LIBRARIES(${PROJECT_NAME})
#LINK_DIRECTORIES (${PROJECT_NAME} ${TUDVISION_DIR}})
# Use pkg-config to find opencv libraries...
INCLUDE(UsePkgConfig)
PKGCONFIG("opencv" OCV_INCLUDE_DIR OCV_LINK_DIR OCV_LINK_FLAGS OCV_CFLAGS)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OCV_LINK_FLAGS})
INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
INSTALL(FILES ${HEADERS} DESTINATION include/vision/components/ObjectMClassDetector)


