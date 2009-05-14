
PROJECT(FeatureLearningProxy)
INCLUDE_DIRECTORIES( ${FeatureLearningCtf_BINARY_DIR}/build )
LINK_DIRECTORIES( ${FeatureLearningCtf_BINARY_DIR}/build )
SET(SOURCES 
   initlib.cpp  
   MatlabHelper.cpp
   FeatureExtractorP.cpp
   RecogniserP.cpp)
SET(HEADERS initlib.h FeatureExtractorP.h RecogniserP.h)
ADD_LIBRARY(FeatureLearningProxy SHARED ${SOURCES})
TARGET_LINK_LIBRARIES(FeatureLearningProxy ${OMNIORB4_LIBRARIES})
TARGET_LINK_LIBRARIES(FeatureLearningProxy VisionIDL Vision)
TARGET_LINK_LIBRARIES(FeatureLearningProxy MLHelperIDL)
TARGET_LINK_LIBRARIES(FeatureLearningProxy FeatureLearningCtf)
INSTALL(TARGETS FeatureLearningProxy LIBRARY DESTINATION lib)
   
   
