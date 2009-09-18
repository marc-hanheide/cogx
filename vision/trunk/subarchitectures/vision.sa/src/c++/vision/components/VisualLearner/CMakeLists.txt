   INCLUDE_DIRECTORIES(../../../../matlab/Proxy)

   PROJECT(VisualLearner)
   SET(SOURCES VisualLearner.cpp)
   SET(HEADERS VisualLearner.h)
   add_cast_component(VisualLearner ${SOURCES})
   link_cast_component(${CAST_COMPONENT_NAME} VisionData VisualLearnerProxy)
#  link_cast_component(${CAST_COMPONENT_NAME} MLHelperIDL)
#  INSTALL(TARGETS FeatureExtractor LIBRARY DESTINATION lib)
#  INSTALL(FILES ${HEADERS} DESTINATION include/vision/components/featurelearning)


   

