   include_directories(../../../../matlab/Proxy)

   project(VisualLearner)
   link_directories(${COGX_ROOT}/output/lib)
   set(SOURCES VisualLearner.cpp)
   set(HEADERS VisualLearner.h)
   add_cast_component(VisualLearner ${SOURCES})
   link_cast_component(${CAST_COMPONENT_NAME} VisionData VisualLearnerProxy VisualLearnerCtf)
#  link_cast_component(${CAST_COMPONENT_NAME} MLHelperIDL)
#  INSTALL(TARGETS FeatureExtractor LIBRARY DESTINATION lib)
#  INSTALL(FILES ${HEADERS} DESTINATION include/vision/components/featurelearning)


   

