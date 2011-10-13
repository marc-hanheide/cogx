SET(V4R_CMODEL_RECOGNISER 0 CACHE BOOL "SIFT based object recogniser." )
IF(V4R_CMODEL_RECOGNISER)

PROJECT(CModelRecogniser)
SET(SOURCE_CPP
  PKeypoint.cc
  View.cc
  CBEntry.cc
  CModel.cc
  CModelHandler.cc
  ProbModel.cc
  fams.cpp
  IMCluster.cc
  KeypointDetector.cc
  KeypointDetectorSIFT.cc
  KeypointDetectorSURF.cc
#  MatchFilter.cc
  MeanShift.cc
  MeanShiftFAMS.cc
  MeanShiftCodebook.cc
  MeanCodebook.cc
#  RobustEstimators.cc
  SphereHistogram.cc
#  VocabularyTree.cc
  LearnerCore.cc
  RecogniserCore.cc
  RecogniserThread.cc
  PSiftGPU.cc
#  ItMoSPlanes.cc
#  Plane.cc
  KeypointDetectorFAST.cc
  Point3dProjs.cc
#  SelectMatchesMRF.cc
#  PKeypointMRF.cc
#  MergePlanesHF.cc
#  IsPlaneGraph.cc
#  GraphFrame.cc
)

SET(SOURCE_H
  PKeypoint.hh
  View.hh
  CBEntry.hh
  ClusterSample.hh
  CModel.hh
  CModelHandler.hh
  Codebook.hh
  ProbModel.hh
  fams.h
  IMCluster.hh
  KeypointDetector.hh
  KeypointDetectorSIFT.hh
  KeypointDetectorSURF.hh
  LearnerCore.hh
#  MatchFilter.hh
  MeanShiftBase.hh
  MeanShiftFAMS.hh
  MeanShift.hh
  MeanShiftCodebook.hh
  MeanCodebook.hh
  ObjectLocation.hh
  polygontest_cv.h
#  RobustEstimators.hh
  SphereHistogram.hh
#  VocabularyTree.hh
  RecogniserCore.hh
  RecogniserThread.hh
  PSiftGPU.hh
#  ItMoSPlanes.hh
#  Plane.hh
  KeypointDetectorFAST.hh
  Point3dProjs.hh
#  SelectMatchesMRF.hh
#  PKeypointMRF.hh
#  MergePlanesHF.hh
#  IsPlaneGraph.hh
#  GraphFrame.hh
)

#add_subdirectory(./svm)

add_library(CModelRecogniser SHARED ${SOURCE_CPP})
target_link_libraries(CModelRecogniser ${OPENCV_LIBRARIES} PGeometry PMath siftgpu)
#target_link_libraries(CModelRecogniser ${OPENCV_LIBRARIES} PGeometry PMath siftgpu SVMClassify)


#add_subdirectory(example)

ENDIF(V4R_CMODEL_RECOGNISER)

