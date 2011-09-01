/**
 * $Id$
 * TODO:
 * - additionally add vocabulary tree
 */

#ifndef P_RECOGNISER_CORE_HH
#define P_RECOGNISER_CORE_HH

#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "KeypointDetector.hh"
#include "CModel.hh"
#include "polygontest_cv.h"
#include "ObjectLocation.hh"
#include "MeanShiftBase.hh"
#include "MeanShift.hh"
#include "MeanShiftFAMS.hh"
#include "v4r/PGeometry/PHomography.hh"
#include "MSCodebook.hh"
#include "ConfValues.hh"
#include "MatchPairs.hh"



namespace P
{

/**
 * RecogniserCore
 */
class RecogniserCore
{
public:
  class Parameter
  {
  public:
    int width, height;
    cv::Mat intrinsic;
    cv::Mat distortion;
    unsigned maxRandTrials;         // max. number of trials for pose ransac
    unsigned numLoTrials;           // number lo-ransac trials
    double etaRansac;               // eta for pose ransac
    double inlDistRansac;           // inlier dist pose ransac
    unsigned numObjectsToRecognise; // num. of objects from the ranked list where the model is tested
    bool useFams;
    double thrDesc;

    Parameter(int w=640, int h=480, 
       unsigned trials=5000, unsigned loTrials=50, double eta=0.01, double dist=2., 
       unsigned numObj=10, bool fams=true, double _thrDesc=0.3) 
     : width(w), height(h), 
       maxRandTrials(trials), numLoTrials(loTrials), etaRansac(eta), inlDistRansac(dist), 
       numObjectsToRecognise(numObj), useFams(fams), thrDesc(_thrDesc) {}
  };

private:
  cv::Ptr<KeypointDetector> detector;
  cv::Ptr<cv::DescriptorExtractor> extractor;
  cv::Ptr<cv::DescriptorMatcher> matcher;

  cv::Ptr<MSCodebook> codebook;

  cv::Mat grayImage;             // gray scale image
  vector< cv::Ptr<PKeypoint> > keys;
  cv::Mat_<float> descriptors;
  vector<cv::KeyPoint> cvKeys;   // just a opencv keypoint container

  vector< cv::Ptr<CModel> > recModels;  // models to recognise


  double ComputeConfidence(vector< cv::Ptr<PKeypoint> > &keys, vector< cv::Ptr<PKeypoint> > &model, Pose &pose);
  void ClusterMatches(unsigned id, vector<cv::Ptr<OVMatches> > &matches, vector<cv::Ptr<MatchPairs> > &mps);
  void GetRandIdx(unsigned size, unsigned num, vector<unsigned> &idx);
  void CountInlier(MatchPairs &mp, Pose &pose, double &sig);
  void RansacObjects(vector<cv::Ptr<MatchPairs> > &mps, vector<ObjectLocation> &objects);

  bool LoRansac(MatchPairs &mp, ObjectLocation &obj, int num);
  int MarkInlier(MatchPairs &mp, ObjectLocation &obj);
  void CountInlier(vector<PKeypoint*> &moKeys, vector<PKeypoint*> &imKeys, double H[9], double &inl);
  void GetInlier(vector<PKeypoint*> &moKeys, vector<PKeypoint*> &imKeys, double H[9], 
         vector<unsigned> &idxInlier);
  int GetUnmarked(MatchPairs &mp, MatchPairs &unexp);


  void GetColor(map<unsigned, cv::Vec3b> &cols, unsigned id, cv::Vec3b &col);



  inline bool Contains(const vector<unsigned> &idx, unsigned num);
  inline unsigned GetObjectIndex(const string id);
  


public:
  Parameter param;
  cv::Mat dbg;

  RecogniserCore(cv::Ptr<KeypointDetector> &keyDetector,
                 cv::Ptr<cv::DescriptorExtractor> &descExtractor,
                 cv::Ptr<cv::DescriptorMatcher> &descMatcher,
                 Parameter _param=Parameter());
  ~RecogniserCore();

  void SetCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_distortion);

  // recognition of keypoint models
  void LoadVocabularyTree(const string &filename);
  void Clear();
  unsigned AddModel(cv::Ptr<CModel> &model);
  void Recognise(const cv::Mat &image, vector<ObjectLocation> &objects, cv::Mat mask=cv::Mat());
  void OptimizeCodebook();
};




/*********************** INLINE METHODES **************************/

inline bool RecogniserCore::Contains(const vector<unsigned> &idx, unsigned num)
{
  for (unsigned i=0; i<idx.size(); i++)
    if (idx[i]==num)
      return true;
  return false;
}

inline unsigned RecogniserCore::GetObjectIndex(const string id)
{
  for (unsigned i=0; i<recModels.size(); i++)
    if (recModels[i]->id.compare(id)==0)
      return i;
  return UINT_MAX;
}




}

#endif

