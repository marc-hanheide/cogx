/**
 * $Id$
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
#include "Codebook.hh"
#include "MeanShiftCodebook.hh"
#include "MeanCodebook.hh"
#include "ProbModel.hh"
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
    cv::Mat distCoeffs;
    unsigned maxRandTrials;         // max. number of trials for pose ransac
    unsigned numLoTrials;           // number lo-ransac trials
    double etaRansac;               // eta for pose ransac
    double inlDistRansac;           // inlier dist pose ransac
    double inlDistAff;              // inlier dist of affine ransac
    bool useFams;
    double thrDesc;
    double sigmaDesc;
    float nnRatio;

    Parameter(int w=640, int h=480, 
       unsigned trials=5000, unsigned loTrials=50, double eta=0.01, double dist=2., double distAff=5., 
       bool fams=false, double _thrDesc=0.4, double _sigmaDesc=.2, double nnr=.8) 
     : width(w), height(h), 
       maxRandTrials(trials), numLoTrials(loTrials), etaRansac(eta), 
       inlDistRansac(dist), inlDistAff(distAff), 
       useFams(fams), thrDesc(_thrDesc), sigmaDesc(_sigmaDesc), nnRatio(nnr) {}
  };

private:
  cv::Ptr<KeypointDetector> detector;
  cv::Ptr<cv::DescriptorExtractor> extractor;
  cv::Ptr<cv::DescriptorMatcher> matcher;

  cv::Ptr<Codebook> codebook;

  cv::Mat grayImage;             // gray scale image
  vector< cv::Ptr<PKeypoint> > keys;
  cv::Mat_<float> descriptors;
  vector<cv::KeyPoint> cvKeys;   // just a opencv keypoint container

  vector< cv::Ptr<CModel> > recModels;  // models to recognise

  map<unsigned, vector<cv::DMatch> >::iterator SelectMaxConf(map<unsigned, vector<cv::DMatch> > &ma);
  void ClusterMatches(unsigned oidx, vector<cv::DMatch> &matches, vector<vector<cv::DMatch> > &clusters);
  bool DetectObjects(unsigned oidx, vector<vector<cv::DMatch> > &clusters, vector<ObjectLocation> &objects);
  bool SortMatchesToViews(vector<cv::DMatch> &matches, vector<vector<cv::DMatch> > &sorted, unsigned &cnt);
  bool LoRansac(unsigned oidx, vector<vector<cv::DMatch> > &matches, int numTotal, ObjectLocation &obj);
  void GetRandIdx(unsigned size, unsigned num, vector<unsigned> &idx);
  void CountInlier(vector<cv::Ptr<PKeypoint> > &queryKeys, vector<cv::Ptr<PKeypoint> > &trainKeys, 
        vector<cv::DMatch> &matches, double H[9], double &inl);
  void GetInlier(vector<cv::Ptr<PKeypoint> > &queryKeys, 
        vector<cv::Ptr<PKeypoint> > &trainKeys, vector<cv::DMatch> &matches, double H[9], 
        vector<PKeypoint*> &querySelected, vector<PKeypoint*> &trainSelected);
  void CountInlier(vector<cv::Ptr<PKeypoint> > &queryKeys, CModel &model,
        vector<vector<cv::DMatch> > &matches, Pose &pose, double &sig);
  void MarkInlier(unsigned oidx, vector<vector<cv::DMatch> > &matches, ObjectLocation &obj, 
        unsigned &numTotal, unsigned &numInlier);




  /*double ComputeConfidence(vector< cv::Ptr<PKeypoint> > &keys, vector< cv::Ptr<PKeypoint> > &model, Pose &pose);
  void CountInlier(MatchPairs &mp, Pose &pose, double &sig);

  int MarkInlier(MatchPairs &mp, ObjectLocation &obj);
  void CountInlier(vector<PKeypoint*> &moKeys, vector<PKeypoint*> &imKeys, double H[9], double &inl);
  void GetInlier(vector<PKeypoint*> &moKeys, vector<PKeypoint*> &imKeys, double H[9], 
         vector<unsigned> &idxInlier);
  int GetUnmarked(MatchPairs &mp, MatchPairs &unexp);*/


  void GetColor(map<unsigned, cv::Vec3b> &cols, unsigned id, cv::Vec3b &col);



  inline bool Contains(const vector<unsigned> &idx, unsigned num);
  inline unsigned GetObjectIndex(const string id);
  


public:
  Parameter param;
  cv::Mat dbg;

  RecogniserCore(cv::Ptr<KeypointDetector> &keyDetector,
                 cv::Ptr<cv::DescriptorExtractor> &descExtractor,
                 cv::Ptr<cv::DescriptorMatcher> &descMatcher,
                 Parameter p=Parameter());
  ~RecogniserCore();

  void SetCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_distCoeffs);

  // recognition of keypoint models
  //void LoadVocabularyTree(const string &filename);
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

