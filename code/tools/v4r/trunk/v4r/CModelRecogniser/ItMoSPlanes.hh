/**
 * $Id$
 */

#ifndef P_ITMOS_PLANES_HH
#define P_ITMOS_PLANES_HH

#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "v4r/PGeometry/PHomography.hh"
#include "v4r/PMath/PVector.hh"
#include "v4r/PMath/PMath.hh"
#include "v4r/PCore/toString.hpp"
#include "PKeypoint.hh"
#include "Plane.hh"
#include "KeypointDetector.hh"
#include "MatchFilter.hh"
#include "DistIdx.hh"
#include "SelectMatchesMRF.hh"



namespace P
{

/**
 * ItMoSPlanes
 */
class ItMoSPlanes
{
public:
  class Parameter
  {
  public:
    int width, height;        // image size
    double minMotion;         // min. motion between key frames(20)
    double maxMotion;         // max. motion within subsequen frames
    double thrDesc;           //.5
    double etaRansac;
    int maxRansacIter;
    int numRandHypotheses;
    double inlDist;
    double sigmaError;
    double kappa1;
    double kappa2;
    int kMatches;
    float nnRatio;             //.8
    double thrDiffMotion;
    bool mrfMatchFilter;             // use a geometric neighbourhood filter or NN-Ratio filtering
    bool planeConstrainedMatching;

    Parameter(int w=640, int h=480, double minMot=2, double maxMot=20, double _thrDesc=.5, 
        double eta=0.01, int iter=1000, int randHyp=4, double _inlDist=1., double sigErr=.4, 
        double _kappa1=6, double _kappa2=.4, int k=3, float nnr=.8, double diffMot=10, 
        bool matchMRF=true, bool constrainedMatch=false) 
      : width(w), height(h), minMotion(minMot), maxMotion(maxMot), thrDesc(_thrDesc), etaRansac(eta), 
        maxRansacIter(iter), numRandHypotheses(randHyp), inlDist(_inlDist), sigmaError(sigErr),
        kappa1(_kappa1), kappa2(_kappa2), kMatches(k), nnRatio(nnr), 
        mrfMatchFilter(matchMRF), planeConstrainedMatching(constrainedMatch) {} 
  };

private:
  cv::Ptr<cv::DescriptorMatcher> matcher;
  cv::Ptr<SelectMatchesMRF> selmat;

  vector< cv::Ptr<Plane> > svPlanes;
  vector< cv::Ptr<PKeypoint> > keys, svKeys, selectedKeys;
  cv::Mat_<float> svDescs;
  vector<vector<cv::DMatch> > matches;
  vector<int> selectedMatches;

  double sqrInlDist;
  double CONST_ERR1, CONST_ERR2;

  MatchFilter filter;

  double GetMedianMot(const vector<cv::Ptr<P::PKeypoint> > &keys, 
    const vector<cv::Ptr<P::PKeypoint> > &svKeys, vector<vector<cv::DMatch> > &matches, 
    vector<int> &selected);
  void CreateGraph(vector<cv::Ptr<P::PKeypoint> > &keys);
  int CountExplainedKeys(vector<cv::Ptr<P::PKeypoint> > &keys, vector< cv::Ptr<Plane> > &planes, 
        int &numKeys);
  void SetTrackingLinks(vector<cv::Ptr<P::PKeypoint> > &queryKeys, 
        vector<cv::Ptr<P::PKeypoint> > &trainKeys, vector<vector<cv::DMatch> > &matches, 
        vector<int> &selected);
  void ClearFWBW(vector<cv::Ptr<P::PKeypoint> > &keys);
  void GetUnusedMatches(vector< cv::Ptr<Plane> > &planes, vector<cv::Ptr<P::PKeypoint> > &keys, 
        vector<cv::Ptr<P::PKeypoint> > &unKeys);
  void GetNapIdx(vector<cv::Ptr<P::PKeypoint> > &keys, int num, vector<int> &idx);
  void AddRandHypotheses(vector<cv::Ptr<P::PKeypoint> > &keys, 
        vector<cv::Ptr<P::PKeypoint> > &unKeys, vector< cv::Ptr<Plane> > &planes);
  void AddRefineHypotheses(vector<cv::Ptr<P::PKeypoint> > &keys, vector< cv::Ptr<Plane> > &planes);
  void AccumulateKeys(vector<cv::Ptr<P::PKeypoint> > &keys, PKeypoint &key, cv::Mat_<double> &H, 
        vector<cv::Ptr<P::PKeypoint> > &planeKeys);
  void SetIds(vector<cv::Ptr<P::PKeypoint> > &keys);
  void CalcLSHomography(vector< cv::Ptr<Plane> > &planes, int method, bool filter=false);
  void AccumulateKeys(vector<cv::Ptr<P::PKeypoint> > &keys, vector< cv::Ptr<Plane> > &planes);
  void SelectPlanes(vector< cv::Ptr<Plane> > &in, vector< cv::Ptr<Plane> > &out);
  void CreateSegmentationMatrixQ(vector< cv::Ptr<Plane> > &planes, cv::Mat_<double> &Q);
  void GetErrProbEuc(double m1[2],double m2[2],double H[9], double err[1]);
  void ComputeQii(Plane &pl, double &weight);
  void ComputeQij(Plane &pi, Plane &pj, double &weight);
  void GreedyQBP(cv::Mat_<double> &Q, cv::Mat_<double> &m);
  void CopyDescriptors(const cv::Mat_<float> &descs, vector< cv::Ptr<Plane> > &planes);
  void TrackPriorPlanes(vector<cv::Ptr<P::PKeypoint> > &keys,
        const vector< cv::Ptr<Plane> > &priorPlanes, vector< cv::Ptr<Plane> > &tentPlanes);
  void MatchKeysNNR(const cv::Mat_<float> &queryDescs, 
        const cv::Mat_<float> &trainDescs, vector<vector<cv::DMatch> > &matches, 
        vector<int> &selectedMatches);
  void SetPriorPlanes(vector< cv::Ptr<Plane> > &planes, vector< cv::Ptr<Plane> > &priorPlanes);
  void ReleaseInteractions(vector< cv::Ptr<Plane> > &planes);
  void GetSelectedQueryKeys(const vector<cv::Ptr<P::PKeypoint> > &queryKeys, 
        vector<vector<cv::DMatch> > &matches, vector<int> &selected, 
        vector<cv::Ptr<P::PKeypoint> > &selectedKeys);
  void ConstrainedMatching(vector<cv::Ptr<Plane> > &planes,vector<cv::Ptr<P::PKeypoint> > &queryKeys, 
        vector<cv::Ptr<P::PKeypoint> > &trainKeys, vector<vector<cv::DMatch> > &matches, 
        vector<int> &selectedMatches, double inlDist, bool markOutlier);
  void SetLastKeypoints(vector<cv::Ptr<P::PKeypoint> > &svKeys, vector< cv::Ptr<Plane> > &planes);
  void MeanKeys(vector< cv::Ptr<P::PKeypoint> > &keys, cv::Point2d &ptMean);



  inline bool Contains(const vector<int> &idx, int id);



public:
  Parameter param;
  cv::Mat dbg;

  ItMoSPlanes(cv::Ptr<cv::DescriptorMatcher> &descMatcher,
              Parameter _param=Parameter());
  ~ItMoSPlanes();

  void Clear();
  bool Operate(const vector< cv::Ptr<P::PKeypoint> > &keys, const cv::Mat_<float> &descs, 
               vector< cv::Ptr<Plane> > &planes);
  //vector<cv::DMatch>& GetMatches(){ return matches; }
  void Draw(cv::Mat &image, vector< cv::Ptr<Plane> > &planes, unsigned detail=0);
};




/*********************** INLINE METHODES **************************/
inline bool ItMoSPlanes::Contains(const vector<int> &idx, int id)
{
  for (unsigned i=0; i<idx.size(); i++)
    if (idx[i]==id)
      return true;
  return false;
}


}

#endif

