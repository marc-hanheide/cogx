/**
 * $Id$
 * Johann Prankl, 2010-11-29 
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_ROBUST_ESTIMATORS_HH
#define P_ROBUST_ESTIMATORS_HH

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "v4r/PGeometry/Pose.hh"
#include "PKeypoint.hh"
#include "v4r/PGeometry/PHomography.hh"



namespace P
{


class RobustEstimators
{
public:
  class Parameter
  {
  public:
    unsigned maxRandTrials;             // max ransac trials
    unsigned numLoTrials;
    double etaRansac;
    double inlDistRansac;               // inlier distance [Px]
    double inlThr3D;                    // test ransac pose against 3d points from kinect [m]
    cv::Mat intrinsic;
    cv::Mat distortion;
    Parameter(unsigned rand=2000, unsigned loRand=50, double eta=0.01, double dist=2., double dist3d=0.2) 
      : maxRandTrials(rand), numLoTrials(loRand), etaRansac(eta), inlDistRansac(dist), inlThr3D(dist3d) {}
  };

private:
  static int NUM_RANSAC_POINTS;

  bool GetPoints(std::vector< cv::Ptr<PKeypoint> > &keys, 
          std::vector< cv::Ptr<PKeypoint> > &model, std::vector<cv::DMatch> &matches, 
          std::vector<PKeypoint*> &ptsImage, std::vector<PKeypoint*> &ptsModel);
  void GetRandIdx(unsigned size, unsigned num, std::vector<unsigned> &idx);
  void FitPoseRANSAC(std::vector<PKeypoint*> &ptsImage, std::vector<PKeypoint*> &ptsModel, Pose &pose, double &conf, bool check3D);
  void CountInlier(std::vector<PKeypoint*> &ptsImage, std::vector<PKeypoint*> &ptsModel, Pose &pose, double &cnt);
  void CountInlier3D(std::vector<PKeypoint*> &ptsImage, std::vector<PKeypoint*> &ptsModel, Pose &pose, double &cnt);

  void FitPoseLoRANSAC(std::vector<PKeypoint*> &ptsImage, std::vector<PKeypoint*> &ptsModel, Pose &pose, double &sig);
  void CountInlier(std::vector<PKeypoint*> &moKeys, std::vector<PKeypoint*> &imKeys, double H[9], double &inl);
  void GetInlier(std::vector<PKeypoint*> &moKeys, std::vector<PKeypoint*> &imKeys, double H[9], std::vector<unsigned> &idxInlier);

  inline bool Contains(const std::vector<unsigned> &idx, unsigned num);

 
public:
  Parameter param;
  cv::Mat dbg;

  RobustEstimators();
  RobustEstimators(Parameter _param);
  ~RobustEstimators();

  double RansacPose(std::vector< cv::Ptr<PKeypoint> > &keys, std::vector< cv::Ptr<PKeypoint> > &model, 
           std::vector<cv::DMatch> &matches, Pose &pose, bool check3D=false);
  double LoRansacPose(std::vector< cv::Ptr<PKeypoint> > &keys, std::vector< cv::Ptr<PKeypoint> > &model, 
           std::vector<cv::DMatch> &matches, Pose &pose);
  void SetCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_distortion);

  void DrawInlier(cv::Mat &img, std::vector< cv::Ptr<PKeypoint> > &keys, std::vector< cv::Ptr<PKeypoint> > &model, std::vector<cv::DMatch> &matches, Pose &pose);

};


/************************** INLINE METHODES ******************************/

inline bool RobustEstimators::Contains(const std::vector<unsigned> &idx, unsigned num)
{
  for (unsigned i=0; i<idx.size(); i++)
    if (idx[i]==num)
      return true;
  return false;
}


}

#endif

