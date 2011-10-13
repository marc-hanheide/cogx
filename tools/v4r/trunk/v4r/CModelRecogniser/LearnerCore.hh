/**
 * $Id$
 */

#ifndef P_LEARNER_CORE_HH
#define P_LEARNER_CORE_HH

#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "KeypointDetector.hh"
//#include "RobustEstimators.hh"
#include "CModel.hh"
#include "polygontest_cv.h"
#include "ObjectLocation.hh"
#include "v4r/PGeometry/Pose.hh"
#include "Codebook.hh"
#include "MeanShiftCodebook.hh"
#include "MeanCodebook.hh"
//#include "MatchFilter.hh"
#include "ProbModel.hh"


namespace P
{


class LearnerCore
{
public:
  class Parameter
  {
  public:
    int width, height;                  //willow 1280 1024
    cv::Mat intrinsic;
    cv::Mat distCoeffs;
    unsigned maxRandTrials;             // max. number of trials for pose ransac (recognition) (1000)
    unsigned numLoTrials;               // number of local ransac trials (50)
    double etaRansac;                   // eta for pose ransac (0.01)
    double inlDistRansac;               // inlier dist pose ransac (2)
    bool forceLearning;                 // force learning of views without links to other views 
    double minAngleBetweenViews;        // test angle between viewpoints [°] (20)
    bool computeObjectCenter;           //
    double thrDesc;                     // (0.4)
    double sigmaDesc;                   // (0.2)
    double alignDist;                   // distance to align projetions of 3d and 2d points
    double maxDistPoint3d;              // max dist point3d [m]
    double maxConfToLearn;

    Parameter(int w=640, int h=480, unsigned trials=1000, unsigned loTrials=50, double eta=0.01, 
              double inlDist=2, double minAngle=20., double _thrDesc=0.4, double _sigmaDesc=.2, 
              double _alignDist=2., bool force=true, bool computeCenter=false, double dist3d=.01,
              double _maxConfToLearn=0.5) 
      : width(w), height(h), maxRandTrials(trials), numLoTrials(loTrials), etaRansac(eta), 
        inlDistRansac(inlDist), minAngleBetweenViews(minAngle), 
        thrDesc(_thrDesc), sigmaDesc(_sigmaDesc), alignDist(_alignDist), 
        forceLearning(force), computeObjectCenter(computeCenter),maxDistPoint3d(dist3d),
        maxConfToLearn(_maxConfToLearn) {}
  };

private:
  cv::Ptr<KeypointDetector> detector;
  cv::Ptr<cv::DescriptorExtractor> extractor;
  cv::Ptr<cv::DescriptorMatcher> matcher;

  //cv::Ptr<RobustEstimators> estimator;

  cv::Mat imgGray;             // gray scale image
  vector<cv::KeyPoint> cvKeys;   // just a opencv keypoint container
  vector<cv::Point3f> cloud3f;
  vector<cv::Point2f> projCloud2f;

  cv::Ptr<View> view;                   // current view
  cv::Ptr<CModel> model;                // object model container for learning
  Pose invRefGlobal;                    // reference pose global coordinates
  Pose refObject;                       // reference pose object coordinates

  cv::Ptr<Codebook> codebook;           // codebook to recognise views

  int Setup(const cv::Mat &image, const cv::Mat_<cv::Vec4f> &cloud, 
         cv::Mat &R, cv::Mat &T, const string &oid);
  double AngleBetween(Pose &pose1, Pose &pose2);
  void AlignKeypoints3D(const cv::Mat_<cv::Vec4f> &cloud, const cv::Mat &mask, Pose &pose, 
        const vector< cv::Ptr<PKeypoint> > &keys, std::vector<cv::Point3f> &pts);
  int AddViewToModel(std::vector<cv::Point3f> &keys3f);
  cv::Point3d GetMean3D(vector<cv::Ptr<View> > &views);
  void Translate3D(vector<cv::Ptr<View> > &views, cv::Point3d T );
  void ComputeViewRay(Pose &pose, cv::Point3d &objCenter, cv::Point3d &vr);
  void SetObjectCenter();
  void AddViewToViewSphere();
  void SetPointCloud(const cv::Mat_<cv::Vec4f> &cloud, const cv::Mat_<uchar> &mask, 
         Pose &pose, vector<cv::Vec4f> &vecCloud);

  inline void ProjectPoint(cv::Point3f &pt3, cv::Mat &intrinsic, cv::Mat &distCoeffs, cv::Point2f &pt);
  inline int GetIndexMax(const vector<int> &tab);


public:
  Parameter param;
  cv::Mat dbg;

  LearnerCore(cv::Ptr<KeypointDetector> &keyDetector,
                 cv::Ptr<cv::DescriptorExtractor> &descExtractor,
                 cv::Ptr<cv::DescriptorMatcher> &descMatcher,
                 Parameter p=Parameter());
  ~LearnerCore();


  void Clear();
  void SetModel(cv::Ptr<CModel> &_model);
  cv::Ptr<CModel>& GetModel();
  int Learn(const cv::Mat &image, const cv::Mat_<cv::Vec4f> &cloud, cv::Mat R, cv::Mat T, const string &oid, cv::Mat mask=cv::Mat());
  void SetReferenceObjectPose(const Pose &pose);
  void GetViewRays(vector<cv::Point3d> &vr);

  void SetCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_distCoeffs);
};




/*********************** INLINE METHODES **************************/

inline void LearnerCore::ProjectPoint(cv::Point3f &pt3, cv::Mat &intrinsic, cv::Mat &distCoeffs, cv::Point2f &pt)
{
  if (distCoeffs.empty())
  {
    ProjectPoint2Image(&pt3.x, intrinsic.ptr<double>(), &pt.x);
  }
  else
  {
    ProjectPoint2Image(&pt3.x, intrinsic.ptr<double>(), distCoeffs.ptr<double>(), &pt.x);
  }
}

inline int LearnerCore::GetIndexMax(const vector<int> &tab)
{
  int max=INT_MIN, idx=INT_MIN;

  for (int i=0; i<tab.size(); i++)
    if (tab[i]>max)
    {
      max=tab[i];
      idx = i;
    }

  return idx;
}



}

#endif

