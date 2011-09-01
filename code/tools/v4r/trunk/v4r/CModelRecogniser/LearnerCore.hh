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
#include "RobustEstimators.hh"
#include "CModel.hh"
#include "polygontest_cv.h"
#include "ObjectLocation.hh"
#include "v4r/PGeometry/Pose.hh"
#include "Codebook.hh"
#include "MSCodebook.hh"
#include "MatchFilter.hh"
#include "ConfValues.hh"


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
    cv::Mat distortion;
    double minProbLearn;                // min conf to insert frame
    double maxProbLearn;                // max conf to insert a frame
    unsigned maxRandTrials;             // max. number of trials for pose ransac (recognition)
    unsigned numLoTrials;               // number of local ransac trials
    double etaRansac;                   // eta for pose ransac
    double inlDistRansac;               // inlier dist pose ransac
    bool forceLearning;
    unsigned maxLinkViews;
    bool useRecognisedPose;
    double cmpViewAngle;                // test recognised poses [°]
    bool computeObjectCenter;
    double thrDesc;

    Parameter(int w=640, int h=480, double _minProbLearn=.25, double _maxProbLearn=.4, 
              unsigned trials=1000, unsigned loTrials=50, double eta=0.01, double inlDist=2, 
              bool force=false, unsigned maxLink=3, bool useRecPose=false, 
              double dAlpha=20., bool center=false, double _thrDesc=0.3) 
      : width(w), height(h), minProbLearn(_minProbLearn), maxProbLearn(_maxProbLearn), 
        maxRandTrials(trials), numLoTrials(loTrials), etaRansac(eta), inlDistRansac(inlDist), 
        forceLearning(force),  maxLinkViews(maxLink), useRecognisedPose(useRecPose), 
        cmpViewAngle(dAlpha), computeObjectCenter(center), thrDesc(_thrDesc) {}
  };

private:
  double scaleWidth, scaleHeight;        // xyImage * scale = xyCloud

  cv::Ptr<KeypointDetector> detector;
  cv::Ptr<cv::DescriptorExtractor> extractor;
  cv::Ptr<cv::DescriptorMatcher> matcher;

  cv::Ptr<RobustEstimators> estimator;

  cv::Mat imgGray;             // gray scale image
  vector<cv::KeyPoint> cvKeys;   // just a opencv keypoint container

  cv::Ptr<View> view;                   // current view
  cv::Ptr<CModel> model;                // object model container for learning
  Pose invRefGlobal;                    // reference pose global coordinates
  Pose refObject;                       // reference pose object coordinates

  cv::Ptr<Codebook> codebook;           // codebook to recognise views

  void Setup(const cv::Mat &image, const cv::Mat_<cv::Vec4f> &cloud, 
         cv::Mat &R, cv::Mat &T, const string &oid);
  void AlignKeypoints3D(const cv::Mat_<cv::Vec4f> &cloud, vector< cv::Ptr<PKeypoint> > &keys, Pose &pose);
  void Recognise(cv::Mat_<float> &queryDescs, vector<cv::Ptr<PKeypoint> > &queryKeys, 
         vector<Pose> &poses, vector<double> &probs, vector<unsigned> &idxViews, 
         vector<vector<cv::DMatch> > &matches);
  void LinkViews(vector<cv::Ptr<PKeypoint> > &queryKeys, vector<cv::Ptr<PKeypoint> > &trainKeys, 
         vector<cv::DMatch> &matches);
  int RecogniseAndLink(double &maxConf);
  cv::Point3d GetMean3D(vector<cv::Ptr<View> > &views);
  bool Translate3D(vector<cv::Ptr<View> > &views, cv::Point3d T );
  int AddViewToModel();
  bool ComparePose(Pose &pose1, Pose &pose2);
  void ComputeViewRay(Pose &pose, cv::Point3d &objCenter, cv::Point3d &vr);
  cv::Point3d GetMean3D(View& view);
  void SetPointCloud(const cv::Mat_<cv::Vec4f> &cloud, const cv::Mat_<uchar> &mask, 
         Pose &pose, vector<cv::Vec4f> &vecCloud);


  inline int toCloudX(double x);
  inline int toCloudY(double y);



public:
  Parameter param;
  cv::Mat dbg;

  LearnerCore(cv::Ptr<KeypointDetector> &keyDetector,
                 cv::Ptr<cv::DescriptorExtractor> &descExtractor,
                 cv::Ptr<cv::DescriptorMatcher> &descMatcher,
                 Parameter _param=Parameter());
  ~LearnerCore();


  void Clear();
  void SetModel(cv::Ptr<CModel> &_model);
  cv::Ptr<CModel>& GetModel();
  int Learn(const cv::Mat &image, const cv::Mat_<cv::Vec4f> &cloud, cv::Mat R, cv::Mat T, const string &oid, cv::Mat mask=cv::Mat());
  void GetViewRays(vector<cv::Point3d> &vr);

  void DetectKeypoints(const cv::Mat &image, vector<cv::Ptr<PKeypoint> > &keys, cv::Mat mask=cv::Mat());
  int InsertToModel(const vector<cv::Ptr<PKeypoint> > &keys,cv::Mat R,cv::Mat T, const string &oid);

  void SetCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_distortion);
};




/*********************** INLINE METHODES **************************/
inline int LearnerCore::toCloudX(double x)
{
  return (int)(x*scaleWidth+.5);
}

inline int LearnerCore::toCloudY(double y)
{
  return (int)(y*scaleHeight+.5);
}



}

#endif

