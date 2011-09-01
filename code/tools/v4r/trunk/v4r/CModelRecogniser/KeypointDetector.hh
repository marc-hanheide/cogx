/**
 * $Id$
 */

#ifndef P_KEYPOINT_DETECTOR_HH
#define P_KEYPOINT_DETECTOR_HH

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <vector>
#include "PKeypoint.hh"

namespace P
{

using namespace std;

class KeypointDetector
{
public:
  int dist;

  KeypointDetector() : dist(1) {};
  ~KeypointDetector(){};


  virtual void Detect(const cv::Mat &img, vector<cv::Ptr<PKeypoint> > &keys, cv::Mat mask=cv::Mat()) = 0;
  virtual void DetectDescriptor(const cv::Mat &img, vector<cv::Ptr<PKeypoint> > &keys, cv::Mat &descriptors, cv::Mat mask=cv::Mat()) { cout<<"Not available!"<<endl;};

  static void Draw(cv::Mat &img, const vector<cv::Ptr<PKeypoint> > &keys, cv::Scalar col=CV_RGB(255,0,0), unsigned detail=0);
  static void Draw(cv::Mat &img, const vector<cv::Ptr<PKeypoint> > &trainKeys, const vector<cv::Ptr<PKeypoint> > &queryKeys, const vector<cv::DMatch> &matches, cv::Scalar col=CV_RGB(255,0,0), unsigned detail=0);
  static void Draw(cv::Mat &img, const vector<cv::Ptr<PKeypoint> > &trainKeys, const vector<cv::Ptr<PKeypoint> > &queryKeys, const vector<vector<cv::DMatch> > &matches, cv::Scalar col=CV_RGB(255,0,0), unsigned detail=0);


};








} //--END--

#endif

