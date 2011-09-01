/**
 * $Id$
 * Johann Prankl, 2010-11-29 
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_KEYPOINT_DETECTOR_FAST_HH
#define P_KEYPOINT_DETECTOR_FAST_HH

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>


#include "KeypointDetector.hh"


namespace P
{


class KeypointDetectorFAST : public KeypointDetector
{
public:
  class Parameter
  {
  public:
    int threshold;
    double mulScale;
    Parameter(int thr=15, double ms=3) : threshold(thr), mulScale(ms) {}
  };

private:
  vector<cv::KeyPoint> cvKeys;
  cv::Ptr<cv::FeatureDetector> detector;
  
public:
  Parameter param;

  KeypointDetectorFAST(Parameter _param=Parameter());
  ~KeypointDetectorFAST();

  void Detect(const cv::Mat &img, vector<cv::Ptr<PKeypoint> > &keys, cv::Mat mask=cv::Mat());
};


/************************** INLINE METHODES ******************************/



}

#endif

