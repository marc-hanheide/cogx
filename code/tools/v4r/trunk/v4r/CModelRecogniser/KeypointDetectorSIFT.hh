/**
 * $Id$
 * Johann Prankl, 2010-11-29 
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_KEYPOINT_DETECTOR_SIFT_HH
#define P_KEYPOINT_DETECTOR_SIFT_HH

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>


#include "KeypointDetector.hh"


namespace P
{


class KeypointDetectorSIFT : public KeypointDetector
{
private:
  cv::Ptr<cv::FeatureDetector> detector;

public:
  KeypointDetectorSIFT();
  ~KeypointDetectorSIFT();

  void Detect(const cv::Mat &img, vector<cv::Ptr<PKeypoint> > &keys, cv::Mat mask=cv::Mat());
};


/************************** INLINE METHODES ******************************/



}

#endif

