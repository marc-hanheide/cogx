/**
 * $Id$
 * Johann Prankl, 2010-11-29 
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_KEYPOINT_DETECTOR_SURF_HH
#define P_KEYPOINT_DETECTOR_SURF_HH

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>


#include "KeypointDetector.hh"


namespace P
{


class KeypointDetectorSURF : public KeypointDetector
{
public:
  class Parameter
  {
  public:
    double hessianThreshold;
    int octaves;
    int octaveLayers;

    Parameter() 
     : hessianThreshold(400), octaves(3), octaveLayers(4) {}
    Parameter(double hes, int oct, int octLayer)
     : hessianThreshold(hes), octaves(oct), octaveLayers(octLayer) {}
  };

private:
  
public:
  Parameter param;

  KeypointDetectorSURF();
  KeypointDetectorSURF(Parameter _param);
  ~KeypointDetectorSURF();

  void Detect(const cv::Mat &img, vector<cv::Ptr<PKeypoint> > &keys, cv::Mat mask=cv::Mat());
};


/************************** INLINE METHODES ******************************/



}

#endif

