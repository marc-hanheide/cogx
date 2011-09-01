/**
 * $Id$
 */

#ifndef P_AR_DETECT_GT_POSE_HH
#define P_AR_DETECT_GT_POSE_HH

#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "v4r/elldetect/EDWrapper.hh"
#include "v4r/PMath/PVector.hh"
#include "arGTPatternDetector.hh"


namespace P 
{


/**
 * arDetectGTPose
 */
class arDetectGTPose
{
public:
  class Parameter
  {
  public:
    int minEllipses;              // to use the pattern for calibration
    double xDist, yDist;          // distance of the ellipses in x an y direction
    int xOffs, yOffs;             // offset of the coordinate system (indices)
    int xMin, xMax, yMin, yMax;   // ellipse indices in x and y direction
    double dist;                  // threshold to accept ellipse location (affine model)

    Parameter(int minElls=9, double xd=30, double yd=30, int _xOffs=4, 
       int _yOffs=0, int _xMin=-2, int _xMax=3, int _yMin=-3, int _yMax=5, double _dist=10.)
     : minEllipses(minElls), xDist(xd), yDist(yd), xOffs(_xOffs), yOffs(_yOffs),
       xMin(_xMin), xMax(_xMax), yMin(_yMin), yMax(_yMax), dist(_dist) {}

  };

private:
  cv::Mat grayImage;

  cv::Ptr<arGTPatternDetector> detector;

  Parameter param;



public:
  cv::Mat dbg;

  arDetectGTPose(){}
  arDetectGTPose(const string &_pat1, const string &_pat2, Parameter p1=Parameter(), Parameter p2=Parameter());
  ~arDetectGTPose();

  bool GetPose(const cv::Mat &img, const cv::Mat& intrinsic, const cv::Mat& distCoeffs, cv::Mat &R, cv::Mat &T, EllPattern &pattern);

};


}
#endif

