/**
 * $Id$
 */

#ifndef P_ELL_CALIBRATE_HH
#define P_ELL_CALIBRATE_HH

#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "v4r/elldetect/EDWrapper.hh"
#include "v4r/PMath/PVector.hh"
#include "EllPatternDetector.hh"


namespace P 
{


/**
 * EllCalibrate
 */
class EllCalibrate
{
public:
  class Parameter
  {
  public:
    double quotientRing;          // to accept the coordinate system
    int minEllipses;              // to use the pattern for calibration
    int maxImages;                // which should be selected for calibration
    double xDist, yDist;          // distance of the ellipses in x an y direction
    int xMin, xMax, yMin, yMax;   // ellipse indices in x and y direction
    double dist;                  // threshold to accept ellipse location (affine model)
    
    Parameter(double quotRing=0.5, int minEll=9, int maxImg=100, double xd=30, double yd=30, 
       int _xMin=-2, int _xMax=3, int _yMin=-3, int _yMax=5, double _dist=10.) 
     : quotientRing(0.5), minEllipses(minEll), maxImages(maxImg), xDist(xd), yDist(yd), 
       xMin(_xMin), xMax(_xMax), yMin(_yMin), yMax(_yMax), dist(_dist) {} 
  };

private:
  Parameter param;
  cv::Mat grayImage;

  cv::Ptr<EllPatternDetector> detector;

  bool IsRingOK(EllPattern &patt);


public:
  cv::Mat dbg;

  vector<cv::Ptr<EllPattern> > patterns;

  EllCalibrate(Parameter p=Parameter());
  ~EllCalibrate();

  void clear();
  bool Add(const cv::Mat &img);
  double Calibrate(cv::Mat& intrinsic, cv::Mat& distCoeffs, vector<cv::Mat>& rvecs, vector<cv::Mat>& tvecs, int flags=0);
  bool GetPose(const cv::Mat &img, const cv::Mat& intrinsic, const cv::Mat& distCoeffs, cv::Mat &R, cv::Mat &T);

};


}
#endif

