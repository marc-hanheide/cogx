/**
 * $Id$
 */


#include "DetectGTPose.hh"

#define DEBUG


namespace P 
{



/************************************************************************************
 * Constructor/Destructor
 */
DetectGTPose::DetectGTPose(Parameter p1, Parameter p2)
 : param(p1)
{ 
  detector = new GTPatternDetector(
                   GTPatternDetector::Parameter(p1.ids[0], p1.ids[1],
                     p1.xDist,p1.yDist,p1.xOffs,p1.yOffs, p1.xMin,p1.xMax,p1.yMin,p1.yMax,p1.dist),
                   GTPatternDetector::Parameter(p2.ids[0], p2.ids[1],
                     p2.xDist,p2.yDist,p2.xOffs,p2.yOffs, p2.xMin,p2.xMax,p2.yMin,p2.yMax,p2.dist));
}

DetectGTPose::~DetectGTPose()
{
}





/******************************* PUBLIC ***************************************/

/**
 * Get the extrinsic camera parameter with respect to the pattern coordinate system
 * @param R 3x3 rotation matrix
 * @param T 3x1 translation vector
 */
bool DetectGTPose::GetPose(const cv::Mat &img, const cv::Mat& intrinsic, const cv::Mat& distCoeffs, cv::Mat &R, cv::Mat &T, EllPattern &pattern)
{
  cv::Mat rod;

  if( img.type() != CV_8U ) cv::cvtColor( img, grayImage, CV_BGR2GRAY );
  else grayImage = img;

  #ifdef DEBUG
  detector->dbg = dbg;
  #endif

  if(detector->Detect(grayImage, pattern))
  {
    if (pattern.objPoints.size() > param.minEllipses)
    {
      cv::solvePnP(cv::Mat(pattern.objPoints), cv::Mat(pattern.imgPoints), intrinsic, distCoeffs, rod, T, false);
      R= cv::Mat(3,3,CV_64F);
      cv::Rodrigues(rod,R);
    }
    return true;
  }
  return false;
}




}












