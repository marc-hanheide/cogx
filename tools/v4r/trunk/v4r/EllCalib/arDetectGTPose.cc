/**
 * $Id$
 */


#include "arDetectGTPose.hh"

#define DEBUG


namespace P 
{



/************************************************************************************
 * Constructor/Destructor
 */
arDetectGTPose::arDetectGTPose(const string &_pat1, const string &_pat2, Parameter p1, Parameter p2)
 : param(p1)
{ 
  detector = new arGTPatternDetector(_pat1, _pat2,
                     arGTPatternDetector::Parameter(
                       p1.xDist,p1.yDist,p1.xOffs,p1.yOffs, p1.xMin,p1.xMax,p1.yMin,p1.yMax,p1.dist),
                     arGTPatternDetector::Parameter(
                       p2.xDist,p2.yDist,p2.xOffs,p2.yOffs, p2.xMin,p2.xMax,p2.yMin,p2.yMax,p2.dist));
}

arDetectGTPose::~arDetectGTPose()
{
}





/******************************* PUBLIC ***************************************/

/**
 * Get the extrinsic camera parameter with respect to the pattern coordinate system
 * @param R 3x3 rotation matrix
 * @param T 3x1 translation vector
 */
bool arDetectGTPose::GetPose(const cv::Mat &img, const cv::Mat& intrinsic, const cv::Mat& distCoeffs, cv::Mat &R, cv::Mat &T, EllPattern &pattern)
{
  cv::Mat rod;

  #ifdef DEBUG
  detector->dbg = dbg;
  #endif

  if(detector->Detect(img, pattern))
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












