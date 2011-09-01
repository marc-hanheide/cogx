/**
 * $Id$
 */


#include "EllCalibrate.hh"

#define DEBUG


namespace P 
{



/************************************************************************************
 * Constructor/Destructor
 */
EllCalibrate::EllCalibrate(Parameter p)
 : param(p)
{ 
  detector = new EllPatternDetector(EllPatternDetector::Parameter(
                   p.xDist,p.yDist,p.xMin,p.xMax,p.yMin,p.yMax,p.dist));
}

EllCalibrate::~EllCalibrate()
{
}

/**
 * Test the quotient of inner and outer ellipse
 * TODO:
 * - that's a really stubide threshold
 */
bool EllCalibrate::IsRingOK(EllPattern &patt)
{
  if (fabs(patt.quotientRing-param.quotientRing) < 0.1)
    return true;
  return false;
}





/******************************* PUBLIC ***************************************/

/**
 * clear
 */
void EllCalibrate::clear()
{
  grayImage.release();
  patterns.clear();
}

/**
 * Detect pattern and add to container for calibration
 */
bool EllCalibrate::Add(const cv::Mat &img)
{
  if( img.type() != CV_8U ) cv::cvtColor( img, grayImage, CV_BGR2GRAY );
  else grayImage = img;

  vector<cv::Ptr<EllPattern> > tmp;

  #ifdef DEBUG
  detector->dbg = dbg;
  #endif

  detector->Detect(grayImage, tmp);
  
  if (tmp.size()>0 && tmp[0]->imgPoints.size()>=param.minEllipses && IsRingOK(*tmp[0]))
  {
    patterns.push_back(tmp[0]);
    #ifdef DEBUG
    cout<<"Pattern number: "<<patterns.size()-1
        <<" with "<<tmp[0]->imgPoints.size()<<" ellipses"<<endl;
    #endif
    return true;
  }
  else
  {
    #ifdef DEBUG
    if (!dbg.empty())
    {
      cv::line(dbg,cv::Point(img.cols,0),cv::Point(0,img.rows), CV_RGB(255,0,0), 2);
      cv::line(dbg,cv::Point(0,0),cv::Point(img.cols,img.rows), CV_RGB(255,0,0), 2);
    }
    cout<<"("<<patterns.size()<<") Skiped frame!"<<endl;
    #endif
    return false;
  }
}

/**
 * Intrinsic and extrinsic calibration of the camera
 * (see cv::calibrateCamera)
 * @return the final calibration error
 */
double EllCalibrate::Calibrate(cv::Mat& intrinsic, cv::Mat& distCoeffs, vector<cv::Mat>& rvecs, vector<cv::Mat>& tvecs, int flags)
{
  if (grayImage.empty() || patterns.size()==0)
    return -1.;

  vector<vector<cv::Point3f> > objectPoints;
  vector<vector<cv::Point2f> > imagePoints;

  int step = (patterns.size()>param.maxImages?patterns.size()/param.maxImages:1);

  for (int i=0; i<patterns.size(); i+=step)
  {
    objectPoints.push_back(patterns[i]->objPoints);
    imagePoints.push_back(patterns[i]->imgPoints);
  }

  return cv::calibrateCamera(objectPoints, imagePoints, grayImage.size(), 
                             intrinsic, distCoeffs, rvecs, tvecs, flags);
}

/**
 * Get the extrinsic camera parameter with respect to the pattern coordinate system
 * @param R 3x3 rotation matrix
 * @param T 3x1 translation vector
 */
bool EllCalibrate::GetPose(const cv::Mat &img, const cv::Mat& intrinsic, const cv::Mat& distCoeffs, cv::Mat &R, cv::Mat &T)
{
  cv::Mat rod;

  if( img.type() != CV_8U ) cv::cvtColor( img, grayImage, CV_BGR2GRAY );
  else grayImage = img;

  vector<cv::Ptr<EllPattern> > tmp;

  detector->Detect(grayImage, tmp);

  if (tmp.size()>0 && tmp[0]->imgPoints.size()>=param.minEllipses && IsRingOK(*tmp[0]))
  {
    cv::solvePnP(cv::Mat(tmp[0]->objPoints), cv::Mat(tmp[0]->imgPoints), intrinsic, distCoeffs, rod, T, false);
    R= cv::Mat(3,3,CV_64F);
    cv::Rodrigues(rod,R);
    return true;
  }
  return false;
}




}












