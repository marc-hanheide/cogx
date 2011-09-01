/**
 * $Id$
 * Johann Prankl, 2010-11-29 
 * prankl@acin.tuwien.ac.at
 */


#include "KeypointDetectorSIFT.hh"


namespace P 
{

KeypointDetectorSIFT::KeypointDetectorSIFT()
{
  detector = new cv::SiftFeatureDetector();
}

KeypointDetectorSIFT::~KeypointDetectorSIFT()
{
}




/************************************** PRIVATE ************************************/






/************************************** PUBLIC ************************************/
/**
 * compute dog and sift descriptor
 */
void KeypointDetectorSIFT::Detect(const cv::Mat &img, vector<cv::Ptr<PKeypoint> > &keys, cv::Mat mask)
{
  cv::Mat grayImage = img;
  if( img.type() != CV_8U ) cv::cvtColor( img, grayImage, CV_BGR2GRAY );

  vector<cv::KeyPoint> cvKeys;

  detector->detect(grayImage,cvKeys,mask);

  PKeypoint::ConvertFromCv(cvKeys,keys);
}


} // -- THE END --
