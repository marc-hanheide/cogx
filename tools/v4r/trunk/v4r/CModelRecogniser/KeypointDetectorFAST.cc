/**
 * $Id$
 * Johann Prankl, 2010-11-29 
 * prankl@acin.tuwien.ac.at
 */


#include "KeypointDetectorFAST.hh"


namespace P 
{

KeypointDetectorFAST::KeypointDetectorFAST(Parameter _param)
 : param(_param)
{
  detector = new cv::FastFeatureDetector(param.threshold, true);

  /*cv::GoodFeaturesToTrackDetector::Params params;
  params.useHarrisDetector = false;
  params.maxCorners = 500;
  params.qualityLevel = 0.1;
  params.minDistance = 3;

  detector = new cv::GoodFeaturesToTrackDetector();*/
}

KeypointDetectorFAST::~KeypointDetectorFAST()
{
}




/************************************** PRIVATE ************************************/






/************************************** PUBLIC ************************************/
/**
 * compute dog and sift descriptor
 */
void KeypointDetectorFAST::Detect(const cv::Mat &img, vector<cv::Ptr<PKeypoint> > &keys, cv::Mat mask)
{
  cv::Mat grayImage = img;
  if( img.type() != CV_8U ) cv::cvtColor( img, grayImage, CV_BGR2GRAY );

  detector->detect(grayImage, cvKeys);

  keys.resize(cvKeys.size());
  for( int i = 0; i < keys.size(); i++)
  {
      keys[i] = new PKeypoint(cvKeys[i]);
      keys[i]->size*=param.mulScale;
      keys[i]->id = i;
  }
}



}

