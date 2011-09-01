/**
 * $Id$
 * Johann Prankl, 2010-11-29 
 * prankl@acin.tuwien.ac.at
 */


#include "KeypointDetectorSURF.hh"


namespace P 
{

KeypointDetectorSURF::KeypointDetectorSURF()
{
}

KeypointDetectorSURF::KeypointDetectorSURF(Parameter _param)
 : param(_param)
{
}

KeypointDetectorSURF::~KeypointDetectorSURF()
{
}




/************************************** PRIVATE ************************************/






/************************************** PUBLIC ************************************/
/**
 * compute dog and sift descriptor
 */
void KeypointDetectorSURF::Detect(const cv::Mat &img, vector<cv::Ptr<PKeypoint> > &keys, cv::Mat mask)
{
  cv::Mat grayImage = img;
  if( img.type() != CV_8U ) cv::cvtColor( img, grayImage, CV_BGR2GRAY );

  CvSURFParams surfParam = CvSURFParams();
  surfParam.hessianThreshold = param.hessianThreshold;
  surfParam.nOctaves = param.octaves;
  surfParam.nOctaveLayers = param.octaveLayers;
 
  CvMat _img = grayImage, _mask, *pmask = 0;
  if( mask.data )
      pmask = &(_mask = mask);
  cv::MemStorage storage(cvCreateMemStorage(0));
  cv::Seq<CvSURFPoint> kp;
  cvExtractSURF(&_img, pmask, &kp.seq, 0, storage, surfParam, 0);
  cv::Seq<CvSURFPoint>::iterator it = kp.begin();
  size_t i, n = kp.size();

  keys.resize(n);
  for( i = 0; i < n; i++, ++it )
  {
      const CvSURFPoint& kpt = *it;
      keys[i] = new PKeypoint(cv::Point2d(kpt.pt.x,kpt.pt.y), kpt.size, kpt.dir, kpt.hessian);
      keys[i]->id = i;
  }
}



}

