#ifndef _SIFT_DETECT_ENGINE_H_
#define _SIFT_DETECT_ENGINE_H_

#include "sift.h"
#include "vision/utils/VisionUtils.h"
#include "vision/idl/Vision.hh"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "Recognizer.h"
#include "extractorI.h"
#include "matcher.h"


using namespace Vision; using namespace cast; using namespace std; 
//using namespace boost; //default useful namespaces, fix to reflect your own code

class SiftDetectEngine {
public:
  SiftDetectEngine(int _img_maxwidth, int _img_maxheight, string strDatabaseFile="");
  ~SiftDetectEngine();

  void detectObject( ImageFrame* _img, const BBox2D &_bbox, string& olabel, double& oProb );
  
  void alignFeatures( FeatureVector &vFeatures, BBox2D &_bbox );

protected:
  void roiFilter( FeatureVector &vFeatures, BBox2D &_bbox );

  Matcher *m_pmatcher;
  ExtractorI *m_pextractor;
  MatchInfo *m_pMatchInfo;

  IplImage *m_srcimg;
};

#endif
