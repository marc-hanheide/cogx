#ifndef _EXTRACTOR_I_H_
#define _EXTRACTOR_I_H_

#include "Recognizer.h"
#include "sift.h"

/** Implementation of the SiftExtractor servant */
class ExtractorI {
public:
  ExtractorI();
  virtual ~ExtractorI();

  FeatureVector * extractFromRgb(char * rgbBuffer,
				 int width, int height);
  
protected:

  SIFT::Image  m_loweImage;
  void rgb2lowe(char *rgbBuffer, int width, int height);
  void allocateLoweImage(int nRows, int nCols);
  FeatureVector * extractFromLoweImage();
};

#endif

