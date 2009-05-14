#include "extractorI.h"

using namespace std;

ExtractorI::ExtractorI() {

}

ExtractorI::~ExtractorI() {

}

//----------------------------------------------------------------------------------
FeatureVector * ExtractorI::extractFromRgb(char * rgbBuffer, int width, int height)
{
    rgb2lowe(rgbBuffer, (int) width, (int) height);
    if ((m_loweImage.rows != height) ||  (m_loweImage.cols != width))
      cerr << "ExtractorI::extractFromRgb: creation of m_loweImage has failed" << endl;

    // extract the features
    return extractFromLoweImage();
} 


//----------------------------------------------------------------------------------


void ExtractorI::rgb2lowe(char * rgbBuffer, int width, int height) {
  int nRows = height;
  int nCols = width;
  
  allocateLoweImage(nRows, nCols);  
  char* pRgb = rgbBuffer;
  for (int iRow = 0;  iRow < nRows;  iRow++) {
    for (int iCol = 0;  iCol < nCols;  iCol++, pRgb += 3) {
      m_loweImage.pixels[iRow][iCol] = 
	((0.299 * (*(pRgb+2)) + 0.587 * (*(pRgb+1)) + 0.114 * (*pRgb))) / 255.0;
    }
  }
} 



//----------------------------------------------------------------------------------

void ExtractorI::allocateLoweImage(int nRows, int nCols) {
    if (m_loweImage.rows != nRows || m_loweImage.cols != nCols) {
        m_loweImage.setSize(nRows, nCols);
    }
}  

//----------------------------------------------------------------------------------

FeatureVector *ExtractorI::extractFromLoweImage() {
  

    // preallocate the sequence
    FeatureVector* vFeatures = new FeatureVector();
    vFeatures->reserve(1000);

    Location objCenter;
    objCenter.x = 0;
    objCenter.y = 0;
    
    // extract keypoints
    SIFT sift;
    SIFT::KeyList keys = sift.getKeypoints(m_loweImage);
    std::vector<SIFT::Key>::const_iterator itk, itk_e;
    for (itk = keys.begin(), itk_e = keys.end();  itk < itk_e;  ++itk) {
      // lrint: round to the nearest integer.
      objCenter.x += lrint(itk->col);
      objCenter.y += lrint(itk->row);
    } // for
    
    // Calculate the object center.
    if (keys.size() == 0) {      
      std::cerr << "no " << std::endl;
      return vFeatures;
    }
    
    objCenter.x /= keys.size();
    objCenter.y /= keys.size();

    for (itk = keys.begin(), itk_e = keys.end();  itk < itk_e;  ++itk) {
      // copy location, scale, and orientation
      Location location;
      location.x = lrint(itk->col);
      location.y = lrint(itk->row);
      
      Feature feature;
      feature.loc = location;
      feature.fScale = itk->scale;
      feature.fOrientation = itk->ori;
      
      feature.dxCenter = location.x - objCenter.x;
      feature.dyCenter = location.y - objCenter.y;
      
      // copy the SIFT vector
      std::memcpy(feature.descriptor, &(itk->ivec)[0], SIFT_LENGTH);
      
      // append new feature
      vFeatures->push_back(feature);
    }
    return vFeatures;
}  // extractFromLoweImage

