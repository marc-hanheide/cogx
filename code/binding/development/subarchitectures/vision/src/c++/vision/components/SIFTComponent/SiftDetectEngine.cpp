#include "SiftDetectEngine.h"
#include <iostream>
#include <fstream>

/**
 * Yay, the constructor...
 */
SiftDetectEngine::SiftDetectEngine(int _img_maxwidth, int _img_maxheight,
				   string strDatabaseFile) {

  m_pmatcher = new Matcher(_img_maxwidth, _img_maxheight);
  m_pextractor = new ExtractorI();  
  m_pMatchInfo = NULL;

  if (strDatabaseFile != "") {
    m_pmatcher->load(strDatabaseFile);
    cout << "\nSiftDetectEngine:: loaded database " << strDatabaseFile.c_str() << endl;  
  }

  //m_srcimg = cvCreateImage(cvSize(_img_maxwidth, _img_maxheight),IPL_DEPTH_8U, 3);
  m_srcimg = NULL;
}


/**
 * Help, help, I am being destroyed...
 */
SiftDetectEngine::~SiftDetectEngine(){
  if (m_pmatcher != NULL)
    delete(m_pmatcher);
  if (m_pextractor != NULL)
    delete(m_pextractor);
  if (m_pMatchInfo != NULL)
    delete(m_pMatchInfo);
  if (m_srcimg != NULL) {
    cvReleaseImage(&m_srcimg);
    m_srcimg = NULL;
  }
}



void SiftDetectEngine::roiFilter(FeatureVector &vFeatures, 
				 BBox2D &_bbox) {
  int tLeft_x, tLeft_y, bRight_x, bRight_y;
  tLeft_x =  (int)(_bbox.m_center.m_x - _bbox.m_size.m_x/2.0);
  tLeft_y =  (int)(_bbox.m_center.m_y - _bbox.m_size.m_y/2.0);
  bRight_x = tLeft_x + (int)_bbox.m_size.m_x;
  bRight_y = tLeft_y + (int)_bbox.m_size.m_y;

  FeatureVector::iterator iter;
  for (iter=vFeatures.begin(); iter!=vFeatures.end();) {
    if (((int)((*iter).loc.x) < tLeft_x) || ((int)((*iter).loc.x) > bRight_x) ||
	((int)((*iter).loc.y) < tLeft_y) || ((int)((*iter).loc.y) > bRight_y)) 
      vFeatures.erase(iter);
    else
      ++iter;
  }
  for( size_t i = 0; i < vFeatures.size(); i++ ) {
    vFeatures[i].dxCenter = vFeatures[i].loc.x - (int)_bbox.m_center.m_x;
    vFeatures[i].dyCenter = vFeatures[i].loc.y - (int)_bbox.m_center.m_y;
  }
}



void SiftDetectEngine::alignFeatures(FeatureVector &vFeatures, 
				     BBox2D &_bbox) {
//   int tLeft_x, tLeft_y, bRight_x, bRight_y;
//   tLeft_x =  (int)(_bbox.m_center.m_x - _bbox.m_size.m_x/2.0);
//   tLeft_y =  (int)(_bbox.m_center.m_y - _bbox.m_size.m_y/2.0);
//   bRight_x = tLeft_x + (int)_bbox.m_size.m_x;
//   bRight_y = tLeft_y + (int)_bbox.m_size.m_y;

//   FeatureVector::iterator iter;
//   for (iter=vFeatures.begin(); iter!=vFeatures.end();) {
//     if (((int)((*iter).loc.x) < tLeft_x) || ((int)((*iter).loc.x) > bRight_x) ||
// 	((int)((*iter).loc.y) < tLeft_y) || ((int)((*iter).loc.y) > bRight_y)) 
//       vFeatures.erase(iter);
//     else
//       ++iter;
//   }

  int center_x = (int)(_bbox.m_center.m_x/2);
  int center_y = (int)(_bbox.m_center.m_y/2);

  for( size_t i = 0; i < vFeatures.size(); i++ ) {
    vFeatures[i].dxCenter = vFeatures[i].loc.x - center_x;
    vFeatures[i].dyCenter = vFeatures[i].loc.y - center_y;
  }
}


/**
 * Function to finally detect the object from the input image region
 * specified by the input bounding box -- sets object label and
 * probability...
 */
void SiftDetectEngine::detectObject( ImageFrame* _pimg, const BBox2D &_bbox, 
				     string& olabel, double& oProb ) {
  //HACK increase size of bbox! ... danger .. better results though?
  unsigned int extension = 25; //*2 for each axis

  BBox2D extendedBox(_bbox); 
  BBox2D orig(_bbox);

  //shift extended bb centre -- WHY WOULD YOU SHIFT THE CENTER??!!
//   extendedBox.m_center.m_x += extension;
//   extendedBox.m_center.m_y += extension;
  
  //increate size byb 2 * extension
  extendedBox.m_size.m_x += (2 * extension);
  extendedBox.m_size.m_y += (2 * extension);

  // if this is oxver image extremes, reset
  if((extendedBox.m_center.m_x + (extendedBox.m_size.m_x/2)) > _pimg->m_width) {
    extendedBox = orig;
  }
  else if((extendedBox.m_center.m_y + (extendedBox.m_size.m_y/2)) > _pimg->m_height) {
    extendedBox = orig;
  }
  else {
    cout<<"extending bbox by "<<extension<<endl;
  }

  IplImage* image = buffer2image(_pimg);
  int topLeft_x  = (int)(extendedBox.m_center.m_x - (extendedBox.m_size.m_x/2.0));
  int topLeft_y  = (int)(extendedBox.m_center.m_y - (extendedBox.m_size.m_y/2.0));  

  cvSetImageROI (image, cvRect(topLeft_x, topLeft_y, 
			       (int)extendedBox.m_size.m_x, (int)extendedBox.m_size.m_y));

  IplImage* roiImg = cvCreateImage(cvSize((int)extendedBox.m_size.m_x,(int)extendedBox.m_size.m_y), 8, 3);
  cvCopy(image, roiImg);
  cvReleaseImage(&image);

  if (m_pMatchInfo) {
    delete m_pMatchInfo;
    m_pMatchInfo = NULL;
  }

  // get features in the bbox
  FeatureVector *pdetectedFeatures = 
    m_pextractor->extractFromRgb(roiImg->imageData, roiImg->width, roiImg->height);
  
  // No longer need the image...
  cvReleaseImage(&roiImg);
  
  // Move the feature positions to correspond to the new center of the
  // extended bounding box...
  alignFeatures((*pdetectedFeatures), extendedBox);
  
  // Now, go ahead and match the test features with the training set
  // -- please, please, let there be some valid matches...
  m_pMatchInfo = m_pmatcher->match(pdetectedFeatures);
 
  // Instead of weird magic number, we should be testing for a certain
  // match probability and a certain object confidence -- this is
  // already done within the matching function...
  if( m_pMatchInfo->strMostProbableObject.c_str() != "" ) {
    cout << "SIFT:SDE: Found valid object label: "
	 << m_pMatchInfo->strMostProbableObject.c_str() << endl;
    olabel = m_pMatchInfo->strMostProbableObject.c_str();
    // Also set the probability -- if the stupid thing is computed,
    // why not use it? :)
    oProb = m_pMatchInfo->objectConfidence;
  }
  else {
    cout << "\n SIFT:SDE: empty object label\n";
  }

  // Write out the feature vectors to a file to be used later...
//   static int ssCount = 0;
//   stringstream ss;
//   ss << "./Databases/sift_multiple_" << ssCount++ << "_samples.txt";
//   string str;
//   ss >> str;

//   ofstream file( str.c_str(), ios::out );
//   ss.clear();

//   int inum; float fnum;
//   // Write out the features, first the location, scale etc, followed
//   // by the vector itself...
//   int nVectors = pdetectedFeatures->size();
//   file.write( (char*)&nVectors, sizeof(nVectors) );
//   for( size_t i = 0; i < pdetectedFeatures->size(); ++i ) {
//     inum = (*pdetectedFeatures)[i].loc.x;
//     file.write( (char*)&inum, sizeof(inum) );

//     inum = (*pdetectedFeatures)[i].loc.y;
//     file.write( (char*)&inum, sizeof(inum) );

//     fnum = (*pdetectedFeatures)[i].fScale;
//     file.write( (char*)&fnum, sizeof(fnum) );

//     fnum = (*pdetectedFeatures)[i].fOrientation;
//     file.write( (char*)&fnum, sizeof(fnum) );
    
//     for( int j = 0; j < SIFT_LENGTH; ++j ) {
//       unsigned char ch = (*pdetectedFeatures)[i].descriptor[j];
//       file.write( (char*)&ch, sizeof(ch) );
//     }
//     inum = (*pdetectedFeatures)[i].dxCenter;
//     file.write( (char*)&inum, sizeof(inum) );
    
//     inum = (*pdetectedFeatures)[i].dyCenter;
//     file.write( (char*)&inum, sizeof(inum) );
//   }

//   file.close();

  // Clean up the vector of feature vectors so that it can be re-used
  // the next time around...
  if( pdetectedFeatures!=NULL ) {
    (*pdetectedFeatures).clear();
    delete(pdetectedFeatures);
  }
}


