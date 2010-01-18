//
// = FILENAME
//    GridObjectFinder.cpp
//
// = FUNCTION
//    Helper class used to extract candidate objects
//    from a discrete grid map, given bitmaps
//    for the objects' shapes
//    Uses naïve brute-force search
//
// = AUTHOR(S)
//    Kristoffer Sjöö
//
// = COPYRIGHT
//    Copyright (c) 2010 Kristoffer Sjöö
//
/*----------------------------------------------------------------------*/

#include "GridObjectFinder.hpp"
#include <iostream>
#include <math.h>
#include <algorithm>
#include <opencv/highgui.h>

using namespace std;

namespace spatial {

GridObjectFinder::GridObjectFinder(IplImage *objectGrid, int objectXCenter,
    int objectYCenter, ObjectFinderSymmetry symmetry, double angStep) : m_angStep(angStep)
{
  double maxAngle;

  switch (symmetry) {
    case NO_SYMMETRY:
      maxAngle = 2*M_PI-m_angStep;
      break;
    case HALF_SYMMETRY:
      maxAngle = M_PI-m_angStep;
      break;
    case QUARTER_SYMMETRY:
      maxAngle = 0.5*M_PI-m_angStep;
      break;
    case CIRCULAR_SYMMETRY:
    default:
      maxAngle = 0;
  }

  int objRadius = (int)ceil(sqrt(2)*max(max(objectGrid->height-objectYCenter, objectYCenter-1), 
	max(objectGrid->width-objectXCenter, objectXCenter-1)));

  CvSize squareObjectSize;
  squareObjectSize.width = objRadius*2+1;
  squareObjectSize.height = objRadius*2+1;

  IplImage *squaredObject = cvCreateImage(squareObjectSize, IPL_DEPTH_8U, 1);
  cvSetZero(squaredObject);
  CvPoint offset;
  offset.x = objRadius - objectXCenter + 2;
  offset.y = objRadius - objectYCenter + 2;
  cvCopyMakeBorder(objectGrid, squaredObject, offset, IPL_BORDER_CONSTANT);

//    cvNamedWindow( "Source", 1 );
//    cvShowImage( "Source", squaredObject );

//    cvWaitKey(0);
  CvPoint2D32f newCenterPoint;
  newCenterPoint.x = objRadius+1;
  newCenterPoint.y = objRadius+1;

  for (double angle = 0.0; angle <=maxAngle; angle += m_angStep) {
    IplImage *newImage = cvCreateImage(squareObjectSize, IPL_DEPTH_8U, 1);

    double degAngle = 180.0/M_PI*angle;
    CvMat* rotationMatrix = cvCreateMat(2, 3, CV_32FC1);

    cv2DRotationMatrix(newCenterPoint, degAngle, 1, rotationMatrix);

    cvWarpAffine(squaredObject, newImage, rotationMatrix);

    m_aspectImages.push_back(newImage);    
    
//    cvShowImage( "Source", newImage );
//    cvWaitKey(0);

    cvReleaseMat(&rotationMatrix);
  }
  cvReleaseImage(&squaredObject);
}

GridObjectFinder::~GridObjectFinder()
{
  for (std::vector<IplImage *>::iterator it = m_aspectImages.begin();
      it != m_aspectImages.end(); it++) {
    cvReleaseImage(&(*it));
  }
}

GridObjectFinder *
createTableFinder(void)
{
  CvSize size = {11, 11};
  IplImage *objIm = cvCreateImage(size, IPL_DEPTH_8U, 1);
  cvSet(objIm, CV_RGB(0,0,0));
  GridObjectFinder *newFinder = new GridObjectFinder(objIm, 6, 6, QUARTER_SYMMETRY);
  cvReleaseImage(&objIm);
  return newFinder;
}

void 
GridObjectFinder::findObject(IplImage *image, int *outX, int *outY, 
    double *outAngle, double *outConfidence)
{
  int objectWidth = m_aspectImages[0]->width;
  int borderWidth = (1+objectWidth)/2;

  CvSize enlargedSize;
  enlargedSize.width = image->width+borderWidth*2;
  enlargedSize.height = image->height+borderWidth*2;

  IplImage *enlargedImage = cvCreateImage(enlargedSize, IPL_DEPTH_8U, 1);
  cvSetZero(enlargedImage);
  CvPoint offset;
  offset.x = borderWidth;
  offset.y = borderWidth;
  cvCopyMakeBorder(image, enlargedImage, offset, IPL_BORDER_CONSTANT);

  CvSize outputSize;
  outputSize.width = enlargedImage->width-objectWidth+1;
  outputSize.height = enlargedImage->height-objectWidth+1;

  IplImage *scoreImage = cvCreateImage(outputSize, IPL_DEPTH_32F, 1);

  double angle = 0.0;
  double bestval = FLT_MAX;
  double bestAngle;
  CvPoint bestPoint;

  cvNamedWindow( "Score", 1 );

  for (std::vector<IplImage *>::iterator it = m_aspectImages.begin();
      it != m_aspectImages.end(); it++) {

    cvMatchTemplate(enlargedImage, *it, scoreImage, CV_TM_SQDIFF);

    double minval, tmp;
    CvPoint minPoint;
    cvNormalize(scoreImage, scoreImage);
    cvMinMaxLoc(scoreImage, &minval, &tmp, &minPoint);

    if (minval < bestval) {
      bestval = minval;
      bestPoint = minPoint;
      bestAngle = angle;
    }
    cvShowImage( "Score", scoreImage );
    cvWaitKey(0);
    angle += m_angStep;
  }

  *outX = bestPoint.x;
  *outY = bestPoint.y;
  *outAngle = bestAngle;
  if (outConfidence != 0)
    *outConfidence = log(bestval);

  cvReleaseImage(&enlargedImage);
  cvReleaseImage(&scoreImage);
}

void 
GridObjectFinder::findObject(Cure::LocalGridMap<unsigned int> &lgm, int *outX, 
    int *outY, double *outAngle, double *outConfidence)
{
  int lgmSize = lgm.getSize(); 
  CvSize size;
  size.width = lgmSize*2+1;
  size.height = size.width;

  IplImage *lgmImage = cvCreateImage(size, IPL_DEPTH_8U, 1);
  int lp = 0;
  for(int x = -lgmSize ; x <= lgmSize; x++){
    for(int y = -lgmSize ; y <= lgmSize; y++){ 
      cvSet2D(lgmImage,y+lgmSize,x+lgmSize,CV_RGB(lgm[lp], 
	    lgm[lp], lgm[lp]));
      lp++;
    }
  }
  findObject(lgmImage, outX, outY, outAngle, outConfidence);
  cvReleaseImage(&lgmImage);
}
};
