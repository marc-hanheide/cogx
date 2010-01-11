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
#include <highgui.h>

using namespace spatial;
using namespace std;

GridObjectFinder::GridObjectFinder(IplImage *objectGrid, int objectXCenter,
    int objectYCenter, ObjectFinderSymmetry symmetry)
{
  double maxAngle;
  double angStep = M_PI/16;
  switch (symmetry) {
    case NO_SYMMETRY:
      maxAngle = 2*M_PI-angStep;
    case HALF_SYMMETRY:
      maxAngle = M_PI-angStep;
    case QUARTER_SYMMETRY:
      maxAngle = 0.5*M_PI-angStep;
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

    cvNamedWindow( "Source", 1 );
    cvShowImage( "Source", squaredObject );

    cvWaitKey(0);
  CvPoint2D32f newCenterPoint;
  newCenterPoint.x = objRadius+1;
  newCenterPoint.y = objRadius+1;

  for (double angle = 0.0; angle <=maxAngle; angle += angStep) {
    IplImage *newImage = cvCreateImage(squareObjectSize, IPL_DEPTH_8U, 1);

    CvMat* rotationMatrix = cvCreateMat(2, 3, CV_32FC1);
    cv2DRotationMatrix(newCenterPoint, angle, 1, rotationMatrix);

    cvGetQuadrangleSubPix(squaredObject, newImage, rotationMatrix);

    m_aspectImages.push_back(newImage);    
    
    cvShowImage( "Source", newImage );

    cvWaitKey(0);

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
