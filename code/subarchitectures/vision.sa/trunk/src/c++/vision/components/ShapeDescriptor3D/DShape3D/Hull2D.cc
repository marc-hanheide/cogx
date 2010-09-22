/**
 * $Id$
 * Johann Prankl, 2010-06-30
 * prankl@acin.tuwien.ac.at
 */


#include "Hull2D.hh"

namespace P 
{


Hull2D::Hull2D()
{
}

Hull2D::~Hull2D()
{
}

/**
 * compute convex hull
 */
void Hull2D::ConvexHull(int *hull, int &hnum)
{
  if (points.Size()<3)
  {
    hnum=0; 
    return;
  }

  CvMat matHull = cvMat( 1, points.Size(), CV_32SC1, hull );
  CvMat matPoints = cvMat( 1, points.Size(), CV_32SC2, &points[0].x);

  cvConvexHull2( &matPoints, &matHull, CV_CLOCKWISE, 0 );
  hnum = matHull.cols;
}

/**
 * Compute convex hull
 */
void Hull2D::ConvexHull( int *points, int pnum, int *hull, int &hnum)  
{
  CvMat matHull = cvMat( 1, pnum, CV_32SC1, hull );
  CvMat matPoints = cvMat( 1, pnum, CV_32SC2, points);

  cvConvexHull2( &matPoints, &matHull, CV_CLOCKWISE, 0 );
  hnum = matHull.cols;
}


}

