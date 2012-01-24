/**
 * $Id$
 *
 * Johann Prankl, 30.11.2006 
 */


#ifndef P_HOMOGRAPHY_HH
#define P_HOMOGRAPHY_HH

#include "PNamespace.hh"
#include "Except.hh"
#include "Array.hh"
#include "Vector2.hh"
#include "matrix.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>


namespace P 
{

bool AffineMatrix(Array<Vector2> &cs1, Array<Vector2> &cs2,         //least square affine transformation
                  Matrix &aff, double &error);
void AffineMatrix2(Array<Vector2> &cs1, Array<Vector2> &cs2, Matrix &aff); //uses (the first) 3 points to calculate the affine transformation
void ComputeHomography(Array<Vector2> &p1, Array<Vector2> &p2, Matrix &H, double *error=0);
void HomographyError(Array<Vector2> &p1, Array<Vector2> &p2, Matrix &H, double &error);
void ComputeSimilarity(Array<Vector2> &cs1, Array<Vector2> &cs2,
                       Matrix &H, double *error=0);
Vector2 MapPoint(Vector2 &p, Matrix &mat);
void MapPoint(double x, double y, Matrix &mat, double &xr, double &yr);
void GetPosScaleAngle( double x, double y, Matrix &H, double &xr, double &yr, double &scale, double &angle);
}

#include "Homography.ic"

#endif

