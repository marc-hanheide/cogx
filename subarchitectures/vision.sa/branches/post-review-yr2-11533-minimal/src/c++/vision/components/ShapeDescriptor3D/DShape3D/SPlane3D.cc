/**
 * $Id$
 * Johann Prankl, 2010-06-30
 * prankl@acin.tuwien.ac.at
 */


#include "SPlane3D.hh"

namespace P 
{

static int CmpDouble(const void *a, const void *b)
{
  if ( *((double*)a) > *((double*)b))
    return -1;  // a is first
  else
    return 1 ;  // b is first
}



SPlane3D::SPlane3D()
{
}

SPlane3D::~SPlane3D()
{
}

/**
 * compute mean vector
 */
void SPlane3D::GetMean(double mean[3])
{
  if (points4.Size()==0)
    throw P::Except(__HERE__,"Empty data structure!");

  mean[0]=mean[1]=mean[2]=0.;

  for (unsigned i=0; i<points4.Size(); i++)
  {
    mean[0]+=points4[i].x;
    mean[1]+=points4[i].y;
    mean[2]+=points4[i].z;
  }

  mean[0]/=points4.Size();
  mean[1]/=points4.Size();
  mean[2]/=points4.Size();
}

/**
 * compute median vector
 */
void SPlane3D::GetMedian(double median[3])
{
  if (points4.Size()<3)
    throw P::Except(__HERE__,"Too few points!");
  
  unsigned halfSize = points4.Size()/2;
  Array<double> data(points4.Size());

  for (unsigned i=0; i<points4.Size(); i++)
    data[i]=points4[i].x;

  data.Sort(CmpDouble);
  median[0] = data[halfSize];

  for (unsigned i=0; i<points4.Size(); i++)
    data[i]=points4[i].y;

  data.Sort(CmpDouble);
  median[1] = data[halfSize];

  for (unsigned i=0; i<points4.Size(); i++)
    data[i]=points4[i].z;

  data.Sort(CmpDouble);
  median[2] = data[halfSize];
}

/**
 * Compute Least Squares plane
 */
void SPlane3D::FitPlaneLS(double n[3], double p[3])
{
  if (points4.Size()<4)
    throw Except(__HERE__,"Too few points!");

  CvMat matA = cvMat(points4.Size(),4,CV_64F,&points4[0].x);
  CvMat *matW = cvCreateMat(points4.Size(), 4, CV_64F);
  CvMat *matU = cvCreateMat(points4.Size(), 4, CV_64F);
  CvMat *matV = cvCreateMat(4, 4, CV_64F);

  cvSVD(&matA,matW,matU,matV);

  n[0] = cvmGet(matV,0,3);
  n[1] = cvmGet(matV,1,3);
  n[2] = cvmGet(matV,2,3);

  double d = cvmGet(matV,3,3)/Norm3(n);
  Normalise3(n);

  double p1[3], p2[3];
  Mul3(n,-d,p1);

  Sub3(&points4[0].x,p1,p2);
  Mul3(n,Dot3(p2,n),p2);
  Sub3(&points4[0].x,p2, p);

  cvReleaseMat(&matW);
  cvReleaseMat(&matU);
  cvReleaseMat(&matV);
}

/**
 * Compute Least Squares plane and return points projected to the plane
 */
void SPlane3D::ProjectPointsToPlane3D(double n[3], double *points)
{
  if (points4.Size()<4)
    throw Except(__HERE__,"Too few points!");

  CvMat matA = cvMat(points4.Size(),4,CV_64F,&points4[0].x);
  CvMat *matW = cvCreateMat(points4.Size(), 4, CV_64F);
  CvMat *matU = cvCreateMat(points4.Size(), 4, CV_64F);
  CvMat *matV = cvCreateMat(4, 4, CV_64F);

  cvSVD(&matA,matW,matU,matV);

  n[0] = cvmGet(matV,0,3);
  n[1] = cvmGet(matV,1,3);
  n[2] = cvmGet(matV,2,3);

  double d = cvmGet(matV,3,3)/Norm3(n);
  Normalise3(n);

  double p1[3], p2[3];
  Mul3(n,-d,p1);

  for (unsigned i=0; i<points4.Size(); i++)
  {
    Sub3(&points4[i].x,p1,p2);
    Mul3(n,Dot3(p2,n),p2);
    Sub3(&points4[i].x,p2, &points[i*3]);
  }

  cvReleaseMat(&matW);
  cvReleaseMat(&matU);
  cvReleaseMat(&matV);
}


}

