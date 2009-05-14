/** @file Intersect.cpp
 *  @brief Useful functions for computing interaction of geometrical structures.
 *
 *
 *  @author Somboon Hongeng
 *  @bug No known bugs.
 */
#include <opencv/cv.h>
#include "Vector3D.h"
#include "Ray.h"
#include "Plane.h"

double intersect(Vector3D &X, Ray &ray, double &t1) {
  t1 = (X-ray.get_X0())*  ray.get_X1() / (ray.get_X1()*ray.get_X1());
  return X.distance( ray.t(t1) );
}


double intersect(Ray &r1, double &t1,Ray &r2, double &t2)
{
  static CvMat A = cvMat(2,2,CV_64F);
  static CvMat b = cvMat(2,1,CV_64F);
  static CvMat l = cvMat(2,1,CV_64F);
  static CvMat invA = cvMat(2,2,CV_64F);
  static int firstcall = 1;
  if( firstcall == 1 ) {
    cvmAlloc(&A);
    cvmAlloc(&b);
    cvmAlloc(&l);
    cvmAlloc(&invA);
    firstcall = 0;
  }

  cvmSet(&A,0,0,  r1.get_X1()*r1.get_X1());
  cvmSet(&A,0,1,-(r1.get_X1()*r2.get_X1()));
  cvmSet(&A,1,0,-(r1.get_X1()*r2.get_X1()));
  cvmSet(&A,1,1,  r2.get_X1()*r2.get_X1());
  cvmSet(&b,0,0,-(r1.get_X1()*(r1.get_X0()-r2.get_X0())) );
  cvmSet(&b,1,0,  r2.get_X1()*(r1.get_X0()-r2.get_X0()) );

  cvmInvert(&A,&invA);
  cvmMul(&invA,&b,&l);
  t1 = cvmGet(&l,0,0);
  t2 = cvmGet(&l,1,0);

  Vector3D X;
  X = r1.t(t1);
  return X.distance( r2.t(t2) );
}

double intersect(Ray &r, double &t1,Plane &p, double &t2,double &t3)
{
  static CvMat A = cvMat(3,3,CV_64F);
  static CvMat b = cvMat(3,1,CV_64F);
  static CvMat l = cvMat(3,1,CV_64F);
  static CvMat invA = cvMat(3,3,CV_64F);
  static int firstcall = 1;
  if( firstcall == 1 ) {
    cvmAlloc(&A);
    cvmAlloc(&b);
    cvmAlloc(&l);
    cvmAlloc(&invA);
    firstcall = 0;
  }

  cvmSet(&A,0,0,  r.get_X1()*r.get_X1() );
  cvmSet(&A,0,1,-(r.get_X1()*p.get_X1()));
  cvmSet(&A,0,2,-(r.get_X1()*p.get_X2()));
  cvmSet(&A,1,0,-(r.get_X1()*p.get_X1()));
  cvmSet(&A,1,1,  p.get_X1()*p.get_X1() );
  cvmSet(&A,1,2,  p.get_X1()*p.get_X2() );
  cvmSet(&A,2,0,-(r.get_X1()*p.get_X2()));
  cvmSet(&A,2,1,  p.get_X1()*p.get_X2() );
  cvmSet(&A,2,2,  p.get_X2()*p.get_X2() );

  cvmSet(&b,0,0,-(r.get_X1()*(r.get_X0()-p.get_X0())) );
  cvmSet(&b,1,0,  p.get_X1()*(r.get_X0()-p.get_X0()) );
  cvmSet(&b,2,0,  p.get_X2()*(r.get_X0()-p.get_X0()) );

  cvmInvert(&A,&invA);
  cvmMul(&invA,&b,&l);
  t1 = cvmGet(&l,0,0);
  t2 = cvmGet(&l,1,0);
  t3 = cvmGet(&l,2,0);

  Vector3D X;
  X = r.t(t1);
  return X.distance( p.t(t2,t3) );
}
