/**
 * $Id: STriangle3D.cc,v 1.1.1.1 2008/02/29 10:12:02 jp Exp $
 * Johann Prankl, 2010-06-30
 * prankl@acin.tuwien.ac.at
 */


#include "STriangle3D.hh"
#include <float.h>

namespace P 
{

STriangle3D::STriangle3D()
 : data(0)
{
}

STriangle3D::STriangle3D(Array<Vector3> &t)
{
  Set(t);
}

STriangle3D::~STriangle3D()
{
  if (data!=0) delete[] data;
}



/**
 * computes the area of a triangle in 3D
 */
double STriangle3D::ComputeArea ( double *t)
{
  double a;
  double alpha;
  double area;
  double b;
  double base;
  double c;
  double dot;
  double height;

  //  Find the projection of (P3-P1) onto (P2-P1).

  dot =
    ( t[0+1*3] - t[0+0*3] ) * ( t[0+2*3] - t[0+0*3] ) +
    ( t[1+1*3] - t[1+0*3] ) * ( t[1+2*3] - t[1+0*3] ) +
    ( t[2+1*3] - t[2+0*3] ) * ( t[2+2*3] - t[2+0*3] );

  base = sqrt ( pow ( t[0+1*3] - t[0+0*3], 2 )
              + pow ( t[1+1*3] - t[1+0*3], 2 )
              + pow ( t[2+1*3] - t[2+0*3], 2 ) );

  //  The height of the triangle is the length of (P3-P1) after its
  //  projection onto (P2-P1) has been subtracted.

  if ( base == 0.0 )
  {
    height = 0.0;
  }
  else
  {
    alpha = dot / ( base * base );

    a = t[0+2*3] - t[0+0*3] - alpha * ( t[0+1*3] - t[0+0*3] );
    b = t[1+2*3] - t[1+0*3] - alpha * ( t[1+1*3] - t[1+0*3] );
    c = t[2+2*3] - t[2+0*3] - alpha * ( t[2+1*3] - t[2+0*3] );

    height = sqrt ( a * a + b * b + c * c );
  }

  area = 0.5 * base * height;

  return area;
}



}

