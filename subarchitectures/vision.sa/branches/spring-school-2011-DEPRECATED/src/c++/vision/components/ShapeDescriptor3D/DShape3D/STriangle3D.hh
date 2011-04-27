/**
 * $Id: STriangle3D.hh,v 1.1.1.1 2008/02/29 10:12:02 jp Exp $
 * Johann Prankl, 2010-06-30
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_STRIANGLE3D_HH
#define P_STRIANGLE3D_HH


#include <float.h>
#include <map> 
#include <stdio.h>
#include <string.h>
#include "Vector3.hh"
#include "PNamespace.hh"
#include "Array.hh"

namespace P
{

class STriangle3D
{
public:
  double *data;      // x1,y1,z1, x2,...
  
  STriangle3D();
  STriangle3D(Array<Vector3> &t);
  ~STriangle3D();

  inline void Set(Array<Vector3> &t);

  inline Vector3 operator[](unsigned i);
  inline STriangle3D &operator=(const STriangle3D &p);

  static double ComputeArea (double *t);
};


/************************ INLINE METHODES *******************************/

inline void STriangle3D::Set(Array<Vector3> &t)
{
  if (t.Size()!=3) 
    throw Except(__HERE__,"Invalide triangle!");

  if (data==0) 
    data = new double[9];
  
  memcpy( data, &t[0], 9*sizeof(double));  
}

inline STriangle3D& STriangle3D::operator=(const STriangle3D &t)
{
  if (this->data==0)
    this->data = new double[9];

  memcpy( this->data, t.data, 9*sizeof(double));

  return *this;
}

inline Vector3 STriangle3D::operator[](unsigned i)
{
  if (i>2)
    throw Except(__HERE__,"Invalide triangle index!");

  return Vector3(data[i*3], data[i*3+1], data[i*3+2]);
}



}

#endif

