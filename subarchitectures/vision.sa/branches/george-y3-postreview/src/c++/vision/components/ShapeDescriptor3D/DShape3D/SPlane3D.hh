/**
 * $Id$
 * Johann Prankl, 2010-06-30
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_SPLANE_3D_HH
#define P_SPLANE_3D_HH

#include "Vector3.hh"
#include "Vector4.hh"
#include "SVector.hh"
#include "Array.hh"
#include "Except.hh"
#include "opencv/cv.h"
#include "opencv/cxcore.h"

namespace P
{

class SPlane3D
{
private:
  Array<Vector4> points4;

public:
  SPlane3D();
  SPlane3D(unsigned size);
  ~SPlane3D();
  
  void GetMean(double mean[3]);
  void GetMedian(double median[3]);

  void FitPlaneLS(double n[3], double p[3]);
  void ProjectPointsToPlane3D(double n[3], double *points);

  inline void Insert(double p[3]);
  inline void Clear();
  inline unsigned Size();
};


/************************** INLINE METHODES ******************************/
/**
 * Insert a 3d point to the data structure
 */
inline void SPlane3D::Insert(double p[3])
{
  points4.PushBack(Vector4(p[0],p[1],p[2],1.));
}

/**
 * Clear the data structure
 */
inline void SPlane3D::Clear()
{
  points4.Clear();
}

/**
 * Get data size
 */
inline unsigned SPlane3D::Size()
{
  return points4.Size();
}



}

#endif

