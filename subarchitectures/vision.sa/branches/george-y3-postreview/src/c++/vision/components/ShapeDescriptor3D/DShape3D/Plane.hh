/**
 * $Id$
 * Johann Prankl, 2010-06-30
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_PLANE_HH
#define P_PLANE_HH

#include "PNamespace.hh"
#include "Array.hh"
#include "opencv/cxcore.h"
#include "KeypointDescriptor.hh"
#include "Vector3.hh"

namespace P
{


class Plane
{
public:
  unsigned id;
  unsigned oid;               //in CModel3D that's the object id
  static unsigned idcnt;

  CvMat *H;
  Array<KeypointDescriptor*> keys;

  double thrDist;                     // just a threshold for tracking (sigma of points..)

  double area;
  Array<Vector3> contour;
  Vector3 p;                          // a point
  Vector3 n;                          // normal vector

  Plane();
  ~Plane();
};

void DeletePlanes(Array<Plane*> &planes);
void ReleaseKeypoints(Array<Plane*> &planes);
void CopyParameter(Plane* src, Plane* dst);

/*********************** INLINE METHODES **************************/


}

#endif

