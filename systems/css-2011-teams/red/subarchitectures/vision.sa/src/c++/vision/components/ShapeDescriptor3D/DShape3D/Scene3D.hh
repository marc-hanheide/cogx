/**
 * $Id$
 * Johann Prankl, 2010-06-30
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_SCENE3D_HH
#define P_SCENE3D_HH

#include "PNamespace.hh"
#include "Array.hh"
#include "opencv/cxcore.h"
#include "Vector3.hh"


namespace P
{


class Scene3D
{
public:
  Array<unsigned> ids;

  Array<Array<Vector3> > cs;
  Array<Array<Vector3> > contours;
 
  Scene3D() {};
  ~Scene3D() {};
};



/*********************** INLINE METHODES **************************/


}

#endif

