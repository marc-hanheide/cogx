/**
 * $Id: SPolygon3D.hh,v 1.1.1.1 2008/02/29 10:12:02 jp Exp $
 *
 * Johann Prankl, 2010-06-30
 * prankl@acin.tuwien.ac.at
 *
 */

#ifndef P_SPOLYGON3D_HH
#define P_SPOLYGON3D_HH


#include <float.h>
#include <map> 
#include "Vector3.hh"
#include "PNamespace.hh"
#include "Array.hh"
#include "STriangle3D.hh"

namespace P
{

class SPolygon3D
{
public:
  Array<Vector3> vs;
  Vector3 center;       //center of gravity
  double area;
  
  SPolygon3D();
  SPolygon3D(Array<Vector3> &p);
  ~SPolygon3D(){};

  inline void Clear(){vs.Clear();}
  inline unsigned Size(){return vs.Size();}
  inline void PushBack(Vector3 &p){vs.PushBack(p);}
  inline unsigned NumVertices(){return vs.Size();}
  inline void Insert(Vector3 &p){vs.PushBack(p);}
  inline double Area(){return area;}
  inline void ComputeArea();
  inline void ComputeCenter();

  static void ComputeArea ( Array<Vector3> &vs, double &area);
  static void ComputeCenter ( Array<Vector3> &vs, Vector3 &center);
  static void Load(ifstream &is, SPolygon3D &p);
  static void Save(ofstream &os, const SPolygon3D &p);

  SPolygon3D &operator=(const SPolygon3D &p);
  Vector3& operator[](unsigned i){return vs[i];}
};


/************************ INLINE METHODES *******************************/

inline void SPolygon3D::ComputeArea()
{
  SPolygon3D::ComputeArea(vs, area);
}

inline void SPolygon3D::ComputeCenter()
{
  SPolygon3D::ComputeCenter(vs, center);
}

}

#endif

