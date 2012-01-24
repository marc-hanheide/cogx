/**
 * $Id: SPolygon3D.cc,v 1.1.1.1 2008/02/29 10:12:02 jp Exp $
 * Johann Prankl, 2010-06-30
 * prankl@acin.tuwien.ac.at
 */


#include "SPolygon3D.hh"
#include <float.h>

namespace P 
{

SPolygon3D::SPolygon3D()
{
}

SPolygon3D& SPolygon3D::operator=(const SPolygon3D &p)
{
  this->vs = p.vs;
  return *this;
}

/**
 * computes the area of a polygon in 3D
 * The area is the sum of the areas of the triangles formed by
 * node N with consecutive pairs of nodes.
 * Reference:
 *   Adrian Bowyer and John Woodwark,
 *   A Programmer's Geometry,
 *   Butterworths, 1983.
 */
void SPolygon3D::ComputeArea ( Array<Vector3> &vs, double &area)
{
  double t[3*3];

  area = 0.0;

  for ( int i = 0; i < (int)vs.Size() - 2; i++ )
  {
    t[0*3+0] = vs[i].x;
    t[0*3+1] = vs[i].y;
    t[0*3+2] = vs[i].z;

    t[1*3+0] = vs[i+1].x;
    t[1*3+1] = vs[i+1].y;
    t[1*3+2] = vs[i+1].z;

    t[2*3+0] = vs.Last().x;
    t[2*3+1] = vs.Last().y;
    t[2*3+2] = vs.Last().z;

    area += STriangle3D::ComputeArea (t);
  }

}

/**
 * Computes the center of a polygon in 3d
 */
void SPolygon3D::ComputeCenter ( Array<Vector3> &vs, Vector3 &center)
{
  center = Vector3(0.,0.,0.);

  for (unsigned i=0; i<vs.Size(); i++)
    center+=vs[i];

  center/=vs.Size();
}

void SPolygon3D::Save(ofstream &os, const SPolygon3D &p)
{
  /*os<<p.v.Size();
  for (unsigned i=0; i<p.v.Size(); i++)
    os<<' '<<p.v[i].x<<' '<<p.v[i].y;
  os<<' '<<p.center.x<<' '<<p.center.y;
  os<<' '<<p.p<<'\n';*/
}

void SPolygon3D::Load(ifstream &is, SPolygon3D &p)
{
  /*unsigned size;
  is >> size;
  p.v.Resize(size);
  for (unsigned i=0; i<size; i++)
    is >>p.v[i].x>>p.v[i].y;
  is >>p.center.x>>p.center.y;
  is >>p.p;*/
}





}

