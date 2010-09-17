/**
 * $Id: Object.cc,v 1.0 2007/12/13 13:47:03 mxz Exp mxz $
 */

#include "TObject.hh"
#include "Cube.hh"

namespace Z
{

TCube::TCube(unsigned i, CvPoint3D64f cPos, double rad)
{
  id = i;
  centerPos = cPos;
  radius = rad;
  age = 0;
  id_tracked = UNDEF_ID;

  cube = Cubes(i);
}

void TCube::Tracked(unsigned i)
{
  id_tracked = i;
}


TCone::TCone(unsigned i, CvPoint3D64f cPos)
{
  id = i;
  centerPos = cPos;
  age = 0;
  id_tracked = UNDEF_ID;
}

void TCone::Tracked(unsigned i)
{
  id_tracked = i;
}

TCylinder::TCylinder(unsigned i, CvPoint3D64f cPos)
{
  id = i;
  centerPos = cPos;
  age = 0;
  id_tracked = UNDEF_ID;
}

void TCylinder::Tracked(unsigned i)
{
  id_tracked = i;
}





}
