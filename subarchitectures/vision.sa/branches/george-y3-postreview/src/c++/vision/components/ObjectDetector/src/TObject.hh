/**
 * @file TObject.hh
 * @author Andreas Richtsfeld
 * @date 2008
 * @version 0.1
 * @brief TObject => old class for tracked Gestalts
 * @TODO OLD - DELETE if not neccessary
 **/


#ifndef Z_TObject_HH
#define Z_TObject_HH

#include "mxCameraModel.h"
#include "mxDrawTools.h"
#include "marker.h"

#include "Vector2.hh"
#include "Cube.hh"

namespace Z
{

class TCube  			// Cubes to track
{
  private:

  public:
  TCube(unsigned i, CvPoint3D64f cPos, double rad);
  void Tracked(unsigned i);	// register tracked cube
  
  unsigned id;			// id of the cube
  unsigned age;			// age of the stored TCube (in steps)
  unsigned id_tracked;		// id of the tracked cube (age+1)
  CvPoint3D64f centerPos;	// center position in 3D-coordinates
  double radius;		// circumscribed circle of the cube-projection (2D)
  Vector2 corner_points[4][2];	// corner points of cube
  Cube *cube;
};

class TCone 			// Cones to track
{
  private:

  public:
  TCone(unsigned i, CvPoint3D64f cPos);
  void Tracked(unsigned i);	// register tracked cone

  unsigned id;			// id of the cone
  unsigned age;			// age of the stored TCone (in steps)
  unsigned id_tracked;		// id of the tracked cone
  CvPoint3D64f centerPos;	// center position in 3D-coordinates
};

class TCylinder 		// Cylinders to track
{
  private:

  public:
  TCylinder(unsigned i, CvPoint3D64f cPos);
  void Tracked(unsigned i);	// register tracked cylinder

  unsigned id;			// id of the cylinder
  unsigned age;			// age of the stored TCylinder (in steps)
  unsigned id_tracked;		// id of the tracked cylinder
  CvPoint3D64f centerPos;	// center position in 3D-coordinates
};


}


#endif
