

#ifndef Z_WalDef_HH
#define Z_WalDef_HH

//#include "Cube.hh"
#include "Namespace.hh"
#include "Vector2.hh"

namespace Z
{
class WalDef
{
public:
  Gestalt::Type type;								// type of the object
	unsigned wall;										// wall-id								// TODO braucht man diese id ???

	// 2D - variables
	bool corner;											// true, if wall with a corner
	Vector2 borderPoints[2];					// the 2 points at the border of the image
	Vector2 points[2];								// connection line points // if corner => corner point and corner edge top point

	Vector2 center;										// TODO center point of the wall
	double radius;										// TODO radius of the wall 
	
	Vector2 groundCenter;							// center point of the wall line (center between border points)
	Vector2 trackedWallGroundCenter;	// center point of exit from tracked exit

	// 3D - variables
	Vector2 groundCenter3D;						// 3D point: center point on ground plane
	Vector2 borderPoints3D[2];					// 3D point: border points of the wall
};
}
#endif
