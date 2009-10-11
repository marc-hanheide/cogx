/**
 * @file FlapProperties.hh
 * @author Andreas Richtsfeld
 * @date Mon Sep 24 2009
 * @version 0.1
 * @brief Header file of flap properties
 **/

#ifndef Z_FlapProp_HH
#define Z_FlapProp_HH

#include "Namespace.hh"
#include "Vector2.hh"
// #include "Vector3.hh"

namespace Z
{
/**
 *	@brief Class, describing the properties of the Gestalt flap.
 */
class FlapProp
{
public:
  Gestalt::Type type;								///< Type of the object
	
	double sig;												///< Significance value

	// 2D - variables
	Vector2 corner_points[2][4];			///< Corner points of the flap (of the two rectangles)
//	Vector2 center;										///< Center point of cube
//	double radius;										///< Radius to the corners from center point

//	Vector2 groundCenter;							///< Center point of ground plane
//	Vector2 trackedCubeGroundCenter;	///< Center point of ground plane from tracked cube
	
	// 3D - variables
//	Vector3 corner_points[2][4];			///< Corner points of the flap (of the two rectangles)
// 	Vector2 groundCenter3D;						///< 3D center point of ground plane
//	Vector2 cubeCenter3D;							///< 3D center of the cube

//	Vector2 corner_points3D[4][2];		///< 3D corner points of the cube
//	Vector2 orientation;							///< 3D orientation of cube (from groundCenter to sharedBottom)
//	double orientation_deg;						///< orientation of the cube in degree


//	double length_a;									///< 3D size of ground plane: from shared to right (bottom) point
//	double length_b;									///< 3D size of ground plane: from left to shared (bottom) point
//	double height;										///< height of the cube in 3D
};
}
#endif
