/**
 * @file CubeDefinition.hh
 * @author Andreas Richtsfeld
 * @date Mon Jan 12 2008
 * @version 0.1
 * @brief Header file of cube definition for tracking
 **/

#ifndef Z_CubeDef_HH
#define Z_CubeDef_HH

//#include "Cube.hh"
#include "Namespace.hh"
#include "Vector2.hh"

namespace Z
{
/**
 *	@brief Class, describing the definition of the Gestalt Cube for tracking.
 */
class CubeDef
{
public:
  Gestalt::Type type;								///< Type of the object
	
	double sig;												///< Significance value of the cube

	// 2D - variables
	Vector2 corner_points[4][2];			///< Corner points of the cube
	Vector2 center;										///< Center point of cube
	double radius;										///< Radius to the corners from center point

	Vector2 groundCenter;							///< Center point of ground plane
	Vector2 trackedCubeGroundCenter;	///< Center point of ground plane from tracked cube
	
	// 3D - variables
	// TODO TODO TODO TODO TODO TODO TODO	Change to Vector3 or other 3D vector class
	Vector2 groundCenter3D;						///< 3D center point of ground plane
	Vector2 cubeCenter3D;							///< 3D center of the cube

	Vector2 corner_points3D[4][2];		///< 3D corner points of the cube
	Vector2 orientation;							///< 3D orientation of cube (from groundCenter to sharedBottom)
	double orientation_deg;						///< orientation of the cube in degree


	double length_a;									///< 3D size of ground plane: from shared to right (bottom) point
	double length_b;									///< 3D size of ground plane: from left to shared (bottom) point
	double height;										///< height of the cube in 3D
};
}
#endif
