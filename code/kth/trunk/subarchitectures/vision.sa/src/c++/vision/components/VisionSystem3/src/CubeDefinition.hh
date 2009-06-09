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
  Gestalt::Type type;								///< type of the object
	
	double sig;												///< Significance value of the cube

	// 2D - variables
	Vector2 corner_points[4][2];			///< corner points of the cube
	Vector2 center;										///< center point of cube
	double radius;										///< radius to the corners from center point

	Vector2 groundCenter;							///< center point of ground plane
	Vector2 trackedCubeGroundCenter;	///< center point of ground plane from tracked cube
	
	// 3D - variables
	Vector2 groundCenter3D;						///< 3D point: center point on ground plane
	Vector2 rightBottom3D;						///< 3D point: right/bottom point of cube
	Vector2 sharedBottom3D;						///< 3D point: shared/bottom point of cube
	Vector2 leftBottom3D;							///< 3D point: left/bottom point of cube

	Vector2 corner_points3D[4][2];		///< 3D corner points of the cube
	Vector2 orientation;							///< 3D orientation of cube (from groundCenter to sharedBottom)
	double orientation_deg;						///< orientation of the cube in degree


	double length_a;									///< 3D size of ground plane: from shared to right (bottom) point
	double length_b;									///< 3D size of ground plane: from left to shared (bottom) point
	double height;										///< TODO height of the cube in 3D
};
}
#endif
