/**
 * @file BallDefinition.hh
 * @author Andreas Richtsfeld
 * @date Mon Jan 12 2008
 * @version 0.1
 * @brief Header file of ball definition for tracking
 **/

#ifndef Z_BalDef_HH
#define Z_BalDef_HH

#include "Namespace.hh"
#include "Vector2.hh"

namespace Z
{
/**
 *	@brief Class, describing the definition of the Gestalt Ball for tracking.
 */
class BalDef
{
public:
  Gestalt::Type type;								// type of the object
	unsigned ball;										// ball-id

	// 2D - variables
	Vector2 center;										// the center point of the ball
	double radius;										// radius of the ball 
	
	Vector2 groundCenter;							// ground center point of the ball
	Vector2 trackedBallGroundCenter;	// tracked ground center point of tracked ball

	// 3D - variables
	Vector2 groundCenter3D;						// 3D point: center point on ground plane
	double radius3D;									// 3D radius of the ball
};
}
#endif
