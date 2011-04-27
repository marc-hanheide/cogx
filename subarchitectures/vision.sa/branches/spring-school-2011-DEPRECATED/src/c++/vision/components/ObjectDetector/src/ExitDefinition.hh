/**
 * @file ExitDefinition.hh
 * @author Andreas Richtsfeld
 * @date June 2008
 * @version 0.1
 * @brief Header file of exit definition for tracking
 **/

#ifndef Z_ExitDef_HH
#define Z_ExitDef_HH

#include "Namespace.hh"
#include "Vector2.hh"

namespace Z
{
/**
 * @brief Class, describing the definition of the Gestalt Exit for tracking.
 */
class ExitDef
{
public:
  Gestalt::Type type;										// type of the object
	unsigned exit;												// exit-id

	// 2D - variables
	Vector2 exitLinesStart[2];						// start points of exit-lines
	Vector2 exitLinesIntersection[2];			// intersection points of exit-lines

	Vector2 center;												// TODO center point of the exit (center between intersection points)
	double radius;												// TODO radius of the exit
	
	Vector2 groundCenter;									// center point of the exit (center between intersection points = center)
	Vector2 trackedExitGroundCenter;			// center point of exit from tracked exit

	// 3D - variables
	Vector2 groundCenter3D;								// 3D point: center point on ground plane
	Vector2 exitLinesIntersection3D[2];		// 3D point: intersection between exit-lines and wall
// 	Vector2 borderPoint3D[2];							// 3D point: border point of the exit

};
}
#endif
