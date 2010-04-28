/**
 * @file ConeDefinition.hh
 * @author Andreas Richtsfeld
 * @date Mon Jan 12 2008
 * @version 0.1
 * @brief Header file of cone definition for tracking
 **/

#ifndef Z_ConeDef_HH
#define Z_ConeDef_HH

//#include "Cube.hh"
#include "Namespace.hh"
#include "Vector2.hh"

namespace Z
{
/**
 * @brief Class, describing the definition of the Gestalt Cone for tracking.
 */
class ConeDef
{
public:
  Gestalt::Type type;								// type of the object
	unsigned cone;										// cone-id

	// 2D - variables
	unsigned ellipse;									// ellipse of the cone
	unsigned ljct;										// ljct of the ellipse (cone end - top)

  double x, y, a, b, phi;  					// ellipse parameters
	Vector2 isct;											// intersection point of l-junction at cone end
	Vector2 vertex[2];								// left / right vertex of the ellipse

	Vector2 center;										// center point of cube
	double radius;										// radius to the corners from center point
	
	Vector2 groundCenter;							// center point of ellipse
	Vector2 trackedConeGroundCenter;	// center point of ellipse from tracked cone

	// 3D - variables
	Vector2 groundCenter3D;						// 3D point: center point on ground plane
	double radius3D;									// 3D radius of the ground plane (circle)
};
}
#endif
