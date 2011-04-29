/**
 * @file CylinderDefinition.hh
 * @author Andreas Richtsfeld
 * @date Mon Jan 12 2008
 * @version 0.1
 * @brief Header file of cylinder definition for tracking
 **/

#ifndef Z_CylinderDef_HH
#define Z_CylinderDef_HH

#include "Namespace.hh"
#include "Vector2.hh"

namespace Z
{
/**
 *	@brief Class, describing the difinition of Gestalt Cylinder for tracking.
 */
class CylDef
{
public:
  Gestalt::Type type;																// type of the object
	unsigned cylinder;																// cylinder-id

	// 2D - variables
	unsigned ellipses[2];															// ellipses of the cylinder // TODO 0 = bottom / 1 = top ellipse ???
  double x[2], y[2], a[2], b[2], phi[2];  					// ellipse parameters

	Vector2 vertex[2][2];															// [first/second ellipse] [left/right] vertex
 	Vector2 dir;																			// direction from first to second ellipse (center)
	bool equalVertex;																	// true if ellipse 0 and 1 are connected by the same ellipse-vertices [left-left or right-right]

	Vector2 center;																		// center point of cylinder (mean value of ellipse-center points)
	double radius;																		// radius to the ellipse-ends (center-point + b)

	Vector2 groundCenter;															// center point of ellipseellipse
	Vector2 trackedCylinderGroundCenter;							// center point of ellipse from tracked cylinder

	// 3D - variables
	bool match;																				// vertex 00-01 == bottom?
	Vector2 vertex3D[2][2];														// 3D vertices: [first/second ellipse] [left/right] vertex
	Vector2 groundCenter3D;														// 3D point: center point on ground plane
	Vector2 topCenter3D;															// 3D point: top center on top plane
 	Vector2 cylinderCenter3D;													// 3D center point in 3D (with z = height/2.)
	double radius3D;																	// 3D radius of the ground plane ellipse
	double topRadius3D;																// 3D radius of the top ellipse
	double height;																		// 3D height of the cylinder


};
}
#endif
