/**
 * @file LJunctionDefinition.hh
 * @author Andreas Richtsfeld
 * @date Februar 2009
 * @version 0.1
 * @brief Header file of L-Junction definition for tracking (class LJunDef)
 **/

#ifndef Z_LJunctionDef_HH
#define Z_LJunctionDef_HH

#include "Namespace.hh"
#include "Vector2.hh"

namespace Z
{

/**
 *	@brief Class, describing the definition of a L-Junction for tracking.
 */
class LJctDef
{
public:
  Gestalt::Type type;								///< type of the object

	// 2D - variables
	unsigned line[2];								///< Line indexes of the two lines
  Vector2 dir[2];									///< Directions of arms, pointing outwards
  Vector2 isct;										///< Intersection point
	double openingAngle;						///< Opening angle between the arms
	
	// 3D - variables

};
}
#endif
