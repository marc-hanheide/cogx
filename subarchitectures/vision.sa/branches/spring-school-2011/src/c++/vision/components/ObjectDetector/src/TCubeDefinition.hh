/**
 * @file TCubeDefinition.hh
 * @author Andreas Richtsfeld
 * @date Februar 2009
 * @version 0.1
 * @brief Header file of TCube Definition
 **/

#ifndef Z_TCubeDefinition_HH
#define Z_TCubeDefinition_HH

#include "Namespace.hh"
#include "Vector2.hh"

namespace Z
{

/**
 *	@brief Class, describing the definition of a cube for tracking.
 */
class TCubeDef
{
public:
	bool hypothesised;							///< Hypothesised Cube?
	double sig;											///< Significance value of the cube

  unsigned flap;									///< Main flap of cube
  unsigned oFlaps[2];							///< The possible two other flaps for closing the cube (maybe undefined == UNDEF_ID)

	/// TODO Rectangles sollte man hier nicht brauchen!
	unsigned rect[3];								///< Rectangles of the cube (maybe undefined == UNDEF_ID)

//	unsigned ljct;										///< L-Junction for closing the cube
//  unsigned closingLJct; 					///< the closing L-Junction (maybe undefined == UNDEF_ID)
//  unsigned closingColl; 					///< the closing Collinearity (maybe undefined == UNDEF_ID)

	Vector2 corner_points[4][2];			///< corner points of the cube

	Vector2 center;										///< center point of cube
	double radius;										///< radius to the corners from center point


};
}
#endif
