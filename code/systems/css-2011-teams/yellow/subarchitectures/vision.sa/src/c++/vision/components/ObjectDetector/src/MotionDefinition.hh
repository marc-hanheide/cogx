/**
 * @file MotionDefinition.hh
 * @author Andreas Richtsfeld
 * @date Februar 2009
 * @version 0.1
 * @brief Header file of Motion Field Definition
 **/

#ifndef Z_MotionDef_HH
#define Z_MotionDef_HH

#include "Namespace.hh"
#include "Vector2.hh"

namespace Z
{

/**
 *	@brief Class, describing the definition of a L-Junction for tracking.
 */
class MotionDef
{
public:
	int motionCase;					///< Motion case: 1/2/3/4 = forward/backward/right/left
	Vector2 FOE;						///< Focus of Expansion
	double disMul;					///< Distance Multiplicator for FOE point
	double sigFOE;					///< Significance value of FOE
};
}
#endif
