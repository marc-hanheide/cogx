/** @file Action.h
 * 
 * 
 * @author	Sergio Roa (DFKI)
 * @author	Manuel Noll (DFKI)
 *
 * @version 1.0
 *
 * 2011      Manuel Noll
 
   This is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This package is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License.
   If not, see <http://www.gnu.org/licenses/>.

 
*/

#ifndef ACTION_H
#define ACTION_H

#include <Golem/Math/Math.h>

using namespace golem;
using namespace std;

namespace smlearning {

/** \brief The very abstract class for an action that is performed by the arm in the SMLearning environment.
*
* The action is a movement which is performed by the Katana arm and is described by the arm state, the current pose, angle
* the push duration and the end pose. Poses are stored in a 3*4 matrix where the 4th row corresponds to the arm orientation 
* in euler coordinates. Additionally obRoll,obPitch and obYaw can be used for the euler coordinates.
* REMARK: The last tree variables and the 4th row of the matrix are not synchronized, if one changes for instance
* the value of obRoll then the value in the 4th row of the matrix isn't changed accordingly. Therefore if one uses
* both represenations one has to keep track on his own that both keep the same values.
*
*/
struct Action 
{
	// Action (const Action& action_to_cpy){(*this)=action_to_cpy; return *this;}
	// Action& operator=(const Action& a)
	// {
	// 	if (this != &a)
	// 	{
	// 		armState = a.armState;
	// 		efRoll = a.efRoll;
	// 		efPitch = a.efPitch;
	// 		efYaw = a.efYaw;
	// 		horizontalAngle = ef.horizontalAngle;
	// 		pushDuration = a.pushDuration;
	// 		effectorPose = a.effectorPose;
	// 		endEffectorPose = a.endEffectorPose;
	// 		endEfRoll = a.endEfRoll;
	// 		endEfPitch = a.endEfPitch;
	// 		endEfYaw = a.endEfYaw;
	// 		featureVector = a.featureVector;
	// 	}
	// 	return *this;
	// }
	/** Arm state - (joint) dynamic configuration */
	golem::GenConfigspaceState armState;
	/** End-effector GLOBAL pose */
	golem::Mat34 effectorPose;
	/** End-effector orientation in Euler coordinates */
	golem::Real efRoll, efPitch, efYaw; 
	/** horizontal angle */
	golem::Real horizontalAngle;
	/** speed ( 3 (fast), 4 (middle), 5 (low) */
	golem::Real pushDuration;
	/** direction vector (target pose) */
	golem::Mat34 endEffectorPose;
	/** target effector orientation in Euler coordinates */
	golem::Real endEfRoll, endEfPitch, endEfYaw;
	/** feature vector (here any vectorial representation is possible) */
	FeatureVector featureVector;
};

}; // namespace smlearning 

#endif
