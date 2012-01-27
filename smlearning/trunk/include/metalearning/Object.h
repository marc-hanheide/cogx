/** @file Object.h
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

#ifndef OBJECT_H
#define OBJECT_H

#include <Golem/Math/Math.h>

using namespace golem;
using namespace std;

/** \brief The very abstract class for an object that is used in the SMLearning environment.
*
* The object is a thing the arm can interact with. It is intented to derive certain classes for
* real and virtual world objects.
* The position of the object is stored in a 3*4 matrix where the 4th row corresponds to the object orientation 
* in euler coordinates.
* Additionally obRoll,obPitch and obYaw can be used for the euler coordinates.
* REMARK: The last tree variables and the 4th row of the matrix are not synchronized, if one changes for instance
* the value of obRoll then the value in the 4th row of the matrix isn't changed accordingly. Therefore if one uses
* both represenations one has to keep track on his own that both keep the same values.
*
* \param objectPose Object GLOBAL pose 
* \param obRoll,obPitch,obYaw Object orientation in Euler coordinates 
* \param featureVector feature vector (here any vectorial representation is possible)
*
*/

namespace smlearning {

struct Object 
{
	// /** cpy constructor */
	// Object (const Object& object_to_cpy) {(*this)=object_to_cpy; return *this;}
	// /** assignment operator */
	// Object& operator=(const Object& o)
	// {
	// 	if (this != &o)
	// 	{
	// 		objectPose = o.objectPose;
	// 		obRoll = o.obRoll;
	// 		obPitch = o.obPitch;
	// 		obYaw = o.obYaw;
	// 		featureVector = o.featureVector;
	// 	}
	// 	return *this;					
	// }

	/** Object GLOBAL pose */
	golem::Mat34 objectPose;
	/** Object orientation in Euler coordinates */
	golem::Real obRoll, obPitch, obYaw;
	/** feature vector (here any vectorial representation is possible) */
	FeatureVector featureVector;
};

}; // namespace smlearning 

#endif
