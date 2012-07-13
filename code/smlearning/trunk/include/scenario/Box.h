/** @file Box.h
 * 
 * 
 * @author	Sergio Roa (DFKI)
 *
 * @version 1.0
 *
 *
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

#ifndef SMLEARNING_BOX_H
#define SMLEARNING_BOX_H

#include <scenario/ConcreteActor.h>

namespace smlearning {

/** \brief The class represents a box.
*
*/
class Box : public ConcreteActor
{

protected:
	/** defines the concrete shape of a polyflap */
	virtual golem::Actor::Desc* getShape (golem::Creator& creator,const golem::Vec3& dimensions,const golem::Real& width)
	{
		// return shapeDesc;
		return creator.createBoxDesc(golem::Real(dimensions.v1*0.5), golem::Real(dimensions.v2*0.5), golem::Real(dimensions.v3*0.5));
	}

	virtual golem::Vec3 computeNormalVector (golem::Bounds::SeqPtr curPol)
	{

		Mat34 x_unit;
		x_unit.p.v1 = 0;
		x_unit.p.v2 = 1;
		x_unit.p.v3 = 0;
		golem::Mat34 pose = curPol->front()->getPose ();
		x_unit.R = pose.R;
		Mat34 tonormal;
		tonormal.multiply (pose, x_unit);
		//Normal vector showing the direction of the lying part of polyflap, and it' orthogonal
		Real obRoll, obPitch, obYaw;
		tonormal.R.toEuler (obRoll, obPitch, obYaw);
		cout << "tonormal: " << tonormal.p.v1 << ", " << tonormal.p.v2 << ", " << tonormal.p.v3 << ", " << obRoll << ", " << obPitch << ", " << obYaw << endl;
		pose.R.toEuler (obRoll, obPitch, obYaw);
		cout << "pose: " << pose.p.v1 << ", " << pose.p.v2 << ", " << pose.p.v3 << ", " << obRoll << ", " << obPitch << ", " << obYaw << endl;
		golem::Vec3 normalVec =
			smlearning::computeNormalVector(
					    Vec3 (pose.p.v1, pose.p.v2, Real(0.0)),
					    Vec3 (tonormal.p.v1, tonormal.p.v2, Real(0.0))
					    );
		cout << "normalVec: " << normalVec.v1 << ", " << normalVec.v2 << ", " << normalVec.v3 << endl;

		return normalVec;
		
	}
	
};

}; // namespace smlearning 

#endif
