/** @file Polyflap.h
 * 
 * 
 * @author	Manuel Noll (DFKI)
 * @author	Sergio Roa (DFKI)
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

#ifndef POLYFLAP_H
#define POLYFLAP_H

#include <scenario/ConcreteActor.h>

namespace smlearning {

/** \brief The class represents a polyflap.
*
*/
class Polyflap : public ConcreteActor
{
protected:
	/** defines the concrete shape of a polyflap */
	virtual golem::Actor::Desc* getShape (golem::Creator& creator,const golem::Vec3& dimensions,const golem::Real& width)
	{
		return creator.createSimple2FlapDesc(Real(dimensions.v1*0.5), Real(dimensions.v1*0.5), Real(dimensions.v1*0.5), Real(width*0.5), REAL_PI_2);
	}

	virtual golem::Vec3 computeNormalVector (golem::Bounds::SeqPtr curPol)
	{
		golem::Mat34 curPolPos1;
		golem::Mat34 curPolPos2;

		//find out bounds of polyflap
		if (curPol->front()->getPose().p.v3 > curPol->back()->getPose().p.v3) {
			curPolPos1 = curPol->front()->getPose();
			curPolPos2 = curPol->back()->getPose();
		}
		else {
			curPolPos1 = curPol->back()->getPose();
			curPolPos2 = curPol->front()->getPose();
		}
		
		//Normal vector showing the direction of the lying part of polyflap, and it' orthogonal
		golem::Vec3 normalVec =
			smlearning::computeNormalVector(
					    Vec3 (curPolPos1.p.v1, curPolPos1.p.v2, Real(0.0)),
					    Vec3 (curPolPos2.p.v1, curPolPos2.p.v2, Real(0.0))
					    );
		return normalVec;
	}

	
};

}; // namespace smlearning 

#endif
