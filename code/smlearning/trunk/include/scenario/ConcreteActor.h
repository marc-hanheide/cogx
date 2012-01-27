/** @file ConcreteActor.h
 * 
 * 
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

#ifndef CONCRETEACTOR_H
#define CONCRETEACTOR_H


#include <Golem/Math/Math.h>
#include <Golem/Phys/Object.h>
#include <Golem/Phys/Scene.h>
#include <Golem/PhysCtrl/Creator.h>

using namespace golem;
using namespace std;

namespace smlearning {

/** \brief The abstract class for an concrete actor in the golem world.
*
*	The ConcreteActor class sets shape, rotation and position of an object within a given scene.
*	This means that the class represents the concrete description of an actor.
*	In order to model a concrete actor one has to derive a class from ConcreteActor and define
*	the getShape function in that derived one.
*
*	The ConcreteActor class is used as an object for the ActorObject class.
*
*/

class ConcreteActor
{
	public:
		/** defines the concrete shape of the actor */
		virtual Actor::Desc* getShape(Creator &,const Vec3&,const Real &)=0;
	
};

}; // namespace smlearning 

#endif
