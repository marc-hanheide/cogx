/** @file ActorObject.cpp
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

#include <scenario/ActorObject.h>

using namespace golem;
using namespace std;

namespace smlearning {

///////// Public //////////
/** \brief sets the shape of the actor 
*
* \param concreteActor is a pointer to a concrete actor that is used as object shape
*/
void ActorObject::setShape(golem::Scene& scene,ConcreteActor* concreteActor)
{

	Creator creator(scene);
	Actor::Desc* _shapeDesc =  concreteActor->getShape(creator,_desc.dimensions,_desc.width); // Create shape of the actor;
	_shapeDesc->nxActorDesc.globalPose.t.set(NxReal(_position.v1), NxReal(_position.v2), NxReal(_position.v3));		//-sets coordinates

	_shapeDesc->nxActorDesc.globalPose.M.rotX(_desc.startRotation.v1); //-sets rotations	
	_shapeDesc->nxActorDesc.globalPose.M.rotY(_desc.startRotation.v2);	
	_shapeDesc->nxActorDesc.globalPose.M.rotZ(_desc.startRotation.v3);

	Actor::create(*_shapeDesc);

	getPose().R.toEuler (_obRoll, _obPitch, _obYaw);

	computeVectors();


}

Mat34 ActorObject::getPose() const {
	NxMat34 nxPose;
	{
		CriticalSectionWrapper csw(universe.getCSPhysX()); // Access to PhysX
		nxPose = pNxActor->getGlobalPose();
	}

	Mat34 pose;
	nxPose.M.getRowMajor(&pose.R.m11);
	nxPose.t.get(&pose.p.v1);
	// std::cout << "getPose "<<pose.p.v1 <<" "<<pose.p.v2<<" "<<pose.p.v3<<std::endl;
	return pose;
}

void ActorObject::setPose(const Mat34& pose) {
	if (!pNxActor->readBodyFlag(NX_BF_KINEMATIC)) {
		context.getMessageStream()->write(Message::LEVEL_ERROR, "Actor::setPose(): Cannot change pose of dynamic actors");
		return;
	}

	NxMat34 nxPose;
	nxPose.M.setRowMajor(&pose.R.m11);
	nxPose.t.set(&pose.p.v1);
	std::cout << "setPose "<<pose.p.v1 <<" "<<pose.p.v2<<" "<<pose.p.v3<<std::endl;
	CriticalSectionWrapper csw(universe.getCSPhysX()); // Access to PhysX
	pNxActor->moveGlobalPose(nxPose);
}


///////// Protected //////////
/**\brief computes normal and orthogonal vector of the object and determines the position of the object 
*/
void ActorObject::computeVectors(){
	Mat34 curPolPos1;
	Mat34 curPolPos2;

	golem::Bounds::SeqPtr curPol = getGlobalBoundsSeq();
	//find out bounds of polyflap and compute the position of the polyflap
	if (curPol->front()->getPose().p.v3 > curPol->back()->getPose().p.v3) {
		curPolPos1 = curPol->front()->getPose();
		curPolPos2 = curPol->back()->getPose();
	}
	else {
		curPolPos1 = curPol->back()->getPose();
		curPolPos2 = curPol->front()->getPose();
	}
		
	_position.set(curPolPos1.p.v1, curPolPos1.p.v2, curPolPos2.p.v3);

	//Normal vector showing the direction of the lying part of polyflap, and it' orthogonal
	_normalVec =
		computeNormalVector(
			    Vec3 (curPolPos1.p.v1, curPolPos1.p.v2, Real(0.0)),
			    Vec3 (curPolPos2.p.v1, curPolPos2.p.v2, Real(0.0))
			    );
		
	_orthogonalVec = computeOrthogonalVec(_normalVec);	
}

///////// Protected //////////
/** \brief The ActorObject specific create function 
* 
* The function is called within scene()->createObject(desc) and ensures that the object is 
* correctly created and initialized within a scene from witch it is called.
* The create function initializes the private member variables such as _objectPose, _obRoll
* with initial values.
*
*/

bool ActorObject::create(const ActorObject::Desc& desc, golem::Scene& scene)
{
	if(desc.isValid())
	{
		setDescription(desc);
	
		setRoll(desc.initialObRoll);
		setPitch(desc.initialObPitch);
		setYaw(desc.initialObYaw);

		setFeatureVector(desc.initialFeatureVector);

		setPosition(desc.startPosition);
		cout << Real(_position.v1) << " " << Real(_position.v2) << " " << Real(_position.v3) << endl;

		return true;
	}
	else 
		return false;

}


///////// Public - Public //////////
/** \brief checks whether the descprition is valid 
*/
bool ActorObject::Desc::isValid() const 
{
	return (golem::Object::Desc::isValid());
}

///////// Public - Public //////////
/** \brief Sets the parameters to the default values 
*/
void ActorObject::Desc::setToDefault()
{
	Actor::Desc::setToDefault();
}

}; // namespace smlearning 
