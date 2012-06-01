/** @file ActorObject.h
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

#ifndef ACTOROBJECT_H
#define ACTOROBJECT_H

#include <scenario/ConcreteActor.h>
#include <Golem/Math/Math.h>
#include <Golem/Phys/Object.h>
#include <metalearning/data_structs.h>

// using namespace golem;
// using namespace std;

namespace smlearning {

/** \brief The super class for all acting objects.
*
* The actor object is an object of the golem universe which is derived from the Golem Actor class and is described
* by its pose as a 3*4 matrix. Actor objects are the typical inhabitans of the golem world, e.g. the golem arm himself  
* as well as real and virtual world objects.
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


class ActorObject : public Actor
{
public:
	/** Object description */
	class Desc : public Actor::Desc 
	{
	public:				
		/** Constructs description object */
		Desc(){Desc::setToDefault();}
		/** checks whether the descprition is valid */
		virtual bool isValid() const;
		/** Sets the parameters to the default values */
		void setToDefault();			

		golem::Vec3 		dimensions;
		FeatureVector 	initialFeatureVector;
		golem::Mat34 	initialObjectPose;
		golem::Real 	initialObRoll, initialObPitch, initialObYaw;
		golem::Vec3 		startRotation;
		golem::Vec3		startPosition;
		/** vertical distance from the ground considering fingertip radius */
		golem::Real over;
		/** distance from the front/back of the polyflap */
		golem::Real dist; 
		/** distance from the side of the polyflap */
		golem::Real side; 
		/** center of the polyflap */
		golem::Real center; 
		/** distance from the top of the polyflap */
		golem::Real top; 
		/** polyflap width */
		golem::Real width;

	protected:
		/** Creates the object from the description. */
		virtual golem::Object::Ptr create(golem::Scene& parameter) const {
			ActorObject *pObject = new ActorObject(parameter);
			golem::Object::Ptr pointer(pObject);
			if (!pObject->create(*this,parameter))
				pointer.release();

			return pointer;
		}
			
	};
	ActorObject(golem::Scene& scene) : Actor(scene){}
	/** returns the description */
	inline ActorObject::Desc getDescription() const {return _desc;}
	/** returns the feature vector of the object */
	inline FeatureVector getFeatureVector() const {return _featureVector;}
	/** returns the normal vector */
	inline golem::Vec3 getNormalVec() const{return _normalVec;}
	/** returns orienation of the object */
	inline golem::Vec3 getOrientation() const{return _orientation;}
	/** returns the orthogonal vector*/
	inline golem::Vec3 getOrthogonalVec() const {return _orthogonalVec;}
	/** returns the pitch of the object */
	inline golem::Real getPitch() const {return _obPitch;}
	/** returns the position of the object */
	inline golem::Vec3 getPosition() const {return _position;} 
	/** returns the roll of the object */
	inline golem::Real getRoll() const {return _obRoll;}
	/** returns the yaw of the object */
	inline golem::Real getYaw() const {return _obYaw;}
	/** sets the shape of the actor  */
	void setShape(golem::Scene&,ConcreteActor*);
	/** sets the description */
	inline void setDescription(const ActorObject::Desc& desc){_desc=desc;}
	/** sets the feature vector of the object */
	inline void setFeatureVector(const FeatureVector& newVec){_featureVector=newVec;}
	/** sets the pitch of the object */
	inline void setPitch(const golem::Real& newPitch){_obPitch=newPitch;}
	/** sets the position of the object */
	inline void setPosition(const golem::Vec3& newPosition){_position=newPosition;}
	/** sets the roll of the object */
	inline void setRoll(const golem::Real& newRoll){_obRoll=newRoll;}
	/** sets the yaw of the object */
	inline void setYaw(const golem::Real& newYaw){_obYaw=newYaw;}
	Mat34 getPose() const;
	void setPose(const Mat34&);
protected:
	/** specific ActorObject create function */
	bool create(const ActorObject::Desc&, golem::Scene&);
	/** computes normal and orthogonal vector of the object and determines the position of the object */
	void computeVectors();

private:
	/** concrete actor shape */
	ConcreteActor*		_concreteActor;
	/** description instance of the ActorObject */
	ActorObject::Desc 	_desc;
	/** feature vector (here any vectorial representation is possible) */
	FeatureVector 		_featureVector;
	/** normal vector of the object */
	golem::Vec3			_normalVec;
	/** Object orientation in Euler coordinates */
	golem::Real 		_obRoll, _obPitch, _obYaw;
	/** orientation of the object */
	golem::Vec3		_orientation;
	/** orthogonal vector of the object */
	golem::Vec3		_orthogonalVec;
	/** coordinates vector used for target description */
	golem::Vec3		_position;
		
		
};

}; // namespace smlearning 


#endif
