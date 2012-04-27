/** @file ArmActor.h
 * 
 * 
 * @author	Manuel Noll (DFKI)
 * @author	Sergio Roa (DFKI)
 * @author      Jan Hanzelka (DFKI)
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

#ifndef ARMACTOR_H
#define ARMACTOR_H

#include <scenario/ActorObject.h>
#include <Golem/PhysCtrl/PhysReacPlanner.h>

namespace smlearning {

class ArmActor 
{
public:
	/** Object description */
	class Desc : public ActorObject::Desc 
	{
	public:
		golem::Mat34 initialObjectPose;
		golem::Real initialObRoll, initialObPitch, initialObYaw;
		FeatureVector initialFeatureVector;

		/** Arm */
		golem::PhysReacPlanner::Desc armDesc;
		/** Effector bounds group */
		golem::U32 effectorGroup;
		/** Finger */
		golem::Bounds::Desc::Seq fingerDesc;
		/** Global end-effector home pose */
		golem::WorkspaceCoord homePose;
		/** minimal duration of a movement (by normal speed) */
		SecTmReal minDuration;
		/** Local end-effector reference pose */
		golem::WorkspaceCoord referencePose;
		/** Motion effects stabilization time period */
		golem::SecTmReal speriod;
		/** Constructs description object */
		Desc(){
			Desc::setToDefault();
		}
		/** Checks if the description is valid. */
		virtual bool isValid() const 
		{
			if (!referencePose.isFinite() || !homePose.isFinite())
				return false;
			for (golem::Bounds::Desc::Seq::const_iterator i = fingerDesc.begin(); i != fingerDesc.end(); i++)
				if (!(*i)->isValid())
					return false;
			
			return true;
		}
		/** Sets the parameters to the default values */
		void setToDefault(){Actor::Desc::setToDefault();}
				
	protected:
		/** Creates the object from the description. */
		/*	virtual golem::Object::Ptr create(golem::Scene& parameter) const {
			ArmActor *pObject = new ArmActor(parameter);
			golem::Object::Ptr pointer(pObject);
			if (!pObject->create(*this))
			pointer.release();
			return pointer;
			}*/
	
	};
		
	/** std cto */
	ArmActor(const ArmActor::Desc& desc, golem::Scene& scene);
	/** std dto */
	~ArmActor ();
	/** try to find a path to given position, if found, move the finegr along it and wait for it to stop */
	void sendPosition(golem::Context&,const golem::GenWorkspaceState&,const golem::ReacPlanner::Action);
	/** turns collision detection on/off */
	void setCollisionDetection(const bool&);
	/** initializes the arm, sets async time, home position and orientation */
	void init(const golem::Context &);
	/** \brief returns the arm */
	inline golem::PhysReacPlanner* getArm() {return _arm;}
	/** \brief returns the state of the arm */
	inline golem::GenConfigspaceState getArmState() const {return _armState;} 
	/** \brief returns the horizontal angle */
	inline golem::Real getHorizontalAngle() const {return _horizontalAngle;}
	/** \brief returns the push duration */
	inline golem::Real getPushDuration() const {return _pushDuration;}
	/** \brief returns the effector */
	inline golem::JointActor* getEffector() {return _effector;}
	/** \brief returns effector bounds */
	inline golem::Bounds::Seq getEffectorBounds() {return _effectorBounds;}
	/** \brief returns the end effector pose */
	inline golem::Mat34 getEndEffectorPose() const {return _endEffectorPose;}		
	/** \brief returns the end effector roll */
	inline golem::Real getEndEfRoll() const {return _endEfRoll;} 
	/** \brief returns the end effector pitch */
	inline golem::Real getEndEfPitch() const {return _endEfPitch;} 
	/** \brief returns the end effector yaw */
	inline golem::Real getEndEfYaw() const {return _endEfYaw;}		
	/** \brief returns the delta async time */
	inline SecTmReal getDeltaAsync() const {return _tmDeltaAsync;}
	/** \brief set the state of the arm */
	inline void setArmState(const golem::GenConfigspaceState& newState){_armState=newState;}
	/** \brief set the horizontal angle */
	inline void setHorizontalAngle(const golem::Real& newAngle){_horizontalAngle=newAngle;}
	/** \brief set push duration */
	/** speed ( 3 (fast), 4 (middle), 5 (low) */
	inline void setPushDuration(const golem::Real& newDuration){_pushDuration=newDuration;}
	/** \brief set end effector pose */
	inline void setEndEffectorPose(const golem::Mat34& newEndPose){_endEffectorPose=newEndPose;}
	/** \brief set end effector roll */
	inline void setEndEfRoll(const golem::Real& newEndRoll){_endEfRoll=newEndRoll;} 
	/** \brief set end effector pitch */
	inline void setEndEfPitch(const golem::Real& newEndPitch){_endEfPitch=newEndPitch;} 
	/** \brief set end effector yaw */
	inline void setEndEfYaw(const golem::Real& newEndYaw){_endEfYaw=newEndYaw;}
	/** \brief set Description */
	inline void setDescription(const ArmActor::Desc& desc){_desc=desc;}
	/** \brief set the maximal number of trials */
	inline void setMaxTrials(const int& newMaxTrials ){_maxTrials=newMaxTrials;}
	/** move the arm to its initial position */
	void moveArmToStartPose(const golem::Context&);
	/** set the end of the experiment trajectory and let the finger move along the experiment trajectory */
	void moveFinger(golem::Context&,golem::GenWorkspaceState&,volatile bool&, SecTmReal&, WorkspaceCoord&);
	/** move finger to initial position */
	void moveFingerToStartPose(golem::Context&);
	/** move finger to start position of pushing */
	void moveFingerToStartPushing(golem::Context&, golem::GenWorkspaceState&);
	/** move finger up in order to increase the chances of finding a suitable path to home position */
	void moveFingerUp(const golem::Context&,golem::GenWorkspaceState&,const Vec3&);

protected:
	/** specific ActorObject create function */
	bool create(const ArmActor::Desc&, golem::Scene&);	

private:
		
	/** Arm */
	golem::PhysReacPlanner* 	_arm;
	/** Arm state - (joint) dynamic configuration */
	golem::GenConfigspaceState 	_armState;
	/** description instance of the arm */
	ArmActor::Desc 			_desc;
	/** End-effector */
	golem::JointActor* 		_effector;
	/** End-effector bounds */
	golem::Bounds::Seq 		_effectorBounds;
	/** direction vector (target pose) */
	golem::Mat34 			_endEffectorPose;
	/** target effector orientation in Euler coordinates */
	golem::Real 			_endEfRoll, _endEfPitch, _endEfYaw;		
	/** statespaces describing the initial position of the arm (first and last position of the arm in every experiment session) */
	golem::GenConfigspaceState 	_initial;
	/** workspace state describing the home position (first and last position of every experiment iteration) */
	GenWorkspaceState 		_home;
	/** horizontal angle */
	golem::Real 			_horizontalAngle;
	/** number of maximal trials in the sendPosition procedure, given by the scenario*/
	int 				_maxTrials;
	/** speed ( 3 (fast), 4 (middle), 5 (low) */
	golem::Real 			_pushDuration;
	/** asynchron time delta*/
	SecTmReal 			_tmDeltaAsync;
	/** rotations vector used for target description */
	Vec3 				_orientationT;

};

}; // namespace smlearning 

#endif
