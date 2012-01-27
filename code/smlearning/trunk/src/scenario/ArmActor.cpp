/** @file ArmActor.cpp
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

#include <scenario/ArmActor.h>

namespace smlearning {

///////// Public //////////
/** \brief initializes the arm, sets async time, home position and orientation 
*/
void ArmActor::init(const golem::Context &context)
{

	_tmDeltaAsync = _arm->getReacPlanner().getTimeDeltaAsync();
	_arm->getArm().lookupState(_initial, context.getTimer().elapsed()); //other possibility: lookupCommand
	_home.pos = _desc.homePose;

	// move the arm with global path planning and collision detection
	_home.t = context.getTimer().elapsed() + getDeltaAsync() + SecTmReal(5.0);
	// movement will last no shorter than 5 sec

	_arm->getReacPlanner().send(_home, ReacPlanner::ACTION_GLOBAL);

	// wait until the arm is ready to accept new commands, but no longer than 60 seconds
	(void)_arm->getReacPlanner().waitForEnd(60000);
	
	// Define the initial pose in the Cartesian workspace
	//_orientationT.set(Real(-0.5*REAL_PI), Real(0.0*REAL_PI), Real(0.0*REAL_PI));

}

///////// Public //////////
/** \brief move the arm to its initial position
*
* \param context referes to the context of the scenario
*/
void ArmActor::moveArmToStartPose(const golem::Context &context)
{

	_initial.t = context.getTimer().elapsed() + getDeltaAsync() + SecTmReal(5.0);
	// movement will last no shorter than 5 sec
	_arm->getReacPlanner().send(_initial, ReacPlanner::ACTION_GLOBAL);
	// wait until the arm is ready to accept new commands, but no longer than 60 seconds
	(void)(_arm->getReacPlanner().waitForEnd(60000));
}

///////// Public //////////
/** \brief set the end of the experiment trajectory and let the finger move along the experiment trajectory
*
* \param context referes to the context of the scenario
* \param target workspace state used to describe the starting point and also the end point of trajectory
* \param bStart thread control variable
* \param duration minimal duration of the movement along the experiment trajectory 
* \param end is the end of the experimental trajectory
*/
void ArmActor::moveFinger(golem::Context &context,golem::GenWorkspaceState& target,volatile bool& bStart, SecTmReal& duration, WorkspaceCoord& end){
	target.pos = end;
	target.t = context.getTimer().elapsed() + getDeltaAsync() + duration;
	
	setCollisionDetection(false);	
	_arm->getReacPlanner().send(target, ReacPlanner::ACTION_LOCAL);
		
	// wait for the movement to start, but no longer than 60 seconds
	(void)_arm->getReacPlanner().waitForBegin(60000);
	context.getTimer().sleep(getDeltaAsync());
	bStart = true;
		
	// wait for the movement end, no longer than 60 seconds
	(void)_arm->getReacPlanner().waitForEnd(60000);
	context.getTimer().sleep(getDeltaAsync() + _desc.speriod);
	bStart = false;
}

///////// Public //////////
/** \brief move finger to initial position
*
* \param context referes to the context of the scenario
*/
void ArmActor::moveFingerToStartPose(golem::Context &context)
{
	_home.t = context.getTimer().elapsed() + getDeltaAsync() + SecTmReal(3.0);

	//turn on collision detection
	setCollisionDetection(true);
	
	//move the finger to home position
	sendPosition(context,_home , ReacPlanner::ACTION_GLOBAL);
	
	context.getMessageStream()->write(Message::LEVEL_INFO, "Moving home...");
	_arm->getReacPlanner().waitForEnd(60000);
}


///////// Public //////////
/** \brief move finger up in order to increase the chances of finding a suitable path to home position
*
* \param context referes to the context of the scenario
* \param target workspace state used to describe the starting point and also the end point of trajectory
*/
void ArmActor::moveFingerUp(const golem::Context &context, golem::GenWorkspaceState& target, const Vec3& dimension)
{
	
	Vec3 positionPreH(target.pos.p.v1, target.pos.p.v2, target.pos.p.v3 += (dimension.v2*1.1));
	// and set target waypoint
	golem::GenWorkspaceState preHome;
	
	preHome.pos.p = positionPreH;
	preHome.pos.R = _home.pos.R;
	preHome.vel.setId(); // it doesn't move

	preHome.t = context.getTimer().elapsed() + getDeltaAsync() + SecTmReal(2.0); // i.e. the movement will last at least 2 sec

	// set the initial pose of the arm, force the global movement (with planning in the entire arm workspace)
	_arm->getReacPlanner().send(preHome, ReacPlanner::ACTION_GLOBAL);
	// wait for completion of the action (until the arm moves to the initial pose)
	_arm->getReacPlanner().waitForEnd();
}


///////// Public //////////
/** \brief try to find a path to given position, if found, move the finegr along it and wait for it to stop
*
* \param context referes to the context of the scenario
* \param position
* \param action
*/
void ArmActor::sendPosition(golem::Context &context,const golem::GenWorkspaceState& position,const golem::ReacPlanner::Action action)
{

	for (int t=0; t < _maxTrials ; t++) {
		if (_arm->getReacPlanner().send(position, action)) {
			break;
		}
		context.getMessageStream()->write(Message::LEVEL_INFO, "Unable to find path to polyflap, trying again.");
	}


	context.getMessageStream()->write(Message::LEVEL_INFO, "Moving...");
	// wait for completion of the action (until the arm stops moving)
	_arm->getReacPlanner().waitForEnd(60000);
}

///////// Public //////////
/** turns collision detection on/off
*
* \param b true for collision detection on and false for off
*/
void ArmActor::setCollisionDetection(const bool& b)
{
	if (b) {
 		// ON collision detection
		_arm->setCollisionBoundsGroup(0xFFFFFFFF);

	}
	else {
		//off collision detection
		_arm->setCollisionBoundsGroup(0x0);
	}
}


///////// Protected //////////
/** \brief specific ActorObject create function 
*/
bool ArmActor::create(const ArmActor::Desc& desc, golem::Scene& scene)
{
	if( (_arm = dynamic_cast<PhysReacPlanner*>(scene.createObject(desc.armDesc))) == NULL)
		return false; // throws

	setDescription(desc);

	_effector = _arm->getJointActors().back();	
	
	Mat34 armPose = _arm->getArm().getGlobalPose();
	_arm->getArm().setGlobalPose (armPose);

	_effectorBounds.clear();	
	
	for (Bounds::Desc::Seq::const_iterator i = desc.fingerDesc.begin(); i != desc.fingerDesc.end(); i++) {
		const Bounds* pBounds = _effector->createBounds(*i);
		if ((*i)->group == desc.effectorGroup)
			_effectorBounds.push_back(pBounds->clone());
	}
	_arm->getArm().setReferencePose(desc.referencePose);
	
	return true;
}

}; // namespace smlearning 
