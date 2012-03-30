/** @file Scenario.cpp
 * 
 * Learning scenario where the arm moves along a straight line
 * using reactive trajectory planner and collision detection
 * simulating a pushing action on a polyflap
 * 
 * Program can be run in two modes:
 * - the first uses real Katana arm
 * - the second runs the arm simulators
 * 
 * @author	Sergio Roa (DFKI)
 *
 * @version 1.0
 *
 * Copyright 2009      Sergio Roa
 *
 * @author	Marek Kopicki (see copyright.txt),
 * 			<A HREF="http://www.cs.bham.ac.uk/~msk">The University Of Birmingham</A>
 * @author      Jan Hanzelka - DFKI

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

#include <metalearning/Scenario.h>
//#include <tools/data_handling.h>

namespace smlearning {

//------------------------------------------------------------------------------

bool XMLData(Scenario::Desc &val, XMLContext* xmlcontext, Context *context) {
	if (xmlcontext == NULL) {
		ASSERT(false)
		return false;
	}
        // arm setup
        //golem::XMLData(val.armDesc.pArmDesc, xmlcontext->getContextFirst("arm")); //doesn't work anymore
	std::string driver;
	XMLData("driver", driver, xmlcontext->getContextFirst("arm")); // Get arm driver name
	val.armDesc.pArmDesc = Arm::Desc::load(*context, driver); // Load driver
	
	// finger setup
	val.fingerDesc.clear();
	//golem::Real baseLength = golem::KatArm::L3; //not present anymore
	golem::Real baseLength = 0.1848;
	XMLData(baseLength, xmlcontext->getContextFirst("effector base_length"));
	golem::Real fingerLength = 0.135;
	XMLData(fingerLength, xmlcontext->getContextFirst("effector finger_length"));
	golem::Real fingerDiam = 0.02;
	XMLData(fingerDiam, xmlcontext->getContextFirst("effector finger_diameter"));
	golem::Real tipRadius = 0.015;
	XMLData(tipRadius, xmlcontext->getContextFirst("effector tip_radius"));
	
	golem::BoundingBox::Desc* pFingerRodShapeDesc = new golem::BoundingBox::Desc;
	pFingerRodShapeDesc->dimensions.set(fingerDiam/2.0, fingerLength/2.0, fingerDiam/2.0);
	pFingerRodShapeDesc->pose.p.v2 += baseLength + fingerLength/2.0;
	pFingerRodShapeDesc->group = val.effectorGroup;
	val.fingerDesc.push_back(golem::Bounds::Desc::Ptr(pFingerRodShapeDesc));
	golem::BoundingSphere::Desc* pFingerTipShapeDesc = new golem::BoundingSphere::Desc;
	pFingerTipShapeDesc->radius = tipRadius;
	pFingerTipShapeDesc->pose.p.v2 += golem::Real(baseLength + fingerLength);
	pFingerTipShapeDesc->group = val.effectorGroup;
	val.fingerDesc.push_back(golem::Bounds::Desc::Ptr(pFingerTipShapeDesc));
	
	// end-effector reference pose
	val.referencePose.setId();
	val.referencePose.p.v2 += golem::Real(baseLength + fingerLength);



	
	//polyflap interaction settings

	//a number that slightly greater then the maximal reachable space of the arm
	//    - used for workspace position normalization and later as a position upper bound
	//      for random polyflap position
	//maxRange = 0.4;
	//XMLData(val.maxRange, xmlcontext->getContextFirst("polyflapInteraction maxRange"));
	//maximum X value for polyflap position
	XMLData(val.featLimits.maxX, xmlcontext->getContextFirst("polyflapInteraction maxX"));
	//maximum Y value for polyflap position
	XMLData(val.featLimits.maxY, xmlcontext->getContextFirst("polyflapInteraction maxY"));
	//maximum Z value for polyflap position
	XMLData(val.featLimits.maxZ, xmlcontext->getContextFirst("polyflapInteraction maxZ"));
	//minimum X value for polyflap position
	XMLData(val.featLimits.minX, xmlcontext->getContextFirst("polyflapInteraction minX"));
	//minimum Y value for polyflap position
	XMLData(val.featLimits.minY, xmlcontext->getContextFirst("polyflapInteraction minY"));
	//minimum Z value for polyflap position
	XMLData(val.featLimits.minZ, xmlcontext->getContextFirst("polyflapInteraction minZ"));
	//minimum duration value for pushing action
	XMLData(val.featLimits.minDuration, xmlcontext->getContextFirst("polyflapInteraction minPushDuration"));
	//maximum duration value for pushing action
	XMLData(val.featLimits.maxDuration, xmlcontext->getContextFirst("polyflapInteraction maxPushDuration"));
	//minimum value for a label
	XMLData(val.featLimits.minValLabel, xmlcontext->getContextFirst("polyflapInteraction minValLabel"));
	//maximum value for a label
	XMLData(val.featLimits.maxValLabel, xmlcontext->getContextFirst("polyflapInteraction maxValLabel"));

	//minimal duration of a movement (by normal speed)
	XMLData(val.minDuration, xmlcontext->getContextFirst("polyflapInteraction minDuration"));


	Real x;
	Real y;
	Real z;
	
	//Polyflap Position and orientation
	XMLData(x, xmlcontext->getContextFirst("polyflapInteraction startPolyflapPosition x"));
	XMLData(y, xmlcontext->getContextFirst("polyflapInteraction startPolyflapPosition y"));
	XMLData(z, xmlcontext->getContextFirst("polyflapInteraction startPolyflapPosition z"));
	val.startPolyflapPosition.set(x, y, z);

	XMLData(x, xmlcontext->getContextFirst("polyflapInteraction startPolyflapRotation x"));
	XMLData(y, xmlcontext->getContextFirst("polyflapInteraction startPolyflapRotation y"));
	XMLData(z, xmlcontext->getContextFirst("polyflapInteraction startPolyflapRotation z"));
	val.startPolyflapRotation.set(y, x, z);

	//Polyflap dimensions		
	XMLData(x, xmlcontext->getContextFirst("polyflapInteraction polyflapDimensions x"));
	XMLData(y, xmlcontext->getContextFirst("polyflapInteraction polyflapDimensions y"));
	XMLData(z, xmlcontext->getContextFirst("polyflapInteraction polyflapDimensions z"));
	val.polyflapDimensions.set(x, y, z);
	//Polyflap width
	XMLData(val.pfWidth, xmlcontext->getContextFirst("polyflapInteraction polyflapDimensions width"));
	

	//vertical distance from the ground
	//const Real over = 0.01;
	//vertical distance from the ground considering fingertip radius
	XMLData(val.over, xmlcontext->getContextFirst("polyflapInteraction over"));
	//distance from the front/back of the polyflap
	XMLData(val.dist, xmlcontext->getContextFirst("polyflapInteraction dist"));

	Real r;

	//distance from the side of the polyflap
	XMLData(r, xmlcontext->getContextFirst("polyflapInteraction side"));
	val.side = val.polyflapDimensions.v1*r;
	//center of the polyflap
	XMLData(r, xmlcontext->getContextFirst("polyflapInteraction center"));
	val.center = val.polyflapDimensions.v2*r;
	//distance from the top of the polyflap
	//const Real top = polyflapDimensions.v2* 1.2;
	XMLData(r, xmlcontext->getContextFirst("polyflapInteraction top"));
	val.top = val.polyflapDimensions.v2 - r;
	//lenght of the movement		
	XMLData(val.distance, xmlcontext->getContextFirst("polyflapInteraction distance"));
	

	XMLData(val.startingPositionsConfig, xmlcontext->getContextFirst("loop startingPositions"));
	
	
	return true;
}

//------------------------------------------------------------------------------


Scenario::Scenario(golem::Scene &scene) : Object(scene), creator(scene) {
	arm = NULL;
	object = NULL;
	obstacles = NULL;
	bStart = bStop = bRec = false;
}

bool Scenario::create(const Scenario::Desc& desc) {
	if (!desc.isValid()) {
		context.getMessageStream()->write(Message::LEVEL_CRIT, "Scenario::create(): invalid description");
		return false;
	}
	this->desc = desc;

	obstacles = dynamic_cast<Actor*>(scene.createObject(*creator.createGroundPlaneDesc())); // throws
	if (obstacles == NULL)
		return false;
	
	arm = dynamic_cast<PhysReacPlanner*>(scene.createObject(desc.armDesc)); // throws
	if (arm == NULL)
		return false;
	effector = arm->getJointActors().back();
	Mat34 armPose = arm->getArm().getGlobalPose();
// 	armPose.p.v3 = 0.03;
	arm->getArm().setGlobalPose (armPose);
	
	effectorBounds.clear();
	for (Bounds::Desc::Seq::const_iterator i = desc.fingerDesc.begin(); i != desc.fingerDesc.end(); i++) {
		const Bounds* pBounds = effector->createBounds(*i);
		if ((*i)->group == desc.effectorGroup)
			effectorBounds.push_back(pBounds->clone());
	}
	arm->getArm().setReferencePose(desc.referencePose);

	return true;
}

void Scenario::release() {
	if (arm != NULL)
		scene.releaseObject(*arm);
	if (obstacles != NULL)
		scene.releaseObject(*obstacles);
}


///
///creates a polyflap object
///setupPolyflap(scene, desc.startPolyflapPosition, desc.startPolyflapRotation, desc.polyflapDimensions, context)
Actor* Scenario::setup_polyflap(/*Scene &scene, Vec3 position, Vec3 rotation, Vec3 dimensions, golem::Context &context*/) {
	// Creator
	Creator creator(scene);
	Actor::Desc *pActorDesc;
	Actor *polyFlapActor;
	
	// Create polyflap
	pActorDesc = creator.createSimple2FlapDesc(Real(desc.polyflapDimensions.v1*0.5), Real(desc.polyflapDimensions.v1*0.5), Real(desc.polyflapDimensions.v1*0.5), Real(desc.pfWidth*0.5), REAL_PI_2);
// 	pActorDesc = creator.createSimple2FlapDesc(Real(desc.polyflapDimensions.v1), Real(desc.polyflapDimensions.v1), Real(desc.polyflapDimensions.v1));
// 	pActorDesc = creator.createSimple2FlapDesc(Real(desc.polyflapDimensions.v1), Real(desc.polyflapDimensions.v1), Real(desc.polyflapDimensions.v1), Real(0.002), Real(REAL_PI_2));

	//-sets coordinates
	pActorDesc->nxActorDesc.globalPose.t.set(NxReal(desc.startPolyflapPosition.v1), NxReal(desc.startPolyflapPosition.v2), NxReal(desc.startPolyflapPosition.v3));	


	/*  not necessary anymore
	Mat34 pose;
	pose.R.fromEuler(
		desc.startPolyflapRotation.v1, 
		desc.startPolyflapRotation.v2, 
		desc.startPolyflapRotation.v3 
	);
	*/
	
	//-sets rotations	
	//pActorDesc->nxActorDesc.globalPose.M.setRowMajor(&pose.R._m._11); //_m not present anymore	
	pActorDesc->nxActorDesc.globalPose.M.rotX(desc.startPolyflapRotation.v1);	
	pActorDesc->nxActorDesc.globalPose.M.rotY(desc.startPolyflapRotation.v2);	
	pActorDesc->nxActorDesc.globalPose.M.rotZ(desc.startPolyflapRotation.v3);	

	//-density
// 	pActorDesc->nxActorDesc.density = NxReal(5.0);	

	polyFlapActor = dynamic_cast<Actor*>(scene.createObject(*pActorDesc));

	return polyFlapActor;
}


///
///creates a polyflap object
///
Actor* Scenario::setup_polyflap(Scene &scene, Mat34& globalPose, Vec3 dimensions) {
	// Creator
	Creator creator(scene);
	Actor::Desc *pActorDesc;
	Actor *polyFlapActor;
	
	// Create polyflap
	pActorDesc = creator.createSimple2FlapDesc(Real(dimensions.v1*0.5), Real(dimensions.v1*0.5), Real(dimensions.v1*0.5), Real(desc.pfWidth*0.5), REAL_PI_2);
// 	pActorDesc = creator.createSimple2FlapDesc(Real(dimensions.v1), Real(dimensions.v1), Real(dimensions.v1));
// 	pActorDesc = creator.createSimple2FlapDesc(Real(dimensions.v1), Real(dimensions.v1), Real(dimensions.v1), Real(0.002), Real(REAL_PI_2));

	//-sets coordinates
	pActorDesc->nxActorDesc.globalPose.t.set(NxReal(globalPose.p.v1), NxReal(globalPose.p.v2), NxReal(globalPose.p.v3));	


	//-sets rotations	
	//pActorDesc->nxActorDesc.globalPose.M.setRowMajor(&globalPose.R._m._11); //_m not present anymore	
	pActorDesc->nxActorDesc.globalPose.M.rotX(desc.startPolyflapRotation.v1);	
	pActorDesc->nxActorDesc.globalPose.M.rotY(desc.startPolyflapRotation.v2);	
	pActorDesc->nxActorDesc.globalPose.M.rotZ(desc.startPolyflapRotation.v3);	

	
	//-density
// 	pActorDesc->nxActorDesc.density = NxReal(5.0);	

	polyFlapActor = dynamic_cast<Actor*>(scene.createObject(*pActorDesc));

	return polyFlapActor;
}


///
///creates the polyflap object, obtains bounds and determines current rotation of the polyflap
///
void Scenario::create_polyflap_object(){

	object = setup_polyflap(/*scene, desc.startPolyflapPosition, desc.startPolyflapRotation, desc.polyflapDimensions, context*/);
	//golem::Bounds::SeqPtr curPol = object->getGlobalBoundsSeq();
	curPol = object->getGlobalBoundsSeq();

	object->getPose().R.toEuler (currentPfRoll, currentPfPitch, currentPfYaw);

}


///
///computes normal and orthogonal vector of the polyflap and determines the position of the polyflap
///
void Scenario::compute_vectors(){
	Mat34 curPolPos1;
	Mat34 curPolPos2;
	//find out bounds of polyflap and compute the position of the polyflap
	if (curPol->front()->getPose().p.v3 > curPol->back()->getPose().p.v3) {
		curPolPos1 = curPol->front()->getPose();
		curPolPos2 = curPol->back()->getPose();
	}
	else {
		curPolPos1 = curPol->back()->getPose();
		curPolPos2 = curPol->front()->getPose();
	}
		
	polyflapPosition.set(curPolPos1.p.v1, curPolPos1.p.v2, curPolPos2.p.v3);

	//Normal vector showing the direction of the lying part of polyflap, and it' orthogonal
	polyflapNormalVec =
		computeNormalVector(
			    Vec3 (curPolPos1.p.v1, curPolPos1.p.v2, Real(0.0)),
			    Vec3 (curPolPos2.p.v1, curPolPos2.p.v2, Real(0.0))
			    );
		
	polyflapOrthogonalVec = computeOrthogonalVec(polyflapNormalVec);	
}


///
///prepares the polyflap to use
///
void Scenario::initialize_polyflap(){

	//creates a polyflap
	create_polyflap_object();

	currentPfY = object->getPose().p.v2;

	//computes needed information about the polyflap
	compute_vectors();
}

///
///select an optimal action from a random set of actions
///
void Scenario::choose_action () {
	//choose starting position
	define_start_position ();

	//pushDuration = floor (randomG.nextUniform (3.0, 6.0));
	pushDuration = 3;

	//chose random horizontal and vertical angle
	horizontalAngle = choose_angle(60.0, 120.0);
	//int verticalAngle = rand() % 7;

}

///
///set current position of the polyflap as default position for computing the starting finger position
///
void Scenario::init_positionT(Vec3& positionT) {
	//initialization of arm target: the center of the polyflap
	//Vec3 positionT(Real(polyflapPosition.v1), Real(polyflapPosition.v2), Real(polyflapPosition.v3));
	positionT.set(Real(polyflapPosition.v1), Real(polyflapPosition.v2), Real(polyflapPosition.v3));
}


///
///choose the starting position
///
void Scenario::define_start_position(){
	


	if (startingPosition == 0)
		//startPosition = floor(randomG.nextUniform (1.0, /*19.0*/25.0));
		startPosition = availableStartingPositions[floor(randomG.nextUniform (0.0,Real(availableStartingPositions.size())))];
	else
		startPosition = startingPosition;
	

	assert (startPosition >= 1 && startPosition <= startingPositionsCount);

	usedStartingPositions.push_back(startPosition);


}


///
///set the variable target so that it obtains the coordinates of the start point of the experiment trajectory
///
void Scenario::prepare_target(){
;
	cout << "Position " << startPosition-1 << endl;
	
	// and set target waypoint
	//golem::GenWorkspaceState target;
	fromCartesianPose(target.pos, positionT, orientationT);
	// cout << "target pos: " << target.pos.p.v1 << ", " << target.pos.p.v2 << ", " << target.pos.p.v3 << endl;
	target.vel.setId(); // it doesn't move
	
	target.t = context.getTimer().elapsed() + tmDeltaAsync + desc.minDuration; // i.e. the movement will last at least 5 sec
	
}


///
///choose and describe the start point of the experiment trajectory
///
void  Scenario::calculate_starting_position_coord(){

	//set default coordinates
	init_positionT(positionT);

	//set the variable target so that it obtains the coordinates of the start point
	//arm target update
	//set_coordinates_into_target(startPosition, positionT, polyflapNormalVec, polyflapOrthogonalVec, desc.dist, desc.side, desc.center, desc.top, desc.over);

	prepare_target();


}

///
///select random angle (discrete or continouos)
///
Real Scenario::choose_angle(Real min, Real max, bool continuous){
	Real res;
	if (!continuous)
		res = floor(randomG.nextUniform (min, max));
	else 
		res = randomG.nextUniform (min, max);
	return res;
}



///
///describe the experiment trajectory
///
void Scenario::set_up_movement(){

		
	// Trajectory duration calculated from a speed constant.
	//pushDuration = floor (randomG.nextUniform (3.0, 5.0));
	//pushDuration = 3.0;
	cout << "push duration: " << pushDuration << endl;
		
	duration = pushDuration;


	// Trajectory end pose equals begin + shift along Y axis
	end = target.pos;

	//normal vector to the center of the polyflap
	Vec3 polyflapCenterNormalVec =
		computeNormalVector(
				    Vec3 (positionT.v1, positionT.v2, positionT.v3),
				    Vec3 (polyflapPosition.v1, polyflapPosition.v2, desc.polyflapDimensions.v2*0.5)
				    );
	//and it's orthogonal
	Vec3 polyflapCenterOrthogonalVec = computeOrthogonalVec(polyflapCenterNormalVec);
	// 		context.getLogger()->post(Message::LEVEL_INFO, "centernormalvec: %f, %1f, %f", polyflapCenterNormalVec.v1, polyflapCenterNormalVec.v2, polyflapCenterNormalVec.v3);


	set_movement_angle(horizontalAngle, end, desc.distance, polyflapCenterNormalVec, polyflapCenterOrthogonalVec);
	cout << "Horizontal direction angle: " << horizontalAngle << " degrees" << endl;

}


///
///storing a label (in this case polyflap abstract behavior)
///
void Scenario::add_label_object_behavior (LearningData::Chunk& chunk) {
	Real polStateOutput = 0; //polyflap moves with the same Y angle
	Real epsilonAngle = 0.005;
	// Real pfFlipThreshold = REAL_PI / 4.0;
	
	if (learningData.currentChunkSeq.size() > 1) {
		if (currentPfRoll < (chunk.object.obRoll - epsilonAngle) ) {//polyflap Y angle increases
			polStateOutput = 1;
			// if (obRoll > pfFlipThreshold)
			// 	polStateOutput = 1;
			// else
			// 	polStateOutput = 0.75;
		}
		if (currentPfRoll > (chunk.object.obRoll + epsilonAngle) )//polyflap Y angle decreases
			polStateOutput = -1;
		Real epsilonPfY = 0.000001;
		if (polStateOutput == 0 && currentPfY < (chunk.object.objectPose.p.v2 - epsilonPfY) ) // polyflap Y position increases
			polStateOutput = 0.5;
		if (polStateOutput == 0 && currentPfY > (chunk.object.objectPose.p.v2 + epsilonPfY) ) //polyflap Y position decreases
			polStateOutput = -0.5;
	}
	
	// 		cout << "polStateOutput: " << polStateOutput << endl;

	// if (obRoll > reachedAngle)
	// 	reachedAngle = obRoll;
	
	chunk.label = polStateOutput;

}


///
///write data chunk (used in postprocess function)
///
void Scenario::write_chunk (LearningData::Chunk& chunk) {
	chunk.timeStamp = trialTime;
	//arm->getArm().lookupInp(chunk.action.armState, context.getTimer().elapsed()); //not present anymore
	arm->getArm().lookupState(chunk.action.armState, context.getTimer().elapsed()); //other possibility: lookupCommand
	chunk.action.effectorPose = effector->getPose();
	chunk.action.effectorPose.multiply (chunk.action.effectorPose, effectorBounds.at(1)->getPose());
	chunk.action.effectorPose.R.toEuler (chunk.action.efRoll, chunk.action.efPitch, chunk.action.efYaw);
	chunk.action.horizontalAngle = horizontalAngle;
	chunk.action.pushDuration = pushDuration;
	chunk.action.endEffectorPose = end;
	chunk.action.endEffectorPose.R.toEuler (chunk.action.endEfRoll, chunk.action.endEfPitch, chunk.action.endEfYaw);

	chunk.object.objectPose = object->getPose();
	chunk.object.objectPose.R.toEuler (chunk.object.obRoll, chunk.object.obPitch, chunk.object.obYaw);

	// golem::Mat34 p = chunk.object.objectPose; 
	// golem::Mat34 p = chunk.action.effectorPose; 
	// golem::Mat34 p = chunk.action.endEffectorPose;
	// cout << "pose: ";

	// cout << p.p.v1 << " " << p.p.v2 << " " << p.p.v3 << " " << chunk.action.endEfRoll << " " << chunk.action.endEfPitch << " " << chunk.action.endEfYaw << endl;

	if (storeLabels) add_label_object_behavior (chunk);
	// trialTime += SecTmReal(1.0)/universe.getRenderFrameRate();

}


void Scenario::postprocess(SecTmReal elapsedTime) {
	if (bStart) {
		CriticalSectionWrapper csw(cs);
		if (object == NULL) {
			return;
		}

		LearningData::Chunk chunk;
		write_chunk (chunk);

		//LearningData::write_chunk_to_featvector (chunk.featureVector, chunk, normalize<Real>, learningData.featLimits);

// 		learningData.data.push_back(chunk);

		learningData.currentChunkSeq.push_back (chunk);

		currentPfRoll = chunk.object.obRoll;
		currentPfY = chunk.object.objectPose.p.v2;
	}
}


///
///initialize the experiment
///
void Scenario::first_init(){

// 	// initialize random seed:
	randomG.setRandSeed (context.getRandSeed());

	//const SecTmReal tmDeltaAsync = arm->getReacPlanner().getTimeDeltaAsync();
	tmDeltaAsync = arm->getReacPlanner().getTimeDeltaAsync();

	// get initial configuration (it is the current joint configuration)
	//golem::GenConfigspaceState initial;
	//arm->getArm().lookupInp(initial, context.getTimer().elapsed()); //not present anymore
	arm->getArm().lookupState(initial, context.getTimer().elapsed()); //other possibility: lookupCommand

	learningData.setToDefault(desc.featLimits);

}


///
///describe the home position (position, where the finger starts and ends every iteration)
///
void Scenario::setup_home(){
	// setup home position
	//GenWorkspaceState home;
	home.pos = desc.homePose;
	// move the arm with global path planning and collision detection
	home.t = context.getTimer().elapsed() + tmDeltaAsync + SecTmReal(5.0);
	// movement will last no shorter than 5 sec
	arm->getReacPlanner().send(home, ReacPlanner::ACTION_GLOBAL);
	// wait until the arm is ready to accept new commands, but no longer than 60 seconds
	(void)arm->getReacPlanner().waitForEnd(60000);
	
	
	// Define the initial pose in the Cartesian workspace
	orientationT.set(Real(-0.5*REAL_PI), Real(0.0*REAL_PI), Real(0.0*REAL_PI));
}

///
///try to find a path to given position, if found, move the finegr along it and wait for it to stop
///
void Scenario::send_position(golem::GenWorkspaceState& position, golem::ReacPlanner::Action action){
// reachedAngle = 0.0;
	for (int t=0; t<MAX_PLANNER_TRIALS; t++) {
		if (arm->getReacPlanner().send(position, action)) {
			break;
		}
		context.getMessageStream()->write(Message::LEVEL_INFO, "Unable to find path to polyflap, trying again.");
	}


	context.getMessageStream()->write(Message::LEVEL_INFO, "Moving...");
	// wait for completion of the action (until the arm stops moving)
	arm->getReacPlanner().waitForEnd(60000);
}


///
///create feature vector and sequence
///
void Scenario::init_writing(){

	/////////////////////////////////////////////////
	//create sequence for this loop run
	learningData.currentChunkSeq.clear();
	/////////////////////////////////////////////////	
}


///
///initialize learning data
///
void Scenario::init_data(){
	// initialize data
	// learningData.effector = effectorBounds;
	// learningData.object = *object->getLocalBoundsSeq();
	// learningData.obstacles = *obstacles->getGlobalBoundsSeq();
}


///
///set the end of the experiment trajectory, initialize learning data and let the finger move along the experiment trajectory
///
void Scenario::move_finger(){
	target.pos = end;
	target.t = context.getTimer().elapsed() + tmDeltaAsync + duration;

	//arm->setCollisionBoundsGroup(0x0);
	set_collision_detection(false);	
	arm->getReacPlanner().send(target, ReacPlanner::ACTION_LOCAL);

	init_data();


		
	// wait for the movement to start, but no longer than 60 seconds
	(void)arm->getReacPlanner().waitForBegin(60000);
	context.getTimer().sleep(tmDeltaAsync);
	bStart = true;
		
	// wait for the movement end, no longer than 60 seconds
	(void)arm->getReacPlanner().waitForEnd(60000);
	context.getTimer().sleep(tmDeltaAsync + desc.speriod);
	bStart = false;
}


///
///turn the finger collision detection on (true) or off (false)
///
void Scenario::set_collision_detection(bool b){
	if (b) {
 		// ON collision detection
		arm->setCollisionBoundsGroup(0xFFFFFFFF);

	}
	else {
		//off collision detection
		arm->setCollisionBoundsGroup(0x0);
	}
}


///
///print out desired sequenc information
///
void Scenario::print_sequence_info(){
	cout << "sequence size: " << learningData.currentChunkSeq.size() << endl;
}


///
///move finger up in order to increase the chances of finding a suitable path to home position
///
void Scenario::move_finger_up(){
	Vec3 positionPreH(target.pos.p.v1, target.pos.p.v2, target.pos.p.v3 += (desc.polyflapDimensions.v2*1.1));
	// and set target waypoint
	golem::GenWorkspaceState preHome;
	preHome.pos.p = positionPreH;
	preHome.pos.R = home.pos.R;
	preHome.vel.setId(); // it doesn't move

	preHome.t = context.getTimer().elapsed() + tmDeltaAsync + SecTmReal(2.0); // i.e. the movement will last at least 2 sec

	// set the initial pose of the arm, force the global movement (with planning in the entire arm workspace)
	arm->getReacPlanner().send(preHome, ReacPlanner::ACTION_GLOBAL);
	// wait for completion of the action (until the arm moves to the initial pose)
	arm->getReacPlanner().waitForEnd();
}


///
///remove the polyflap object from the scene
///
void Scenario::remove_polyflap(){
	// remove object
	scene.releaseObject(*object);
	object = NULL;
//	scene.releaseObject(*predictedPolyflapObject);
}


///
///movement to home position
///
void Scenario::move_finger_home() {
	home.t = context.getTimer().elapsed() + tmDeltaAsync + SecTmReal(3.0);

	//turn on collision detection
	set_collision_detection(true);
	
	//move the finger to home position
	send_position(home , ReacPlanner::ACTION_GLOBAL);
	
	context.getMessageStream()->write(Message::LEVEL_INFO, "Moving home...");
	arm->getReacPlanner().waitForEnd(60000);

	
}


///
///print out desired information at the end of a sequence
///
void Scenario::iteration_end_info(){
	context.getMessageStream()->write(Message::LEVEL_INFO, "Done");
	cout << "Iteration " << iteration << " completed!" << endl;
}


///
///finish current iteration
///
void Scenario::check_interrupted(){
	if (universe.interrupted())
		throw Interrupted();
}


///
///move the arm to its initial position
///
void Scenario::move_to_initial(){
	initial.t = context.getTimer().elapsed() + tmDeltaAsync + SecTmReal(5.0);
	// movement will last no shorter than 5 sec
	arm->getReacPlanner().send(initial, ReacPlanner::ACTION_GLOBAL);
	// wait until the arm is ready to accept new commands, but no longer than 60 seconds
	(void)arm->getReacPlanner().waitForEnd(60000);
}


///
///write obtained dataset into a binary file
///
void Scenario::write_data (){
	
	/////////////////////////////////////////////////
	//writing the dataset into binary file
	LearningData::write_dataset (dataFileName, data, learningData.featLimits);
	/////////////////////////////////////////////////

	string stpFileName = dataFileName + ".stp";
	ofstream writeToFile (stpFileName.c_str(), ios::out | ios::binary);
	write_vector<double> (writeToFile, usedStartingPositions);
}


///
///The experiment performed in this method behaves as follows:
///The arm randomly selects any of the possible actions.
///Data are gathered and stored in a binary file for future use
///with learning machines running offline learning experiments.
///
void Scenario::run(int argc, char* argv[]) {

	//set: random seed, tmDeltaAsync; get initial config
	first_init();

	//setup and move to home position; define fingertip orientation
	setup_home();


	//start of the experiment loop
	for (iteration = 0; iteration<numSequences; iteration++) {
		
		// experiment main loops
		creator.setToDefault();
		//polyflap actor

		//create and setup polyflap object, compute its vectors
		initialize_polyflap();
cout << Real(polyflapPosition.v1) << " " << Real(polyflapPosition.v2) << " " << Real(polyflapPosition.v3) << endl;
cout << Real(positionT.v1) << " " << Real(positionT.v2) << " " << Real(positionT.v3) << endl;
cout << Real(target.pos.p.v1) << " " << Real(target.pos.p.v2) << " " << Real(target.pos.p.v3) << endl;
		//select a random action
		choose_action ();

		//compute coordinates of start position
		calculate_starting_position_coord ();
		
		//move the finger to the beginnign of experiment trajectory
		send_position(target , ReacPlanner::ACTION_GLOBAL);
		
		//create feature sequence and vector
		init_writing();	

		//compute direction and other features of trajectory
		set_up_movement();

		//move the finger along described experiment trajectory
		move_finger();

		//write sequence into dataset
		data.push_back(learningData.currentChunkSeq);

		//turn off collision detection
		set_collision_detection(false);		

		//print out information about current sequence
		print_sequence_info();

		//move the finger up to avoid path finding problems 
		move_finger_up();

		//remove polyflap object from the scene
		remove_polyflap();

		//move finger to initial position
		move_finger_home ();

		//print out end information of this iteration
		iteration_end_info();		
	
		//finish this iteration
		check_interrupted();

	}

	//move the arm to its initial position
	move_to_initial();
	
	//write obtained data into a binary file
	write_data ();


}

//------------------------------------------------------------------------------

///
///obtain a discretization of starting finger poses from a canonical set of actions
///
map<Vec3, int, compare_Vec3> Scenario::get_canonical_positions (Desc& desc) {
	//generate all possible polyflap poses
	//TODO: the following data should be obtained from an xml file

	//Polyflap Position and orientation
	/*const Vec3 startPolyflapPosition(Real(0.2), Real(0.2), Real(0.0));
	const Vec3 startPolyflapRotation(Real(0.0*REAL_PI), Real(0.0*REAL_PI), Real(0.0*REAL_PI));//Y,X,Z
	//Polyflap dimensions		
	const Vec3 polyflapDimensions(Real(0.1), Real(0.1), Real(0.1)); //w,h,l
		

// 	//vertical distance from the ground
// 	const Real over = 0.01;
	//vertical distance from the ground considering fingertip radius
	Real over = 0.002 + 0.015;
	//distance from the front/back of the polyflap
	const Real dist = 0.05;
	//distance from the side of the polyflap
	const Real side = polyflapDimensions.v1*0.6;
	//center of the polyflap
	const Real center = polyflapDimensions.v2*0.6;
	//distance from the top of the polyflap
	//const Real top = polyflapDimensions.v2* 1.2;
	const Real top = polyflapDimensions.v2 - 0.02;
	//lenght of the movement		
	const Real distance = 0.2;*/

	//initialization of arm target: the center of the polyflap
	//Vec3 positionT (0.2, 0.2, 0.001);
	Vec3 positionT (desc.startPolyflapPosition.v1, desc.startPolyflapPosition.v2, desc.pfWidth * 0.5);
	//Normal vector showing the direction of the lying part of polyflap, and it' orthogonal
	//Vec3 polyflapNormalVec = computeNormalVector(Vec3 (0.2, 0.2, 0.0),Vec3 (0.2, 0.25, 0.0));
	Vec3 standingPolyflapPosition (desc.startPolyflapPosition.v1, desc.startPolyflapPosition.v2 + desc.polyflapDimensions.v2 * 0.5, desc.startPolyflapPosition.v3);
	cout << "standingPolPos: " << standingPolyflapPosition.v1 << "," << standingPolyflapPosition.v2 << "," << standingPolyflapPosition.v3 << endl;
	Vec3 polyflapNormalVec = computeNormalVector(desc.startPolyflapPosition, standingPolyflapPosition);
	
	Vec3 polyflapOrthogonalVec = computeOrthogonalVec(polyflapNormalVec);

	map<Vec3, int, compare_Vec3> positionsT;

	vector<int> availableStartingPositions = parse_startingPositions(desc.startingPositionsConfig, startingPositionsCount);


	//int smRegionsCount = 18;
	//for (int i=1; i <= smRegionsCount; i++) {
	for (int i=0; i<availableStartingPositions.size(); i++) {
		//arm target update
		
		Vec3 pos (positionT);
		///*Scenario::*/set_coordinates_into_target(i, pos, polyflapNormalVec, polyflapOrthogonalVec, dist, side, center, top, over);
		set_coordinates_into_target(availableStartingPositions[i], pos, polyflapNormalVec, polyflapOrthogonalVec, desc.dist, desc.side, desc.center, desc.top, desc.over);
		positionsT[pos] = availableStartingPositions[i];
		
	}

	return positionsT;
}


///
///Hack to solve a collision problem (don't know if it is still there):
///Function that checks if arm hitted the polyflap while approaching it
///
bool Scenario::check_pf_position(const Actor* polyFlapActor, const Mat34& refPos) {
	return (polyFlapActor->getPose().equals(refPos, Real(0.001)));
}


///
///calculate final pose according to the given direction angle
///
void Scenario::set_movement_angle(const Real angle, golem::WorkspaceCoord& pose,const Real& distance,const Vec3& normVec,const Vec3& orthVec) {
	pose.p.v1 += Real(sin(angle/180.0*REAL_PI)*(distance*normVec.v1)); 
	pose.p.v2 += Real(sin(angle/180.0*REAL_PI)*(distance*normVec.v2)); 
	pose.p.v1 += Real(cos(angle/180.0*REAL_PI)*(distance*orthVec.v1)); 
	pose.p.v2 += Real(cos(angle/180.0*REAL_PI)*(distance*orthVec.v2)); 
	pose.p.v3 += 0.0;	
}



///
///set experiment default values
///
void Scenario::init(boost::program_options::variables_map vm) {
	numSequences = 10000;
	startingPosition = 0;
	storeLabels = false;
	
	if (vm.count("numSequences")) {
		numSequences = atoi(vm["numSequences"].as<string>().c_str());
	}

	if (vm.count("startingPosition")) {
		startingPosition = atoi(vm["startingPosition"].as<string>().c_str());
	}

	if (vm.count("storeLabels")) {
		storeLabels = true;
	}



	dataFileName = get_base_filename_from_time ();

	availableStartingPositions = parse_startingPositions(desc.startingPositionsConfig, startingPositionsCount);

	
}

//------------------------------------------------------------------------------

void PushingApplication::define_program_options_desc() {


	try {

	prgOptDesc.add_options()
		("help,h", "produce help message")
		("numSequences,S", po::value<string>(), "number of sequences")
		("startingPosition,P", po::value<string>(), "only starting position to use")
		("storeLabels,L", "store labels")
		("configFile,C", po::value<string>(), "name of the xml config file")
	;



	} catch(std::exception& e) {
		cerr << "error: " << e.what() << "\n";
	} catch(...) {
		cerr << "Exception of unknown type!\n";

	}
	
}




int PushingApplication::read_program_options(int argc, char *argv[]) {


	try {
		po::store(po::parse_command_line(argc, argv, prgOptDesc), vm);
		po::notify(vm);

		if (vm.count("help")) {
			cout << prgOptDesc << endl;
			return 1;
		}


	}catch(std::exception& e) {
		cerr << "error: " << e.what() << "\n";
	}catch(...) {
		cerr << "Exception of unknown type!\n";

	}


}



int PushingApplication::start_experiment(char *argv[]) {
	int problemOccured;
	if (vm.count("configFile")) {
		char* arr[2];
		//char* arr [] = (char *){argv[0], vm["configFile"].as<string>().c_str()};
		arr[0] = argv[0];
		arr[1] =  (char*)vm["configFile"].as<string>().c_str();
		problemOccured = Application::main(2, arr);
	} else {
		char* arr [] = {argv[0]};
		problemOccured = Application::main(1, arr);
	}
	if (problemOccured) {
		cout << endl << "Please use the option -C or --configFile for the configuration file path argument." << endl;
		cout << endl << prgOptDesc << endl;
		return 1;
	}
		
}



int PushingApplication::main(int argc, char *argv[]) {


	try {

		define_program_options_desc();

		if (read_program_options(argc, argv)) {
			return 1;
		}


		start_experiment(argv);




	} catch(std::exception& e) {
		cerr << "error: " << e.what() << "\n";
	} catch(...) {
		cerr << "Exception of unknown type!\n";
	}

	return 1;
	
}



void PushingApplication::run(int argc, char *argv[]) {


	Scenario::Desc desc;
	XMLData(desc, xmlcontext(), context());

	Scenario *pScenario = dynamic_cast<Scenario*>(scene()->createObject(desc)); // throws
	if (pScenario == NULL) {
		context()->getMessageStream()->write(Message::LEVEL_CRIT, "PushingApplication::run(): unable to cast to Scenario");
		return;
	}

	// Random number generator seed
	context()->getMessageStream()->write(Message::LEVEL_INFO, "Random number generator seed %d", context()->getRandSeed()._U32[0]);
	
	try {
		//pScenario->init(arguments);
		pScenario->init(vm);
		pScenario->run(argc, argv);
	}
	catch (const Scenario::Interrupted&) {
	}
	
	scene()->releaseObject(*pScenario);
}













}; /* namespace smlearning */
