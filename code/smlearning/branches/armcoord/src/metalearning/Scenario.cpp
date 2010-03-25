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

namespace smlearning {

//------------------------------------------------------------------------------

bool XMLData(Scenario::Desc &val, XMLContext* context, bool create) {
	if (context == NULL) {
		ASSERT(false)
		return false;
	}
        // arm setup
        golem::XMLData(val.armDesc.pArmDesc, context->getContextFirst("arm"));
	
	// finger setup
	val.fingerDesc.clear();
	golem::Real baseLength = golem::KatArm::L3;
	XMLData(baseLength, context->getContextFirst("effector base_length"));
	golem::Real fingerLength = 0.135;
	XMLData(fingerLength, context->getContextFirst("effector finger_length"));
	golem::Real fingerDiam = 0.02;
	XMLData(fingerDiam, context->getContextFirst("effector finger_diameter"));
	golem::Real tipRadius = 0.015;
	XMLData(tipRadius, context->getContextFirst("effector tip_radius"));
	
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
	XMLData(val.maxRange, context->getContextFirst("polyflapInteraction maxRange"));

	//minimal duration of a movement (by normal speed)
	XMLData(val.minDuration, context->getContextFirst("polyflapInteraction minDuration"));


	Real x;
	Real y;
	Real z;
	
	//Polyflap Position and orientation
	XMLData(x, context->getContextFirst("polyflapInteraction startPolyflapPosition x"));
	XMLData(y, context->getContextFirst("polyflapInteraction startPolyflapPosition y"));
	XMLData(z, context->getContextFirst("polyflapInteraction startPolyflapPosition z"));
	val.startPolyflapPosition.set(x, y, z);

	XMLData(x, context->getContextFirst("polyflapInteraction startPolyflapRotation x"));
	XMLData(y, context->getContextFirst("polyflapInteraction startPolyflapRotation y"));
	XMLData(z, context->getContextFirst("polyflapInteraction startPolyflapRotation z"));
	val.startPolyflapRotation.set(y, x, z);

	//Polyflap dimensions		
	XMLData(x, context->getContextFirst("polyflapInteraction polyflapDimensions x"));
	XMLData(y, context->getContextFirst("polyflapInteraction polyflapDimensions y"));
	XMLData(z, context->getContextFirst("polyflapInteraction polyflapDimensions z"));
	val.polyflapDimensions.set(x, y, z);

	//vertical distance from the ground
	//const Real over = 0.01;
	//vertical distance from the ground considering fingertip radius
	XMLData(val.over, context->getContextFirst("polyflapInteraction over"));
	//distance from the front/back of the polyflap
	XMLData(val.dist, context->getContextFirst("polyflapInteraction dist"));

	Real r;

	//distance from the side of the polyflap
	XMLData(r, context->getContextFirst("polyflapInteraction side"));
	val.side = val.polyflapDimensions.v1*r;
	//center of the polyflap
	XMLData(r, context->getContextFirst("polyflapInteraction center"));
	val.center = val.polyflapDimensions.v2*r;
	//distance from the top of the polyflap
	//const Real top = polyflapDimensions.v2* 1.2;
	XMLData(r, context->getContextFirst("polyflapInteraction top"));
	val.top = val.polyflapDimensions.v2 - r;
	//lenght of the movement		
	XMLData(val.distance, context->getContextFirst("polyflapInteraction distance"));
	

	
	
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
		context.getLogger()->post(Message::LEVEL_CRIT, "Scenario::create(): invalid description");
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
Actor* Scenario::setupPolyflap(/*Scene &scene, Vec3 position, Vec3 rotation, Vec3 dimensions, golem::Context &context*/) {
	// Creator
	Creator creator(scene);
	Actor::Desc *pActorDesc;
	Actor *polyFlapActor;
	
	// Create polyflap
	pActorDesc = creator.createSimple2FlapDesc(Real(desc.polyflapDimensions.v1*0.5), Real(desc.polyflapDimensions.v1*0.5), Real(desc.polyflapDimensions.v1*0.5), Real(0.002*0.5), REAL_PI_2);
// 	pActorDesc = creator.createSimple2FlapDesc(Real(desc.polyflapDimensions.v1), Real(desc.polyflapDimensions.v1), Real(desc.polyflapDimensions.v1));
// 	pActorDesc = creator.createSimple2FlapDesc(Real(desc.polyflapDimensions.v1), Real(desc.polyflapDimensions.v1), Real(desc.polyflapDimensions.v1), Real(0.002), Real(REAL_PI_2));

	//-sets coordinates
	pActorDesc->nxActorDesc.globalPose.t.set(NxReal(desc.startPolyflapPosition.v1), NxReal(desc.startPolyflapPosition.v2), NxReal(desc.startPolyflapPosition.v3));	

	Mat34 pose;
	pose.R.fromEuler(
		desc.startPolyflapRotation.v1, 
		desc.startPolyflapRotation.v2, 
		desc.startPolyflapRotation.v3 
	);

	//-sets rotations	
	pActorDesc->nxActorDesc.globalPose.M.setRowMajor(&pose.R._m._11);	
	
	//-density
// 	pActorDesc->nxActorDesc.density = NxReal(5.0);	

	polyFlapActor = dynamic_cast<Actor*>(scene.createObject(*pActorDesc));

	return polyFlapActor;
}

///
///creates a polyflap object
///
Actor* Scenario::setupPolyflap(Scene &scene, Mat34& globalPose, Vec3 dimensions) {
	// Creator
	Creator creator(scene);
	Actor::Desc *pActorDesc;
	Actor *polyFlapActor;
	
	// Create polyflap
	pActorDesc = creator.createSimple2FlapDesc(Real(dimensions.v1*0.5), Real(dimensions.v1*0.5), Real(dimensions.v1*0.5), Real(0.002), REAL_PI_2);
// 	pActorDesc = creator.createSimple2FlapDesc(Real(dimensions.v1), Real(dimensions.v1), Real(dimensions.v1));
// 	pActorDesc = creator.createSimple2FlapDesc(Real(dimensions.v1), Real(dimensions.v1), Real(dimensions.v1), Real(0.002), Real(REAL_PI_2));

	//-sets coordinates
	pActorDesc->nxActorDesc.globalPose.t.set(NxReal(globalPose.p.v1), NxReal(globalPose.p.v2), NxReal(globalPose.p.v3));	


	//-sets rotations	
	pActorDesc->nxActorDesc.globalPose.M.setRowMajor(&globalPose.R._m._11);	
	
	//-density
// 	pActorDesc->nxActorDesc.density = NxReal(5.0);	

	polyFlapActor = dynamic_cast<Actor*>(scene.createObject(*pActorDesc));

	return polyFlapActor;
}


void Scenario::initializePolyflap(){

	object = setupPolyflap(/*scene, desc.startPolyflapPosition, desc.startPolyflapRotation, desc.polyflapDimensions, context*/);
	golem::Bounds::SeqPtr curPol = object->getGlobalBoundsSeq();

	object->getPose().R.toEuler (currentPfRoll, currentPfPitch, currentPfYaw);

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






void  Scenario::initializeMovement(const SecTmReal tmDeltaAsync, int startingPosition){

		//initialization of arm target: the center of the polyflap
		//Vec3 positionT(Real(polyflapPosition.v1), Real(polyflapPosition.v2), Real(polyflapPosition.v3));
		positionT.set(Real(polyflapPosition.v1), Real(polyflapPosition.v2), Real(polyflapPosition.v3));


	        //int startPosition;

		if (startingPosition == 0)
			startPosition = floor(randomG.nextUniform (1.0, 18.0));
		else
			startPosition = startingPosition;


		//arm target update
		setCoordinatesIntoTarget(startPosition, positionT, polyflapNormalVec, polyflapOrthogonalVec, desc.dist, desc.side, desc.center, desc.top, desc.over);
		cout << "Position " << startPosition-1 << endl;

		// and set target waypoint
		//golem::GenWorkspaceState target;
		fromCartesianPose(target.pos, positionT, orientationT);
		target.vel.setId(); // it doesn't mov

		target.t = context.getTimer()->elapsed() + tmDeltaAsync + desc.minDuration; // i.e. the movement will last at least 5 sec

		//tuple<golem::GenWorkspaceState, Vec3, int> t = make_tuple(target, positionT, startPosition);
		//return t;
		//return startPosition;
}





void Scenario::setUpMovement(){

		
		// Trajectory duration calculated from a speed constant.
		speed = floor (randomG.nextUniform (3.0, 5.0));
		cout << "speed: " << speed << endl;
		
		duration = speed;


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


		//the lenght of the movement
		Real currDistance = desc.distance;

		//chose random horizontal and vertical angle
		int horizontalAngle = floor(randomG.nextUniform (60.0, 120.0));
		
		//int verticalAngle = rand() % 7;

		setMovementAngle(horizontalAngle, end, currDistance, polyflapCenterNormalVec, polyflapCenterOrthogonalVec);
		cout << "Horizontal direction angle: " << horizontalAngle << " degrees" << endl;

		//return horizontalAngle;


}




void Scenario::postprocess(SecTmReal elapsedTime) {
	if (bStart) {
		CriticalSectionWrapper csw(cs);
		if (object == NULL) {
			return;
		}

		FeatureVector currentFeatureVector;
		LearningData::Chunk chunk;
		chunk.timeStamp = trialTime;
		arm->getArm().lookupInp(chunk.armState, context.getTimer()->elapsed());
		chunk.effectorPose = effector->getPose();
		chunk.objectPose = object->getPose();
// 		golem::Mat34 p = chunk.objectPose; 
// 		cout << "pose: ";
// 		printf("%f %f %f %f %f %f %f %f %f %f %f %f\n", p.p.v1, p.p.v2, p.p.v3, p.R._m._11, p.R._m._12, p.R._m._13, p.R._m._21, p.R._m._22, p.R._m._23, p.R._m._31, p.R._m._32, p.R._m._33);  

		Real efRoll, efPitch, efYaw;
		chunk.effectorPose.R.toEuler (efRoll, efPitch, efYaw);
		Real obRoll, obPitch, obYaw;
		chunk.objectPose.R.toEuler (obRoll, obPitch, obYaw);

		Real polStateOutput = 0; //polyflap moves with the same Y angle
		Real epsilonAngle = 0.005;
		if (learningData.currentSeq.size() > 1) {
			if (currentPfRoll < (obRoll - epsilonAngle) ) //polyflap Y angle increases
				polStateOutput = 1;
			if (currentPfRoll > (obRoll + epsilonAngle) ) //polyflap Y angle decreases
				polStateOutput = -1;
			
			Real epsilonPfY = 0.000001;
			if (polStateOutput == 0 && currentPfY < (chunk.objectPose.p.v2 - epsilonPfY) ) // polyflap Y position increases
				polStateOutput = 0.5;
			if (polStateOutput == 0 && currentPfY > (chunk.objectPose.p.v2 + epsilonPfY) ) //polyflap Y position decreases
				polStateOutput = -0.5;
		}
		
// 		cout << "polStateOutput: " << polStateOutput << endl;
		currentPfRoll = obRoll;
		currentPfY = chunk.objectPose.p.v2;

		if (obRoll > reachedAngle) {
			reachedAngle = obRoll;
		}

		
// 		learningData.data.push_back(chunk);
// 		trialTime += SecTmReal(1.0)/universe.getRenderFrameRate();
		/////////////////////////////////////////////////
		//storing the feature vector

		currentFeatureVector.push_back(normalize(chunk.effectorPose.p.v1, 0.0, desc.maxRange));
		currentFeatureVector.push_back(normalize(chunk.effectorPose.p.v2, 0.0, desc.maxRange));
		currentFeatureVector.push_back(normalize(chunk.effectorPose.p.v3, 0.0, desc.maxRange));
		currentFeatureVector.push_back(normalize(efRoll, -REAL_PI, REAL_PI));
		currentFeatureVector.push_back(normalize(efPitch, -REAL_PI, REAL_PI));
		currentFeatureVector.push_back(normalize(efYaw, -REAL_PI, REAL_PI));

		currentFeatureVector.push_back(normalize(chunk.objectPose.p.v1, 0.0, desc.maxRange));
		currentFeatureVector.push_back(normalize(chunk.objectPose.p.v2, 0.0, desc.maxRange));
		currentFeatureVector.push_back(normalize(chunk.objectPose.p.v3, 0.0, desc.maxRange));
		currentFeatureVector.push_back(normalize(obRoll, -REAL_PI, REAL_PI));
		currentFeatureVector.push_back(normalize(obPitch, -REAL_PI, REAL_PI));
		currentFeatureVector.push_back(normalize(obYaw, -REAL_PI, REAL_PI));

		currentFeatureVector.push_back(polStateOutput);
		
		learningData.currentSeq.push_back(currentFeatureVector);
	
	
	}
}

















///
///The experiment performed in this method behaves as follows:
///The arm randomly selects any of the possible actions.
///Data are gathered and stored in a binary file for future use
///with learning machines running offline learning experiments.
///
void Scenario::run(int argc, char* argv[]) {

// 	// initialize random seed:
	randomG.setRandSeed (context.getRandSeed());

	const SecTmReal tmDeltaAsync = arm->getReacPlanner().getTimeDeltaAsync();

	// get initial configuration (it is the current joint configuration)
	golem::GenConfigspaceState initial;
	arm->getArm().lookupInp(initial, context.getTimer()->elapsed());

	
	// setup home position
	GenWorkspaceState home;
	home.pos = desc.homePose;
	// move the arm with global path planning and collision detection
	home.t = context.getTimer()->elapsed() + tmDeltaAsync + SecTmReal(5.0);
	// movement will last no shorter than 5 sec
	arm->getReacPlanner().send(home, ReacPlanner::ACTION_GLOBAL);
	// wait until the arm is ready to accept new commands, but no longer than 60 seconds
	(void)arm->getReacPlanner().waitForEnd(60000);
	
	
	// Define the initial pose in the Cartesian workspace
	//Vec3 orientationT(Real(-0.5*REAL_PI), Real(0.0*REAL_PI), Real(0.0*REAL_PI));
	orientationT.set(Real(-0.5*REAL_PI), Real(0.0*REAL_PI), Real(0.0*REAL_PI));

	numSequences = 10000;
	int startingPosition = 0;
	if (argc > 2)
		numSequences = atoi(argv[2]);
	if (argc > 3)
		startingPosition = atoi(argv[3]);
	
	
	//start of the experiment loop
	for (iteration = 0; iteration<numSequences; iteration++) {
		
		// experiment main loops
		creator.setToDefault();
		//polyflap actor


		initializePolyflap();
		
		initializeMovement(tmDeltaAsync, startingPosition);


		reachedAngle = 0.0;
		for (int t=0; t<MAX_PLANNER_TRIALS; t++) {
			if (arm->getReacPlanner().send(target , ReacPlanner::ACTION_GLOBAL)) {
				break;
			}
			context.getLogger()->post(Message::LEVEL_INFO, "Unable to find path to polyflap, trying again.");
		}

		// wait for completion of the action (until the arm moves to the initial pose)
		arm->getReacPlanner().waitForEnd(60000);

			/////////////////////////////////////////////////
			//create sequence for this loop run and initial (motor command) vector
			learningData.currentSeq.clear();
			learningData.currentMotorCommandVector.clear();
			/////////////////////////////////////////////////	

			/////////////////////////////////////////////////
			//writing in the initial vector	
			//initial position, normalized
			learningData.currentMotorCommandVector.push_back(normalize<double>(positionT.v1, 0.0, desc.maxRange));
			learningData.currentMotorCommandVector.push_back(normalize<double>(positionT.v2, 0.0, desc.maxRange));
			learningData.currentMotorCommandVector.push_back(normalize<double>(positionT.v3, 0.0, desc.maxRange));
			//initial orientation, normalized
//	 		currentMotorCommandVector.push_back(normalize<double>(orientationT.v1, -REAL_PI, REAL_PI));
//	 		currentMotorCommandVector.push_back(normalize<double>(orientationT.v2, -REAL_PI, REAL_PI));
//	 		currentMotorCommandVector.push_back(normalize<double>(orientationT.v3, -REAL_PI, REAL_PI));
			//end pose info missing (must be added later 
			/////////////////////////////////////////////////


		setUpMovement();



			/////////////////////////////////////////////////
			//writing in the initial vector
			learningData.currentMotorCommandVector.push_back(normalize<double>(speed, 3.0, 5.0));
			/////////////////////////////////////////////////




			/////////////////////////////////////////////////
			//writing in the initial vector
			learningData.currentMotorCommandVector.push_back(normalize(Real(horizontalAngle/180.0*REAL_PI), -REAL_PI, REAL_PI));
			/////////////////////////////////////////////////

			/////////////////////////////////////////////////
			//writing of the initial vector into sequence
			LearningData::Chunk chunk;
			learningData.currentSeq.push_back(learningData.currentMotorCommandVector);
			/////////////////////////////////////////////////

	
		target.pos = end;
		target.t = context.getTimer()->elapsed() + tmDeltaAsync + duration;
	
		arm->setCollisionBoundsGroup(0x0);
		arm->getReacPlanner().send(target, ReacPlanner::ACTION_LOCAL);

		// initialize data
		learningData.setToDefault();
		learningData.effector = effectorBounds;
		learningData.object = *object->getLocalBoundsSeq();
		learningData.obstacles = *obstacles->getGlobalBoundsSeq();


		
		// wait for the movement to start, but no longer than 60 seconds
		(void)arm->getReacPlanner().waitForBegin(60000);
		context.getTimer()->sleep(tmDeltaAsync);
		bStart = true;
			
		// wait for the movement end, no longer than 60 seconds
		(void)arm->getReacPlanner().waitForEnd(60000);
		context.getTimer()->sleep(tmDeltaAsync + desc.speriod);
		bStart = false;

		Real polState = -1; //polyflap was smoothly moved
		if (reachedAngle > 0.1) { //polyflap was tilted more than threshold
			polState = 0;
		}
		if (reachedAngle > 1.5) {//polyflap was flipped
			polState = 1;
		}

		Sequence::iterator n;
		for (n=learningData.currentSeq.begin()+1; n!=learningData.currentSeq.end();n++)
			n->at(n->size()-1) = polState;
				
			/////////////////////////////////////////////////
			//writing the sequence into the dataset
			data.push_back(learningData.currentSeq);
			/////////////////////////////////////////////////

		
		//off collision detection
		arm->setCollisionBoundsGroup(0x0);

			
		cout << "sequence size: " << learningData.currentSeq.size() << endl;

		Vec3 positionPreH(target.pos.p.v1, target.pos.p.v2, target.pos.p.v3 += (desc.polyflapDimensions.v2*1.1));
		// and set target waypoint
		golem::GenWorkspaceState preHome;
		preHome.pos.p = positionPreH;
		preHome.pos.R = home.pos.R;
		preHome.vel.setId(); // it doesn't move

		preHome.t = context.getTimer()->elapsed() + tmDeltaAsync + SecTmReal(2.0); // i.e. the movement will last at least 2 sec

		// set the initial pose of the arm, force the global movement (with planning in the entire arm workspace)
		arm->getReacPlanner().send(preHome, ReacPlanner::ACTION_GLOBAL);
		// wait for completion of the action (until the arm moves to the initial pose)
		arm->getReacPlanner().waitForEnd();

		// remove object
		scene.releaseObject(*object);
		object = NULL;
// 		scene.releaseObject(*predictedPolyflapObject);
		
		home.t = context.getTimer()->elapsed() + tmDeltaAsync + SecTmReal(3.0);

// 		// ON collision detection
		arm->setCollisionBoundsGroup(0xFFFFFFFF);

		for (int t=0; t<MAX_PLANNER_TRIALS; t++) {
			if (arm->getReacPlanner().send(home, ReacPlanner::ACTION_GLOBAL)) {
				break;
			}
			
			context.getLogger()->post(Message::LEVEL_INFO, "Unable to find path home, trying again.");
		}





		context.getLogger()->post(Message::LEVEL_INFO, "Moving home...");
		arm->getReacPlanner().waitForEnd(60000);

		context.getLogger()->post(Message::LEVEL_INFO, "Done");

		cout << "Iteration " << iteration << " completed!" << endl;

		if (universe.interrupted())
			throw Interrupted();
	}
	initial.t = context.getTimer()->elapsed() + tmDeltaAsync + SecTmReal(5.0);
	// movement will last no shorter than 5 sec
	arm->getReacPlanner().send(initial, ReacPlanner::ACTION_GLOBAL);
	// wait until the arm is ready to accept new commands, but no longer than 60 seconds
	(void)arm->getReacPlanner().waitForEnd(60000);
	
		/////////////////////////////////////////////////
		//writing the dataset into binary file
		writeDownCollectedData(data);
		/////////////////////////////////////////////////
	
}

//------------------------------------------------------------------------------

///
///Hack to solve a collision problem (don't know if it is still there):
///Function that checks if arm hitted the polyflap while approaching it
///
bool Scenario::checkPfPosition(const Actor* polyFlapActor, const Mat34& refPos) {
	return (polyFlapActor->getPose().equals(refPos, Real(0.001)));
}

///
///calculate final pose according to the given direction angle
///
void Scenario::setMovementAngle(const int angle, golem::WorkspaceCoord& pose,const Real& distance,const Vec3& normVec,const Vec3& orthVec) {
	pose.p.v1 += Real(sin(angle/180.0*REAL_PI)*(distance*normVec.v1)); 
	pose.p.v2 += Real(sin(angle/180.0*REAL_PI)*(distance*normVec.v2)); 
	pose.p.v1 += Real(cos(angle/180.0*REAL_PI)*(distance*orthVec.v1)); 
	pose.p.v2 += Real(cos(angle/180.0*REAL_PI)*(distance*orthVec.v2)); 
	pose.p.v3 += 0.0;	
}

///
///calculate position to direct the arm given parameters set in the learning scenario
///
void Scenario::setPointCoordinates(Vec3& position, const Vec3& normalVec, const Vec3& orthogonalVec, const Real& spacing, const Real& horizontal, const Real& vertical) {
	position.v1 += (spacing*normalVec.v1); 
	position.v2 += (spacing*normalVec.v2); 
	position.v1 += (horizontal*orthogonalVec.v1); 
	position.v2 +=(horizontal*orthogonalVec.v2); 
	position.v3 += vertical; 
}

///
///calls setPointCoordinates for a discrete canonical number of different actions
///setCoordinatesIntoTarget(startPosition, positionT, polyflapNormalVec, polyflapOrthogonalVec, desc.dist, desc.side, desc.center, desc.top, desc.over);
void Scenario::setCoordinatesIntoTarget(const int startPosition, Vec3& positionT,const Vec3& polyflapNormalVec, const Vec3& polyflapOrthogonalVec,const Real& dist, const Real& side, const Real& center, const Real& top, const Real& over) {


	//set it's coordinates into target
	switch (startPosition) {
	case 1: 
		setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, side, over);
		//context->getLogger()->post(Message::LEVEL_INFO, "Front down left (1)");
		break;

	case 2:
		setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, Real(0.0), over);
		//context->getLogger()->post(Message::LEVEL_INFO, "Front down middle (2)");
		break;

	case 3:
		setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, -side, over);
		//context->getLogger()->post(Message::LEVEL_INFO, "Front down right (3)");
		break;

	case 4:
		setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, side, center);
		//context->getLogger()->post(Message::LEVEL_INFO, "Front center left (4)");
		break;

	case 5:
		setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, Real(0.0), center);
		//context->getLogger()->post(Message::LEVEL_INFO, "Front center middle (5)");
		break;

	case 6:
		setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, -side, center);
		//context->getLogger()->post(Message::LEVEL_INFO, "Front center right (6)");
		break;

	case 7:
		setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, side, top);
		//context->getLogger()->post(Message::LEVEL_INFO, "Front up left (7)");
		break;

	case 8:
		setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, Real(0.0), top);
		//context->getLogger()->post(Message::LEVEL_INFO, "Front up middle (8)");
		break;

	case 9:
		setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, -side, top);
		//context->getLogger()->post(Message::LEVEL_INFO, "Front up right (9)");
		break;

	case 10:
		setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, -dist, side, over);
		//context->getLogger()->post(Message::LEVEL_INFO, "Back down left (10)");
		break;

	case 11:
		setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, -dist, Real(0.0), over);
		//context->getLogger()->post(Message::LEVEL_INFO, "Back down middle (11)");
		break;

	case 12:
		setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, -dist, -side, over);
		//context->getLogger()->post(Message::LEVEL_INFO, "Back down right (12)");
		break;

	case 13:
		setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, -dist, side, center);
		//context->getLogger()->post(Message::LEVEL_INFO, "Back center left (13)");
		break;

	case 14:
		setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, -dist, Real(0.0), center);
		//context->getLogger()->post(Message::LEVEL_INFO, "Back center middle (14)");
		break;

	case 15:
		setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, -dist, -side, center);
		//context->getLogger()->post(Message::LEVEL_INFO, "Back center right (15)");
		break;

	case 16:
		setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, -dist, side, top);
		//context->getLogger()->post(Message::LEVEL_INFO, "Back up left (16)");
		break;

	case 17:
		setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, -dist, Real(0.0), top);
		//context->getLogger()->post(Message::LEVEL_INFO, "Back up middle (17)");
		break;

	case 18:
		setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, -dist, -side, top);
		//context->getLogger()->post(Message::LEVEL_INFO, "Back up right (18)");
		break;

	case 19:
		setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, Real(0.0), side, over);
		//context->getLogger()->post(Message::LEVEL_INFO, "Side down left (19)");
		break;

	case 20:
		setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, Real(0.0), -side, over);
		//context->getLogger()->post(Message::LEVEL_INFO, "Side down right (20)");
		break;

	case 21:
		setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, Real(0.0), side, center);
		//context->getLogger()->post(Message::LEVEL_INFO, "Side center left (21)");
		break;

	case 22:
		setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, Real(0.0), -side, center);
		//context->getLogger()->post(Message::LEVEL_INFO, "Side center right (22)");
		break;

	case 23:
		setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, Real(0.0), side, top);
		//context->getLogger()->post(Message::LEVEL_INFO, "Side up left (23)");
		break;

	case 24:
		setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, Real(0.0), -side, top);
		//context->getLogger()->post(Message::LEVEL_INFO, "Side up right (24)");
		break;

	};

}

//------------------------------------------------------------------------------

void PushingApplication::run(int argc, char *argv[]) {

	Scenario::Desc desc;
	XMLData(desc, xmlcontext());

	Scenario *pScenario = dynamic_cast<Scenario*>(scene()->createObject(desc)); // throws
	if (pScenario == NULL) {
		context()->getLogger()->post(Message::LEVEL_CRIT, "PushingApplication::run(): unable to cast to Scenario");
		return;
	}

	// Random number generator seed
	context()->getLogger()->post(Message::LEVEL_INFO, "Random number generator seed %d", context()->getRandSeed()._U32[0]);
	
	try {
		pScenario->run(argc, argv);
	}
	catch (const Scenario::Interrupted&) {
	}
	
	scene()->releaseObject(*pScenario);
}




}; /* namespace smlearning */
