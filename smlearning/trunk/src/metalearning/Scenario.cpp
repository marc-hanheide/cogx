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
#include <tools/data_handling.h>

namespace smlearning {

//------------------------------------------------------------------------------

bool XMLData(Scenario::Desc &val, XMLContext* context) {
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
	//XMLData(val.maxRange, context->getContextFirst("polyflapInteraction maxRange"));
	//minimum X value for polyflap position
	XMLData(val.maxX, context->getContextFirst("polyflapInteraction maxX"));
	//minimum Y value for polyflap position
	XMLData(val.maxY, context->getContextFirst("polyflapInteraction maxY"));
	//minimum Z value for polyflap position
	XMLData(val.maxZ, context->getContextFirst("polyflapInteraction maxZ"));
	//minimum X value for polyflap position
	XMLData(val.minX, context->getContextFirst("polyflapInteraction minX"));
	//minimum Y value for polyflap position
	XMLData(val.minY, context->getContextFirst("polyflapInteraction minY"));
	//minimum Z value for polyflap position
	XMLData(val.minZ, context->getContextFirst("polyflapInteraction minZ"));

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
	

	XMLData(val.startingPositionsConfig, context->getContextFirst("loop startingPositions"));
	
	
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
Actor* Scenario::setup_polyflap(/*Scene &scene, Vec3 position, Vec3 rotation, Vec3 dimensions, golem::Context &context*/) {
	// Creator
	Creator creator(scene);
	Actor::Desc *pActorDesc;
	Actor *polyFlapActor;
	
	// Create polyflap
	pActorDesc = creator.createSimple2FlapDesc(Real(desc.polyflapDimensions.v1*0.5), Real(desc.polyflapDimensions.v1*0.5), Real(desc.polyflapDimensions.v1*0.5), Real(pfWidth*0.5), REAL_PI_2);
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
Actor* Scenario::setup_polyflap(Scene &scene, Mat34& globalPose, Vec3 dimensions) {
	// Creator
	Creator creator(scene);
	Actor::Desc *pActorDesc;
	Actor *polyFlapActor;
	
	// Create polyflap
	pActorDesc = creator.createSimple2FlapDesc(Real(dimensions.v1*0.5), Real(dimensions.v1*0.5), Real(dimensions.v1*0.5), Real(pfWidth*0.5), REAL_PI_2);
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

	speed = floor (randomG.nextUniform (3.0, 6.0));
	//speed = 3.0;

	horizontalAngle = choose_angle(60.0, 120.0, "cont");

}

///
///set current position of the polyflap as default position for computing of the starting position
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
	//arm target update
	set_coordinates_into_target(startPosition, positionT, polyflapNormalVec, polyflapOrthogonalVec, desc.dist, desc.side, desc.center, desc.top, desc.over);
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


///
///choose and describe the start point of the experiment trajectory
///
void  Scenario::calculate_starting_position_coord(){

	//set default coordinates
	init_positionT(positionT);

	//set the variable target so that it obtains the coordinates of the start point
	prepare_target();
}

///
///select random angle (discrete or continouos)
///
Real Scenario::choose_angle(Real min, Real max, string form){
	Real res;
	if (form == "disc") {
		res = floor(randomG.nextUniform (min, max));
	}
	else if (form == "cont") {
		res = randomG.nextUniform (min, max);
	}
	else {
		res = -1.0;
	}
	return res;
}



///
///describe the experiment trajectory
///
void Scenario::set_up_movement(){

		
	// Trajectory duration calculated from a speed constant.
	//speed = floor (randomG.nextUniform (3.0, 5.0));
	// speed = 3.0;
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
	// use disc for integer values and cont for non-integer values
	//horizontalAngle = floor(randomG.nextUniform (60.0, 120.0));
	//horizontalAngle = choose_angle(60.0, 120.0, "disc");
	// horizontalAngle = choose_angle(60.0, 120.0, "cont");
				
		
	//int verticalAngle = rand() % 7;

	set_movement_angle(horizontalAngle, end, currDistance, polyflapCenterNormalVec, polyflapCenterOrthogonalVec);
	cout << "Horizontal direction angle: " << horizontalAngle << " degrees" << endl;

	//return horizontalAngle;
}

///
///storing a feature vector
///
void Scenario::add_feature_vector (FeatureVector& currentFeatureVector, LearningData::Chunk& chunk) {

	
	currentFeatureVector.push_back(normalize(chunk.effectorPose.p.v1, desc.minX, desc.maxX));
	currentFeatureVector.push_back(normalize(chunk.effectorPose.p.v2, desc.minY, desc.maxY));
	currentFeatureVector.push_back(normalize(chunk.effectorPose.p.v3, desc.minZ, desc.maxZ));
	currentFeatureVector.push_back(normalize(chunk.efRoll, -REAL_PI, REAL_PI));
	currentFeatureVector.push_back(normalize(chunk.efPitch, -REAL_PI, REAL_PI));
	currentFeatureVector.push_back(normalize(chunk.efYaw, -REAL_PI, REAL_PI));

	assert (currentFeatureVector.size() == efVectorSize);
	
	currentFeatureVector.push_back(normalize(chunk.objectPose.p.v1, desc.minX, desc.maxX));
	currentFeatureVector.push_back(normalize(chunk.objectPose.p.v2, desc.minY, desc.maxY));
	currentFeatureVector.push_back(normalize(chunk.objectPose.p.v3, desc.minZ, desc.maxZ));
	currentFeatureVector.push_back(normalize(chunk.obRoll, -REAL_PI, REAL_PI));
	currentFeatureVector.push_back(normalize(chunk.obPitch, -REAL_PI, REAL_PI));
	currentFeatureVector.push_back(normalize(chunk.obYaw, -REAL_PI, REAL_PI));

	assert (currentFeatureVector.size() == efVectorSize + pfVectorSize);
	
}

///
///storing a label (in this case polyflap status)
///
void Scenario::add_label (FeatureVector& currentFeatureVector, LearningData::Chunk& chunk) {
	Real polStateOutput = 0; //polyflap moves with the same Y angle
	Real epsilonAngle = 0.005;
	// Real pfFlipThreshold = REAL_PI / 4.0;
	
	if (learningData.currentSeq.size() > 1) {
		if (currentPfRoll < (chunk.obRoll - epsilonAngle) ) {//polyflap Y angle increases
			polStateOutput = 1;
			// if (obRoll > pfFlipThreshold)
			// 	polStateOutput = 1;
			// else
			// 	polStateOutput = 0.75;
		}
		if (currentPfRoll > (chunk.obRoll + epsilonAngle) )//polyflap Y angle decreases
			polStateOutput = -1;
		Real epsilonPfY = 0.000001;
		if (polStateOutput == 0 && currentPfY < (chunk.objectPose.p.v2 - epsilonPfY) ) // polyflap Y position increases
			polStateOutput = 0.5;
		if (polStateOutput == 0 && currentPfY > (chunk.objectPose.p.v2 + epsilonPfY) ) //polyflap Y position decreases
			polStateOutput = -0.5;
	}
	
	// 		cout << "polStateOutput: " << polStateOutput << endl;

	// if (obRoll > reachedAngle)
	// 	reachedAngle = obRoll;
	
	currentFeatureVector.push_back(polStateOutput);

}

///
///add the feature vector to the current sequence
///
void Scenario::write_feature_vector_into_current_sequence(FeatureVector& featureVector){

	if (storeLabels)
		assert (featureVector.size() == featureVectorSize + 1);
	else
		assert (featureVector.size() == featureVectorSize);
	/////////////////////////////////////////////////
	//writing of the feature vector into sequence
	learningData.currentSeq.push_back(featureVector);
	/////////////////////////////////////////////////
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
		// golem::Mat34 p = chunk.objectPose; 
		// golem::Mat34 p = chunk.effectorPose; 
		// cout << "pose: ";
		// printf("%f %f %f %f %f %f %f %f %f %f %f %f\n", p.p.v1, p.p.v2, p.p.v3, p.R._m._11, p.R._m._12, p.R._m._13, p.R._m._21, p.R._m._22, p.R._m._23, p.R._m._31, p.R._m._32, p.R._m._33);  
	
		chunk.effectorPose.R.toEuler (chunk.efRoll, chunk.efPitch, chunk.efYaw);
		chunk.objectPose.R.toEuler (chunk.obRoll, chunk.obPitch, chunk.obYaw);

		add_feature_vector (currentFeatureVector, chunk);
		if (storeLabels) add_label (currentFeatureVector, chunk);

// 		learningData.data.push_back(chunk);
// 		trialTime += SecTmReal(1.0)/universe.getRenderFrameRate();

		write_feature_vector_into_current_sequence (currentFeatureVector);

		currentPfRoll = chunk.obRoll;
		currentPfY = chunk.objectPose.p.v2;
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
	arm->getArm().lookupInp(initial, context.getTimer()->elapsed());
}


///
///describe the home position (position, where the finger starts and ends every iteration)
///
void Scenario::setup_home(){
	// setup home position
	//GenWorkspaceState home;
	home.pos = desc.homePose;
	// move the arm with global path planning and collision detection
	home.t = context.getTimer()->elapsed() + tmDeltaAsync + SecTmReal(5.0);
	// movement will last no shorter than 5 sec
	arm->getReacPlanner().send(home, ReacPlanner::ACTION_GLOBAL);
	// wait until the arm is ready to accept new commands, but no longer than 60 seconds
	(void)arm->getReacPlanner().waitForEnd(60000);
	
	
	// Define the initial pose in the Cartesian workspace
	orientationT.set(Real(-0.5*REAL_PI), Real(0.0*REAL_PI), Real(0.0*REAL_PI));
}


///
///set the lenght of experiment (number of sequences), starting position, and other default values
///
void Scenario::setup_loop(int argc, char* argv[]){
	numSequences = 10000;
	startingPosition = 0;
	storeLabels = false;
	if (argc > 2)
		numSequences = atoi(argv[2]);
	if (argc > 3)
		startingPosition = atoi(argv[3]);
	if (argc > 4)
		storeLabels = atoi(argv[4]);

	data.second = make_tuple (make_tuple((int)motorVectorSize, (int)featureVectorSize, (int)pfVectorSize, (int)efVectorSize), storeLabels, make_tuple(desc.minX, desc.minY, desc.minZ, desc.maxX, desc.maxY, desc.maxZ));

	dataFileName = get_base_filename_from_time ();

	availableStartingPositions = parse_startingPositions(desc.startingPositionsConfig, startingPositionsCount);

}


///
///try to find a path to given position, if found, move the finegr along it and wait for it to stop
///
void Scenario::send_position(golem::GenWorkspaceState position, golem::ReacPlanner::Action action){
// reachedAngle = 0.0;
		for (int t=0; t<MAX_PLANNER_TRIALS; t++) {
			if (arm->getReacPlanner().send(position, action)) {
				break;
			}
			context.getLogger()->post(Message::LEVEL_INFO, "Unable to find path to polyflap, trying again.");
		}


		context.getLogger()->post(Message::LEVEL_INFO, "Moving...");
		// wait for completion of the action (until the arm stops moving)
		arm->getReacPlanner().waitForEnd(60000);
}


///
///create feature vector and sequence
///
void Scenario::init_writing(){

	/////////////////////////////////////////////////
	//create sequence for this loop run and initial (motor command) vector
	learningData.currentSeq.clear();
	learningData.currentMotorCommandVector.clear();
	//learningData.currentFeatureVector.clear();
	/////////////////////////////////////////////////	
}


///
///write finger features to the vector
///
void Scenario::write_finger_pos_and_or(FeatureVector& featureVector, const Vec3& positionT){

	/////////////////////////////////////////////////
	//writing in the initial vector	
	//initial position, normalized
	featureVector.push_back(normalize<double>(positionT.v1, desc.minX, desc.maxX));
	featureVector.push_back(normalize<double>(positionT.v2, desc.minY, desc.maxY));
	featureVector.push_back(normalize<double>(positionT.v3, desc.minZ, desc.maxZ));
	//initial orientation, normalized
//	currentMotorCommandVector.push_back(normalize<double>(orientationT.v1, -REAL_PI, REAL_PI));
//	currentMotorCommandVector.push_back(normalize<double>(orientationT.v2, -REAL_PI, REAL_PI));
//	currentMotorCommandVector.push_back(normalize<double>(orientationT.v3, -REAL_PI, REAL_PI));
	//end pose info missing (must be added later 
	/////////////////////////////////////////////////
}


///
///write finger features to the vector
///
void Scenario::write_finger_speed_and_angle(FeatureVector& featureVector, const int speed, const Real horizontalAngle){

	/////////////////////////////////////////////////
	//writing in the initial vector
	featureVector.push_back(normalize<double>(speed, 3.0, 5.0));
	/////////////////////////////////////////////////



	/////////////////////////////////////////////////
	//writing in the initial vector
	featureVector.push_back(normalize(Real(horizontalAngle/180.0*REAL_PI), -REAL_PI, REAL_PI));
	/////////////////////////////////////////////////
}


///
///add the motor vector to the current sequence
///
void Scenario::write_motor_vector_into_current_sequence(){

	assert (learningData.currentMotorCommandVector.size() == motorVectorSize);
	/////////////////////////////////////////////////
	//writing of the initial vector into sequence
	learningData.currentSeq.push_back(learningData.currentMotorCommandVector);
	/////////////////////////////////////////////////
}


///
///initialize learning data
///
void Scenario::init_data(){
	// initialize data
	learningData.setToDefault();
	learningData.effector = effectorBounds;
	learningData.object = *object->getLocalBoundsSeq();
	learningData.obstacles = *obstacles->getGlobalBoundsSeq();
}


///
///set the end of the experiment trajectory, initialize learning data and let the finger move along the experiment trajectory
///
void Scenario::move_finger(){
	target.pos = end;
	target.t = context.getTimer()->elapsed() + tmDeltaAsync + duration;

	//arm->setCollisionBoundsGroup(0x0);
	set_collision_detection(false);	
	arm->getReacPlanner().send(target, ReacPlanner::ACTION_LOCAL);

	init_data();


		
	// wait for the movement to start, but no longer than 60 seconds
	(void)arm->getReacPlanner().waitForBegin(60000);
	context.getTimer()->sleep(tmDeltaAsync);
	bStart = true;
		
	// wait for the movement end, no longer than 60 seconds
	(void)arm->getReacPlanner().waitForEnd(60000);
	context.getTimer()->sleep(tmDeltaAsync + desc.speriod);
	bStart = false;
}


///
///write vector sequence into current dataset
///
void Scenario::write_current_sequence_into_dataset(DataSet& data){
	/////////////////////////////////////////////////
	//writing the sequence into the dataset
	data.push_back(learningData.currentSeq);
	/////////////////////////////////////////////////
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
	cout << "sequence size: " << learningData.currentSeq.size() << endl;
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

	preHome.t = context.getTimer()->elapsed() + tmDeltaAsync + SecTmReal(2.0); // i.e. the movement will last at least 2 sec

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
///describe the movement to home position
///
void Scenario::prepare_home_movement(){
	home.t = context.getTimer()->elapsed() + tmDeltaAsync + SecTmReal(3.0);
}


///
///print out desired information at the end of a sequence
///
void Scenario::iteration_end_info(){
	context.getLogger()->post(Message::LEVEL_INFO, "Done");
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
	initial.t = context.getTimer()->elapsed() + tmDeltaAsync + SecTmReal(5.0);
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
	//writedown_collected_data(data);
	write_dataset (dataFileName, data);
	/////////////////////////////////////////////////

	string stpFileName = dataFileName + ".stp";
	ofstream writeToFile (stpFileName.c_str(), ios::out | ios::binary);
	write_intvector(writeToFile, usedStartingPositions);
	
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

	//define numSequences and startingPosition
	setup_loop(argc, argv);
	
	
	
	//start of the experiment loop
	for (iteration = 0; iteration<numSequences; iteration++) {
		
		// experiment main loops
		creator.setToDefault();
		//polyflap actor

		//create and setup polyflap object, compute its vectors
		initialize_polyflap();

		//select a random action
		choose_action ();

		//compute coordinates of start position
		calculate_starting_position_coord ();
		
		//move the finger to the beginnign of experiment trajectory
		send_position(target , ReacPlanner::ACTION_GLOBAL);
		
		//create feature sequence and vector
		init_writing();	

		//write initial position and orientation of the finger
		write_finger_pos_and_or(learningData.currentMotorCommandVector, positionT);

		//write chosen speed and angle of the finger experiment trajectory
		write_finger_speed_and_angle(learningData.currentMotorCommandVector, speed, horizontalAngle);
		//add motor feature vector to the sequence
		write_motor_vector_into_current_sequence();

		//compute direction and other features of trajectory
		set_up_movement();

		//move the finger along described experiment trajectory
		move_finger();

		//write sequence into dataset
		write_current_sequence_into_dataset(data.first);

		//turn off collision detection
		set_collision_detection(false);		

		//print out information about current sequence
		print_sequence_info();

		//move the finger up to avoid path finding problems 
		move_finger_up();

		//remove polyflap object from the scene
		remove_polyflap();

		//set up needed data for home movement
		prepare_home_movement();
		
		//turn on collision detection
		set_collision_detection(true);
		
		//move the finger to home position
		send_position(home , ReacPlanner::ACTION_GLOBAL);

		//context.getLogger()->post(Message::LEVEL_INFO, "Moving home...");
		//arm->getReacPlanner().waitForEnd(60000);
		
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
