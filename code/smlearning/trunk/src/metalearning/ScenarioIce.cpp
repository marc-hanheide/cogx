/** @file ScenarioIce.cpp
 * 
 * Learning scenario where the arm moves along a straight line
 * using reactive trajectory planner and collision detection
 * simulating a pushing action on a polyflap
 * 
 * Program can be run in two modes:
 * - the first uses real Katana arm
 * - the second runs the arm simulators
 * Ice Interface
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

#include <metalearning/ScenarioIce.h>

namespace smlearning {


///
///creates a polyflap object given position and rotation angle
///
void ScenarioIce::setupPolyflap(TinyPrx &pTiny, RigidBodyPrx& pObject, golem::tinyice::Vec3 position, Real rotationZ, golem::tinyice::Vec3 dimensions) {

	// setup an object (polyflap) as a set of two boxes
	RigidBodyDesc* pObjectDesc = new RigidBodyDescI;
	const Real objectWidthY = dimensions.v1 * 0.5;
	const Real objectWidthZ = dimensions.v1 * 0.5;
	const Real objectLength = dimensions.v2 * 0.5;
	const Real objectHeight = dimensions.v3 * 0.5;
	const Real objectAngle = 0.5*REAL_PI;
	const Real objectThickness = 0.001;
	// Y-up shape
	BoxShapeDesc* pYShapeDesc = new BoxShapeDescI;
	pYShapeDesc->dimensions.v1 = objectWidthY;
	pYShapeDesc->dimensions.v2 = objectLength;
	pYShapeDesc->dimensions.v3 = objectThickness;
	pYShapeDesc->localPose.p.v1 = 0.0;
	pYShapeDesc->localPose.p.v2 = objectLength;
	pYShapeDesc->localPose.p.v3 = objectThickness;
	pYShapeDesc->density = 0.5;
	pObjectDesc->shapes.push_back(ShapeDescPtr(pYShapeDesc));
	// Y-up shape
	BoxShapeDesc* pZShapeDesc = new BoxShapeDescI;
	double objectSin = ::sin(objectAngle), objectCos = ::cos(objectAngle);
	pZShapeDesc->dimensions.v1 = objectWidthZ;
	pZShapeDesc->dimensions.v2 = objectHeight;
	pZShapeDesc->dimensions.v3 = objectThickness;
	pZShapeDesc->localPose.p.v1 = 0.0;
	pZShapeDesc->localPose.p.v2 = objectCos*objectHeight;
	pZShapeDesc->localPose.p.v3 = objectSin*objectHeight + objectThickness;
	pZShapeDesc->density = 0.5;
	rotX(pZShapeDesc->localPose.R, objectAngle);
	pObjectDesc->shapes.push_back(ShapeDescPtr(pZShapeDesc));
	// global pose
	pObjectDesc->globalPose.p.v1 = position.v1;
	pObjectDesc->globalPose.p.v2 = position.v2;
	pObjectDesc->globalPose.p.v3 = position.v3;
	rotZ(pObjectDesc->globalPose.R, rotationZ);
	// delete previous object, create a new one. Optionally only global pose can be set
	if (pObject)
		pTiny->releaseActor(pObject);
	pObject = RigidBodyPrx::checkedCast(pTiny->createActor(ActorDescPtr(pObjectDesc)));
}

///
///creates a polyflap object given pose
///
void ScenarioIce::setupPolyflap(TinyPrx &pTiny, RigidBodyPrx& pObject, golem::tinyice::Mat34 pose, golem::tinyice::Vec3 dimensions) {

	// setup an object (polyflap) as a set of two boxes
	RigidBodyDesc* pObjectDesc = new RigidBodyDescI;
	const Real objectWidthY = dimensions.v1 * 0.5;
	const Real objectWidthZ = dimensions.v1 * 0.5;
	const Real objectLength = dimensions.v2 * 0.5;
	const Real objectHeight = dimensions.v3 * 0.5;
	const Real objectAngle = 0.5*REAL_PI;
	const Real objectThickness = 0.001;
	// Y-up shape
	BoxShapeDesc* pYShapeDesc = new BoxShapeDescI;
	pYShapeDesc->dimensions.v1 = objectWidthY;
	pYShapeDesc->dimensions.v2 = objectLength;
	pYShapeDesc->dimensions.v3 = objectThickness;
	pYShapeDesc->localPose.p.v1 = 0.0;
	pYShapeDesc->localPose.p.v2 = objectLength;
	pYShapeDesc->localPose.p.v3 = objectThickness;
	pYShapeDesc->density = 0.5;
	pYShapeDesc->color.r = 0.0;
	pYShapeDesc->color.g = 0.0;
	pYShapeDesc->color.b = 255.0;
	
	pObjectDesc->shapes.push_back(ShapeDescPtr(pYShapeDesc));
	// Y-up shape
	BoxShapeDesc* pZShapeDesc = new BoxShapeDescI;
	double objectSin = ::sin(objectAngle), objectCos = ::cos(objectAngle);
	pZShapeDesc->dimensions.v1 = objectWidthZ;
	pZShapeDesc->dimensions.v2 = objectHeight;
	pZShapeDesc->dimensions.v3 = objectThickness;
	pZShapeDesc->localPose.p.v1 = 0.0;
	pZShapeDesc->localPose.p.v2 = objectCos*objectHeight;
	pZShapeDesc->localPose.p.v3 = objectSin*objectHeight + objectThickness;
	pZShapeDesc->density = 0.5;
	pZShapeDesc->color.r = 0.0;
	pZShapeDesc->color.g = 0.0;
	pZShapeDesc->color.b = 255.0;
	rotX(pZShapeDesc->localPose.R, objectAngle);
	pObjectDesc->shapes.push_back(ShapeDescPtr(pZShapeDesc));
	// global pose
	pObjectDesc->globalPose = pose;
	// delete previous object, create a new one. Optionally only global pose can be set
	if (pObject)
		pTiny->releaseActor(pObject);
	pObject = RigidBodyPrx::checkedCast(pTiny->createActor(ActorDescPtr(pObjectDesc)));
}



///
///create finger
///
void ScenarioIce::createFinger(JointPrx& pEffector, ArmPrx& pArm) {

	// get the end-effector reference pose
	golem::tinyice::Mat34 referencePose = pArm->getReferencePose();

	const double fingerLength = 0.1;
	const double fingerDiam = 0.005;
	const double fingerTipRadius = 0.015;
	BoxShapeDesc* pFingerRodShapeDesc = new BoxShapeDescI;
	pFingerRodShapeDesc->dimensions.v1 = fingerDiam/2.0;
	pFingerRodShapeDesc->dimensions.v2 = fingerLength/2.0;
	pFingerRodShapeDesc->dimensions.v3 = fingerDiam/2.0;
	pFingerRodShapeDesc->localPose = referencePose;
	pFingerRodShapeDesc->localPose.p.v2 += fingerLength/2.0;
	ShapePrx pFingerRodShape = pEffector->createShape(ShapeDescPtr(pFingerRodShapeDesc));
	SphereShapeDesc* pFingerTipShapeDesc = new SphereShapeDescI;
	pFingerTipShapeDesc->radius = fingerTipRadius;
	pFingerTipShapeDesc->localPose = referencePose;
	pFingerTipShapeDesc->localPose.p.v2 += fingerLength;
	ShapePrx pFingerTipShape = pEffector->createShape(ShapeDescPtr(pFingerTipShapeDesc));
	// change reference pose, so the end-effector pose will be further referred to the finger tip
	referencePose.p.v2 += fingerLength;
	pArm->setReferencePose(referencePose);
	
}

///
///create gripper
///
void ScenarioIce::createGripper(JointPrx& pEffector, ArmPrx& pArm) {

	// get the end-effector reference pose
	golem::tinyice::Mat34 referencePose = pArm->getReferencePose();

	const double gripperLength = 0.135;
	const double gripperDiam = 0.02;
	BoxShapeDesc* pGripperRodShapeDesc = new BoxShapeDescI;
	pGripperRodShapeDesc->dimensions.v1 = gripperDiam/2.0;
	pGripperRodShapeDesc->dimensions.v2 = gripperLength/2.0;
	pGripperRodShapeDesc->dimensions.v3 = gripperDiam/2.0;
	pGripperRodShapeDesc->localPose = referencePose;
	pGripperRodShapeDesc->localPose.p.v2 += gripperLength/2.0;
	ShapePrx pGripperRodShape = pEffector->createShape(ShapeDescPtr(pGripperRodShapeDesc));
	// change reference pose, so the end-effector pose will be further referred to the gripper tip
	referencePose.p.v2 += gripperLength;
	pArm->setReferencePose(referencePose);
	
}



///
///The experiment performed in this method behaves as follows:
///The arm randomly selects any of the possible actions.
///Data are gathered and stored in a binary file for future use
///with learning machines running offline learning experiments.
///
int ScenarioIce::run (int argc, char *argv[]) {


	Ice::ObjectPrx base = communicator()->stringToProxy("GolemTiny:default -p 8172");
		
	TinyPrx pTiny = TinyPrx::checkedCast(base);
	if (!pTiny)
		throw ExTiny("Invalid proxy");
	if (argc < 2) {
		cout << "Usage: " << argv[0] << " 0/1 (0 for real arm, 1 for simulation) [nr. experiments] [starting position (1-24)]" << endl;
		throw ExTiny("Invalid number of arguments");
	}
	
	shutdownOnInterrupt();


	// initialize random seed:
	srand ((unsigned)time(NULL) );

	//-----------------------------------------------------------------------------

	// create ground plane using plane shape
	RigidBodyDesc* pGroundPlaneDesc = new RigidBodyDescI;
	PlaneShapeDesc* pGroundPlaneShapeDesc = new PlaneShapeDescI;
	pGroundPlaneDesc->shapes.push_back(ShapeDescPtr(pGroundPlaneShapeDesc));
	RigidBodyPrx pGroundPlane = RigidBodyPrx::checkedCast(pTiny->createActor(ActorDescPtr(pGroundPlaneDesc)));

	ArmPrx pArm;
	// create arm
	printf("Creating the arm...\n");
	if (atoi(argv[1]) == 0) {
		KatArmDesc* pArmDesc = new KatArmDescI; // Katana
		pArmDesc->bGripper = true;
		pArm = ArmPrx::checkedCast(pTiny->createActor(ActorDescPtr(pArmDesc)));
		KatSensorDataSet sensorData;
		KatSensorDataSet thresholdData;
		
		KatArmPrx pKatArm = KatArmPrx::checkedCast(pArm);

		KatGripperEncoderData encoderData;
		if (pKatArm->gripperRecvSensorData(5.0, sensorData)) {
			for (KatSensorDataSet::const_iterator iter = sensorData.begin(); iter != sensorData.end(); iter++) {
				KatSensorData sensor = KatSensorData(*iter);
				sensor.value = 150;
				thresholdData.push_back (sensor);
// 				cout << "thr. index " << sensor.index <<  ": " << sensor.value << endl;
// 				cout << "read index " << (*iter).index <<  ": " << (*iter).value << endl;
			}			

		}
		pKatArm->gripperClose(10.0, thresholdData);

		
	}
	else {
		KatSimArmDesc* pArmDesc = new KatSimArmDescI; // Katana simulator
		pArm = ArmPrx::checkedCast(pTiny->createActor(ActorDescPtr(pArmDesc)));
	}
	//GenSimArmDesc* pArmDesc = new GenSimArmDescI; // 6DOF manipulator
	
	// attached a finger to the end-effector (the last joint)
	JointPrx pEffector = pArm->getJoints().back();

	
	createFinger(pEffector, pArm);

	
	// Display arm information
	//armInfo(arm);
	//sleep (1);

	// Reactive arm controller is capable to make the arm to move on almost arbirtary trajectories
	// (within velocity and acceleration limits) using planner with collision detection.
	// Trajectories are created by sequential sending trajectory waypoints to the arm
	// Waypoints can be specified in jointspace or in Cartesian workspace
	// Each waypoints has its own time stamp, and the arm controller takes care of time synchronization between
	// the arm and the program

	// Program has its own local time which is the same for all threads
	// and it is the time that has elapsed since the start of the program
	// Each arm controller has two characteristic time constants:
	// (Synchronous) Time Delta - a minimum elapsed time between two consecutive waypoints sent to the controller
	Real timeDelta = pArm->getTimeDelta();
	// Asynchronous Time Delta - a minimum elapsed time between the current time and the first waypoint sent to the controller
	// (no matter what has been sent before)
	Real timeDeltaAsync = pArm->getTimeDeltaAsync();


	
	//a number that slightly greater than the maximal reachable space of the arm
	//    - used for workspace position normalization and later as a position upper bound
	//      for random polyflap position
	const Real maxRange = 0.7;

	//minimal duration of a movement (by normal speed)
	const Real minDuration = 5.0;
	
	//Polyflap Position and orientation
	golem::tinyice::Vec3 startPolyflapPosition;
	startPolyflapPosition.v1 = 0.2;
	startPolyflapPosition.v2 = 0.2;
	startPolyflapPosition.v3 = 0.0;
	Real startPolyflapZRotation = 0.0*REAL_PI;  //Z
	//Polyflap dimensions		
	golem::tinyice::Vec3 polyflapDimensions;
	polyflapDimensions.v1 = 0.1;//w
	polyflapDimensions.v2 = 0.1;//h
	polyflapDimensions.v3 = 0.1;//l

	//vertical distance from the ground considering fingertip radius
	Real over = 0.002 + 0.015;
	//distance from the front/back of the polyflap
	Real dist = 0.05;
	//distance from the side of the polyflap
	Real side = polyflapDimensions.v1*0.6;
	//center of the polyflap
	Real center = polyflapDimensions.v2*0.5;
	//distance from the top of the polyflap
	//const Real top = polyflapDimensions.v2* 1.2;
	Real top = polyflapDimensions.v2 - over;
	//lenght of the movement		
	Real distance = 0.2;

		
	//Dataset in which all sequences are stored; sequences and featureVectors are created
	//in every loop run
	DataSet data;

// 	//getting the maximal and minimal Velocities of arm joints	
// 	const int numOfJoints = pArm->getJoints().size();
// 	// 		Real minVelocities [MAX_JOINTS];
// 	// 		Real maxVelocities [MAX_JOINTS];
// 	Real minVelocities [numOfJoints];
// 	Real maxVelocities [numOfJoints];
// 	for (int i = 0; i < numOfJoints; i++) {
// 		const IceProxy::golem::tinyice::Joint &joint = *pArm->getJoints()[i];
// 		minVelocities[i] = joint.getMin().vel;
// 	}
// 	for (int i = 0; i < numOfJoints; i++) {
// 		const Joint &joint = *pArm->getJoints()[i];
// 		maxVelocities[i] = joint.getMax().vel;
// 	}


	// get initial configuration (it is the current joint configuration)
	golem::tinyice::GenConfigspaceState initial = pArm->getGenConfigspaceState(pTiny->getTime());


	
	// Define the Home pose in the Cartesian workspace
	golem::tinyice::Vec3 positionH;
	positionH.v1 = 0.0;
	positionH.v2 = 0.1;
	positionH.v3 = 0.1;
	golem::tinyice::Vec3 orientationH;
	orientationH.v1 = -0.5*REAL_PI;
	orientationH.v2 = 0.0*REAL_PI;
	orientationH.v3 = 0.0*REAL_PI;
		
	// and set target waypoint
	golem::tinyice::GenWorkspaceState home;
	golem::GenWorkspaceState home1;
	golem::tools::fromCartesianPose(home1.pos, make(positionH), make(orientationH));
	home = make (home1);
	//home.vel.setId(); // it doesn't move

	//The movement will last at least minDuration sec
	home.t = pTiny->getTime() + timeDeltaAsync + minDuration;


	// set the initial pose of the arm, force the global movement (with planning in the entire arm workspace)
	pArm->setGenWorkspaceState(home, ActionTypeGlobal);
	// wait for completion of the action (until the arm moves to the initial pose),
	//but no longer than 60 sec.
	pArm->waitForEnd(60.0);


	// Define the initial pose in the Cartesian workspace
	golem::tinyice::Vec3 orientationT;
	orientationT.v1 = -0.5*REAL_PI;
	orientationT.v2 = 0.0*REAL_PI;
	orientationT.v3 = 0.0*REAL_PI;

	// Object pointer
	RigidBodyPrx pPolyflapObject;

	int numSequences = 10;
	int startingPosition = 0;
	if (argc > 2)
		numSequences = atoi(argv[2]);
	if (argc > 3)
		startingPosition = atoi(argv[3]);
	
	//start of the experiment loop
	for (int e=0; e<numSequences && !pTiny->interrupted(); e++) {
		//polyflap object
		setupPolyflap(pTiny, pPolyflapObject, startPolyflapPosition, startPolyflapZRotation, polyflapDimensions);
		/*golem::Bounds::SeqPtr curPol = pPolyflapObject->getGlobalBoundsSeq();*/
		golem::tinyice::Mat34 referencePolyflapPos = pPolyflapObject->getGlobalPose ();


		/////////////////////////////////////////////////
		//create sequence for this loop run and initial vector
		smlearning::Sequence seq;
		FeatureVector infoVector;
		/////////////////////////////////////////////////

		golem::tinyice::Mat34 polPosZShape = pPolyflapObject->getShapes().back()->getLocalPose();
		polPosZShape.p.v1 += referencePolyflapPos.p.v1;
		polPosZShape.p.v2 += referencePolyflapPos.p.v2;
		polPosZShape.p.v3 += referencePolyflapPos.p.v3;		
		golem::tinyice::Mat34 polPosYShape = pPolyflapObject->getShapes().front()->getLocalPose();
		polPosYShape.p.v1 += referencePolyflapPos.p.v1;
		polPosYShape.p.v2 += referencePolyflapPos.p.v2;
		polPosYShape.p.v3 += referencePolyflapPos.p.v3;
		

		golem::tinyice::Vec3 polPosZShape2D (polPosZShape.p);
		polPosZShape2D.v3 = 0.0;
		golem::tinyice::Vec3 polPosYShape2D (polPosYShape.p);
		polPosYShape2D.v3 = 0.0;
		
		//Normal vector showing the direction of the lying part of polyflap, and it' orthogonal
		golem::tinyice::Vec3 polyflapNormalVec = computeNormalVector(polPosZShape2D, polPosYShape2D);
		golem::tinyice::Vec3 polyflapOrthogonalVec = computeOrthogonalVec(polyflapNormalVec);


		//initial target of the arm: the center of the polyflap
		golem::tinyice::Vec3 positionT;
		positionT.v1 = polPosZShape.p.v1;
		positionT.v2 = polPosZShape.p.v2;
		positionT.v3 = polPosYShape.p.v3;  //considering object thickness and fingertip radius

		int startPosition;
		if (startingPosition == 0)
			startPosition = rand() % 18 + 1;
		else
			startPosition = startingPosition;
	
		setCoordinatesIntoTarget(startPosition, positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, side, center, top, over);
		cout << "Position " << startPosition << endl;

		// and set target waypoint
		golem::tinyice::GenWorkspaceState target;
		golem::GenWorkspaceState target1;
		golem::tools::fromCartesianPose(target1.pos, make(positionT), make(orientationT));
		target = make (target1);
		//target.vel.setId(); // it doesn't mov

		//The movement will last at least minDuration sec
		target.t = pTiny->getTime() + timeDeltaAsync + minDuration;

		pArm->setGenWorkspaceState(target, ActionTypeGlobal);

		// wait for completion of the action (until the arm moves to the initial pose)
		pArm->waitForEnd(60.0);
		
		// Trajectory profile can be defined by e.g. a simple 3rd degree polynomial
		golem::Polynomial4::Desc polynomDesc;
		golem::Profile::Ptr pProfile(/*Polynomial4::Desc().create()*/ polynomDesc.create());

		//initializing infoVector
		golem::Polynomial4 polynom;
		polynom.create(polynomDesc);
		const Real* coefs  = polynom.getCoeffs();
			
		/////////////////////////////////////////////////
		//writing in the initial vector
		//Trajectory curve coefficients
		infoVector.push_back(normalize<Real>(coefs[0], -5, 5));
		infoVector.push_back(normalize<Real>(coefs[1], -5, 5));
		infoVector.push_back(normalize<Real>(coefs[2], -5, 5));
		infoVector.push_back(normalize<Real>(coefs[3], -5, 5));
		//initial position, normalized
		infoVector.push_back(normalize(positionT.v1, -maxRange, maxRange));
		infoVector.push_back(normalize(positionT.v2, -maxRange, maxRange));
		infoVector.push_back(normalize(positionT.v3, -maxRange, maxRange));
		//innitial orientation, normalized
		infoVector.push_back(normalize(orientationT.v1, -REAL_PI, REAL_PI));
		infoVector.push_back(normalize(orientationT.v2, -REAL_PI, REAL_PI));
		infoVector.push_back(normalize(orientationT.v3, -REAL_PI, REAL_PI));
		//end pose info missing (must be added later 
		/////////////////////////////////////////////////


		// It consists of 70 parts
		int n = 70;
		// Trajectory duration is a multiplicity of Time Delta [sec], but with a speed coefficient
		int speed = -1 + (rand() % 3);
		// 			cout << "speed: " << speed << endl;
		// 			int speed = 0;
		Real duration = timeDelta * n* pow(2.0, -speed*2);
				
		/////////////////////////////////////////////////
		//writing in the initial vector
		infoVector.push_back(Real(speed));
		/////////////////////////////////////////////////

		// Trajectory end pose equals begin + shift along Y axis
		golem::WorkspaceCoord begin = make (target.pos), end = make(target.pos);

		//normal vector to the center of the polyflap
		golem::tinyice::Vec3 polyflapCenterNormalVec = computeNormalVector(positionT, polPosZShape.p);
		//and it's orthogonal
		golem::tinyice::Vec3 polyflapCenterOrthogonalVec = computeOrthogonalVec(polyflapCenterNormalVec);
		//cout << "centernormalvec: " << polyflapCenterNormalVec.v1 << "," << polyflapCenterNormalVec.v2 << "," << polyflapCenterNormalVec.v3 << endl;


		//the lenght of the movement
		Real currDistance = distance;
		if (speed < 0) {
			currDistance *= 5.0;
		}


		//chose random horizontal and vertical angle
		int horizontalAngle = rand() % 61 + 60;
		// 			int horizontalAngle = 90;
			
		//int verticalAngle = rand() % 7;

		setMovementAngle(horizontalAngle, end, currDistance, polyflapCenterNormalVec, polyflapCenterOrthogonalVec);
		cout << horizontalAngle << " degree horizontally" << endl;

// 		/////////////////////////////////////////////////
// 		//writing in the initial vector
// 		//add info about end position
// 		infoVector.push_back(normalize(end.p.v1, -maxRange, maxRange));
// 		infoVector.push_back(normalize(end.p.v2, -maxRange, maxRange));
// 		infoVector.push_back(normalize(end.p.v3, -maxRange, maxRange));
// 		//end orientation, normalized
// 		infoVector.push_back(normalize(orientationT.v1, -REAL_PI, REAL_PI));
// 		infoVector.push_back(normalize(orientationT.v2, -REAL_PI, REAL_PI));
// 		infoVector.push_back(normalize(orientationT.v3, -REAL_PI, REAL_PI));
// 		/////////////////////////////////////////////////
		
		infoVector.push_back(normalize(Real(horizontalAngle/180.0*REAL_PI), -REAL_PI, REAL_PI));

		/////////////////////////////////////////////////
		//writing of the initial vector into sequence
		seq.push_back(infoVector);
		/////////////////////////////////////////////////


		// Current time is the same for all threads and it is the time that has elapsed since the start of the program
		Real timeBegin = pTiny->getTime();
	
		cout << "Moving on the line..." << endl;

		// move the arm with global path planning and WITHOUT collision detection - clear collision group mask
		pArm->setCollisionGroup(0x0);


// 		Mat34 polyFlapPose = polyFlapActor->getPose();
// 		Real roll, pitch, yaw;
// 		referencePolyflapPos.R.toEuler (roll, pitch, yaw);
// 		context->getLogger()->post(Message::LEVEL_INFO, "referencePose: %1.20f, %1.20f, %1.20f, %1.20f, %1.20f, %1.20f", referencePolyflapPos.p.v1, referencePolyflapPos.p.v2, referencePolyflapPos.p.v3, roll, pitch, yaw);
// 		polyFlapPose.R.toEuler (roll, pitch, yaw);
// 		context->getLogger()->post(Message::LEVEL_INFO, "polyflapPose: %1.20f, %1.20f, %1.20f, %1.20f, %1.20f, %1.20f", polyFlapPose.p.v1, polyFlapPose.p.v2, polyFlapPose.p.v3, roll, pitch, yaw);

			
		//if the arm didn't touch the polyflap when approaching, we can proceed with the experiment
		//otherwise we skip it
		//if (checkPfPosition(pScene, polyFlapActor, referencePolyflapPosVec1, referencePolyflapPosVec2)) {
		if (checkPfPosition(pPolyflapObject, referencePolyflapPos)) {

			// Generate and send a simple straight line trajectory
			for (int i = 0; i <= n; i++) {

				// create a new target waypoint on a straight line at normalized time timeDelta * i
				golem::GenWorkspaceState target1; 
				golem::tools::waypointFromLineTrajectory(target1, *pProfile, begin, end, duration, timeDelta * i);
				target = make (target1);
				// set the target waypoint absolute time (in future)
				target.t = timeBegin + timeDeltaAsync + timeDelta * i;
		
				//send to the controller, force local movement (without planning in the entire arm workspace)
				//pArm->waitForEnd() will wait approx 'timeDelta' seconds, until a next waypoint can be sent
				pArm->setGenWorkspaceState (target, ActionTypeMove);
				if (!pArm->waitForEnd(60.0)) {
					// woops something went wrong
				}


				// arm state at a time t and finishes at some time later
				golem::tinyice::GenConfigspaceState state = pArm->getGenConfigspaceState(target.t); // last sent trajectory waypoint
			
				//to get polyflap pose information
				golem::tinyice::Mat34 polyFlapPose = pPolyflapObject->getGlobalPose();


				if ( i == i // | i == n
				     ) {
					/////////////////////////////////////////////////
					//creating current featureVector
					FeatureVector features;
					/////////////////////////////////////////////////
						
					/////////////////////////////////////////////////
					//writing in the feature vector
					// joint position and joint velocity
// 					for (int i = 0; i < numOfJoints; i++) {
// 						features.push_back(normalize(state.pos[i], -REAL_PI, REAL_PI));
// 						features.push_back(normalize(state.vel[i], minVelocities[i], maxVelocities[i]));
// 					}
					/////////////////////////////////////////////////
						
					/////////////////////////////////////////////////
					//writing in the feature vector
					//the position of the polyflap (position of both sides separately)
					features.push_back(normalize(polyFlapPose.p.v1, -maxRange, maxRange));
					features.push_back(normalize(polyFlapPose.p.v2, -maxRange, maxRange));
					features.push_back(normalize(polyFlapPose.p.v3, -maxRange, maxRange));
					features.push_back(normalize(polyFlapPose.R.m11, -1.0, 1.0));
					features.push_back(normalize(polyFlapPose.R.m12, -1.0, 1.0));
					features.push_back(normalize(polyFlapPose.R.m13, -1.0, 1.0));
					features.push_back(normalize(polyFlapPose.R.m21, -1.0, 1.0));
					features.push_back(normalize(polyFlapPose.R.m22, -1.0, 1.0));
					features.push_back(normalize(polyFlapPose.R.m23, -1.0, 1.0));
					features.push_back(normalize(polyFlapPose.R.m31, -1.0, 1.0));
					features.push_back(normalize(polyFlapPose.R.m32, -1.0, 1.0));
					features.push_back(normalize(polyFlapPose.R.m33, -1.0, 1.0));
					/////////////////////////////////////////////////
						
					/////////////////////////////////////////////////
					//writing in the feature vector
					//normal vectors of the polyflap
// 					if (pfNormVec1 != NULL) {
// 						features.push_back(normalize(pfNormVec1.v1, -1.0, 1.0));
// 						features.push_back(normalize(pfNormVec1.v2, -1.0, 1.0));
// 						features.push_back(normalize(pfNormVec1.v3, -1.0, 1.0));
// 					}
// 					if (pfNormVec2 != NULL) {
// 						features.push_back(normalize(-1.0*pfNormVec2.v1, -1.0, 1.0));
// 						features.push_back(normalize(-1.0*pfNormVec2.v2, -1.0, 1.0));
// 						features.push_back(normalize(-1.0*pfNormVec2.v3, -1.0, 1.0));
// 					}
					/////////////////////////////////////////////////


					/////////////////////////////////////////////////
					//writing the feature vector in the sequence
					seq.push_back(features);
					/////////////////////////////////////////////////
				}
	
				
				//end of the movement
			}

			// wait until the arm stops
			pTiny->sleep(timeDeltaAsync - timeDelta);
				
			/////////////////////////////////////////////////
			//writing the sequence into the dataset
			data.push_back(seq);
			/////////////////////////////////////////////////
			
		} //end of the if(checkPosition...) block


		//OFF collision detection
		pArm->setCollisionGroup(0x0);

		golem::tinyice::Vec3 positionPreH;
		positionPreH.v1 = target.pos.p.v1;
		positionPreH.v2 = target.pos.p.v2;
		positionPreH.v3 = target.pos.p.v3 + (polyflapDimensions.v2*1.1);
		// and set target waypoint
		golem::tinyice::GenWorkspaceState preHome;
		golem::GenWorkspaceState preHome1;
		golem::tools::fromCartesianPose(preHome1.pos, make(positionPreH), make(orientationH));
		preHome = make(preHome1);
		//preHome.vel.setId(); // it doesn't move

		preHome.t = pTiny->getTime() + timeDeltaAsync + 2.0; // i.e. the movement will last at least 2 sec

		// set the initial pose of the arm, force the global movement (with planning in the entire arm workspace)
		pArm->setGenWorkspaceState(preHome, ActionTypeGlobal);
		// wait for completion of the action (until the arm moves to the initial pose)
		pArm->waitForEnd(60.0);




		// ON collision detection
		// fill collision group mask with 1s to indicate shapes with all possible group masks
		pArm->setCollisionGroup(0xFFFFFFFF);
		pArm->setGenWorkspaceState(home, ActionTypeGlobal);
		cout << "Moving home..." << endl;
		pArm->waitForEnd(60.0);
		cout << "Done" << endl;
		cout << "Iteration " << e << " completed!" << endl;

				
}// end of the for-loop (experiment loop)


	/////////////////////////////////////////////////
	//writing the dataset into binary file
	writeDownCollectedData(data);
	/////////////////////////////////////////////////

	// setup initial configuration
	initial.t = pTiny->getTime() + pArm->getTimeDeltaAsync() + 5.0; // movement will last no shorter than 5 sec
	// return to the initial configuration
	printf("Moving back to the initial configuration...\n");
	pArm->setGenConfigspaceState(initial, ActionTypeGlobal);
	// wait until the arm is ready to accept new commands, but no longer than 60 seconds
	(void)pArm->waitForEnd(60.0);
	
	if (pPolyflapObject)
		pTiny->releaseActor(pPolyflapObject);
	pTiny->releaseActor(pArm);
	pTiny->releaseActor(pGroundPlane);

	return 0;
}


///
///Hack to solve a collision problem (don't know if it is still there):
///Function that checks if arm hitted the polyflap while approaching it
///
bool ScenarioIce::checkPfPosition(RigidBodyPrx& pObject, const golem::tinyice::Mat34& refPos) {
	golem::WorkspaceCoord pose = make (pObject->getGlobalPose());
	return (pose.equals(make(refPos), 0.001));
}

///
///calculate final pose according to the given direction angle
///
void ScenarioIce::setMovementAngle(const int angle, golem::WorkspaceCoord& pose,const Real& distance,const golem::tinyice::Vec3& normVec,const golem::tinyice::Vec3& orthVec) {
	pose.p.v1 += Real(sin(angle/180.0*REAL_PI)*(distance*normVec.v1)); 
	pose.p.v2 += Real(sin(angle/180.0*REAL_PI)*(distance*normVec.v2)); 
	pose.p.v1 += Real(cos(angle/180.0*REAL_PI)*(distance*orthVec.v1)); 
	pose.p.v2 += Real(cos(angle/180.0*REAL_PI)*(distance*orthVec.v2)); 
	pose.p.v3 += 0.0;	
}

///
///calculate position to direct the arm given parameters set in the learning scenario
///
void ScenarioIce::setPointCoordinates(golem::tinyice::Vec3& position, const golem::tinyice::Vec3& normalVec, const golem::tinyice::Vec3& orthogonalVec, const Real& spacing, const Real& horizontal, const Real& vertical) {
	position.v1 += (spacing*normalVec.v1); 
	position.v2 += (spacing*normalVec.v2); 
	position.v1 += (horizontal*orthogonalVec.v1); 
	position.v2 +=(horizontal*orthogonalVec.v2); 
	position.v3 += vertical; 
}

///
///calls setPointCoordinates for a discrete number of different actions
///
void ScenarioIce::setCoordinatesIntoTarget(const int startPosition, golem::tinyice::Vec3& positionT,const golem::tinyice::Vec3& polyflapNormalVec, const golem::tinyice::Vec3& polyflapOrthogonalVec,const Real& dist, const Real& side, const Real& center, const Real& top, const Real& over) {


			//set it's coordinates into target
			switch (startPosition) {
			case 1: 
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, side, over);
				cout << "Front down left (1)" << endl;
				break;

			case 2:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, Real(0.0), over);
				cout << "Front down middle (2)" << endl;
				break;

			case 3:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, -side, over);
				cout << "Front down right (3)" << endl;
				break;

			case 4:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, side, center);
				cout << "Front center left (4)" << endl;
				break;

			case 5:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, Real(0.0), center);
				cout << "Front center middle (5)" << endl;
				break;

			case 6:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, -side, center);
				cout << "Front center right (6)" << endl;
				break;

			case 7:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, side, top);
				cout << "Front up left (7)" << endl;
				break;

			case 8:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, Real(0.0), top);
				cout << "Front up middle (8)" << endl;
				break;

			case 9:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, -side, top);
				cout << "Front up right (9)" << endl;
				break;

			case 10:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, -dist, side, over);
				cout << "Back down left (10)" << endl;
				break;

			case 11:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, -dist, Real(0.0), over);
				cout << "Back down middle (11)" << endl;
				break;

			case 12:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, -dist, -side, over);
				cout << "Back down right (12)" << endl;
				break;

			case 13:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, -dist, side, center);
				cout << "Back center left (13)" << endl;
				break;

			case 14:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, -dist, Real(0.0), center);
				cout << "Back center middle (14)" << endl;
				break;

			case 15:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, -dist, -side, center);
				cout << "Back center right (15)" << endl;
				break;

			case 16:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, -dist, side, top);
				cout << "Back up left (16)" << endl;
				break;

			case 17:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, -dist, Real(0.0), top);
				cout << "Back up middle (17)" << endl;
				break;

			case 18:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, -dist, -side, top);
				cout << "Back up right (18)" << endl;
				break;

			case 19:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, Real(0.0), side, over);
				cout << "Side down left (19)" << endl;
				break;

			case 20:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, Real(0.0), -side, over);
				cout << "Side down right (20)" << endl;
				break;

			case 21:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, Real(0.0), side, center);
				cout << "Side center left (21)" << endl;
				break;

			case 22:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, Real(0.0), -side, center);
				cout << "Side center right (22)" << endl;
				break;

			case 23:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, Real(0.0), side, top);
				cout << "Side up left (23)" << endl;
				break;

			case 24:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, Real(0.0), -side, top);
				cout << "Side up right (24)" << endl;
				break;

			};

}




}; /* namespace smlearning */
