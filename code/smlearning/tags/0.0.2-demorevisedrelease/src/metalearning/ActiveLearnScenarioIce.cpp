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

#include <metalearning/ActiveLearnScenarioIce.h>

namespace smlearning {

///
///The experiment performed in this method behaves as follows:
///The arm selects an action according to a learning progress
///associated to the RNN machine.
///

int ActiveLearnScenarioIce::run (int argc, char *argv[]) {
	//argc = argnr;
	//argv = arguments;

	Ice::ObjectPrx base1 = communicator()->stringToProxy("GolemTiny:default -p 8172");

		
	TinyPrx pTiny = TinyPrx::checkedCast(base1);
	if (!pTiny)
		throw ExTiny("Invalid proxy");
	if (argc < 2) {
		cout << "Usage: " << argv[0] << " 0/1 (0 for real arm, 1 for simulation) [nr. experiments] [starting position (1-24)]" << endl;
		throw ExTiny("Invalid number of arguments");
	}
	
	shutdownOnInterrupt();

	try {

	Ice::ObjectPrx base2 = communicator()->stringToProxy("DataPlotter:default -p 8174");
	smlearning::plotting::PlottingAppPrx plotApp = smlearning::plotting::PlottingAppPrx::checkedCast(base2);
	if (!plotApp)
		throw "Invalid proxy";
	int smregionsCount = 18;
	plotApp->init (smregionsCount, learner.SMOOTHING + learner.TIMEWINDOW);
	plotApp->resize(600,400);
	
	plotApp->show();

	
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
		pKatArm->gripperClose(5.0, thresholdData);
		
	}
	else {
		KatSimArmDesc* pArmDesc = new KatSimArmDescI; // Katana simulator
		pArm = ArmPrx::checkedCast(pTiny->createActor(ActorDescPtr(pArmDesc)));
	}
	//GenSimArmDesc* pArmDesc = new GenSimArmDescI; // 6DOF manipulator
	
	// attached a finger to the end-effector (the last joint)
	JointPrx pEffector = pArm->getJoints().back();

// 	createGripper(pEffector, pArm);
	createFinger(pEffector, pArm);

	golem::tinyice::Mat34 armGlobalPose = pArm->getGlobalPose ();
	armGlobalPose.p.v3 = 0.03;
	pArm->setGlobalPose (armGlobalPose);
	
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
	const Real maxRange = 0.4;

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
// 	//vertical distance from the ground
// 	Real over = 0.01;
	//distance from the front/back of the polyflap
	Real dist = 0.05;
	//distance from the side of the polyflap
	Real side = polyflapDimensions.v1*0.6;
	//center of the polyflap
	Real center = polyflapDimensions.v2*0.5;
	//distance from the top of the polyflap
	//const Real top = polyflapDimensions.v2* 1.2;
	//Real top = polyflapDimensions.v2 - over;
	Real top = polyflapDimensions.v2 - 0.02;
	//lenght of the movement		
	Real distance = 0.2;

		
	//Datasets in which all sequences are stored
	//Arranged in a vector associated to region indices
	//sequences and featureVectors are created
	//in every loop run
	vector<DataSet> data;
	for (int i=0; i<smregionsCount; i++) {
		DataSet currentDataset;
		data.push_back (currentDataset);
	}

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

	int numSequences = 10000;
	int startingPosition = 0;
	if (argc > 2)
		numSequences = atoi(argv[2]);
	if (argc > 3)
		startingPosition = atoi(argv[3]);
	
	//start of the experiment loop
	int e = 0;
	while (e<numSequences && !pTiny->interrupted()) {
		// Object pointer
		RigidBodyPrx pPolyflapObject;
		//polyflap object
		setupPolyflap(pTiny, pPolyflapObject, startPolyflapPosition, startPolyflapZRotation, polyflapDimensions);
		/*golem::Bounds::SeqPtr curPol = pPolyflapObject->getGlobalBoundsSeq();*/
		golem::tinyice::Mat34 referencePolyflapPos = pPolyflapObject->getGlobalPose ();


		/////////////////////////////////////////////////
		//create sequence for this loop run and initial (motor command) vector
		smlearning::Sequence seq;
		FeatureVector motorcommandVector;
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
			if ( e < smregionsCount )
				startPosition = rand() % smregionsCount + 1;
			else {
				//active selection of samples
				double neargreedyActionRand = rand() / (double) RAND_MAX;
				cout << "neargreedyRand: " << neargreedyActionRand << endl;
				if (neargreedyActionRand <= learner.neargreedyActionProb)
					startPosition = rand() % smregionsCount + 1;
				else
					assert ((startPosition = learner.chooseSMRegion () + 1) != -1);
			}
		else
			startPosition = startingPosition;
		
		setCoordinatesIntoTarget(startPosition, positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, side, center, top, over);
		cout << "Position " << startPosition-1 << endl;

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

		//initializing motorcommandVector
		golem::Polynomial4 polynom;
		polynom.create(polynomDesc);
		const Real* coefs  = polynom.getCoeffs();
			
		/////////////////////////////////////////////////
		//writing in the initial vector
		//Trajectory curve coefficients
		motorcommandVector.push_back(normalize<Real>(coefs[0], -5, 5));
		motorcommandVector.push_back(normalize<Real>(coefs[1], -5, 5));
		motorcommandVector.push_back(normalize<Real>(coefs[2], -5, 5));
		motorcommandVector.push_back(normalize<Real>(coefs[3], -5, 5));
		//initial position, normalized
		motorcommandVector.push_back(normalize(positionT.v1, /*-maxRange*/0.0, maxRange));
		motorcommandVector.push_back(normalize(positionT.v2, /*-maxRange*/0.0, maxRange));
		motorcommandVector.push_back(normalize(positionT.v3, /*-maxRange*/0.0, maxRange));
		//innitial orientation, normalized
		motorcommandVector.push_back(normalize(orientationT.v1, -REAL_PI, REAL_PI));
		motorcommandVector.push_back(normalize(orientationT.v2, -REAL_PI, REAL_PI));
		motorcommandVector.push_back(normalize(orientationT.v3, -REAL_PI, REAL_PI));
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
		motorcommandVector.push_back(Real(speed));
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
// 		motorcommandVector.push_back(normalize(end.p.v1, -maxRange, maxRange));
// 		motorcommandVector.push_back(normalize(end.p.v2, -maxRange, maxRange));
// 		motorcommandVector.push_back(normalize(end.p.v3, -maxRange, maxRange));
// 		//end orientation, normalized
// 		motorcommandVector.push_back(normalize(orientationT.v1, -REAL_PI, REAL_PI));
// 		motorcommandVector.push_back(normalize(orientationT.v2, -REAL_PI, REAL_PI));
// 		motorcommandVector.push_back(normalize(orientationT.v3, -REAL_PI, REAL_PI));
// 		/////////////////////////////////////////////////
		
		motorcommandVector.push_back(normalize(Real(horizontalAngle/180.0*REAL_PI), -REAL_PI, REAL_PI));

		/////////////////////////////////////////////////
		//writing of the initial vector into sequence
		seq.push_back(motorcommandVector);
		/////////////////////////////////////////////////

		// Current time is the same for all threads and it is the time that has elapsed since the start of the program
		Real timeBegin = pTiny->getTime();
	
		cout << "Moving on the line..." << endl;

		// move the arm with global path planning and WITHOUT collision detection - clear collision group mask
		pArm->setCollisionGroup(0x0);


		
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
					features.push_back(normalize(polyFlapPose.p.v1, /*-maxRange*/0.0, maxRange));
					features.push_back(normalize(polyFlapPose.p.v2, /*-maxRange*/0.0, maxRange));
					features.push_back(normalize(polyFlapPose.p.v3, /*-maxRange*/0.0, maxRange));
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
			data[startPosition-1].push_back(seq);
			/////////////////////////////////////////////////
			
		} //end of the if(checkPosition...) block

		if (pPolyflapObject)
			pTiny->releaseActor(pPolyflapObject);
		//initialize RNN learner with a dummy dataset
		//TODO: this is a hack!
		//string dummyDataFile = "tmp_dummy";
		if (e == 0) {
			//write_nc_file_basis (dummyDataFile, data[startPosition-1]);
			learner.build (/*dummyDataFile, */smregionsCount, seq[0].size() );
		}

		//update RNN learner with current sequence
		rnnlib::DataSequence trainseq(learner.header->inputSize, learner.header->outputSize);
		vector<int> inputShape, targetShape;
		inputShape.push_back (seq.size() - 1);
		targetShape.push_back (seq.size() - 1);
		trainseq.inputs.reshape(inputShape);
		trainseq.targetPatterns.reshape(targetShape);
		load_sequence (trainseq.inputs.data, trainseq.targetPatterns.data, seq);
		learner.update (trainseq, startPosition-1);
		vector<double> learnProgData = learner.learnProg_errorsMap[startPosition-1].first;
		vector<double> errorData = learner.learnProg_errorsMap[startPosition-1].second;		
		plotApp->updateData(startPosition-1, learnProgData, errorData);
		golem::tinyice::Mat34 predictedPfPose = getPfPoseFromOutputActivations (learner.net->outputLayer->outputActivations, motorcommandVector.size(), maxRange);
		RigidBodyPrx predictedPolyflapObject;
		setupPolyflap(pTiny, predictedPolyflapObject, predictedPfPose, polyflapDimensions);
		

		
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

		if (predictedPolyflapObject)
			pTiny->releaseActor(predictedPolyflapObject);



		// ON collision detection
		// fill collision group mask with 1s to indicate shapes with all possible group masks
		pArm->setCollisionGroup(0xFFFFFFFF);
		pArm->setGenWorkspaceState(home, ActionTypeGlobal);
		cout << "Moving home..." << endl;
		pArm->waitForEnd(60.0);
		cout << "Done" << endl;
		cout << "Iteration " << e++ << " completed!" << endl;

				
	}// end of the for-loop (experiment loop)


	/////////////////////////////////////////////////
	//writing the dataset into binary file
// 	for (int i=0; i<smregionsCount; i++)
// 		writeDownCollectedData(data[i]);
	/////////////////////////////////////////////////

	// setup initial configuration
	initial.t = pTiny->getTime() + pArm->getTimeDeltaAsync() + 5.0; // movement will last no shorter than 5 sec
	// return to the initial configuration
	printf("Moving back to the initial configuration...\n");
	pArm->setGenConfigspaceState(initial, ActionTypeGlobal);
	// wait until the arm is ready to accept new commands, but no longer than 60 seconds
	(void)pArm->waitForEnd(60.0);
	
	pTiny->releaseActor(pArm);
	pTiny->releaseActor(pGroundPlane);

	} catch (const Ice::Exception& ex) {
		std::cerr << ex << std::endl;
	} catch (const char* msg) {
		std::cerr << msg << std::endl;
	}

	
	return 0;
}

golem::tinyice::Mat34  ActiveLearnScenarioIce::getPfPoseFromOutputActivations (rnnlib::SeqBuffer<double> outputActivations, int startIndex, Real maxRange) {
	int finalActIndex = outputActivations.shape[0] - 1;
// 	cout << finalActIndex << endl;
	int outputsize = outputActivations.shape[1];
// 	cout << outputsize << endl;

// 	cout << startIndex << endl;

	golem::tinyice::Mat34 predictedPfPose;

	predictedPfPose.p.v1 = denormalize(outputActivations[finalActIndex][startIndex++], /*-maxRange*/0.0, maxRange);
	predictedPfPose.p.v2 = denormalize(outputActivations[finalActIndex][startIndex++], /*-maxRange*/0.0, maxRange);
	predictedPfPose.p.v3 = denormalize(outputActivations[finalActIndex][startIndex++], /*-maxRange*/0.0, maxRange);
	predictedPfPose.R.m11 = denormalize(outputActivations[finalActIndex][startIndex++], -1.0, 1.0);
	predictedPfPose.R.m12 = denormalize(outputActivations[finalActIndex][startIndex++], -1.0, 1.0);
	predictedPfPose.R.m13 = denormalize(outputActivations[finalActIndex][startIndex++], -1.0, 1.0);
	predictedPfPose.R.m21 = denormalize(outputActivations[finalActIndex][startIndex++], -1.0, 1.0);
	predictedPfPose.R.m22 = denormalize(outputActivations[finalActIndex][startIndex++], -1.0, 1.0);
	predictedPfPose.R.m23 = denormalize(outputActivations[finalActIndex][startIndex++], -1.0, 1.0);
	predictedPfPose.R.m31 = denormalize(outputActivations[finalActIndex][startIndex++], -1.0, 1.0);
	predictedPfPose.R.m32 = denormalize(outputActivations[finalActIndex][startIndex++], -1.0, 1.0);
	predictedPfPose.R.m33 = denormalize(outputActivations[finalActIndex][startIndex++], -1.0, 1.0);
	assert (startIndex == outputsize);
	
	return predictedPfPose;
	


}


}; /* namespace smlearning */
