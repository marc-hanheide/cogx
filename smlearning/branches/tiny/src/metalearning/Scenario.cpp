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

///
///creates object in the scene
///
void Scenario::setupSimulatedObjects(Scene &scene, golem::Context &context) {
	{
		CriticalSectionWrapper csw(scene.getUniverse().getCSPhysX());
		
		//set physical parameters of simulation
		NxMaterial* defaultMaterial = scene.getNxScene()->getMaterialFromIndex(0);
		defaultMaterial->setRestitution((NxReal)0.05);
		defaultMaterial->setStaticFriction((NxReal)0.4);
		defaultMaterial->setDynamicFriction((NxReal)0.2);
	}

	// Creator
	Creator creator(scene);
	Actor::Desc *pActorDesc;
	
	// Create ground plane.
	pActorDesc = creator.createGroundPlaneDesc();
	scene.createObject(*pActorDesc);
}




///
///creates a polyflap object
///
Actor* Scenario::setupPolyflap(Scene &scene, Vec3 position, Vec3 rotation, Vec3 dimensions, golem::Context &context) {
	// Creator
	Creator creator(scene);
	Actor::Desc *pActorDesc;
	Actor *polyFlapActor;
	
	// Create polyflap
	pActorDesc = creator.createSimple2FlapDesc(Real(dimensions.v1*0.5), Real(dimensions.v1*0.5), Real(dimensions.v1*0.5), Real(0.002), REAL_PI_2);
// 	pActorDesc = creator.createSimple2FlapDesc(Real(dimensions.v1), Real(dimensions.v1), Real(dimensions.v1));
// 	pActorDesc = creator.createSimple2FlapDesc(Real(dimensions.v1), Real(dimensions.v1), Real(dimensions.v1), Real(0.002), Real(REAL_PI_2));

	//-sets coordinates
	pActorDesc->nxActorDesc.globalPose.t.set(NxReal(position.v1), NxReal(position.v2), NxReal(position.v3));	

	Mat34 pose;
	pose.R.fromEuler(
		rotation.v1, 
		rotation.v2, 
		rotation.v3 
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


void trn(Vec3 v [], U32 n, const Mat34 &pose) {
	while (n--)
		pose.multiply(v[n], v[n]);
}


///
///create finger
///
void Scenario::createFinger(std::vector<Bounds::Desc::Ptr> &bounds, Mat34 &referencePose, const Mat34 &pose, MemoryStream &buffer) {
	// mace characteristic dimensions
	const Real length = Real(0.1);
	const Real begin = Real(0.01);
	const Real end = Real(0.01);

	// reference pose at the end (on Y-axis)
	referencePose.setId();
	referencePose.p.v2 += length;
	referencePose.multiply(pose, referencePose);
	
	// objects data
	Vec3 finger[8];
	BoundingConvexMesh::Desc desc;
	desc.verticesStrideBytes = sizeof(Vec3);
	desc.bCook = true;

	// create hilt (along Y-axis)
	finger[0].set(-begin, Real(0.0), -begin);
	finger[1].set(+begin, Real(0.0), -begin);
	finger[2].set(-begin, Real(0.0), +begin);
	finger[3].set(+begin, Real(0.0), +begin);
	finger[4].set(-end, length, -end);
	finger[5].set(+end, length, -end);
	finger[6].set(-end, length, +end);
	finger[7].set(+end, length, +end);
	trn(finger, 8, pose);
	desc.numOfVertices = 8;
	buffer.write(finger, 8);
	desc.vertices = buffer.get().back();
	bounds.push_back(desc.clone());
}



///
///Add new bounds to the Joint Actor.
///
void Scenario::addBounds(Actor* pActor, std::vector<const Bounds*> &boundsSeq, const std::vector<Bounds::Desc::Ptr> &boundsDescSeq) {
	ASSERT(pActor)
	boundsSeq.clear();
	for (std::vector<Bounds::Desc::Ptr>::const_iterator i = boundsDescSeq.begin(); i != boundsDescSeq.end(); i++)
		boundsSeq.push_back(pActor->createBounds(*i));
}


///
///The experiment performed in this method behaves as follows:
///The arm randomly selects any of the possible actions.
///Data are gathered and stored in a binary file for future use
///with learning machines running offline learning experiments.
///
bool Scenario::runSimulatedOfflineExperiment (int argc, char *argv[], int nSeq, int startPos) {
// 	argc = argnr;
// 	argv = arguments;
	numSequences = nSeq;
	startingPosition = startPos;

// 	if (!thread.start (this)) {
// 		cout << "Unable to launch thread!" << endl;
// 		return 0;
// 	}
// 	run();
// }


// void Scenario::run () {
	// Determine configuration file name
	std::string cfg;
	//if (argc == 1) {
		// default configuration file name
	cfg.assign(argv[0] );
		size_t pos = cfg.rfind(".exe"); // Windows only
		if (pos != std::string::npos) cfg.erase(pos);
		cfg.append(".xml");
	//}
	//else
		//cfg.assign(argv[1]);

	// Create XML parser and load configuration file
	XMLParser::Desc parserDesc;
	XMLParser::Ptr parser = parserDesc.create();
	if (!parser->load(FileReadStream(cfg.c_str()))) {
		printf("unable to load configuration file: %s\n", cfg.c_str());
		printf("%s <configuration_file>\n", argv[0]);
		return false;
	}

	// Find program XML root context
	XMLContext* xmlContext = parser->getContextRoot()->getContextFirst("golem");
	if (xmlContext == NULL) {
		printf("unknown configuration file: %s\n", cfg.c_str());
		return false;
	}

	// Create program context
	golem::Context::Desc contextDesc;
	setupContext(contextDesc, xmlContext);
	golem::Context::Ptr context = contextDesc.create();
	if (context == NULL) {
		printf("unable to create program context");
		return false;
	}

	printf("Use the arrow keys to move the camera.\n");
	printf("Use the mouse to rotate the camera.\n");
	printf("Press p to pause simulations.\n");
	printf("Press pgup/pgdn/space to switch between simulations.\n");
	printf("Press v to show Actors reference frames.\n");
	printf("Use z, x, c to change randering mode.\n");
	printf("Use F1-F12 to display program specific debug information:\n");
	printf("\tF1 to display/hide the current destination pose.\n");
	printf("\tF2 to display/hide the current trajectory.\n");
	printf("\tF3 to display/hide the goal/desired pose.\n");
	printf("\tF4 to display/hide the global waypoint graph nodes.\n");
	printf("\tF5 to display/hide the global waypoint path.\n");
	printf("\tF6 to display/hide the local waypoint graph nodes.\n");
	printf("\tF7 to display/hide the local waypoint path.\n");
	printf("\tF8 to display/hide the optimised waypoint path.\n");
	printf("Press esc to exit.\n");
	
// 	// Stream all messages to std::cerr
// 	Streamer streamer(context->getLogger(), std::cout);
// 	// Throw exception if the message level at least LEVEL_CRIT but only from the current thread
// 	context->getLogger()->setExFilter(MessageFilter::Ptr(new ThreadFilter<StdMsg>(StdMsg::LEVEL_CRIT)));
	// Do not display LEVEL_DEBUG messages (only with level at least LEVEL_INFO)
	//context->getLogger()->setMsgFilter(MessageFilter::Ptr(new LevelFilter<StdMsg>(StdMsg::LEVEL_ERR)));

	// Random number generator seed
	context->getLogger()->post(Message::LEVEL_INFO, "Random number generator seed %d", context->getRandSeed()._U32[0]);

	//-----------------------------------------------------------------------------

	// Create Universe
	Universe::Desc universeDesc;
	setupUniverse(universeDesc, xmlContext->getContextFirst("universe"), *context);
	universeDesc.name = "Golem (Pushing)";
	universeDesc.argc = argc;
	universeDesc.argv = argv;
	Universe::Ptr pUniverse = universeDesc.create(*context);

	cout << "pasamos por aca" << endl;
	
	// Create scene
	Scene::Desc sceneDesc;
	setupScene(sceneDesc, xmlContext->getContextFirst("scene"), *context);
	sceneDesc.name = "Robotic arm controller demo";
	Scene *pScene = pUniverse->createScene(sceneDesc);
	
	// Determine arm type
	std::string armType;
	if (!XMLData(armType, xmlContext->getContextFirst("arm type"))) {
		context->getLogger()->post(Message::LEVEL_CRIT, "Unspecified arm type");
		return false;
	}
	
	// Setup PhysReacPlanner controller description
	PhysReacPlanner::Desc physReacPlannerDesc;
	if (!armType.compare("kat_serial_arm")) {
		KatSerialArm::Desc *pDesc = new KatSerialArm::Desc();
		physReacPlannerDesc.pArmDesc.reset(pDesc);
		XMLData(pDesc->cfgPath, xmlContext->getContextFirst("arm kat_serial_arm path"));
		XMLData(pDesc->serialDesc.commPort, xmlContext->getContextFirst("arm kat_serial_arm comm_port"));
	}
	else if (!armType.compare("kat_sim_arm")) {
		KatSimArm::Desc *pDesc = new KatSimArm::Desc();
		physReacPlannerDesc.pArmDesc.reset(pDesc);
	}
	else if (!armType.compare("gen_sim_arm")) {
		GenSimArm::Desc *pDesc = new GenSimArm::Desc();
		physReacPlannerDesc.pArmDesc.reset(pDesc);
	}


	
	else {
		context->getLogger()->post(Message::LEVEL_CRIT, "Unknown arm type");
		return false;
	}

	// Create PhysReacPlanner
	context->getLogger()->post(Message::LEVEL_INFO, "Initialising reactive planner...");
	PhysReacPlanner *pPhysReacPlanner = dynamic_cast<PhysReacPlanner*>(pScene->createObject(physReacPlannerDesc));
	if (pPhysReacPlanner == NULL) {
		context->getLogger()->post(Message::LEVEL_CRIT, "Unable to create ReacPlanner");
		return false;
	}

	// some useful pointers
	ReacPlanner &reacPlanner = pPhysReacPlanner->getReacPlanner();
	Planner &planner = pPhysReacPlanner->getPlanner();
	Arm &arm = pPhysReacPlanner->getArm();

	//if (!armType.compare("kat_sim_arm")) {
		setupSimulatedObjects(*pScene, *context);
		// Create bounds to be attached to the end-effector (the last joint) 
		const U32 jointIndex = (U32)arm.getJoints().size() - 1;
		std::vector<Bounds::Desc::Ptr> boundsDescSeq;
		MemoryWriteStream buffer;
		Mat34 referencePose;
		createFinger(boundsDescSeq, referencePose, arm.getReferencePose(), buffer);
		// set new arm reference pose
		arm.setReferencePose(referencePose);
		// Modify shape of the joint by adding new bounds to the Actor representing the end-effector.
		std::vector<const Bounds*> boundsSeq;
		addBounds(pPhysReacPlanner->getJointActors().back(), boundsSeq, boundsDescSeq);
		pPhysReacPlanner->getPlanner().getHeuristic()->syncArmBoundsDesc(); // sync new arm bounds
		//}

/*		
// 		Mat34 p = pRobot->getFinger()->getFingerActor().getBounds()->front()->getPose();
		Real roll, pitch, yaw;
// 		p.R.toEuler (roll,pitch,yaw);
// 		context->getLogger()->post(Message::LEVEL_INFO, "Getting finger bounds pose...");
// 		context->getLogger()->post(Message::LEVEL_INFO, "%f, %f, %f, %f, %f, %f", p.p.v1, p.p.v2, p.p.v3, roll, pitch, yaw);

		Mat34 p;
		//arm.setReferencePose (p);
		//arm.setReferencePose (arm.getGlobalPose());
		p = arm.getReferencePose ();
		p.R.toEuler (roll, pitch, yaw);
		
		context->getLogger()->post(Message::LEVEL_INFO, "Getting arm reference pose...");
		context->getLogger()->post(Message::LEVEL_INFO, "%f, %f, %f, %f, %f, %f", p.p.v1, p.p.v2, p.p.v3, roll, pitch, yaw);

		p = arm.getGlobalPose ();
		p.R.toEuler (roll, pitch, yaw);
		
		context->getLogger()->post(Message::LEVEL_INFO, "Getting arm global pose...");
		context->getLogger()->post(Message::LEVEL_INFO, "%f, %f, %f, %f, %f, %f", p.p.v1, p.p.v2, p.p.v3, roll, pitch, yaw);
*/
		
	// Display arm information
	armInfo(arm);
	//sleep (1);

	// Big Bang!
	context->getLogger()->post(Message::LEVEL_INFO, "Launching Universe...");
	pUniverse->launch();


	// Reactive arm controller is capable to make the arm to move on almost arbirtary trajectories
	// (within velocity and acceleration limits) using planner with collision detection
	// Trajectories are created by sequential sending trajectory waypoints to the arm
	// Waypoints can be specified in jointspace or in Cartesian workspace
	// Each waypoints has its own time stamp, and the arm controller takes care of time synchronization between
	// the arm and the program

	// Program has its own local time which is the same for all threads
	// and it is the time that has elapsed since the start of the program
	// Each arm controller has two characteristic time constants:
	// (Synchronous) Time Delta - a minimum elapsed time between two consecutive waypoints sent to the controller
	SecTmReal timeDelta = reacPlanner.getTimeDelta();
	// Asynchronous Time Delta - a minimum elapsed time between the current time and the first waypoint sent to the controller
	// (no matter what has been sent before)
	SecTmReal timeDeltaAsync = reacPlanner.getTimeDeltaAsync();


	
	//a number that slightly greater then the maximal reachable space of the arm
	//    - used for workspace position normalization and later as a position upper bound
	//      for random polyflap position
	const Real maxRange = 0.7;

	//minimal duration of a movement (by normal speed)
	const SecTmReal minDuration = SecTmReal(5.0);
	
	//Polyflap Position and orientation
	const Vec3 startPolyflapPosition(Real(0.2), Real(0.2), Real(0.0));
	const Vec3 startPolyflapRotation(Real(-0.0*REAL_PI), Real(-0.0*REAL_PI), Real(0.0*REAL_PI));//Y,X,Z
	//Polyflap dimensions		
	const Vec3 polyflapDimensions(Real(0.1), Real(0.1), Real(0.1)); //w,h,l
		

	//vertical distance from the ground
	const Real over = 0.01;
	//distance from the front/back of the polyflap
	const Real dist = 0.05;
	//distance from the side of the polyflap
	const Real side = polyflapDimensions.v1*0.6;
	//center of the poolyflop
	const Real center = polyflapDimensions.v2*0.6;
	//distance from the top of the polyflap
	//const Real top = polyflapDimensions.v2* 1.2;
	const Real top = polyflapDimensions.v2 - over;
	//lenght of the movement		
	const Real distance = 0.2;

		
	//Dataset in which all sequences are stored; sequences and featureVectors are created
	//in every loop run
	DataSet data;

	//getting the maximal and minimal Velocities of arm joints	
	const U32 numOfJoints = (U32)arm.getJoints().size();
	// 		Real minVelocities [MAX_JOINTS];
	// 		Real maxVelocities [MAX_JOINTS];
	Real minVelocities [numOfJoints];
	Real maxVelocities [numOfJoints];
	for (U32 i = 0; i < numOfJoints; i++) {
		const Joint &joint = *arm.getJoints()[i];
		minVelocities[i] = joint.getMin().vel;
	}
	for (U32 i = 0; i < numOfJoints; i++) {
		const Joint &joint = *arm.getJoints()[i];
		maxVelocities[i] = joint.getMax().vel;
	}




	
	// Define the Home pose in the Cartesian workspace
	Vec3 positionH(Real(0.1), Real(0.1), Real(0.1));
	Vec3 orientationH(Real(-0.5*REAL_PI), Real(0.0*REAL_PI), Real(0.0*REAL_PI));
		
	// and set target waypoint
	golem::GenWorkspaceState home;
	fromCartesianPose(home.pos, positionH, orientationH);
	home.vel.setId(); // it doesn't move
		
	home.t = context->getTimer()->elapsed() + timeDeltaAsync + minDuration; // i.e. the movement will last at least 5 sec


	// set the initial pose of the arm, force the global movement (with planning in the entire arm workspace)
	reacPlanner.send(home, ReacPlanner::ACTION_GLOBAL);
	// wait for completion of the action (until the arm moves to the initial pose)
	reacPlanner.waitForEnd();



	// Define the initial pose in the Cartesian workspace
	Vec3 orientationT(Real(-0.5*REAL_PI), Real(0.0*REAL_PI), Real(0.0*REAL_PI));



	//start of the experiment loop
	for (int e=0; e<numSequences; e++) {
		//polyflap actor
		Actor *polyFlapActor = setupPolyflap(*pScene, startPolyflapPosition, startPolyflapRotation, polyflapDimensions, *context);
		golem::Bounds::SeqPtr curPol = polyFlapActor->getGlobalBoundsSeq();
		Mat34 referencePolyflapPos = polyFlapActor->getPose ();


		/////////////////////////////////////////////////
		//create sequence for this loop run and initial vector
		smlearning::Sequence seq;
		FeatureVector infoVector;
		/////////////////////////////////////////////////

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

		Vec3 polyflapPosition(curPolPos1.p.v1, curPolPos1.p.v2, curPolPos2.p.v3);

		//Normal vector showing the direction of the lying part of polyflap, and it' orthogonal
		Vec3 polyflapNormalVec =
			computeNormalVector(
					    Vec3 (curPolPos1.p.v1, curPolPos1.p.v2, Real(0.0)),
					    Vec3 (curPolPos2.p.v1, curPolPos2.p.v2, Real(0.0))
					    );
			
		Vec3 polyflapOrthogonalVec = computeOrthogonalVec(polyflapNormalVec);	


		//initial target of the arm: the center of the polyflap
		Vec3 positionT(Real(polyflapPosition.v1), Real(polyflapPosition.v2), Real(polyflapPosition.v3));


		//chose random point int the vicinity of the polyflap
		srand(context->getRandSeed()._U32[0]  + e);
		int startPosition;
		if (startingPosition == 0)
			startPosition = rand() % 18 + 1;
		else
			startPosition = startingPosition;
		//			int startPosition = 1;
	
		setCoordinatesIntoTarget(startPosition, positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, side, center, top, over);
		context->getLogger()->post(Message::LEVEL_INFO, "Position %i", startPosition);

		// and set target waypoint
		golem::GenWorkspaceState target;
		fromCartesianPose(target.pos, positionT, orientationT);
		target.vel.setId(); // it doesn't mov

		//target.t = context->getTimer()->elapsed() + timeDeltaAsync + SecTmReal(5.0); // i.e. the movement will last at least 5 sec
		target.t = context->getTimer()->elapsed() + timeDeltaAsync + minDuration; // i.e. the movement will last at least 5 sec

		for (int t=0; t<MAX_PLANNER_TRIALS; t++) {
			if (reacPlanner.send(target , ReacPlanner::ACTION_GLOBAL)) {
				break;
			}
			context->getLogger()->post(Message::LEVEL_INFO, "Unable to find path to polyflap, trying again.");
		}

		// wait for completion of the action (until the arm moves to the initial pose)
		reacPlanner.waitForEnd();
		
		// Trajectory profile can be defined by e.g. a simple 3rd degree polynomial
		Polynomial4::Desc polynomDesc;
		Profile::Ptr pProfile(/*Polynomial4::Desc().create()*/ polynomDesc.create());

		//initializing infoVector
		Polynomial4 polynom;
		polynom.create(polynomDesc);
		const Real* coefs  = polynom.getCoeffs();
			
		/////////////////////////////////////////////////
		//writing in the initial vector
		//Trajectory curve coefficients
		infoVector.push_back(normalize<double>(coefs[0], -5, 5));
		infoVector.push_back(normalize<double>(coefs[1], -5, 5));
		infoVector.push_back(normalize<double>(coefs[2], -5, 5));
		infoVector.push_back(normalize<double>(coefs[3], -5, 5));
		//initial position, normalized
		infoVector.push_back(normalize<double>(positionT.v1, -maxRange, maxRange));
		infoVector.push_back(normalize<double>(positionT.v2, -maxRange, maxRange));
		infoVector.push_back(normalize<double>(positionT.v3, -maxRange, maxRange));
		//innitial orientation, normalized
		infoVector.push_back(normalize<double>(orientationT.v1, -REAL_PI, REAL_PI));
		infoVector.push_back(normalize<double>(orientationT.v2, -REAL_PI, REAL_PI));
		infoVector.push_back(normalize<double>(orientationT.v3, -REAL_PI, REAL_PI));
		//end pose info missing (must be added later 
		/////////////////////////////////////////////////


		// It consists of 70 parts
		U32 n = 70;
		// Trajectory duration is a multiplicity of Time Delta [sec], but with a speed coefficient
		int speed = -1 + (rand() % 3);
		// 			cout << "speed: " << speed << endl;
		// 			int speed = 0;
		SecTmReal duration = timeDelta * n*(Math::pow(Real(2.0), Real(-speed)*2));
				
		/////////////////////////////////////////////////
		//writing in the initial vector
		infoVector.push_back(Real(speed));
		/////////////////////////////////////////////////

		// Trajectory end pose equals begin + shift along Y axis
		WorkspaceCoord begin = target.pos, end = target.pos;

		//normal vector to the center of the polyflap
		Vec3 polyflapCenterNormalVec =
			computeNormalVector(
					    Vec3 (positionT.v1, positionT.v2, positionT.v3),
					    Vec3 (polyflapPosition.v1, polyflapPosition.v2, polyflapDimensions.v2*0.5)
					    );
		//and it's orthogonal
		Vec3 polyflapCenterOrthogonalVec = computeOrthogonalVec(polyflapCenterNormalVec);
		context->getLogger()->post(Message::LEVEL_INFO, "centernormalvec: %f, %1f, %f", polyflapCenterNormalVec.v1, polyflapCenterNormalVec.v2, polyflapCenterNormalVec.v3);


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
		context->getLogger()->post(Message::LEVEL_INFO, "%d degree horizontaly", horizontalAngle);

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
		SecTmReal timeBegin = context->getTimer()->elapsed();
		
		context->getLogger()->post(Message::LEVEL_INFO, "Moving on the line...");
			

		// ON/OFF collision detection
		planner.getHeuristic()->setCollisionDetection(false);


// 		Mat34 polyFlapPose = polyFlapActor->getPose();
// 		Real roll, pitch, yaw;
// 		referencePolyflapPos.R.toEuler (roll, pitch, yaw);
// 		context->getLogger()->post(Message::LEVEL_INFO, "referencePose: %1.20f, %1.20f, %1.20f, %1.20f, %1.20f, %1.20f", referencePolyflapPos.p.v1, referencePolyflapPos.p.v2, referencePolyflapPos.p.v3, roll, pitch, yaw);
// 		polyFlapPose.R.toEuler (roll, pitch, yaw);
// 		context->getLogger()->post(Message::LEVEL_INFO, "polyflapPose: %1.20f, %1.20f, %1.20f, %1.20f, %1.20f, %1.20f", polyFlapPose.p.v1, polyFlapPose.p.v2, polyFlapPose.p.v3, roll, pitch, yaw);

			
		//if the arm didn't touch the polyflap when approaching, we can proceed with the experiment
		//otherwise we skip it
		//if (checkPfPosition(pScene, polyFlapActor, referencePolyflapPosVec1, referencePolyflapPosVec2)) {
		if (checkPfPosition(polyFlapActor, referencePolyflapPos)) {

			// Generate and send a simple straight line trajectory
			for (U32 i = 0; i <= n; i++) {

				// create a new target waypoint on a straight line at normalized time timeDelta * i
				waypointFromLineTrajectory(target, *pProfile, begin, end, duration, timeDelta * i);
				// set the target waypoint absolute time (in future)
				target.t = timeBegin + timeDeltaAsync + timeDelta * i;
		
				// send to the controller, force local movement (without planning in the entire arm workspace)
				// reacPlanner.wait() will wait approx 'timeDelta' seconds, until a next waypoint can be sent
				// reacPlanner.wait() can be used only if signal synchronization is enabled (see 'setupPlanner()' above)
				if (!reacPlanner.send(target, ReacPlanner::ACTION_MOVE) || !reacPlanner.waitForEnd()) {
					// woops something went wrong
				}


				// arm state at a time t and finishes at some time later
				// 					golem::JointState state;
				golem::GenConfigspaceState state;
				arm.lookupInp(state, target.t); // last sent trajectory waypoint
			
				//to get polyflap pose information
				Mat34 polyFlapPose = polyFlapActor->getPose();

// 				Real roll, pitch, yaw;
// 				polyFlapPose.R.toEuler (roll, pitch, yaw);
// 				context->getLogger()->post(Message::LEVEL_INFO, "iteration %d: polyflapPose: %1.20f, %1.20f, %1.20f, %1.20f, %1.20f, %1.20f", i, polyFlapPose.p.v1, polyFlapPose.p.v2, polyFlapPose.p.v3, roll, pitch, yaw);
// 				context->getLogger()->post(Message::LEVEL_INFO, "iteration %d: polyflap pos.: %f, %f, %f", i, polyFlapPose.p.v1, polyFlapPose.p.v2, polyFlapPose.p.v3);
// 				context->getLogger()->post(Message::LEVEL_INFO, "iteration %d: polyflap Rot matrix: %f, %f, %f, %f, %f, %f, %f, %f, %f", i, polyFlapPose.R._m._11, polyFlapPose.R._m._12, polyFlapPose.R._m._13, polyFlapPose.R._m._21, polyFlapPose.R._m._22, polyFlapPose.R._m._23, polyFlapPose.R._m._31, polyFlapPose.R._m._32, polyFlapPose.R._m._33);

				//getting the first component of the polyflap
				golem::Bounds::SeqPtr set = polyFlapActor->getGlobalBoundsSeq();
				const BoundingBox* pBox1 = dynamic_cast<const BoundingBox*>(set->front().get()); // obj_ptr::get()
				//and the vector, which represents the first normal vector of the polyflap
				Vec3 pfNormVec1;
				if (pBox1 != NULL) {
					pfNormVec1 = pBox1->getNormals()[2]; // nr2 is the one we need
				}

				//getting the second component of the polyflap
				const BoundingBox* pBox2 = dynamic_cast<const BoundingBox*>(set->back().get()); // obj_ptr::get()
				//and the vector, which represents the second normal vector of the polyflap
				Vec3 pfNormVec2;
				if (pBox2 != NULL) {
					pfNormVec2 = pBox2->getNormals()[2]; //here it is also nr2, but this one must be multiplied by -1 before storing!!
				}

				if ( i == i // | i == n
				     ) {
					/////////////////////////////////////////////////
					//creating current featureVector
					FeatureVector features;
					/////////////////////////////////////////////////
						
					/////////////////////////////////////////////////
					//writing in the feature vector
					// joint position and joint velocity
					for (U32 i = 0; i < numOfJoints; i++) {
						features.push_back(normalize(state.pos[i], -REAL_PI, REAL_PI));
						features.push_back(normalize(state.vel[i], minVelocities[i], maxVelocities[i]));
					}
					/////////////////////////////////////////////////
						
					/////////////////////////////////////////////////
					//writing in the feature vector
					//the position of the polyflap (position of both sides separately)
					features.push_back(normalize(polyFlapPose.p.v1, -maxRange, maxRange));
					features.push_back(normalize(polyFlapPose.p.v2, -maxRange, maxRange));
					features.push_back(normalize(polyFlapPose.p.v3, -maxRange, maxRange));
// 					features.push_back(normalize(roll, -REAL_PI, REAL_PI));
// 					features.push_back(normalize(pitch, -REAL_PI, REAL_PI));
// 					features.push_back(normalize(yaw, -REAL_PI, REAL_PI));
					features.push_back(normalize(polyFlapPose.R._m._11, -1.0, 1.0));
					features.push_back(normalize(polyFlapPose.R._m._12, -1.0, 1.0));
					features.push_back(normalize(polyFlapPose.R._m._13, -1.0, 1.0));
					features.push_back(normalize(polyFlapPose.R._m._21, -1.0, 1.0));
					features.push_back(normalize(polyFlapPose.R._m._22, -1.0, 1.0));
					features.push_back(normalize(polyFlapPose.R._m._23, -1.0, 1.0));
					features.push_back(normalize(polyFlapPose.R._m._31, -1.0, 1.0));
					features.push_back(normalize(polyFlapPose.R._m._32, -1.0, 1.0));
					features.push_back(normalize(polyFlapPose.R._m._33, -1.0, 1.0));
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
			context->getTimer()->sleep(timeDeltaAsync - timeDelta);
				
			/////////////////////////////////////////////////
			//writing the sequence into the dataset
			data.push_back(seq);
			/////////////////////////////////////////////////
			
		} //end of the if(checkPosition...) block


		// ON/OFF collision detection
		planner.getHeuristic()->setCollisionDetection(false);

	


		Vec3 positionPreH(target.pos.p.v1, target.pos.p.v2, target.pos.p.v3 += (polyflapDimensions.v2*1.1));
		// and set target waypoint
		golem::GenWorkspaceState preHome;
		fromCartesianPose(preHome.pos, positionPreH, orientationH);
		preHome.vel.setId(); // it doesn't move

		preHome.t = context->getTimer()->elapsed() + timeDeltaAsync + SecTmReal(2.0); // i.e. the movement will last at least 2 sec

		// set the initial pose of the arm, force the global movement (with planning in the entire arm workspace)
		reacPlanner.send(preHome, ReacPlanner::ACTION_GLOBAL);
		// wait for completion of the action (until the arm moves to the initial pose)
		reacPlanner.waitForEnd();




		// ON/OFF collision detection
		planner.getHeuristic()->setCollisionDetection(true);

		for (int t=0; t<MAX_PLANNER_TRIALS; t++) {
			if (reacPlanner.send(home, ReacPlanner::ACTION_GLOBAL)) {
				break;
			}
			
			context->getLogger()->post(Message::LEVEL_INFO, "Unable to find path home, trying again.");
		}





		context->getLogger()->post(Message::LEVEL_INFO, "Moving home...");
		reacPlanner.waitForEnd();

		context->getLogger()->post(Message::LEVEL_INFO, "Done");

		context->getLogger()->post(Message::LEVEL_INFO, "trying to delete polyflap");

		// 			context->getTimer()->sleep(1);
		pScene->releaseObject(*polyFlapActor);
		// wait a bit before new actor is created to avoid simulation crash
		// 			context->getTimer()->sleep(3);
		context->getLogger()->post(Message::LEVEL_INFO, "deleting succeded");
		context->getLogger()->post(Message::LEVEL_INFO, "Iteration %d completed!", e);

				
	}// end of the for-loop (experiment loop)


	/////////////////////////////////////////////////
	//writing the dataset into binary file
	writeDownCollectedData(data);
	/////////////////////////////////////////////////
	/*return true;*/
}

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
///calls setPointCoordinates for a discrete number of different actions
///
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




}; /* namespace smlearning */
