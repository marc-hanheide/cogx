/** @file ActiveLearnScenario.cpp
 * 
 * Program demonstrates conditional distribution learning.
 * 
 * Program can be run in two modes:
 * - the first uses real Katana arm
 * - the second runs the Katana arm simulator
 * 
 * @author	Sergio Roa (DFKI)
 * @author	Marek Kopicki (The University Of Birmingham)
 * @author	Jan Hanzelka (DFKI)
 *
 * @version 1.0
 *
 */

//------------------------------------------------------------------------------

#include <metalearning/ActiveLearnScenario.h>
#include <Golem/Renderer.h>
#include <Golem/Creator.h>
#include <algorithm>

// using namespace push;
using namespace golem;

namespace smlearning {


//------------------------------------------------------------------------------

bool XMLData(ActiveLearnScenario::Desc &val, XMLContext* context, bool create) {
	if (context == NULL) {
		ASSERT(false)
		return false;
	}

	// Determine arm type
	std::string armType = "kat_sim_arm";
	XMLData(armType, context->getContextFirst("arm type"));
	
	// Setup controller description
	if (!armType.compare("kat_serial_arm")) {
		golem::KatSerialArm::Desc *pDesc = new golem::KatSerialArm::Desc();
		val.armDesc.pArmDesc.reset(pDesc);
		XMLData(pDesc->cfgPath, context->getContextFirst("arm kat_serial_arm path"));
		XMLData(pDesc->serialDesc.commPort, context->getContextFirst("arm kat_serial_arm comm_port"));
	}
	else if (!armType.compare("kat_sim_arm")) {
		golem::KatSimArm::Desc *pDesc = new golem::KatSimArm::Desc();
		val.armDesc.pArmDesc.reset(pDesc);
	}
	else if (!armType.compare("gen_sim_arm")) {
		golem::GenSimArm::Desc *pDesc = new golem::GenSimArm::Desc();
		val.armDesc.pArmDesc.reset(pDesc);
	}
	XMLDataEuler(val.armDesc.pArmDesc->globalPose, context->getContextFirst("arm pose"));
	
	// finger setup
	val.fingerDesc.clear();
	golem::Real baseLength = golem::KatArm::L3;
	XMLData(baseLength, context->getContextFirst("effector base_length"));
	golem::Real fingerLength = 0.135;
	XMLData(fingerLength, context->getContextFirst("effector finger_length"));
	golem::Real fingerDiam = 0.02;
// 	XMLData(fingerDiam, context->getContextFirst("effector finger_diameter"));
	golem::Real tipRadius = 0.015;
	XMLData(tipRadius, context->getContextFirst("effector tip_radius"));
	
	golem::BoundingBox::Desc* pFingerRodShapeDesc = new golem::BoundingBox::Desc;
	pFingerRodShapeDesc->dimensions.set(fingerDiam/2.0, fingerLength/2.0, fingerDiam/2.0);
	pFingerRodShapeDesc->pose.p.v2 += baseLength + fingerLength/2.0;
	pFingerRodShapeDesc->group = val.effectorGroup;
	val.fingerDesc.push_back(golem::Bounds::Desc::Ptr(pFingerRodShapeDesc));
// 	golem::BoundingSphere::Desc* pFingerTipShapeDesc = new golem::BoundingSphere::Desc;
// 	pFingerTipShapeDesc->radius = tipRadius;
// 	pFingerTipShapeDesc->pose.p.v2 += golem::Real(baseLength + fingerLength);
// 	pFingerTipShapeDesc->group = val.effectorGroup;
// 	val.fingerDesc.push_back(golem::Bounds::Desc::Ptr(pFingerTipShapeDesc));
	
	// end-effector reference pose
	val.referencePose.setId();
	val.referencePose.p.v2 += golem::Real(baseLength + fingerLength);
	
// 	// trials
// 	val.trials.clear();
// 	XMLData(val.trials, context->getContextFirst("experiment"), "trial");

	return true;
}

//------------------------------------------------------------------------------

bool LearningData::load(const Stream &stream) {
//	if (stream.read(bArmState) != 1 || stream.read(bEffectorPose) != 1 || stream.read(bObjectPose) || stream.read(bFtsData) || stream.read(bImageIndex) || stream.read(bEffector) || stream.read(bObject) || stream.read(bObstacles))
//		false;
	effector.clear();
	if (stream.read(effector, effector.begin()) != 1)
		false;
	object.clear();
	if (stream.read(object, object.begin()) != 1)
		false;
	obstacles.clear();
	if (stream.read(obstacles, obstacles.begin()) != 1)
		false;
	data.clear();
	if (stream.read(data, data.begin()) != 1)
		false;

	return true;
}

bool LearningData::store(Stream &stream) const {
//	if (stream.write(bArmState) != 1 || stream.write(bEffectorPose) != 1 || stream.write(bObjectPose) || stream.write(bFtsData) || stream.write(bImageIndex) || stream.write(bEffector) || stream.write(bObject) || stream.write(bObstacles))
//		false;
	if (stream.write(effector.begin(), effector.end()) != 1)
		false;
	if (stream.write(object.begin(), object.end()) != 1)
		false;
	if (stream.write(obstacles.begin(), obstacles.end()) != 1)
		false;
	if (stream.write(data.begin(), data.end()) != 1)
		false;

	return true;
}

//------------------------------------------------------------------------------

void ActiveLearnScenario::render () {
	CriticalSectionWrapper csw (cs);
	if (currentPredictedObjSeq.size() == 0)
		return;

	for (int i=0; i<currentPredictedObjSeq.size(); i++) {
		golem::BoundsRenderer boundsRenderer;
		Mat34 currentPose = currentPredictedObjSeq[i];
		
		boundsRenderer.setMat(currentPose);
		boundsRenderer.setWireColour (RGBA::BLUE);
		boundsRenderer.renderWire (objectLocalBounds->begin(), objectLocalBounds->end());
	}
}


//------------------------------------------------------------------------------

ActiveLearnScenario::ActiveLearnScenario(golem::Scene &scene) : Object(scene), creator(scene) {
	arm = NULL;
	object = NULL;
	obstacles = NULL;
	bStart = bStop = bRec = false;
}

bool ActiveLearnScenario::create(const ActiveLearnScenario::Desc& desc) {
	if (!desc.isValid()) {
		context.getLogger()->post(Message::LEVEL_CRIT, "ActiveLearnScenario::create(): invalid description");
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
	armPose.p.v2 = 0.03;
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

void ActiveLearnScenario::release() {
	if (arm != NULL)
		scene.releaseObject(*arm);
	if (obstacles != NULL)
		scene.releaseObject(*obstacles);
}

void ActiveLearnScenario::postprocess(SecTmReal elapsedTime) {
	if (bStart) {
// 		bRec = true;
// 		trialTime = SEC_TM_REAL_ZERO;
// 		cout << "starting postprocess" << endl;
// 	}
// 	if (bRec) {
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
	
		learningData.data.push_back(chunk);
// 		trialTime += SecTmReal(1.0)/universe.getRenderFrameRate();
		/////////////////////////////////////////////////
		//storing the feature vector

		currentFeatureVector.push_back(normalize(chunk.objectPose.p.v1, 0.0, maxRange));
		currentFeatureVector.push_back(normalize(chunk.objectPose.p.v2, 0.0, maxRange));
		currentFeatureVector.push_back(normalize(chunk.objectPose.p.v3, 0.0, maxRange));
		currentFeatureVector.push_back(normalize(chunk.objectPose.R._m._11, -1.0, 1.0));
		currentFeatureVector.push_back(normalize(chunk.objectPose.R._m._12, -1.0, 1.0));
		currentFeatureVector.push_back(normalize(chunk.objectPose.R._m._13, -1.0, 1.0));
		currentFeatureVector.push_back(normalize(chunk.objectPose.R._m._21, -1.0, 1.0));
		currentFeatureVector.push_back(normalize(chunk.objectPose.R._m._22, -1.0, 1.0));
		currentFeatureVector.push_back(normalize(chunk.objectPose.R._m._23, -1.0, 1.0));
		currentFeatureVector.push_back(normalize(chunk.objectPose.R._m._31, -1.0, 1.0));
		currentFeatureVector.push_back(normalize(chunk.objectPose.R._m._32, -1.0, 1.0));
		currentFeatureVector.push_back(normalize(chunk.objectPose.R._m._33, -1.0, 1.0));
		/////////////////////////////////////////////////
		currentSeq.push_back(currentFeatureVector);
		
	}
// 	if (bStop) {
// 		CriticalSectionWrapper csw (cs);
// 		bStart = false;
// 		bRec = false;
// 		bStop = false;
	        
// 		//bStart = bStop = bRec = false;
// // 		ev.set(true);
// 	}
}

void ActiveLearnScenario::run(int argc, char* argv[]) {

	Ice::CommunicatorPtr ic;
	try {
	ic = Ice::initialize(argc, argv);

	Ice::ObjectPrx base2 = ic->stringToProxy("DataPlotter:default -p 8174");
	smlearning::plotting::PlottingAppPrx plotApp = smlearning::plotting::PlottingAppPrx::checkedCast(base2);

	if (!plotApp)
		throw "Invalid proxy";
	int smregionsCount = 18;
	plotApp->init (smregionsCount, learner.SMOOTHING + learner.TIMEWINDOW);
	plotApp->resize(600,400);
	
	plotApp->show();

	
// 	// initialize random seed:
// 	srand ((unsigned)time(NULL) );
	randomG.setRandSeed (context.getRandSeed());

	//a number that slightly greater then the maximal reachable space of the arm
	//    - used for workspace position normalization and later as a position upper bound
	//      for random polyflap position
	maxRange = 0.4;

	//minimal duration of a movement (by normal speed)
	const SecTmReal minDuration = SecTmReal(5.0);
	
	//Polyflap Position and orientation
	const Vec3 startPolyflapPosition(Real(0.2), Real(0.2), Real(0.0));
	const Vec3 startPolyflapRotation(Real(-0.0*REAL_PI), Real(-0.0*REAL_PI), Real(0.0*REAL_PI));//Y,X,Z
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
	const Real distance = 0.2;

	//Datasets in which all sequences are stored
	//Arranged in a vector associated to region indices
	//sequences and featureVectors are created
	//in every loop run
	vector<DataSet> data;
	for (int i=0; i<smregionsCount; i++) {
		DataSet currentDataset;
		data.push_back (currentDataset);
	}


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
	Vec3 orientationT(Real(-0.5*REAL_PI), Real(0.0*REAL_PI), Real(0.0*REAL_PI));

	int numSequences = 10000;
	int startingPosition = 0;
	if (argc > 2)
		numSequences = atoi(argv[2]);
	if (argc > 3)
		startingPosition = atoi(argv[3]);

	//start of the experiment loop
	int e = 0;
	while (e<numSequences) {
		
		// experiment main loops
		creator.setToDefault();
		//polyflap actor

		object = setupPolyflap(scene, startPolyflapPosition, startPolyflapRotation, polyflapDimensions, context);
		golem::Bounds::SeqPtr curPol = object->getGlobalBoundsSeq();
		if (e == 0)
			objectLocalBounds = object->getLocalBoundsSeq();
		

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


// 		//chose random point int the vicinity of the polyflap
// 		srand(context.getRandSeed()._U32[0]  + e);

	        int startPosition;

		if (startingPosition == 0)
			if ( e < smregionsCount ) 
// 				startPosition = rand() % smregionsCount + 1;
 				startPosition = floor(randomG.nextUniform (1.0, 18.0));
			else {
				//active selection of samples
// 				double neargreedyActionRand = rand() / (double) RAND_MAX;
				double neargreedyActionRand = randomG.nextUniform (0.0, 1.0);
				
				cout << "neargreedyRand: " << neargreedyActionRand << endl;
				if (neargreedyActionRand <= learner.neargreedyActionProb)
// 					startPosition = rand() % smregionsCount + 1;
					startPosition = floor(randomG.nextUniform (1.0, 18.0));
				else
					assert ((startPosition = learner.chooseSMRegion () + 1) != -1);
			}
		else
			startPosition = startingPosition;



		
		setCoordinatesIntoTarget(startPosition, positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, side, center, top, over);
		cout << "Position " << startPosition-1 << endl;

		// and set target waypoint
		golem::GenWorkspaceState target;
		fromCartesianPose(target.pos, positionT, orientationT);
		target.vel.setId(); // it doesn't mov

		target.t = context.getTimer()->elapsed() + tmDeltaAsync + minDuration; // i.e. the movement will last at least 5 sec

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
		currentSeq.clear();
		currentMotorCommandVector.clear();
		/////////////////////////////////////////////////

		/////////////////////////////////////////////////
		//writing in the initial vector
		//initial position, normalized
		currentMotorCommandVector.push_back(normalize<double>(positionT.v1, 0.0, maxRange));
		currentMotorCommandVector.push_back(normalize<double>(positionT.v2, 0.0, maxRange));
		currentMotorCommandVector.push_back(normalize<double>(positionT.v3, 0.0, maxRange));
		//initial orientation, normalized
// 		currentMotorCommandVector.push_back(normalize<double>(orientationT.v1, -REAL_PI, REAL_PI));
// 		currentMotorCommandVector.push_back(normalize<double>(orientationT.v2, -REAL_PI, REAL_PI));
// 		currentMotorCommandVector.push_back(normalize<double>(orientationT.v3, -REAL_PI, REAL_PI));
		//end pose info missing (must be added later 
		/////////////////////////////////////////////////


		// Trajectory duration calculated from a speed constant.
// 		int speed = rand() % 3 + 3;
		int speed = floor (randomG.nextUniform (3.0, 5.0));
		cout << "speed: " << speed << endl;
		// 			int speed = 0;
// 		SecTmReal duration = timeDelta * n*(Math::pow(Real(2.0), Real(-speed)*2));

		
		SecTmReal duration = speed;
				
		/////////////////////////////////////////////////
		//writing in the initial vector
		currentMotorCommandVector.push_back(Real(speed));
		/////////////////////////////////////////////////

		// Trajectory end pose equals begin + shift along Y axis
		WorkspaceCoord /*begin = target.pos, */end = target.pos;

		//normal vector to the center of the polyflap
		Vec3 polyflapCenterNormalVec =
			computeNormalVector(
					    Vec3 (positionT.v1, positionT.v2, positionT.v3),
					    Vec3 (polyflapPosition.v1, polyflapPosition.v2, polyflapDimensions.v2*0.5)
					    );
		//and it's orthogonal
		Vec3 polyflapCenterOrthogonalVec = computeOrthogonalVec(polyflapCenterNormalVec);
// 		context.getLogger()->post(Message::LEVEL_INFO, "centernormalvec: %f, %1f, %f", polyflapCenterNormalVec.v1, polyflapCenterNormalVec.v2, polyflapCenterNormalVec.v3);


		//the lenght of the movement
		Real currDistance = distance;
// 		if (speed < 0) {
// 			currDistance *= 5.0;
// 		}

		//chose random horizontal and vertical angle
// 		int horizontalAngle = rand() % 61 + 60;
		int horizontalAngle = floor(randomG.nextUniform (60.0, 120.0));
		
		// 			int horizontalAngle = 90;
			
		//int verticalAngle = rand() % 7;

		setMovementAngle(horizontalAngle, end, currDistance, polyflapCenterNormalVec, polyflapCenterOrthogonalVec);
		cout << "Horizontal direction angle: " << horizontalAngle << " degrees" << endl;


		currentMotorCommandVector.push_back(normalize(Real(horizontalAngle/180.0*REAL_PI), -REAL_PI, REAL_PI));

		/////////////////////////////////////////////////
		//writing of the initial vector into sequence
		currentSeq.push_back(currentMotorCommandVector);
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
			
		// wait for completion
// 		if (!ev.wait(60000))
// 			context.getLogger()->post(Message::LEVEL_ERR, "ActiveLearnScenario::run(): event timeout");
// 		ev.set(false);

// 		{
// 			CriticalSectionWrapper csw(cs);
				
// 			// save data
// 			const int indices [] = { dataIndex++ };
// 			FileWriteStream fstream(desc.dataPath.create(indices, 1).c_str());
// 			learningData.store(fstream);
				
			// remove object
			scene.releaseObject(*object);
			object = NULL;
// 		}
		/////////////////////////////////////////////////
		//writing the sequence into the dataset
		data[startPosition-1].push_back(currentSeq);
		/////////////////////////////////////////////////


		//initialize RNN learner with a dummy dataset
		//TODO: this is a hack!
		string dummyDataFile = "tmp_dummy";
		if (e == 0) {
			write_nc_file_basis (dummyDataFile, data[startPosition-1]);
			learner.build (dummyDataFile, smregionsCount, currentSeq[0].size() );
		}

		//update RNN learner with current sequence
		rnnlib::DataSequence trainseq(learner.header->inputSize, learner.header->outputSize);
		vector<int> inputShape, targetShape;
		inputShape.push_back (currentSeq.size() - 1);
		targetShape.push_back (currentSeq.size() - 1);
		trainseq.inputs.reshape(inputShape);
		trainseq.targetPatterns.reshape(targetShape);
		load_sequence (trainseq.inputs.data, trainseq.targetPatterns.data, currentSeq);
		learner.feed_forward (trainseq);
		calculatePfSeqFromOutputActivations (learner.net->outputLayer->outputActivations, currentMotorCommandVector.size(), maxRange);
		learner.update (trainseq, startPosition-1);
		vector<double> learnProgData = learner.learnProg_errorsMap[startPosition-1].first;
		vector<double> errorData = learner.learnProg_errorsMap[startPosition-1].second;		
		plotApp->updateData(startPosition-1, learnProgData, errorData);
// 		calculatePfSeqFromOutputActivations (learner.net->outputLayer->outputActivations, currentMotorCommandVector.size(), maxRange);
// 		golem::Mat34 predictedPfPose = getPfPoseFromOutputActivations (learner.net->outputLayer->outputActivations, currentMotorCommandVector.size(), maxRange);
// 		Actor* predictedPolyflapObject = setupPolyflap(scene, predictedPfPose, polyflapDimensions);

		//off collision detection
		arm->setCollisionBoundsGroup(0x0);

			
		cout << "sequence size: " << currentSeq.size() << endl;


		Vec3 positionPreH(target.pos.p.v1, target.pos.p.v2, target.pos.p.v3 += (polyflapDimensions.v2*1.1));
		// and set target waypoint
		golem::GenWorkspaceState preHome;
		preHome.pos.p = positionPreH;
		preHome.pos.R = home.pos.R;
// 		fromCartesianPose(preHome.pos, positionPreH, orientationH);
		preHome.vel.setId(); // it doesn't move

		preHome.t = context.getTimer()->elapsed() + tmDeltaAsync + SecTmReal(2.0); // i.e. the movement will last at least 2 sec

		// set the initial pose of the arm, force the global movement (with planning in the entire arm workspace)
		arm->getReacPlanner().send(preHome, ReacPlanner::ACTION_GLOBAL);
		// wait for completion of the action (until the arm moves to the initial pose)
		arm->getReacPlanner().waitForEnd();

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

		cout << "Iteration " << e++ << " completed!" << endl;

		currentPredictedObjSeq.clear();

		
		if (universe.interrupted())
			throw Interrupted();
	}
	initial.t = context.getTimer()->elapsed() + tmDeltaAsync + SecTmReal(5.0);
	// movement will last no shorter than 5 sec
	arm->getReacPlanner().send(initial, ReacPlanner::ACTION_GLOBAL);
	// wait until the arm is ready to accept new commands, but no longer than 60 seconds
	(void)arm->getReacPlanner().waitForEnd(60000);
	
	} catch (const Ice::Exception& ex) {
		std::cerr << ex << std::endl;
	} catch (const char* msg) {
		std::cerr << msg << std::endl;
	}
	if (ic)
		ic->destroy();
	
}

//------------------------------------------------------------------------------


golem::Mat34  ActiveLearnScenario::getPfPoseFromOutputActivations (rnnlib::SeqBuffer<double> outputActivations, int startIndex, Real maxRange) {
	int finalActIndex = outputActivations.shape[0] - 1;
// 	cout << finalActIndex << endl;
	int outputsize = outputActivations.shape[1];
// 	cout << outputsize << endl;

// 	cout << startIndex << endl;
	assert (startIndex == outputsize);

	golem::Mat34 predictedPfPose;

	predictedPfPose.p.v1 = denormalize(outputActivations[finalActIndex][startIndex++], /*-maxRange*/0.0, maxRange);
	predictedPfPose.p.v2 = denormalize(outputActivations[finalActIndex][startIndex++], /*-maxRange*/0.0, maxRange);
	predictedPfPose.p.v3 = denormalize(outputActivations[finalActIndex][startIndex++], /*-maxRange*/0.0, maxRange);
	predictedPfPose.R._m._11 = denormalize(outputActivations[finalActIndex][startIndex++], -1.0, 1.0);
	predictedPfPose.R._m._12 = denormalize(outputActivations[finalActIndex][startIndex++], -1.0, 1.0);
	predictedPfPose.R._m._13 = denormalize(outputActivations[finalActIndex][startIndex++], -1.0, 1.0);
	predictedPfPose.R._m._21 = denormalize(outputActivations[finalActIndex][startIndex++], -1.0, 1.0);
	predictedPfPose.R._m._22 = denormalize(outputActivations[finalActIndex][startIndex++], -1.0, 1.0);
	predictedPfPose.R._m._23 = denormalize(outputActivations[finalActIndex][startIndex++], -1.0, 1.0);
	predictedPfPose.R._m._31 = denormalize(outputActivations[finalActIndex][startIndex++], -1.0, 1.0);
	predictedPfPose.R._m._32 = denormalize(outputActivations[finalActIndex][startIndex++], -1.0, 1.0);
	predictedPfPose.R._m._33 = denormalize(outputActivations[finalActIndex][startIndex++], -1.0, 1.0);
	
	return predictedPfPose;
	


}


void ActiveLearnScenario::calculatePfSeqFromOutputActivations (rnnlib::SeqBuffer<double> outputActivations, int startIndex, Real maxRange) {
	
	for (int i=0; i < outputActivations.shape[0]; i++) {
		golem::Mat34 currentPredictedPfPose;
		
		currentPredictedPfPose.p.v1 = denormalize(outputActivations[i][startIndex], /*-maxRange*/0.0, maxRange);
		currentPredictedPfPose.p.v2 = denormalize(outputActivations[i][startIndex+1], /*-maxRange*/0.0, maxRange);
		currentPredictedPfPose.p.v3 = denormalize(outputActivations[i][startIndex+2], /*-maxRange*/0.0, maxRange);
		currentPredictedPfPose.R._m._11 = denormalize(outputActivations[i][startIndex+3], -1.0, 1.0);
		currentPredictedPfPose.R._m._12 = denormalize(outputActivations[i][startIndex+4], -1.0, 1.0);
		currentPredictedPfPose.R._m._13 = denormalize(outputActivations[i][startIndex+5], -1.0, 1.0);
		currentPredictedPfPose.R._m._21 = denormalize(outputActivations[i][startIndex+6], -1.0, 1.0);
		currentPredictedPfPose.R._m._22 = denormalize(outputActivations[i][startIndex+7], -1.0, 1.0);
		currentPredictedPfPose.R._m._23 = denormalize(outputActivations[i][startIndex+8], -1.0, 1.0);
		currentPredictedPfPose.R._m._31 = denormalize(outputActivations[i][startIndex+9], -1.0, 1.0);
		currentPredictedPfPose.R._m._32 = denormalize(outputActivations[i][startIndex+10], -1.0, 1.0);
		currentPredictedPfPose.R._m._33 = denormalize(outputActivations[i][startIndex+11], -1.0, 1.0);
		currentPredictedObjSeq.push_back (currentPredictedPfPose);
	}
}

//------------------------------------------------------------------------------

void MyApplication::run(int argc, char *argv[]) {

	ActiveLearnScenario::Desc desc;
	XMLData(desc, xmlcontext());

	ActiveLearnScenario *pActiveLearnScenario = dynamic_cast<ActiveLearnScenario*>(scene()->createObject(desc)); // throws
	if (pActiveLearnScenario == NULL) {
		context()->getLogger()->post(Message::LEVEL_CRIT, "MyApplication::run(): unable to cast to ActiveLearnScenario");
		return;
	}

	// Random number generator seed
	context()->getLogger()->post(Message::LEVEL_INFO, "Random number generator seed %d", context()->getRandSeed()._U32[0]);
	
	try {
		pActiveLearnScenario->run(argc, argv);
	}
	catch (const ActiveLearnScenario::Interrupted&) {
	}
	
	scene()->releaseObject(*pActiveLearnScenario);
}


};
