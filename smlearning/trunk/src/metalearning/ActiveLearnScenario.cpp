/** @file ActiveLearnScenario.cpp
 * 
 * Program demonstrates affordances learning with LSTMs.
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

// using namespace push;
using namespace golem;

namespace smlearning {



//------------------------------------------------------------------------------

void ActiveLearnScenario::render () {
	CriticalSectionWrapper csw (cs);

	if ( learningData.currentPredictedObjSeq.size() == 0 || object == NULL)
		return;

	for (int i=0; i<learningData.currentPredictedObjSeq.size(); i++) {
		golem::BoundsRenderer boundsRenderer;
		if (i == 0 || i == learningData.currentPredictedObjSeq.size()-1) {
		Mat34 currentPose = learningData.currentPredictedObjSeq[i];
// 		Mat34 currentPose = learningData.currentPredictedObjSeq[learningData.currentPredictedObjSeq.size()-1];
		
		boundsRenderer.setMat(currentPose);
		if (i == learningData.currentPredictedObjSeq.size()-1)
			boundsRenderer.setWireColour (RGBA::RED);
		else if (i == 0)
			boundsRenderer.setWireColour (RGBA::BLUE);			

		boundsRenderer.renderWire (objectLocalBounds->begin(), objectLocalBounds->end());
		}

		
	}

}


//------------------------------------------------------------------------------


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
// 		golem::Mat34 p = chunk.objectPose; 
// 		cout << "pose: ";
// 		printf("%f %f %f %f %f %f %f %f %f %f %f %f\n", p.p.v1, p.p.v2, p.p.v3, p.R._m._11, p.R._m._12, p.R._m._13, p.R._m._21, p.R._m._22, p.R._m._23, p.R._m._31, p.R._m._32, p.R._m._33);  

		Real roll, pitch, yaw;
		chunk.objectPose.R.toEuler (roll, pitch, yaw);
// 		cout << "roll: " << roll << " pitch: " << pitch << " yaw: " << yaw << endl;  
	
// 		learningData.data.push_back(chunk);
// 		trialTime += SecTmReal(1.0)/universe.getRenderFrameRate();
		/////////////////////////////////////////////////
		//storing the feature vector

		currentFeatureVector.push_back(normalize(chunk.objectPose.p.v1, 0.0, maxRange));
		currentFeatureVector.push_back(normalize(chunk.objectPose.p.v2, 0.0, maxRange));
		currentFeatureVector.push_back(normalize(chunk.objectPose.p.v3, 0.0, maxRange));
		currentFeatureVector.push_back(normalize(roll, -REAL_PI, REAL_PI));
		currentFeatureVector.push_back(normalize(pitch, -REAL_PI, REAL_PI));
		currentFeatureVector.push_back(normalize(yaw, -REAL_PI, REAL_PI));

		learningData.currentSeq.push_back(currentFeatureVector);
	
		/////////////////////////////////////////////////
		//initialize RNN learner with a dummy dataset
		if (!netBuilt) {
			learner.build (smregionsCount, learningData.currentMotorCommandVector.size() + currentFeatureVector.size() );
			netBuilt = true;
		}
		loadCurrentTrainSeq (learner.header->inputSize, learner.header->outputSize);
		learner.feed_forward (*trainSeq);
// 		golem::Mat34 predictedPfPose = getPfPoseFromOutputActivations (learner.net->outputLayer->outputActivations, learningData.currentMotorCommandVector.size(), maxRange);
// 		learningData.currentPredictedObjSeq.push_back (predictedPfPose);
		getPfSeqFromOutputActivations (learner.net->outputLayer->outputActivations, learningData.currentMotorCommandVector.size(), maxRange, learningData.currentPredictedObjSeq);
	
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
	plotApp->init (smregionsCount, learner.SMOOTHING + learner.TIMEWINDOW);
	plotApp->resize(640,480);
	
	plotApp->show();

	
// 	// initialize random seed:
	randomG.setRandSeed (context.getRandSeed());
	netBuilt = false;

	//a number that slightly greater then the maximal reachable space of the arm
	//    - used for workspace position normalization and later as a position upper bound
	//      for random polyflap position
	maxRange = 0.4;

	//minimal duration of a movement (by normal speed)
	const SecTmReal minDuration = SecTmReal(5.0);
	
	//Polyflap Position and orientation
	const Vec3 startPolyflapPosition(Real(0.2), Real(0.2), Real(0.0));
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
	const Real distance = 0.2;

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
	for (iteration = 0; iteration<numSequences; iteration++) {
		
		// experiment main loops
		creator.setToDefault();
		//polyflap actor

		object = setupPolyflap(scene, startPolyflapPosition, startPolyflapRotation, polyflapDimensions, context);
		golem::Bounds::SeqPtr curPol = object->getGlobalBoundsSeq();
		if (iteration == 0)
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


	        int startPosition;

		if (startingPosition == 0)
			if ( iteration < smregionsCount ) 
 				startPosition = floor(randomG.nextUniform (1.0, 18.0));
			else {
				//active selection of samples
				double neargreedyActionRand = randomG.nextUniform (0.0, 1.0);
				
				cout << "neargreedyRand: " << neargreedyActionRand << endl;
				if (neargreedyActionRand <= learner.neargreedyActionProb)
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
		learningData.currentSeq.clear();
		learningData.currentMotorCommandVector.clear();
		/////////////////////////////////////////////////

		/////////////////////////////////////////////////
		//writing in the initial vector
		//initial position, normalized
		learningData.currentMotorCommandVector.push_back(normalize<double>(positionT.v1, 0.0, maxRange));
		learningData.currentMotorCommandVector.push_back(normalize<double>(positionT.v2, 0.0, maxRange));
		learningData.currentMotorCommandVector.push_back(normalize<double>(positionT.v3, 0.0, maxRange));
		//initial orientation, normalized
// 		currentMotorCommandVector.push_back(normalize<double>(orientationT.v1, -REAL_PI, REAL_PI));
// 		currentMotorCommandVector.push_back(normalize<double>(orientationT.v2, -REAL_PI, REAL_PI));
// 		currentMotorCommandVector.push_back(normalize<double>(orientationT.v3, -REAL_PI, REAL_PI));
		//end pose info missing (must be added later 
		/////////////////////////////////////////////////


		// Trajectory duration calculated from a speed constant.
		int speed = floor (randomG.nextUniform (3.0, 5.0));
		cout << "speed: " << speed << endl;
		
		SecTmReal duration = speed;
				
		/////////////////////////////////////////////////
		//writing in the initial vector
		learningData.currentMotorCommandVector.push_back(Real(speed));
		/////////////////////////////////////////////////

		// Trajectory end pose equals begin + shift along Y axis
		WorkspaceCoord end = target.pos;

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

		//chose random horizontal and vertical angle
		int horizontalAngle = floor(randomG.nextUniform (60.0, 120.0));
		
		//int verticalAngle = rand() % 7;

		setMovementAngle(horizontalAngle, end, currDistance, polyflapCenterNormalVec, polyflapCenterOrthogonalVec);
		cout << "Horizontal direction angle: " << horizontalAngle << " degrees" << endl;


		learningData.currentMotorCommandVector.push_back(normalize(Real(horizontalAngle/180.0*REAL_PI), -REAL_PI, REAL_PI));

		/////////////////////////////////////////////////
		//writing of the initial vector into sequence
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
			
			
		/////////////////////////////////////////////////
		//writing the sequence into the dataset
		learningData.data.push_back(learningData.currentSeq);
		/////////////////////////////////////////////////

		
		//update RNN learner with current sequence
		{
			CriticalSectionWrapper csw (cs);
			learner.update (*trainSeq, startPosition-1);
			vector<double> learnProgData = learner.learnProg_errorsMap[startPosition-1].first;
			vector<double> errorData = learner.learnProg_errorsMap[startPosition-1].second;		
			plotApp->updateData(startPosition-1, learnProgData, errorData);
		}

		//off collision detection
		arm->setCollisionBoundsGroup(0x0);

			
		cout << "sequence size: " << learningData.currentSeq.size() << endl;
		cout << "predicted poses 1 size: " << learningData.currentPredictedObjSeq.size() << endl;


		Vec3 positionPreH(target.pos.p.v1, target.pos.p.v2, target.pos.p.v3 += (polyflapDimensions.v2*1.1));
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

		learningData.currentPredictedObjSeq.clear();

		
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
	assert (startIndex < outputsize);

	golem::Mat34 predictedPfPose;

	predictedPfPose.p.v1 = denormalize(outputActivations[finalActIndex][startIndex++], /*-maxRange*/0.0, maxRange);
	predictedPfPose.p.v2 = denormalize(outputActivations[finalActIndex][startIndex++], /*-maxRange*/0.0, maxRange);
	predictedPfPose.p.v3 = denormalize(outputActivations[finalActIndex][startIndex++], /*-maxRange*/0.0, maxRange);

	Real roll, pitch, yaw;
	roll = denormalize(outputActivations[finalActIndex][startIndex++], -REAL_PI, REAL_PI);
	pitch = denormalize(outputActivations[finalActIndex][startIndex++], -REAL_PI, REAL_PI);
	yaw = denormalize(outputActivations[finalActIndex][startIndex++], -REAL_PI, REAL_PI);
	predictedPfPose.R.fromEuler(roll, pitch, yaw);

	return predictedPfPose;
	


}


void ActiveLearnScenario::getPfSeqFromOutputActivations (rnnlib::SeqBuffer<double> outputActivations, int startIndex, Real maxRange, vector<golem::Mat34>& currentPredictedObjSeq) {
	
	currentPredictedObjSeq.clear();
	for (int i=0; i < outputActivations.shape[0]; i++) {
		golem::Mat34 currentPredictedPfPose;
		
		currentPredictedPfPose.p.v1 = denormalize(outputActivations[i][startIndex], /*-maxRange*/0.0, maxRange);
		currentPredictedPfPose.p.v2 = denormalize(outputActivations[i][startIndex+1], /*-maxRange*/0.0, maxRange);
		currentPredictedPfPose.p.v3 = denormalize(outputActivations[i][startIndex+2], /*-maxRange*/0.0, maxRange);
		Real roll, pitch, yaw;
		roll = denormalize(outputActivations[i][startIndex+3], -REAL_PI, REAL_PI);
		pitch = denormalize(outputActivations[i][startIndex+4], -REAL_PI, REAL_PI);
 		yaw = denormalize(outputActivations[i][startIndex+5], -REAL_PI, REAL_PI);
		currentPredictedPfPose.R.fromEuler(roll, pitch, yaw);		

		currentPredictedObjSeq.push_back (currentPredictedPfPose);
	}
}

//------------------------------------------------------------------------------
void ActiveLearnScenario::loadCurrentTrainSeq (int inputSize, int outputSize) {
	if (trainSeq)
		delete trainSeq;
	trainSeq = new rnnlib::DataSequence (inputSize, outputSize);
	vector<int> inputShape, targetShape;
	inputShape.push_back (learningData.currentSeq.size() - 1);
	targetShape.push_back (learningData.currentSeq.size() - 1);
	trainSeq->inputs.reshape(inputShape);
	trainSeq->targetPatterns.reshape(targetShape);
	load_sequence (trainSeq->inputs.data, trainSeq->targetPatterns.data, learningData.currentSeq);
}

//------------------------------------------------------------------------------

void ActivePushingApplication::run(int argc, char *argv[]) {

	ActiveLearnScenario::Desc desc;
	XMLData(desc, xmlcontext());

	ActiveLearnScenario *pActiveLearnScenario = dynamic_cast<ActiveLearnScenario*>(scene()->createObject(desc)); // throws
	if (pActiveLearnScenario == NULL) {
		context()->getLogger()->post(Message::LEVEL_CRIT, "ActivePushingApplication::run(): unable to cast to ActiveLearnScenario");
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
