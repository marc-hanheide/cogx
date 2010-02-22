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

using namespace golem;

namespace smlearning {

//------------------------------------------------------------------------------

bool XMLData(ActiveLearnScenario::Desc &val, XMLContext* context, bool create) {
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


	

	
	return true;
}


bool XMLData2(ActiveLearnScenario* p, XMLContext* context) {



	//a number that slightly greater then the maximal reachable space of the arm
	//    - used for workspace position normalization and later as a position upper bound
	//      for random polyflap position
	//maxRange = 0.4;
	XMLData(p->maxRange, context->getContextFirst("polyflapInteraction maxRange"));

	//minimal duration of a movement (by normal speed)
	XMLData(p->minDuration, context->getContextFirst("polyflapInteraction minDuration"));


	Real x;
	Real y;
	Real z;
	
	//Polyflap Position and orientation
	XMLData(x, context->getContextFirst("polyflapInteraction startPolyflapPosition x"));
	XMLData(y, context->getContextFirst("polyflapInteraction startPolyflapPosition y"));
	XMLData(z, context->getContextFirst("polyflapInteraction startPolyflapPosition z"));
	p->startPolyflapPosition.set(x, y, z);

	XMLData(x, context->getContextFirst("polyflapInteraction startPolyflapRotation x"));
	XMLData(y, context->getContextFirst("polyflapInteraction startPolyflapRotation y"));
	XMLData(z, context->getContextFirst("polyflapInteraction startPolyflapRotation z"));
	p->startPolyflapRotation.set(y, x, z);

	//Polyflap dimensions		
	XMLData(x, context->getContextFirst("polyflapInteraction polyflapDimensions x"));
	XMLData(y, context->getContextFirst("polyflapInteraction polyflapDimensions y"));
	XMLData(z, context->getContextFirst("polyflapInteraction polyflapDimensions z"));
	p->polyflapDimensions.set(x, y, z);

	//vertical distance from the ground
	//const Real over = 0.01;
	//vertical distance from the ground considering fingertip radius
	XMLData(p->over, context->getContextFirst("polyflapInteraction over"));
	//distance from the front/back of the polyflap
	XMLData(p->dist, context->getContextFirst("polyflapInteraction dist"));

	Real r;

	//distance from the side of the polyflap
	XMLData(r, context->getContextFirst("polyflapInteraction side"));
	p->side = p->polyflapDimensions.v1*r;
	//center of the polyflap
	XMLData(r, context->getContextFirst("polyflapInteraction center"));
	p->center = p->polyflapDimensions.v2*r;
	//distance from the top of the polyflap
	//const Real top = polyflapDimensions.v2* 1.2;
	XMLData(p->top, context->getContextFirst("polyflapInteraction top"));
	p->top = p->polyflapDimensions.v2 - r;
	//lenght of the movement		
	XMLData(p->distance, context->getContextFirst("polyflapInteraction distance"));





return true;
}



//------------------------------------------------------------------------------

void ActiveLearnScenario::render () {
	CriticalSectionWrapper csw (cs);

	if ( learningData.currentPredictedPfSeq.size() == 0 || object == NULL)
		return;

	golem::BoundsRenderer boundsRenderer;
	vector<int> idx;
	idx.push_back(0);
	idx.push_back(learningData.currentPredictedPfSeq.size()-1);
	for (int i=0; i<2; i++) {
		Mat34 currentPose = learningData.currentPredictedPfSeq[idx[i]];
		
		boundsRenderer.setMat(currentPose);
		if (idx[i] == learningData.currentPredictedPfSeq.size()-1)
			boundsRenderer.setWireColour (RGBA::RED);
		else if (idx[i] == 0)
			boundsRenderer.setWireColour (RGBA::BLUE);			
		
		boundsRenderer.renderWire (objectLocalBounds->begin(), objectLocalBounds->end());
	}

	idx.pop_back();
	idx.push_back(learningData.currentPredictedEfSeq.size()-1);
	for (int i=0; i<2; i++) {
		Mat34 currentPose = learningData.currentPredictedEfSeq[idx[i]];			
		boundsRenderer.setMat(currentPose);
		if (idx[i] == learningData.currentPredictedEfSeq.size()-1)
			boundsRenderer.setWireColour (RGBA::RED);
		else if (idx[i] == 0)
			boundsRenderer.setWireColour (RGBA::BLUE);			
		
		boundsRenderer.renderWire (effectorBounds.begin(), effectorBounds.end());
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
// 		golem::Mat34 p = chunk./*object*/effectorPose; 
// 		cout << "pose: ";
// 		printf("%f %f %f %f %f %f %f %f %f %f %f %f\n", p.p.v1, p.p.v2, p.p.v3, p.R._m._11, p.R._m._12, p.R._m._13, p.R._m._21, p.R._m._22, p.R._m._23, p.R._m._31, p.R._m._32, p.R._m._33);  

		Real efRoll, efPitch, efYaw;
		chunk.effectorPose.R.toEuler (efRoll, efPitch, efYaw);
// 		cout << "Effector roll: " << efRoll << " pitch: " << efPitch << " yaw: " << efYaw << endl;
		Real obRoll, obPitch, obYaw;
		chunk.objectPose.R.toEuler (obRoll, obPitch, obYaw);
// 		cout << "Object roll: " << obRoll << " pitch: " << obPitch << " yaw: " << obYaw << endl;  
	
// 		learningData.data.push_back(chunk);
// 		trialTime += SecTmReal(1.0)/universe.getRenderFrameRate();
		/////////////////////////////////////////////////
		//storing the feature vector

		currentFeatureVector.push_back(normalize(chunk.effectorPose.p.v1, 0.0, maxRange));
		currentFeatureVector.push_back(normalize(chunk.effectorPose.p.v2, 0.0, maxRange));
		currentFeatureVector.push_back(normalize(chunk.effectorPose.p.v3, 0.0, maxRange));
		currentFeatureVector.push_back(normalize(efRoll, -REAL_PI, REAL_PI));
		currentFeatureVector.push_back(normalize(efPitch, -REAL_PI, REAL_PI));
		currentFeatureVector.push_back(normalize(efYaw, -REAL_PI, REAL_PI));

		currentFeatureVector.push_back(normalize(chunk.objectPose.p.v1, 0.0, maxRange));
		currentFeatureVector.push_back(normalize(chunk.objectPose.p.v2, 0.0, maxRange));
		currentFeatureVector.push_back(normalize(chunk.objectPose.p.v3, 0.0, maxRange));
		currentFeatureVector.push_back(normalize(obRoll, -REAL_PI, REAL_PI));
		currentFeatureVector.push_back(normalize(obPitch, -REAL_PI, REAL_PI));
		currentFeatureVector.push_back(normalize(obYaw, -REAL_PI, REAL_PI));

		learningData.currentSeq.push_back(currentFeatureVector);
	
		/////////////////////////////////////////////////
		//initialize RNN learner
		if (!netBuilt) {
			learner.build (smregionsCount, learningData.currentMotorCommandVector.size() + currentFeatureVector.size() );
			netBuilt = true;
		}
		loadCurrentTrainSeq (learner.header->inputSize, learner.header->outputSize);
		learner.feed_forward (*trainSeq);
// 		golem::Mat34 predictedPfPose = getPfPoseFromOutputActivations (learner.net->outputLayer->outputActivations, learningData.currentMotorCommandVector.size(), maxRange);
// 		learningData.currentPredictedPfSeq.push_back (predictedPfPose);
		getPfEfSeqFromOutputActivations (learner.net->outputLayer->outputActivations, learningData.currentMotorCommandVector.size(), maxRange, learningData.currentPredictedPfSeq, learningData.currentPredictedEfSeq);
	
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


		//initialization of arm target: the center of the polyflap
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



		//arm target position update
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
		cout << "predicted poses 1 size: " << learningData.currentPredictedPfSeq.size() << endl;


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

		learningData.currentPredictedPfSeq.clear();
		learningData.currentPredictedEfSeq.clear();

		
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
	learner.save_net_data (writeDownCollectedData(learningData.data));
	
	/////////////////////////////////////////////////
	
	} catch (const Ice::Exception& ex) {
		std::cerr << ex << std::endl;
	} catch (const char* msg) {
		std::cerr << msg << std::endl;
	}
	if (ic)
		ic->destroy();
	
}

//------------------------------------------------------------------------------


golem::Mat34  ActiveLearnScenario::getPfEfPoseFromOutputActivations (rnnlib::SeqBuffer<double> outputActivations, int startIndex, Real maxRange, golem::Mat34& predictedPfPose, golem::Mat34& predictedEfPose) {
	int finalActIndex = outputActivations.shape[0] - 1;
// 	cout << finalActIndex << endl;
	int outputsize = outputActivations.shape[1];
// 	cout << outputsize << endl;

// 	cout << startIndex << endl;
	assert (startIndex < outputsize);

	//extract effector Pose
	predictedEfPose.p.v1 = denormalize(outputActivations[finalActIndex][startIndex++], 0.0, maxRange);
	predictedEfPose.p.v2 = denormalize(outputActivations[finalActIndex][startIndex++], 0.0, maxRange);
	predictedEfPose.p.v3 = denormalize(outputActivations[finalActIndex][startIndex++], 0.0, maxRange);
	Real efRoll, efPitch, efYaw;
	efRoll = denormalize(outputActivations[finalActIndex][startIndex++], -REAL_PI, REAL_PI);
	efPitch = denormalize(outputActivations[finalActIndex][startIndex++], -REAL_PI, REAL_PI);
	efYaw = denormalize(outputActivations[finalActIndex][startIndex++], -REAL_PI, REAL_PI);
	predictedEfPose.R.fromEuler (efRoll, efPitch, efYaw);

	//extract polyflap Pose
	predictedPfPose.p.v1 = denormalize(outputActivations[finalActIndex][startIndex++], 0.0, maxRange);
	predictedPfPose.p.v2 = denormalize(outputActivations[finalActIndex][startIndex++], 0.0, maxRange);
	predictedPfPose.p.v3 = denormalize(outputActivations[finalActIndex][startIndex++], 0.0, maxRange);
	Real pfRoll, pfPitch, pfYaw;
	pfRoll = denormalize(outputActivations[finalActIndex][startIndex++], -REAL_PI, REAL_PI);
	pfPitch = denormalize(outputActivations[finalActIndex][startIndex++], -REAL_PI, REAL_PI);
	pfYaw = denormalize(outputActivations[finalActIndex][startIndex++], -REAL_PI, REAL_PI);
	predictedPfPose.R.fromEuler(pfRoll, pfPitch, pfYaw);



}


void ActiveLearnScenario::getPfEfSeqFromOutputActivations (rnnlib::SeqBuffer<double> outputActivations, int sIndex, Real maxRange, vector<golem::Mat34>& currentPredictedPfSeq, vector<golem::Mat34>& currentPredictedEfSeq) {

	currentPredictedPfSeq.clear();
	for (int i=0; i < outputActivations.shape[0]; i++) {
		golem::Mat34 currentPredictedPfPose;
		golem::Mat34 currentPredictedEfPose;
		int startIndex = sIndex;

		//extract effector Pose
		currentPredictedEfPose.p.v1 = denormalize(outputActivations[i][startIndex++], 0.0, maxRange);
		currentPredictedEfPose.p.v2 = denormalize(outputActivations[i][startIndex++], 0.0, maxRange);
		currentPredictedEfPose.p.v3 = denormalize(outputActivations[i][startIndex++], 0.0, maxRange);
		Real efRoll, efPitch, efYaw;
		efRoll = denormalize(outputActivations[i][startIndex++], -REAL_PI, REAL_PI);
		efPitch = denormalize(outputActivations[i][startIndex++], -REAL_PI, REAL_PI);
 		efYaw = denormalize(outputActivations[i][startIndex++], -REAL_PI, REAL_PI);
		currentPredictedEfPose.R.fromEuler(efRoll, efPitch, efYaw);
		currentPredictedEfSeq.push_back (currentPredictedEfPose);
		
		//extract polyflap Pose
		currentPredictedPfPose.p.v1 = denormalize(outputActivations[i][startIndex++], 0.0, maxRange);
		currentPredictedPfPose.p.v2 = denormalize(outputActivations[i][startIndex++], 0.0, maxRange);
		currentPredictedPfPose.p.v3 = denormalize(outputActivations[i][startIndex++], 0.0, maxRange);
		Real pfRoll, pfPitch, pfYaw;
		pfRoll = denormalize(outputActivations[i][startIndex++], -REAL_PI, REAL_PI);
		pfPitch = denormalize(outputActivations[i][startIndex++], -REAL_PI, REAL_PI);
 		pfYaw = denormalize(outputActivations[i][startIndex++], -REAL_PI, REAL_PI);
		currentPredictedPfPose.R.fromEuler(pfRoll, pfPitch, pfYaw);
		currentPredictedPfSeq.push_back (currentPredictedPfPose);
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

XMLData2(pActiveLearnScenario, xmlcontext());


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
