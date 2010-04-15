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
// 		golem::Mat34 p = chunk.objectPose; 
// 		cout << "pose: ";
// 		printf("%.20f %.20f %.20f %f %f %f %f %f %f %f %f %f\n", p.p.v1, p.p.v2, p.p.v3, p.R._m._11, p.R._m._12, p.R._m._13, p.R._m._21, p.R._m._22, p.R._m._23, p.R._m._31, p.R._m._32, p.R._m._33);  

		Real efRoll, efPitch, efYaw;
		chunk.effectorPose.R.toEuler (efRoll, efPitch, efYaw);
// 		cout << "Effector roll: " << efRoll << " pitch: " << efPitch << " yaw: " << efYaw << endl;
		Real obRoll, obPitch, obYaw;
		chunk.objectPose.R.toEuler (obRoll, obPitch, obYaw);
		cout << "Object roll: " << obRoll << " pitch: " << obPitch << " yaw: " << obYaw << endl;

		Real polStateOutput = 0; //polyflap moves with the same Y angle
		Real epsilonAngle = 0.005;
		if (currentPfRoll < (obRoll - epsilonAngle) ) //polyflap Y angle increases
			polStateOutput = 1;
		if (currentPfRoll > (obRoll + epsilonAngle) ) //polyflap Y angle decreases
			polStateOutput = -1;

		if (polStateOutput == 0 && currentPfY < chunk.objectPose.p.v2) // polyflap Y position increases
			polStateOutput = 0.5;
		if (polStateOutput == 0 && currentPfY > chunk.objectPose.p.v2) //polyflap Y position decreases
			polStateOutput = -0.5;
		
// 		cout << "polStateOutput: " << polStateOutput << endl;
		currentPfRoll = obRoll;
		currentPfY = chunk.objectPose.p.v2;

		
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
		currentFeatureVector.push_back(normalize(chunk.objectPose.p.v3, -0.01, desc.maxRange));
		currentFeatureVector.push_back(normalize(obRoll, -REAL_PI, REAL_PI));
		currentFeatureVector.push_back(normalize(obPitch, -REAL_PI, REAL_PI));
		currentFeatureVector.push_back(normalize(obYaw, -REAL_PI, REAL_PI));
		currentFeatureVector.push_back(polStateOutput);

		learningData.currentSeq.push_back(currentFeatureVector);

		/////////////////////////////////////////////////
		//initialize RNN learner
		if (!netBuilt) {
			learner.build (smregionsCount, learningData.currentMotorCommandVector.size() + currentFeatureVector.size() );
			netBuilt = true;
		}
		loadCurrentTrainSeq (learner.header->inputSize, learner.header->outputSize);
		learner.feed_forward (*trainSeq);
// 		golem::Mat34 predictedPfPose = getPfPoseFromOutputActivations (learner.net->outputLayer->outputActivations, learningData.currentMotorCommandVector.size(), desc.maxRange);
// 		learningData.currentPredictedPfSeq.push_back (predictedPfPose);
		getPfEfSeqFromOutputActivations (learner.net->outputLayer->outputActivations, learningData.currentMotorCommandVector.size(), desc.maxRange, learningData.currentPredictedPfSeq, learningData.currentPredictedEfSeq);
	
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


///
///prepares the polyflap to use
///
void ActiveLearnScenario::initialize_polyflap(){
	
	//creates a polyflap
	create_polyflap_object();
	
	if (iteration == 0) {
			objectLocalBounds = object->getLocalBoundsSeq();		
	}
	currentPfY = object->getPose().p.v2;

	//computes needed information about the polyflap
	compute_vectors();
}

///
///choose and describe the start point of the experiment trajectory
///
void  ActiveLearnScenario::initialize_movement(){

	//set default coordinates
	set_positionT();

	//choose start point
	define_start_position();

	//edit the coordinates so that they describe chosen start point
	prepare_target();	
}


///
///choose the starting position
///
void ActiveLearnScenario::define_start_position(){
	cout << "ALS: define_start_position" << endl;

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

}

void ActiveLearnScenario::write_dataset_into_binary(){
	cout << "ALS: write_dataset_into_binary" << endl;
	/////////////////////////////////////////////////
	//writing the dataset into binary file
	learner.save_net_data (writeDownCollectedData(data));
	/////////////////////////////////////////////////
	
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

	netBuilt = false;

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
		
		//compute coordinates of start position
		initialize_movement();

		//move the finger to the beginnign of experiment trajectory
		send_position(target, ReacPlanner::ACTION_GLOBAL);
		
		//create feature sequence and vector
		init_writing();

		//write initial position and orientation of the finger
		write_finger_pos_and_or();

		//compute direction and other features of trajectory
		set_up_movement();

		//write chosen speed and angle of the finger experiment trajectory	
		write_finger_speed_and_angle();

		//add feature vector to the sequence
		write_f_vector_into_sequence();

		//move the finger along described experiment trajectory
		move_finger();

		//write sequence into dataset
		write_sequence_into_dataset();


		//update RNN learner with current sequence
		{
			CriticalSectionWrapper csw (cs);
			learner.update (*trainSeq, startPosition-1);
			vector<double> learnProgData = learner.learnProg_errorsMap[startPosition-1].first;
			vector<double> errorData = learner.learnProg_errorsMap[startPosition-1].second;		
			plotApp->updateData(startPosition-1, learnProgData, errorData);
		}


		//turn off collision detection
		set_collision_detection(false);

		//print out information about current sequence
		print_sequence_info();
		cout << "predicted poses 1 size: " << learningData.currentPredictedPfSeq.size() << endl;

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

		//print out end information of this iteration
		iteration_end_info();		
		
		
		learningData.currentPredictedPfSeq.clear();
		learningData.currentPredictedEfSeq.clear();
	
		//finish this iteration
		finish_iteration();

}

	//move the arm to its initial position
	move_to_initial();
	
	//write obtained data into a binary file
	write_dataset_into_binary();


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
