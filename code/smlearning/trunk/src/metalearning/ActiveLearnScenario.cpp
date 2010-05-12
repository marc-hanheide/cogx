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
	if (learningData.currentPredictedEfSeq.size() > 0) {
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

		chunk.effectorPose.R.toEuler (chunk.efRoll, chunk.efPitch, chunk.efYaw);
// 		cout << "Effector roll: " << efRoll << " pitch: " << efPitch << " yaw: " << efYaw << endl;
		chunk.objectPose.R.toEuler (chunk.obRoll, chunk.obPitch, chunk.obYaw);
		// cout << "Object roll: " << obRoll << " pitch: " << obPitch << " yaw: " << obYaw << endl;

// 		learningData.data.push_back(chunk);
// 		trialTime += SecTmReal(1.0)/universe.getRenderFrameRate();
		add_feature_vector (currentFeatureVector, chunk); 
		//add_label (currentFeatureVector, chunk);
		
		learningData.currentSeq.push_back(currentFeatureVector);

		int size = learningData.currentSeq.size();
		// for (int i=0; i<learningData.currentSeq[size-1].size(); i++) 
		// 	cout << learningData.currentSeq[size-1][i] << " ";
		// cout << endl;

		currentPfRoll = chunk.obRoll;
		currentPfY = chunk.objectPose.p.v2;

		/////////////////////////////////////////////////
		//initialize RNN learner
		if (!netBuilt) {
			learner.build (startingPositionsCount, learningData.currentMotorCommandVector.size() + currentFeatureVector.size(),  /*learningData.currentMotorCommandVector.size() + */currentFeatureVector.size() / 2);
			//learner.build (startingPositionsCount, learningData.currentMotorCommandVector.size() - 3 + currentFeatureVector.size(), currentFeatureVector.size() );
			netBuilt = true;
		}
		load_current_trainSeq (learner.header->inputSize, learner.header->outputSize);
		learner.feed_forward (*trainSeq);
		//get_pfefSeq_from_outputActivations (learner.net->outputLayer->outputActivations, learningData.currentMotorCommandVector.size(), desc.maxRange, desc.minZ, learningData.currentPredictedPfSeq, learningData.currentPredictedEfSeq);
		get_pfefSeq_from_outputActivations (learner.net->outputLayer->outputActivations, /*learningData.currentMotorCommandVector.size() - 3*/0, desc.maxRange, desc.minZ, learningData.currentPredictedPfSeq, learningData.currentPredictedEfSeq);
	
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
///get the action that maximizes learning progress
///
pair<FeatureVector, ActiveLearnScenario::Action> ActiveLearnScenario::get_actionsIdx_maxLearningProgress (const ActionsVector& candidateActions) {
	double maxLearningProgress = -1e6;
	int index = -1;
	for (int i=0; i < candidateActions.size(); i++) {
		SMRegion contextRegion = regions[get_SMRegion (candidateActions[i].first)];
		double learningProgress = 0.0/*contextRegion.learningProgressHistory.back()*/;
		if ( learningProgress > maxLearningProgress  ) {
			maxLearningProgress = learningProgress;
			index = i;
		}
	}
	assert (index != -1);
	return candidateActions[index];
}



///
///select an optimal action from a random set of actions
///
void ActiveLearnScenario::choose_action () {
	
	if (startingPosition == 0) {

		//active selection of samples
		double neargreedyActionRand = randomG.nextUniform (0.0, 1.0);
		
		cout << "neargreedyRand: " << neargreedyActionRand << endl;
		if (neargreedyActionRand <= neargreedyActionProb) {
			Scenario::choose_action ();
		}
		
		else {

			ActionsVector candidateActions;
			
			for (int i=0; i<maxNumberCandidateActions; i++) {
				Action action;
				action.startPosition = floor(randomG.nextUniform (1.0, 19.0));
				//action.speed = floor (randomG.nextUniform (3.0, 5.0));
				action.speed = 3.0;
				action.horizontalAngle = choose_angle(60.0, 120.0, "cont");
				FeatureVector motorVector;
				Vec3 pos;
				init_positionT (pos);
				set_coordinates_into_target(startPosition, pos, polyflapNormalVec, polyflapOrthogonalVec, desc.dist, desc.side, desc.center, desc.top, desc.over);
				write_finger_pos_and_or (motorVector, pos);
				write_finger_speed_and_angle (motorVector, action.speed, action.horizontalAngle);
				candidateActions.push_back (make_pair (motorVector, action));
			}

			pair<FeatureVector, Action> chosenAction = get_actionsIdx_maxLearningProgress(candidateActions);
			
			this->startPosition = chosenAction.second.startPosition;
			this->speed = chosenAction.second.speed;
			this->horizontalAngle = chosenAction.second.horizontalAngle;

		}
	}
	else
		Scenario::choose_action ();
	
}


void ActiveLearnScenario::write_dataset_into_binary(){
	cout << "ALS: write_dataset_into_binary" << endl;
	/////////////////////////////////////////////////
	//writing the dataset into binary file
	learner.save_net_data (writedown_collected_data(data));
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
	plotApp->init (startingPositionsCount, learner.SMOOTHING + learner.TIMEWINDOW);
	plotApp->resize(640,480);
	
	plotApp->show();

	netBuilt = false;
	regionsCount = 0;
	SMRegion firstRegion (regionsCount, motorVectorSize );
	regions[regionsCount] = firstRegion;

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

		//select an optimal action from a random set of actions
		{
			CriticalSectionWrapper csw (cs);
			choose_action ();
		}

		//compute coordinates of start position
		calculate_starting_position_coord ();

		//move the finger to the beginnign of experiment trajectory
		send_position(target, ReacPlanner::ACTION_GLOBAL);
		
		//create feature sequence and vector
		init_writing();

		//write initial position and orientation of the finger
		write_finger_pos_and_or(learningData.currentMotorCommandVector, positionT);

		//write chosen speed and angle of the finger experiment trajectory	
		write_finger_speed_and_angle(learningData.currentMotorCommandVector, speed, horizontalAngle);
		//add motor vector to the sequence
		write_motor_vector_into_sequence();

		currentRegion = regions[get_SMRegion (learningData.currentMotorCommandVector)];

		//compute direction and other features of trajectory
		set_up_movement();

		//move the finger along described experiment trajectory
		move_finger();

		//write sequence into dataset
		write_sequence_into_dataset(data);


		//update RNN learner with current sequence
		{
			CriticalSectionWrapper csw (cs);
			learner.update (*trainSeq, startPosition-1);
			//update_learners ();
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
		check_interrupted();

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


golem::Mat34  ActiveLearnScenario::get_pfefPose_from_outputActivations (const rnnlib::SeqBuffer<double>& outputActivations, int startIndex, Real maxRange, Real minZ, golem::Mat34& predictedPfPose, golem::Mat34& predictedEfPose) {
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
	predictedPfPose.p.v3 = denormalize(outputActivations[finalActIndex][startIndex++], minZ, maxRange);
	Real pfRoll, pfPitch, pfYaw;
	pfRoll = denormalize(outputActivations[finalActIndex][startIndex++], -REAL_PI, REAL_PI);
	pfPitch = denormalize(outputActivations[finalActIndex][startIndex++], -REAL_PI, REAL_PI);
	pfYaw = denormalize(outputActivations[finalActIndex][startIndex++], -REAL_PI, REAL_PI);
	predictedPfPose.R.fromEuler(pfRoll, pfPitch, pfYaw);



}


void ActiveLearnScenario::get_pfefSeq_from_outputActivations (const rnnlib::SeqBuffer<double>& outputActivations, int sIndex, Real maxRange, Real minZ, vector<golem::Mat34>& currentPredictedPfSeq, vector<golem::Mat34>& currentPredictedEfSeq) {

	currentPredictedPfSeq.clear();
	for (int i=0; i < outputActivations.shape[0]; i++) {
		golem::Mat34 currentPredictedPfPose;
		golem::Mat34 currentPredictedEfPose;
		int startIndex = sIndex;

		//extract effector Pose
		// currentPredictedEfPose.p.v1 = denormalize(outputActivations[i][startIndex++], 0.0, maxRange);
		// currentPredictedEfPose.p.v2 = denormalize(outputActivations[i][startIndex++], 0.0, maxRange);
		// currentPredictedEfPose.p.v3 = denormalize(outputActivations[i][startIndex++], 0.0, maxRange);
		// Real efRoll, efPitch, efYaw;
		// efRoll = denormalize(outputActivations[i][startIndex++], -REAL_PI, REAL_PI);
		// efPitch = denormalize(outputActivations[i][startIndex++], -REAL_PI, REAL_PI);
 		// efYaw = denormalize(outputActivations[i][startIndex++], -REAL_PI, REAL_PI);
		// currentPredictedEfPose.R.fromEuler(efRoll, efPitch, efYaw);
		// currentPredictedEfSeq.push_back (currentPredictedEfPose);
		
		//extract polyflap Pose
		currentPredictedPfPose.p.v1 = denormalize(outputActivations[i][startIndex++], 0.0, maxRange);
		currentPredictedPfPose.p.v2 = denormalize(outputActivations[i][startIndex++], 0.0, maxRange);
		currentPredictedPfPose.p.v3 = denormalize(outputActivations[i][startIndex++], minZ, maxRange);
		Real pfRoll, pfPitch, pfYaw;
		pfRoll = denormalize(outputActivations[i][startIndex++], -REAL_PI, REAL_PI);
		pfPitch = denormalize(outputActivations[i][startIndex++], -REAL_PI, REAL_PI);
 		pfYaw = denormalize(outputActivations[i][startIndex++], -REAL_PI, REAL_PI);
		currentPredictedPfPose.R.fromEuler(pfRoll, pfPitch, pfYaw);
		currentPredictedPfSeq.push_back (currentPredictedPfPose);
	}
}

//------------------------------------------------------------------------------
void ActiveLearnScenario::load_current_trainSeq (int inputSize, int outputSize) {
	if (trainSeq)
		delete trainSeq;
	trainSeq = new rnnlib::DataSequence (inputSize, outputSize);
	vector<int> inputShape, targetShape;
	inputShape.push_back (learningData.currentSeq.size() - 1);
	targetShape.push_back (learningData.currentSeq.size() - 1);
	trainSeq->inputs.reshape(inputShape);
	trainSeq->targetPatterns.reshape(targetShape);
	load_sequence_basis (trainSeq->inputs.data, trainSeq->targetPatterns.data, learningData.currentSeq);
	//load_sequence_Markov (trainSeq->inputs.data, trainSeq->targetPatterns.data, learningData.currentSeq);

}

//------------------------------------------------------------------------------

///
///Find the appropriate region index according to the given sensorimotor context
///
int ActiveLearnScenario::get_SMRegion (const FeatureVector& sMContext) {
	for (RegionsMap::const_iterator regionIter = regions.begin(); regionIter != regions.end(); regionIter) {
		bool wrongCuttingValue = false;
		SMRegion currentRegion = regionIter->second;
		assert (sMContext.size() == currentRegion.sMContextSize);
		for (int i=0; i<currentRegion.sMContextSize; i++) {
			if ( currentRegion.minValuesSMVector[i] > sMContext[i] ||
			     currentRegion.maxValuesSMVector[i] < sMContext[i]) {
				wrongCuttingValue = true;
				break;
			}
		}
		if (wrongCuttingValue) continue; else return currentRegion.index;
	}
	return -1;
}

//------------------------------------------------------------------------------

///
///Update learners according to a sensorimotor region splitting criterion
///
void ActiveLearnScenario::update_learners () {


	int currentRegion = get_SMRegion (learningData.currentMotorCommandVector);
	
}

///
///variance calculation of a vector
///
double ActiveLearnScenario::variance (const DataSet& data, int sMContextSize) {
	vector<double> means;
	vector<double> variances;

	assert (data.size() >= 2);
	double featVectorSize;
	assert ((featVectorSize = data[0][1].size()) > 0);
	means.resize (featVectorSize, 0.0);
	variances.resize (featVectorSize, 0.0);
	int numInstances = 0;
	
	for (int i=0; i<data.size(); i++)
		for (int j=1; j<data[i].size(); j++) {
			assert (data[i][j].size() == featVectorSize);
			numInstances++;
			for (int k=0; k<featVectorSize; k++)
				means[k] += data[i][j][k];
	}
	for (int k=0; k<featVectorSize; k++)
		means[k] /= numInstances;

	for (int k=0; k<featVectorSize; k++) {
		for (int i=0; i<data.size(); i++)
			for (int j=1; j<data[i].size(); j++) {
				double meanDiff = data[i][j][k] - means[k];
				variances[k] += (meanDiff * meanDiff);
			}
		variances[k] /= numInstances;
	}
	double variance = 0.0;
	for (int k=0; k<featVectorSize; k++)
		variance += variances[k];
	variance /= featVectorSize;

	return variance;
}

///
///minimality criterion for splitting a sensorimotor region
///
double ActiveLearnScenario::evaluate_minimality (const DataSet& firstSplittingSet, const DataSet& secondSplittingSet, int sMContextSize) {
	//calculate variance of sequence S(t+1),...S(t+n) for each splitting set
	double varianceLastSC1stSet = variance (firstSplittingSet, sMContextSize);
	double varianceLastSC2ndSet = variance (secondSplittingSet, sMContextSize);
	return (firstSplittingSet.size()  * varianceLastSC1stSet +
		secondSplittingSet.size() * varianceLastSC2ndSet);
}


///
///partition of regions according to variance of dataset instances
///
void ActiveLearnScenario::split_region (int regionIdx) {
	double minimalQuantity = 1e6;
	DataSet firstSplittingSet;
	DataSet secondSplittingSet;
	double cuttingValue = -1.0;
	int cuttingIdx = -1;
	SMRegion& region = regions[regionIdx];
	for (int i=0; i<region.sMContextSize; i++) {
		for (double j=-0.999; j<1.0; j=j+0.001) {
			DataSet firstSplittingSetTry;
			DataSet secondSplittingSetTry;
			for (int k=0; k < region.data.size(); k++) {
				FeatureVector currentSMContext = region.data[k][0];
				//cutting criterion for splitting
				if (currentSMContext[i] < j)
					firstSplittingSetTry.push_back (region.data[k]);
				else
					secondSplittingSetTry.push_back (region.data[k]);
			}
			double quantityEval = evaluate_minimality (firstSplittingSetTry, secondSplittingSetTry, region.sMContextSize);
			if (quantityEval < minimalQuantity) {
				minimalQuantity = quantityEval;
				firstSplittingSet = firstSplittingSetTry;
				secondSplittingSet = secondSplittingSetTry;
				cuttingValue = j;
				cuttingIdx = i;
			}
		}
	}
	assert (firstSplittingSet.size () > 0);
	assert (secondSplittingSet.size() > 0);
	assert (cuttingValue != -1.0);
	assert (cuttingIdx != -1);
	SMRegion firstRegion (region, ++regionsCount, cuttingValue, cuttingIdx, true);
	SMRegion secondRegion (region, ++regionsCount, cuttingValue, cuttingIdx, false);
	regions[firstRegion.index] = firstRegion;
	regions[secondRegion.index] = secondRegion;
	firstRegion.data = firstSplittingSet;
	secondRegion.data = secondSplittingSet;
	firstRegion.learner = region.learner;
	secondRegion.learner = region.learner;
	
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
