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
		if (storeLabels) add_label (currentFeatureVector, chunk);
		
		write_feature_vector_into_current_sequence (currentFeatureVector, chunk);

		// int size = learningData.currentSeq.size();
		// for (int i=0; i<learningData.currentSeq[size-1].size(); i++) 
		// 	cout << learningData.currentSeq[size-1][i] << " ";
		// cout << endl;

		currentPfRoll = chunk.obRoll;
		currentPfY = chunk.objectPose.p.v2;

		//trainSeq = load_trainSeq (learningData.currentSeq, currentRegion->learner.header->inputSize, currentRegion->learner.header->outputSize, motorVectorSize, efVectorSize);
		if (learningData._currentSeq.second.size() > 1) {
			if (featureSelectionMethod == "basis") 
				trainSeq = LearningData::load_NNtrainSeq (learningData._currentSeq, LearningData::load_NNsequence_basis<float(*)(float const&, float const&, float const&)>, normalize<float>, learningData.coordLimits);
			else if (featureSelectionMethod == "markov") 
				trainSeq = LearningData::load_NNtrainSeq (learningData._currentSeq, LearningData::load_NNsequence_markov<float(*)(float const&, float const&, float const&)>, normalize<float>, learningData.coordLimits);
				
				
			//trainSeq = load_trainSeq (learningData.currentSeq, data.second);
			currentRegion->learner.feed_forward (*trainSeq);
		
			//get_pfefSeq_from_outputActivations (learner.net->outputLayer->outputActivations, learningData.currentMotorCommandVector.size(), desc.maxRange, desc.minZ, learningData.currentPredictedPfSeq, learningData.currentPredictedEfSeq);
			get_pfefSeq_from_outputActivations (currentRegion->learner.net->outputLayer->outputActivations, /*learningData.currentMotorCommandVector.size() - 3*/0, /*desc.maxRange, desc.minZ,*/ learningData.currentPredictedPfSeq, learningData.currentPredictedEfSeq, denormalize<Real>);
		}
	
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
	
	if (iteration == 0) objectLocalBounds = object->getLocalBoundsSeq();
	
	currentPfY = object->getPose().p.v2;

	//computes needed information about the polyflap
	compute_vectors();
}

///
///get the action that maximizes learning progress
///
pair<FeatureVector, LearningData::MotorCommand> ActiveLearnScenario::get_action_maxLearningProgress (const ActionsVector& candidateActions) {
	double maxLearningProgress = -1e6;
	int index = -1;
	for (int i=0; i < candidateActions.size(); i++) {
		SMRegion contextRegion = regions[SMRegion::get_SMRegion (regions, candidateActions[i].first)];
		double learningProgress;
		if (contextRegion.learningProgressHistory.size() > 0)
			learningProgress = contextRegion.learningProgressHistory.back();
		else
			learningProgress = -1e5;
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
				LearningData::MotorCommand action;
				int startPosition = availableStartingPositions[floor(randomG.nextUniform (0.0,Real(availableStartingPositions.size())))];
				//action.pushDuration = floor (randomG.nextUniform (3.0, 6.0));
				action.pushDuration = 3.0;
				action.horizontalAngle = choose_angle(60.0, 120.0, "cont");
				FeatureVector motorVector;
				Vec3 pos;
				init_positionT (pos);
				set_coordinates_into_target(/*action.*/startPosition, pos, polyflapNormalVec, polyflapOrthogonalVec, desc.dist, desc.side, desc.center, desc.top, desc.over);
				write_finger_pos_and_or (motorVector, action, pos);
				write_finger_speed_and_angle (motorVector, action, action.pushDuration, action.horizontalAngle);
				candidateActions.push_back (make_pair (motorVector, action));
			}

			pair<FeatureVector, LearningData::MotorCommand> chosenAction = get_action_maxLearningProgress(candidateActions);
			
			this->pushDuration = chosenAction.second.pushDuration;
			this->horizontalAngle = chosenAction.second.horizontalAngle;
			this->positionT = chosenAction.second.initEfPosition;
			this->startPosition = positionsT[this->positionT];

			usedStartingPositions.push_back(startPosition);
		}
	}
	else
		Scenario::choose_action ();
	
}


void ActiveLearnScenario::write_data (bool final){
	//cout << "ALS: writing data..." << endl;
	/////////////////////////////////////////////////
	//writing the dataset into binary file
	if (final) {
		write_dataset (dataFileName, data);
		string stpFileName = dataFileName + ".stp";
		ofstream writeToFile (stpFileName.c_str(), ios::out | ios::binary);
		//write_intvector(writeToFile, usedStartingPositions);
		//write_realvector(writeToFile, usedStartingPositions);
		write_vector<double>(writeToFile, usedStartingPositions);
	}

	for (SMRegion::RegionsMap::iterator regionIter = regions.begin(); regionIter != regions.end(); regionIter++) {
		stringstream name;
		name << dataFileName << "_region" << regionIter->first;
		if (final)
			name << "_final";
		if (!regionIter->second.write_data (name.str()))
			cerr << "Saving region " << regionIter->first << " data was unsuccesful!" << endl;
	}
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

	//set: random seed, tmDeltaAsync; get initial config
	first_init();

	//setup and move to home position; define fingertip orientation
	setup_home();

	positionsT = get_canonical_positions (desc);

	regionsCount = 0;
	SMRegion firstRegion (regionsCount, /*motorVectorSize, */splittingCriterion1);
	regions[regionsCount] = firstRegion;
	if (netconfigFileName.empty())
		regions[regionsCount].learner.init (/*learningData.motorVectorSize*/2 + learningData.efVectorSize + learningData.pfVectorSize,  learningData.pfVectorSize);
	else
		regions[regionsCount].learner.init (/*learningData.motorVectorSize*/2 + learningData.efVectorSize + learningData.pfVectorSize, learningData.pfVectorSize, netconfigFileName);

	plotApp->init (regionsCount+1, firstRegion.smoothing + firstRegion.timewindow);
	plotApp->resize(640,480);
	
	plotApp->show();


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
			//choose motor command
			choose_action ();
		}

		//compute coordinates of start position
		prepare_target();

		//move the finger to the beginnign of experiment trajectory
		send_position(target, ReacPlanner::ACTION_GLOBAL);
		
		//create feature sequence and vector
		init_writing();

		//write initial position and orientation of the finger
		write_finger_pos_and_or(learningData.currentMotorCommandVector, learningData.currentMotorCommand, positionT);

		//write chosen speed and angle of the finger experiment trajectory	
		write_finger_speed_and_angle(learningData.currentMotorCommandVector, learningData.currentMotorCommand, pushDuration, horizontalAngle);
		//add motor vector to the sequence
		write_motor_vector_into_current_sequence();

		//update context region
		update_currentRegion ();

		//compute direction and other features of trajectory
		set_up_movement();

		//move the finger along described experiment trajectory
		move_finger();

		//write sequence into dataset
		write_current_sequence_into_dataset(data.first, _data);
		write_current_sequence_into_dataset(currentRegion->data, currentRegion->_data);


		//update RNN learner with current sequence
		{
			CriticalSectionWrapper csw (cs);
			//update_learners ();
			int windowSize = currentRegion->smoothing + currentRegion->timewindow;
			vector<double> learnProgData;
			vector<double> errorData;
			if (currentRegion->learningProgressHistory.size() > windowSize) {
				learnProgData = vector<double> (currentRegion->learningProgressHistory.end() - windowSize, currentRegion->learningProgressHistory.end());
				errorData = vector<double> (currentRegion->errorsHistory.end() - windowSize, currentRegion->errorsHistory.end());
			}
			else {
				learnProgData = currentRegion->learningProgressHistory;
				errorData = currentRegion->errorsHistory;
			}
			assert (learnProgData.size() <= windowSize);
			assert (errorData.size() <= windowSize);
			plotApp->updateData(currentRegion->index, learnProgData, errorData);
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

		//move finger to initial position
		move_finger_home ();

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
	write_data (true);


	} catch (const Ice::Exception& ex) {
		std::cerr << ex << std::endl;
	} catch (const char* msg) {
		std::cerr << msg << std::endl;
	}
	if (ic)
		ic->destroy();
	
}

//------------------------------------------------------------------------------


golem::Mat34  ActiveLearnScenario::get_pfefPose_from_outputActivations (const rnnlib::SeqBuffer<double>& outputActivations, int startIndex/*, Real maxRange, Real minZ*/, golem::Mat34& predictedPfPose, golem::Mat34& predictedEfPose) {
	int finalActIndex = outputActivations.shape[0] - 1;
// 	cout << finalActIndex << endl;
	int outputsize = outputActivations.shape[1];
// 	cout << outputsize << endl;

// 	cout << startIndex << endl;
	assert (startIndex < outputsize);

	//extract effector Pose
	predictedEfPose.p.v1 = denormalize(outputActivations[finalActIndex][startIndex++], desc.coordLimits.minX, desc.coordLimits.maxX);
	predictedEfPose.p.v2 = denormalize(outputActivations[finalActIndex][startIndex++], desc.coordLimits.minY, desc.coordLimits.maxY);
	predictedEfPose.p.v3 = denormalize(outputActivations[finalActIndex][startIndex++], desc.coordLimits.minZ, desc.coordLimits.maxZ);
	Real efRoll, efPitch, efYaw;
	efRoll = denormalize(outputActivations[finalActIndex][startIndex++], -REAL_PI, REAL_PI);
	efPitch = denormalize(outputActivations[finalActIndex][startIndex++], -REAL_PI, REAL_PI);
	efYaw = denormalize(outputActivations[finalActIndex][startIndex++], -REAL_PI, REAL_PI);
	predictedEfPose.R.fromEuler (efRoll, efPitch, efYaw);

	//extract polyflap Pose
	predictedPfPose.p.v1 = denormalize(outputActivations[finalActIndex][startIndex++], desc.coordLimits.minX, desc.coordLimits.maxX);
	predictedPfPose.p.v2 = denormalize(outputActivations[finalActIndex][startIndex++], desc.coordLimits.minY, desc.coordLimits.maxY);
	predictedPfPose.p.v3 = denormalize(outputActivations[finalActIndex][startIndex++], desc.coordLimits.minZ, desc.coordLimits.maxY);
	Real pfRoll, pfPitch, pfYaw;
	pfRoll = denormalize(outputActivations[finalActIndex][startIndex++], -REAL_PI, REAL_PI);
	pfPitch = denormalize(outputActivations[finalActIndex][startIndex++], -REAL_PI, REAL_PI);
	pfYaw = denormalize(outputActivations[finalActIndex][startIndex++], -REAL_PI, REAL_PI);
	predictedPfPose.R.fromEuler(pfRoll, pfPitch, pfYaw);



}




//------------------------------------------------------------------------------



///
///Obtain current context region pointer
///
void ActiveLearnScenario::update_currentRegion () {

	int regionIdx = SMRegion::get_SMRegion (regions, learningData.currentMotorCommandVector);
	assert (regionIdx != -1);
	currentRegion = &regions[regionIdx];

}


//------------------------------------------------------------------------------

///
///Update learners according to a sensorimotor region splitting criterion
///
void ActiveLearnScenario::update_learners () {

	
	if (currentRegion->data.size() > splittingCriterion1 && startingPosition == 0) {
		write_data ();
		split_region (*currentRegion);
	}

	//Evaluate learning progress
	cout << "Region: " << currentRegion->index << endl;
	currentRegion->update_learning_progress (*trainSeq);

	currentRegion->startingPositionsHistory.push_back (startPosition);

	SMRegion::RegionsMap::iterator it;
	for ( it=regions.begin() ; it != regions.end(); it++ ) {
		if (&(it->second) != currentRegion) {
			double lastLP = it->second.learningProgressHistory.back();
			double lastEr = it->second.errorsHistory.back();
			it->second.learningProgressHistory.push_back(lastLP);
			it->second.errorsHistory.push_back(lastEr);
			it->second.startingPositionsHistory.push_back(0);
		}
	
	}


	
	
}

///
///variance calculation of a dataset
///
double ActiveLearnScenario::variance (const DataSet& data, int sMContextSize) {
	vector<double> means;
	vector<double> variances;

	if (data.size() == 0)
		return 0.0;
	
	assert (data[0].size() >= 2);
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
void ActiveLearnScenario::split_region (SMRegion& region) {
	double minimalQuantity = 1e6;
	DataSet firstSplittingSet;
	DataSet secondSplittingSet;
	double cuttingValue = -1.0;
	int cuttingIdx = -1;
	// SMRegion& region = regions[regionIdx];
	//for (int i=0; i<region.sMContextSize; i++) {
	for (int i=0; i<LearningData::motorVectorSize; i++) {
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
			double quantityEval = evaluate_minimality (firstSplittingSetTry, secondSplittingSetTry, learningData.motorVectorSize);
			if (quantityEval < minimalQuantity) {
				minimalQuantity = quantityEval;
				firstSplittingSet = firstSplittingSetTry;
				secondSplittingSet = secondSplittingSetTry;
				cuttingValue = j;
				cuttingIdx = i;
			}
		}
	}
	//assert (firstSplittingSet.size () > 0);
	//assert (secondSplittingSet.size() > 0);
	assert (cuttingValue != -1.0);
	assert (cuttingIdx != -1);
	SMRegion firstRegion (region, ++regionsCount, cuttingValue, cuttingIdx, firstSplittingSet, true);
	SMRegion secondRegion (region, ++regionsCount, cuttingValue, cuttingIdx, secondSplittingSet, false);
	regions[firstRegion.index] = firstRegion;
	regions[secondRegion.index] = secondRegion;

	regions[firstRegion.index].learner.init (learningData.motorVectorSize + learningData.efVectorSize + learningData.pfVectorSize,  learningData.pfVectorSize, region.learner.net->weightContainer);
	regions[secondRegion.index].learner.init (learningData.motorVectorSize + learningData.efVectorSize + learningData.pfVectorSize,  learningData.pfVectorSize, region.learner.net->weightContainer);

	assert (regions.erase (region.index) == 1);
	
	update_currentRegion ();
	cout << "region split..." << endl;


	
}


//void ActiveLearnScenario::init(map<string, string> m) {
void ActiveLearnScenario::init(boost::program_options::variables_map vm) {
	
	Scenario::init(vm);

	splittingCriterion1 = numSequences / startingPositionsCount;

	if (splittingCriterion1 < 5)
		splittingCriterion1 = 4;

	cout << "splitting criterion: " << splittingCriterion1 << endl;
	
	netconfigFileName = "";

/*	if (m.count("netconfigFileName")) {
		netconfigFileName = m["netconfigFileName"];
	}
*/
	if (vm.count("netconfigFileName")) {
		netconfigFileName = vm["netconfigFileName"].as<string>();
	}

	featureSelectionMethod = "markov";

	if (vm.count("featureSelectionMethod")) {
		featureSelectionMethod = vm["featureSelectionMethod"].as<string>();
	}



	
}



//------------------------------------------------------------------------------


void ActivePushingApplication::define_program_options_desc() {

try {

	PushingApplication::define_program_options_desc();

	prgOptDesc.add_options()
		("netconfigFileName,N", po::value<string>(), "name of the netconfig file")
		("featureSelectionMethod,F", po::value<string>(), "basis/markov (feature selection method, default:markov)")
	;
	

}catch(std::exception& e) {
	cerr << "error: " << e.what() << "\n";
}catch(...) {
	cerr << "Exception of unknown type!\n";

}


}



/*

void ActivePushingApplication::read_program_options(int argc, char *argv[]) {

	try {
		PushingApplication::read_program_options(argc, argv);

		if (vm.count("netconfigFileName")) {
			arguments["netconfigFileName"]=vm["netconfigFileName"].as<string>();
		}

	} catch(std::exception& e) {
		cerr << "error: " << e.what() << "\n";
	} catch(...) {
		cerr << "Exception of unknown type!\n";

	}

}
*/



int ActivePushingApplication::main(int argc, char *argv[]) {

	//PushingApplication::main(argc, argv);

	define_program_options_desc();

	if (read_program_options(argc, argv)) {
		return 1;
	}

	start_experiment(argv);

}

void ActivePushingApplication::run(int argc, char *argv[]) {

	ActiveLearnScenario::Desc desc;
	XMLData((Scenario::Desc&)desc, xmlcontext());

	ActiveLearnScenario *pActiveLearnScenario = dynamic_cast<ActiveLearnScenario*>(scene()->createObject(desc)); // throws

	if (pActiveLearnScenario == NULL) {
		context()->getLogger()->post(Message::LEVEL_CRIT, "ActivePushingApplication::run(): unable to cast to ActiveLearnScenario");
		return;
	}




	// Random number generator seed
	context()->getLogger()->post(Message::LEVEL_INFO, "Random number generator seed %d", context()->getRandSeed()._U32[0]);
	
	try {
		//pActiveLearnScenario->init(arguments);
		pActiveLearnScenario->init(vm);
		pActiveLearnScenario->run(argc, argv);
	}
	catch (const ActiveLearnScenario::Interrupted&) {
	}
	
	scene()->releaseObject(*pActiveLearnScenario);
}


};
