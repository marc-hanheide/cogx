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
		CriticalSectionWrapper csw(cs);
		if (object == NULL) {
			return;
		}

		LearningData::Chunk chunk;
		write_chunk (chunk);

// 		learningData.data.push_back(chunk);
		LearningData::write_chunk_to_featvector (chunk.featureVector, chunk, normalize<Real>, learningData.coordLimits, _end_effector_pos | _effector_pos | _object );
		
		learningData.currentChunkSeq.push_back (chunk);

		currentPfRoll = chunk.object.obRoll;
		currentPfY = chunk.object.objectPose.p.v2;

		if (learningData.currentChunkSeq.size() > 1) {
			trainSeq = LearningData::load_NNtrainSeq (learningData.currentChunkSeq, featureSelectionMethod, normalize<float>, learningData.coordLimits);
				
			currentRegion->learner.feed_forward (*trainSeq);
		
			get_pfefSeq_from_outputActivations (currentRegion->learner.net->outputLayer->outputActivations, /*learningData.currentMotorCommandVector.size() - 3*/0, /*desc.maxRange, desc.minZ,*/ learningData.currentPredictedPfSeq, learningData.currentPredictedEfSeq, denormalize<Real>);
		}	
	}
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
LearningData::Chunk::Action ActiveLearnScenario::get_action_maxLearningProgress (const ActionsVector& candidateActions) {
	double maxLearningProgress = -1e6;
	int index = -1;
	for (int i=0; i < candidateActions.size(); i++) {
		SMRegion contextRegion = regions[SMRegion::get_SMRegion (regions, candidateActions[i].featureVector)];
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
				LearningData::Chunk chunk_cand;
				int startPosition = availableStartingPositions[floor(randomG.nextUniform (0.0,Real(availableStartingPositions.size())))];
				//action.pushDuration = floor (randomG.nextUniform (3.0, 6.0));
				chunk_cand.action.pushDuration = 3.0;
				chunk_cand.action.horizontalAngle = choose_angle(60.0, 120.0);
				Vec3 pos;
				init_positionT (pos);
				set_coordinates_into_target(startPosition, pos, polyflapNormalVec, polyflapOrthogonalVec, desc.dist, desc.side, desc.center, desc.top, desc.over);
				chunk_cand.action.effectorPose.p = pos;
				chunk_cand.action.efRoll = orientationT.v1;
				chunk_cand.action.efPitch = orientationT.v2;
				chunk_cand.action.efYaw = orientationT.v3;
				Vec3 polyflapCenterNormalVec = computeNormalVector(pos, Vec3 (polyflapPosition.v1, polyflapPosition.v2, desc.polyflapDimensions.v2*0.5));
				Vec3 polyflapCenterOrthogonalVec = computeOrthogonalVec(polyflapCenterNormalVec);
				fromCartesianPose(chunk_cand.action.endEffectorPose, pos, orientationT);
				set_movement_angle(chunk_cand.action.horizontalAngle, chunk_cand.action.endEffectorPose, desc.distance, polyflapCenterNormalVec, polyflapCenterOrthogonalVec);
				chunk_cand.action.endEffectorPose.R.toEuler (chunk_cand.action.endEfRoll, chunk_cand.action.endEfPitch, chunk_cand.action.endEfYaw);

				LearningData::write_chunk_to_featvector (chunk_cand.action.featureVector, chunk_cand, normalize<Real>, learningData.coordLimits, _end_effector_pos | _effector_pos /*| _action_params*/ );
				candidateActions.push_back (chunk_cand.action);
			}

			// current chosen Action
			LearningData::Chunk::Action chosenAction = get_action_maxLearningProgress(candidateActions);
			
			this->pushDuration = chosenAction.pushDuration;
			this->horizontalAngle = chosenAction.horizontalAngle;
			this->positionT = chosenAction.effectorPose.p;
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
		LearningData::write_dataset (dataFileName, data, learningData.coordLimits);
		string stpFileName = dataFileName + ".stp";
		ofstream writeToFile (stpFileName.c_str(), ios::out | ios::binary);
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

	//Set first sensorimotor region and corresponding learner
	regionsCount = 0;
	SMRegion firstRegion (regionsCount, motorContextSize, splittingCriterion1);
	regions[regionsCount] = firstRegion;
	if (netconfigFileName.empty())
		regions[regionsCount].learner.init (motorVectorSize + learningData.efVectorSize + learningData.pfVectorSize,  learningData.pfVectorSize);
	else
		regions[regionsCount].learner.init (motorVectorSize + learningData.efVectorSize + learningData.pfVectorSize, learningData.pfVectorSize, netconfigFileName);
	currentRegion = &regions[regionsCount];

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

		//compute direction and other features of trajectory
		set_up_movement();

		//move the finger along described experiment trajectory
		move_finger();
		
		//update context region
		update_currentRegion ();

		//write sequence into dataset
		data.push_back(learningData.currentChunkSeq);
		currentRegion->data.push_back(learningData.currentChunkSeq);

		//update RNN learner with current sequence
		{
			CriticalSectionWrapper csw (cs);
			update_learners ();
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

	int regionIdx = SMRegion::get_SMRegion (regions, learningData.currentChunkSeq[0].featureVector);
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
double ActiveLearnScenario::variance (const LearningData::DataSet& data, int sMContextSize) {
	vector<double> means;
	vector<double> variances;

	if (data.size() == 0)
		return 0.0;
	
	assert (data[0].size() >= 2);
	double featVectorSize;
	assert ((featVectorSize = data[0][1].featureVector.size()) > 0);
	means.resize (featVectorSize, 0.0);
	variances.resize (featVectorSize, 0.0);
	int numInstances = 0;
	
	for (int i=0; i<data.size(); i++)
		for (int j=1; j<data[i].size(); j++) {
			assert (data[i][j].featureVector.size() == featVectorSize);
			numInstances++;
			for (int k=0; k<featVectorSize; k++)
				means[k] += data[i][j].featureVector[k];
	}
	for (int k=0; k<featVectorSize; k++)
		means[k] /= numInstances;

	for (int k=0; k<featVectorSize; k++) {
		for (int i=0; i<data.size(); i++)
			for (int j=1; j<data[i].size(); j++) {
				double meanDiff = data[i][j].featureVector[k] - means[k];
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
double ActiveLearnScenario::evaluate_minimality (const LearningData::DataSet& firstSplittingSet, const LearningData::DataSet& secondSplittingSet, int sMContextSize) {
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
	LearningData::DataSet firstSplittingSet;
	LearningData::DataSet secondSplittingSet;
	double cuttingValue = -1.0;
	int cuttingIdx = -1;
	//for (int i=0; i<region.sMContextSize; i++) {
	for (int i=0; i<motorContextSize; i++) {
		for (double j=-0.999; j<1.0; j=j+0.001) {
			LearningData::DataSet firstSplittingSetTry;
			LearningData::DataSet secondSplittingSetTry;
			for (int k=0; k < region.data.size(); k++) {
				FeatureVector currentSMContext = region.data[k][0].featureVector;
				//cutting criterion for splitting
				if (currentSMContext[i] < j)
					firstSplittingSetTry.push_back (region.data[k]);
				else
					secondSplittingSetTry.push_back (region.data[k]);
			}
			double quantityEval = evaluate_minimality (firstSplittingSetTry, secondSplittingSetTry, motorContextSize);
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

	regions[firstRegion.index].learner.init (motorVectorSize + learningData.efVectorSize + learningData.pfVectorSize,  learningData.pfVectorSize, region.learner.net->weightContainer);
	regions[secondRegion.index].learner.init (motorVectorSize + learningData.efVectorSize + learningData.pfVectorSize,  learningData.pfVectorSize, region.learner.net->weightContainer);

	assert (regions.erase (region.index) == 1);
	
	update_currentRegion ();
	cout << "region split..." << endl;


	
}


void ActiveLearnScenario::init(boost::program_options::variables_map vm) {
	
	Scenario::init(vm);

	splittingCriterion1 = numSequences / startingPositionsCount;

	if (splittingCriterion1 < 5)
		splittingCriterion1 = 4;

	cout << "splitting criterion: " << splittingCriterion1 << endl;
	
	netconfigFileName = "";

	if (vm.count("netconfigFileName")) {
		netconfigFileName = vm["netconfigFileName"].as<string>();
	}

	string fSMethod;
	if (vm.count("featureSelectionMethod")) {
		fSMethod = vm["featureSelectionMethod"].as<string>();
	}

	if (fSMethod == "basis" ) {
		featureSelectionMethod = _basis;
		motorVectorSize = LearningData::motorVectorSizeBasis;
	}
	else if (fSMethod == "markov" || fSMethod == "" ) {
		featureSelectionMethod = _markov;
		motorVectorSize = LearningData::motorVectorSizeMarkov;
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
	XMLData((Scenario::Desc&)desc, xmlcontext(), context());

	ActiveLearnScenario *pActiveLearnScenario = dynamic_cast<ActiveLearnScenario*>(scene()->createObject(desc)); // throws

	if (pActiveLearnScenario == NULL) {
		context()->getMessageStream()->write(Message::LEVEL_CRIT, "ActivePushingApplication::run(): unable to cast to ActiveLearnScenario");
		return;
	}




	// Random number generator seed
	context()->getMessageStream()->write(Message::LEVEL_INFO, "Random number generator seed %d", context()->getRandSeed()._U32[0]);
	
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
