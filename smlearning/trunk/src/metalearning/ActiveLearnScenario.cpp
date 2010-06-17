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
		
		write_feature_vector_into_current_sequence (currentFeatureVector);

		// int size = learningData.currentSeq.size();
		// for (int i=0; i<learningData.currentSeq[size-1].size(); i++) 
		// 	cout << learningData.currentSeq[size-1][i] << " ";
		// cout << endl;

		currentPfRoll = chunk.obRoll;
		currentPfY = chunk.objectPose.p.v2;

		//trainSeq = load_trainSeq (learningData.currentSeq, currentRegion->learner.header->inputSize, currentRegion->learner.header->outputSize, motorVectorSize, efVectorSize);
		trainSeq = load_trainSeq (learningData.currentSeq, data.second);
		currentRegion->learner.feed_forward (*trainSeq);
		//get_pfefSeq_from_outputActivations (learner.net->outputLayer->outputActivations, learningData.currentMotorCommandVector.size(), desc.maxRange, desc.minZ, learningData.currentPredictedPfSeq, learningData.currentPredictedEfSeq);
		get_pfefSeq_from_outputActivations (currentRegion->learner.net->outputLayer->outputActivations, /*learningData.currentMotorCommandVector.size() - 3*/0, /*desc.maxRange, desc.minZ,*/ learningData.currentPredictedPfSeq, learningData.currentPredictedEfSeq);
	
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
///Set the lenght of experiment (number of sequences) and if given, the starting position.
///Calculate the splittingCriterion1 constant according to nr. of sequences. 
///Get previously trained neural network if given.
void ActiveLearnScenario::setup_loop(int argc, char* argv[]) {
	Scenario::setup_loop (argc, argv);

	splittingCriterion1 = numSequences / startingPositionsCount;

	if (splittingCriterion1 < 5)
		splittingCriterion1 = 4;

	cout << "splitting criterion: " << splittingCriterion1 << endl;

	if (argc > 5)
		netconfigFileName = string (argv[5]);

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
pair<FeatureVector, ActiveLearnScenario::Action> ActiveLearnScenario::get_action_maxLearningProgress (const ActionsVector& candidateActions) {
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
				Action action;
				action.startPosition = availableStartingPositions[floor(randomG.nextUniform (0.0,Real(availableStartingPositions.size())))];
				//action.startPosition = floor(randomG.nextUniform (1.0, 25.0));
				action.speed = floor (randomG.nextUniform (3.0, 6.0));
				//action.speed = 3.0;
				action.horizontalAngle = choose_angle(60.0, 120.0, "cont");
				FeatureVector motorVector;
				Vec3 pos;
				init_positionT (pos);
				set_coordinates_into_target(action.startPosition, pos, polyflapNormalVec, polyflapOrthogonalVec, desc.dist, desc.side, desc.center, desc.top, desc.over);
				write_finger_pos_and_or (motorVector, pos);
				write_finger_speed_and_angle (motorVector, action.speed, action.horizontalAngle);
				candidateActions.push_back (make_pair (motorVector, action));
			}

			pair<FeatureVector, Action> chosenAction = get_action_maxLearningProgress(candidateActions);
			
			this->startPosition = chosenAction.second.startPosition;
			this->speed = chosenAction.second.speed;
			this->horizontalAngle = chosenAction.second.horizontalAngle;

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
	//string basename = writedown_collected_data(data);
	if (final) {
		write_dataset (dataFileName, data);
		string stpFileName = dataFileName + ".stp";
		ofstream writeToFile (stpFileName.c_str(), ios::out | ios::binary);
		//write_intvector(writeToFile, usedStartingPositions);
		write_realvector(writeToFile, usedStartingPositions);
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

	//define numSequences and startingPosition
	setup_loop(argc, argv);

	regionsCount = 0;
	SMRegion firstRegion (regionsCount, /*motorVectorSize, */splittingCriterion1);
	regions[regionsCount] = firstRegion;
	if (netconfigFileName.empty())
		regions[regionsCount].learner.init (motorVectorSize + featureVectorSize,  pfVectorSize);
	else
		regions[regionsCount].learner.init (motorVectorSize + featureVectorSize, pfVectorSize, netconfigFileName);

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
		write_motor_vector_into_current_sequence();

		//update context region
		update_currentRegion ();

		//compute direction and other features of trajectory
		set_up_movement();

		//move the finger along described experiment trajectory
		move_finger();

		//write sequence into dataset
		write_current_sequence_into_dataset(data.first);
		write_current_sequence_into_dataset(currentRegion->data);


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
	predictedEfPose.p.v1 = denormalize(outputActivations[finalActIndex][startIndex++], desc.minX, desc.maxX);
	predictedEfPose.p.v2 = denormalize(outputActivations[finalActIndex][startIndex++], desc.minY, desc.maxY);
	predictedEfPose.p.v3 = denormalize(outputActivations[finalActIndex][startIndex++], desc.minZ, desc.maxZ);
	Real efRoll, efPitch, efYaw;
	efRoll = denormalize(outputActivations[finalActIndex][startIndex++], -REAL_PI, REAL_PI);
	efPitch = denormalize(outputActivations[finalActIndex][startIndex++], -REAL_PI, REAL_PI);
	efYaw = denormalize(outputActivations[finalActIndex][startIndex++], -REAL_PI, REAL_PI);
	predictedEfPose.R.fromEuler (efRoll, efPitch, efYaw);

	//extract polyflap Pose
	predictedPfPose.p.v1 = denormalize(outputActivations[finalActIndex][startIndex++], desc.minX, desc.maxX);
	predictedPfPose.p.v2 = denormalize(outputActivations[finalActIndex][startIndex++], desc.minY, desc.maxY);
	predictedPfPose.p.v3 = denormalize(outputActivations[finalActIndex][startIndex++], desc.minZ, desc.maxY);
	Real pfRoll, pfPitch, pfYaw;
	pfRoll = denormalize(outputActivations[finalActIndex][startIndex++], -REAL_PI, REAL_PI);
	pfPitch = denormalize(outputActivations[finalActIndex][startIndex++], -REAL_PI, REAL_PI);
	pfYaw = denormalize(outputActivations[finalActIndex][startIndex++], -REAL_PI, REAL_PI);
	predictedPfPose.R.fromEuler(pfRoll, pfPitch, pfYaw);



}


void ActiveLearnScenario::get_pfefSeq_from_outputActivations (const rnnlib::SeqBuffer<double>& outputActivations, int sIndex/*, Real maxRange, Real minZ*/, vector<golem::Mat34>& currentPredictedPfSeq, vector<golem::Mat34>& currentPredictedEfSeq) {

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
		currentPredictedPfPose.p.v1 = denormalize(outputActivations[i][startIndex++], desc.minX, desc.maxX);
		currentPredictedPfPose.p.v2 = denormalize(outputActivations[i][startIndex++], desc.minY, desc.maxY);
		currentPredictedPfPose.p.v3 = denormalize(outputActivations[i][startIndex++], desc.minZ, desc.maxZ);
		Real pfRoll, pfPitch, pfYaw;
		pfRoll = denormalize(outputActivations[i][startIndex++], -REAL_PI, REAL_PI);
		pfPitch = denormalize(outputActivations[i][startIndex++], -REAL_PI, REAL_PI);
 		pfYaw = denormalize(outputActivations[i][startIndex++], -REAL_PI, REAL_PI);
		currentPredictedPfPose.R.fromEuler(pfRoll, pfPitch, pfYaw);
		currentPredictedPfSeq.push_back (currentPredictedPfPose);
	}
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
//cout << "--------------------------------------------------------" << regions.size() << endl;
	for ( it=regions.begin() ; it != regions.end(); it++ ) {
		if (&(it->second) != currentRegion) {
//cout << "--------------------------------------------------------" << endl;
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
	for (int i=0; i<SMRegion::motorVectorSize; i++) {
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
			double quantityEval = evaluate_minimality (firstSplittingSetTry, secondSplittingSetTry, motorVectorSize);
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

	regions[firstRegion.index].learner.init (motorVectorSize + featureVectorSize,  pfVectorSize, region.learner.net->weightContainer);
	regions[secondRegion.index].learner.init (motorVectorSize + featureVectorSize,  pfVectorSize, region.learner.net->weightContainer);

	assert (regions.erase (region.index) == 1);
	
	update_currentRegion ();
	cout << "region split..." << endl;


	
}


void ActiveLearnScenario::init(map<string, string> m) {
	
	Scenario::init(m);

	netconfigFileName = "";

	if (m.count("netconfigFileName")) {
		netconfigFileName = m["netconfigFileName"];
	}

	
}



//------------------------------------------------------------------------------


namespace po = boost::program_options;
using namespace boost;



void ActivePushingApplication::define_program_options_desc() {

try {

	PushingApplication::define_program_options_desc();

	prgOptDesc.add_options()
		("netconfigFileName,N", po::value<string>(), "name of the netconfig file")
	;
	

}catch(std::exception& e) {
	cerr << "error: " << e.what() << "\n";
}catch(...) {
	cerr << "Exception of unknown type!\n";

}


}





void ActivePushingApplication::read_program_options(int argc, char *argv[]) {

try {
	PushingApplication::read_program_options(argc, argv);

	if (vm.count("netconfigFileName")) {
		arguments["netconfigFileName"]=vm["netconfigFileName"].as<string>();
	}

}catch(std::exception& e) {
	cerr << "error: " << e.what() << "\n";
}catch(...) {
	cerr << "Exception of unknown type!\n";

}

}





int ActivePushingApplication::main(int argc, char *argv[]) {

	PushingApplication::main(argc, argv);

 //try {
	
	//po::options_description desc("Allowed options");
	//prgOptDesc.add_options()
/*		("help", "produce help message")
//		("argument1, a1", po::value<int>(&opt)->default_value(10), "first argument")
//		("argument2, a2", po::value<string>(), "first argument")
//		("include-path,I", po::value< vector<string> >(),"include path")
//		("input-file", po::value< vector<string> >(), "input file")
		("numSequences,S", po::value<string>(), "number of sequences")
		("startingPosition,P", po::value<string>(), "only starting position to use")
		("storeLabels,L", po::value<string>(), "use true or false as argument")
*/	//	("netconfigFileName,N", po::value<string>(), "name of the netconfig file")
/*		("configFile", po::value<string>(), "name of the xml config file")
*/	;

/*	po::positional_options_description p;
	p.add("configFile", -1);

	po::variables_map vm;
//	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
	po::notify(vm);

	if (vm.count("help")) {
		cout << desc << endl;
		return 1;
	}

	if (vm.count("numSequences")) {
		arguments["numSequences"]=vm["numSequences"].as<string>();
	}

	if (vm.count("startingPosition")) {
		arguments["startingPosition"]=vm["startingPosition"].as<string>();
	}

	if (vm.count("storeLabels")) {
		arguments["storeLabels"]=vm["storeLabels"].as<string>();
	}

*/	//if (vm.count("netconfigFileName")) {
	//	arguments["netconfigFileName"]=vm["netconfigFileName"].as<string>();
	//}

/*	if (vm.count("configFile")) {
		char* arr[2];
		//char* arr [] = (char *){argv[0], vm["configFile"].as<string>().c_str()};
		arr[0] = argv[0];
		arr[1] =  vm["configFile"].as<string>().c_str();
		Application::main(2, arr);
	} else {
		char* arr [] = {argv[0]};
		Application::main(1, arr);
	}


}catch(std::exception& e) {
	cerr << "error: " << e.what() << "\n";
}catch(...) {
	cerr << "Exception of unknown type!\n";
}

return 1;
*/
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
		pActiveLearnScenario->init(arguments);
		pActiveLearnScenario->run(argc, argv);
	}
	catch (const ActiveLearnScenario::Interrupted&) {
	}
	
	scene()->releaseObject(*pActiveLearnScenario);
}


};
