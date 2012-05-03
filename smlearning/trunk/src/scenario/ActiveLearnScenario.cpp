/** @file ActiveLearnScenario.cpp
 * 
 * Program demonstrates affordances learning with LSTMs.
 * 
 * Program can be run in two modes:
 * - the first uses real Katana arm
 * - the second runs the Katana arm simulator
 * 
 * @author	Sergio Roa (DFKI)
 *
 * @version 2.0 beta
 *
 */

//------------------------------------------------------------------------------

#include <scenario/ActiveLearnScenario.h>


using namespace golem;

namespace smlearning {

ActiveLearnScenario::ActiveLearnScenario(golem::Scene &scene) : Scenario (scene)
{
	normalization = normalize<Real>;
	denormalization = denormalize<Real>;
	chosenAction = NULL;
	saveMDLHistory = false;
	currentRegion = NULL;
}

ActiveLearnScenario::~ActiveLearnScenario ()
{
}


void ActiveLearnScenario::init(boost::program_options::variables_map vm) {
	
	Scenario::init(vm);

	// splittingCriterion1 = numSequences / startingPositionsCount;
	// if (splittingCriterion1 < 5)
	//  	splittingCriterion1 = 4;

	splittingCriterion1 = vm["splitting"].as<unsigned int>();
	if (splittingCriterion1 < 2)
		splittingCriterion1 = 2;
	
	cout << "splitting criterion: " << splittingCriterion1 << endl;
	
	string fSMethod = vm["featuresel"].as<string>();

	if (fSMethod == "obpose")
		featureSelectionMethod = _obpose;
	else if (fSMethod == "efobpose")
		featureSelectionMethod = _efobpose;
	else if (fSMethod == "obpose_direction")
		featureSelectionMethod = _obpose_direction;
	else if (fSMethod == "efobpose_direction")
		featureSelectionMethod = _efobpose_direction;
	else if (fSMethod == "mcobpose_obpose_direction")
		featureSelectionMethod = _mcobpose_obpose_direction;
	else if (fSMethod == "obpose_rough_direction")
		featureSelectionMethod = _obpose_rough_direction;
	else if (fSMethod == "efobpose_rough_direction")
		featureSelectionMethod = _efobpose_rough_direction;
	else if (fSMethod == "obpose_slide_flip_tilt")
		featureSelectionMethod = _obpose_slide_flip_tilt;
	else if (fSMethod == "efobpose_slide_flip_tilt")
		featureSelectionMethod = _efobpose_slide_flip_tilt;

	if (vm.count("mdl"))
		saveMDLHistory = true;


	//Set first sensorimotor region and corresponding learner
	regionsCount = 0;
	GNGSMRegion firstRegion (regionsCount, motorContextSize);
	regions[regionsCount] = firstRegion;

	currentRegion = &regions[regionsCount];

	// create new quantizers
	// Initialize Input Quantizer
	if (featureSelectionMethod == _obpose || featureSelectionMethod == _obpose_direction || featureSelectionMethod == _obpose_label || featureSelectionMethod == _obpose_rough_direction || featureSelectionMethod == _obpose_slide_flip_tilt) //suitable for Mealy machines
		currentRegion->cryssmex.initializeInputQuantizer (learningData.motorVectorSizeMarkov + learningData.efVectorSize);
	else if (featureSelectionMethod == _efobpose || featureSelectionMethod == _efobpose_direction || featureSelectionMethod == _efobpose_label || featureSelectionMethod == _efobpose_rough_direction || featureSelectionMethod == _efobpose_slide_flip_tilt) //suitable for Moore machines
		currentRegion->cryssmex.initializeInputQuantizer (learningData.motorVectorSizeMarkov);
	else if (featureSelectionMethod == _mcobpose_obpose_direction)
		currentRegion->cryssmex.initializeInputQuantizer (2*learningData.motorVectorSizeMarkov + learningData.pfVectorSize);

	if (featureSelectionMethod == _obpose || featureSelectionMethod == _obpose_direction || featureSelectionMethod == _efobpose || featureSelectionMethod == _efobpose_direction || featureSelectionMethod == _mcobpose_obpose_direction)
		// Initialize Output Quantizer 
		currentRegion->cryssmex.initializeOutputQuantizer (learningData.pfVectorSize);
	
	GNG_Quantizer* inputQuantizer = static_cast<GNG_Quantizer*>(currentRegion->cryssmex.getInputQuantizer());
	GNG_Quantizer* outputQuantizer = static_cast<GNG_Quantizer*>(currentRegion->cryssmex.getOutputQuantizer());
	
	inputQuantizer->setMaxEpochsErrorReduction (vm["maxepochserror"].as<unsigned int>());
	inputQuantizer->setMaxEpochsMDLReduction (vm["maxepochsmdl"].as<unsigned int>());
	inputQuantizer->setModelEfficiencyConst (vm["modelefficiency"].as<double>());
	inputQuantizer->setLearningRates (vm["learningrate"].as<double>(), 0.001);
	inputQuantizer->setDataAccuracy (vm["accuracy"].as<double>());
	inputQuantizer->setTimeWindows (20, 12, 100);
	inputQuantizer->setAdaptationThreshold (0.0);
	inputQuantizer->setMaximalEdgeAge (50);
	inputQuantizer->setSamplingMode (randomly);
	// inputQuantizer->setMeanDistanceMode (arithmetic);
	inputQuantizer->setStoppingCriterion (stability);

	outputQuantizer->setMaxEpochsErrorReduction (vm["maxepochserror"].as<unsigned int>());
	outputQuantizer->setMaxEpochsMDLReduction (vm["maxepochsmdl"].as<unsigned int>());
	outputQuantizer->setModelEfficiencyConst (vm["output_modelefficiency"].as<double>());
	outputQuantizer->setLearningRates (vm["learningrate"].as<double>(), 0.001);
	outputQuantizer->setDataAccuracy (vm["output_accuracy"].as<double>());
	outputQuantizer->setTimeWindows (20, 12, 100);
	outputQuantizer->setAdaptationThreshold (0.0);
	outputQuantizer->setMaximalEdgeAge (50);
	outputQuantizer->setSamplingMode (randomly);
	// outputQuantizer->setMeanDistanceMode (arithmetic);
	outputQuantizer->setStoppingCriterion (stability);

	currentRegion->redirectOutputToNull ();
	
}

///
///get the action that maximizes learning progress
///
int ActiveLearnScenario::getActionMaxAvgError (const ActionsVector& candidateActions) {
	double max_avgerror = -1e6;
	int index = -1;
	for (int i=0; i < candidateActions.size(); i++) {
		int regionIdx = GNGSMRegion::getSMRegion (regions, candidateActions[i].featureVector);
		assert (regionIdx != -1);
		GNGSMRegion* contextRegion = &regions[regionIdx];
		double avgerror;
		if (contextRegion->inputqErrorsHistory.size() > 0)
			avgerror = contextRegion->getAvgError ();
		else
			avgerror = -1e5;
		if ( avgerror > max_avgerror  ) {
			max_avgerror = avgerror;
			index = i;
		}
	}
	assert (index != -1);
	return index;
}



///
///select an optimal action from a random set of actions
///
void ActiveLearnScenario::chooseAction () {
	
	if (startingPosition == 0) {

		//active selection of samples
		double neargreedyActionRand = randomG.nextUniform (0.0, 1.0);
		
		cout << "neargreedyRand: " << neargreedyActionRand << endl;
		if (neargreedyActionRand <= neargreedyActionProb) {
			if (chosenAction != NULL)
				delete chosenAction;
			chosenAction = NULL;
			Scenario::chooseAction ();
		}
		
		else {

			ActorObject::Desc tmpDesc 	= object->getDescription();
			Vec3 position 			= object->getPosition();
			Vec3 normal			= object->getNormalVec();
			Vec3 orthogonal			= object->getOrthogonalVec();
			Vec3 orientation		= object->getOrientation();
			ActionsVector candidateActions;
			
			for (int i=0; i<maxNumberCandidateActions; i++) {
				LearningData::Chunk chunk_cand;
				int startPosition = availableStartingPositions[floor(randomG.nextUniform (0.0,Real(availableStartingPositions.size())))];
				//action.pushDuration = floor (randomG.nextUniform (3.0, 6.0));
				chunk_cand.action.pushDuration = 3.0;
				chunk_cand.action.horizontalAngle = chooseAngle(Real(60.0), Real(120.0));
				Vec3 pos;
				pos.set (Real(position.v1), Real(position.v2), Real(position.v3));
				set_coordinates_into_target(startPosition, pos, normal, orthogonal, tmpDesc.dist, tmpDesc.side, tmpDesc.center, tmpDesc.top, tmpDesc.over);
				chunk_cand.action.effectorPose.p = pos;
				chunk_cand.action.efRoll = orientationTarget.v1;
				chunk_cand.action.efPitch = orientationTarget.v2;
				chunk_cand.action.efYaw = orientationTarget.v3;
				Vec3 centerNormalVec = computeNormalVector(pos, Vec3 (position.v1, position.v2, tmpDesc.dimensions.v2*0.5));
				Vec3 centerOrthogonalVec = computeOrthogonalVec(centerNormalVec);
				fromCartesianPose(chunk_cand.action.endEffectorPose, pos, orientationTarget);
				setMovementAngle(chunk_cand.action.horizontalAngle, chunk_cand.action.endEffectorPose, desc.distance, centerNormalVec, centerOrthogonalVec);
				chunk_cand.action.endEffectorPose.R.toEuler (chunk_cand.action.endEfRoll, chunk_cand.action.endEfPitch, chunk_cand.action.endEfYaw);

				LearningData::write_chunk_to_featvector (chunk_cand.action.featureVector, chunk_cand, normalization, learningData.featLimits, _end_effector_pos | _effector_pos /*| _action_params*/ );
				candidateActions.push_back (chunk_cand.action);
			}

			// current chosen Action
			if (chosenAction != NULL)
				delete chosenAction;
			chosenAction = new Action(candidateActions[getActionMaxAvgError(candidateActions)]);
			// chosenAction = new Action(candidateActions[floor(randomG.nextUniform (0.0,Real(candidateActions.size())))]);
			
			// this->pushDuration = chosenAction.pushDuration;
			// this->horizontalAngle = chosenAction.horizontalAngle;
			// this->positionT = chosenAction.effectorPose.p;
			// this->startPosition = positionsT[this->positionT];

			// usedStartingPositions.push_back(startPosition);
		}
	}
	else
	{
		if (chosenAction != NULL)
			delete chosenAction;
		chosenAction = NULL;
		Scenario::chooseAction ();
	}
	
}


/** \brief calculate the start coordinates of the arm
*/
void ActiveLearnScenario::calculateStartCoordinates() {
	if (chosenAction != NULL)
	{
		target.pos = chosenAction->effectorPose;
		target.vel.setId(); // it doesn't move
		
		target.t = context.getTimer().elapsed() + arm->getDeltaAsync() + desc.descArmActor.minDuration; // i.e. the movement will last at least 5 sec

	}
	else
		Scenario::calculateStartCoordinates ();
}

/** \brief Describe the experiment trajectory
*
*/
void ActiveLearnScenario::initMovement() {
	if (chosenAction != NULL)
	{
		duration = chosenAction->pushDuration;
		cout << "push duration: " << duration << endl;
		end = target.pos;
		Vec3 normal 		= object->getNormalVec();
		Vec3 orthogonal 	= object->getOrthogonalVec();
		Vec3 position		= object->getPosition();

		Vec3 centerNormalVec =
			computeNormalVector(
					    Vec3 (target.pos.p.v1, target.pos.p.v2, target.pos.p.v3),
					    Vec3 (position.v1, position.v2, object->getDescription().dimensions.v2*0.5)
					    );

		Vec3 centerOrthogonalVec = computeOrthogonalVec(centerNormalVec);
		setMovementAngle(chosenAction->horizontalAngle, end, desc.distance, centerNormalVec, centerOrthogonalVec);
		cout << "Horizontal direction angle: " << chosenAction->horizontalAngle << " degrees" << endl;


	}
	else
		Scenario::initMovement ();
}


void ActiveLearnScenario::writeData (bool final){
	//cout << "ALS: writing data..." << endl;
	/////////////////////////////////////////////////
	//writing the dataset into binary file
	if (final) {
		LearningData::write_dataset (dataFileName, data, learningData.featLimits);
		// string stpFileName = dataFileName + ".stp";
		// ofstream writeToFile (stpFileName.c_str(), ios::out | ios::binary);
		// write_vector<double>(writeToFile, usedStartingPositions);
	}

	for (GNGSMRegion::RegionsMap::iterator regionIter = regions.begin(); regionIter != regions.end(); regionIter++) {
		stringstream name;
		name << dataFileName << "_region" << regionIter->first;
		if (final)
			name << "_final";
		if (!regionIter->second.writeData (name.str()))
			cerr << "Saving region " << regionIter->first << " data was unsuccesful!" << endl;

	}
	/////////////////////////////////////////////////
	
}

void ActiveLearnScenario::postprocess(SecTmReal elapsedTime) {
	if (bStart) {
		CriticalSectionWrapper csw(cs);
		if (object == NULL) {
			return;
		}

		LearningData::Chunk chunk;
		writeChunk (chunk);

// 		learningData.data.push_back(chunk);
		LearningData::write_chunk_to_featvector (chunk.featureVector, chunk, normalization, learningData.featLimits, _end_effector_pos | _effector_pos | _object );
		
		learningData.currentChunkSeq.push_back (chunk);

	}
}


void ActiveLearnScenario::run(int argc, char* argv[]) {

	//set: random seed, tmDeltaAsync; get initial config
	_init();

	// positionsT = get_canonical_positions (desc);

	//start of the experiment loop
	for (int iteration = 0; iteration<numSequences; iteration++) {
		
		// experiment main loops
		creator.setToDefault();

		//create and setup object, compute its vectors
		std::cout <<"creating object "<<std::endl;
		object = dynamic_cast<ActorObject*>(scene.createObject(desc.descActorObject));
		object->setShape(scene,_concreteActor);
		//create sequence for this loop run
		learningData.currentChunkSeq.clear ();
		
		//select an optimal action from a random set of actions
		{
			CriticalSectionWrapper csw (cs);
			//choose motor command
			chooseAction ();
		}

		//compute coordinates of start position	      
		std::cout <<"calculate starting coordinates "<<std::endl;
		calculateStartCoordinates ();
		std::cout << "sending position ";
		cout << Real(target.pos.p.v1) << " " << Real(target.pos.p.v2) << " " << Real(target.pos.p.v3) << endl;		

		//move the finger to the beginning of experiment trajectory
		arm->sendPosition(context, target, ReacPlanner::ACTION_GLOBAL);
		std::cout << "init movement"<<std::endl;
		//compute direction and other features of trajectory
		initMovement();
		std::cout << "move finger"<<std::endl;
		//move the finger along described experiment trajectory
		arm->moveFinger(context,target, bStart, duration,end);
		//update context region
		updateCurrentRegion ();
		//write sequence into dataset
		data.push_back(learningData.currentChunkSeq);
		currentRegion->data.push_back(learningData.currentChunkSeq);
		//turn off collision detection
		arm->setCollisionDetection(false);
		//move the finger up to avoid path finding problems 
		arm->moveFingerUp(context,target, object->getDescription().dimensions);
		//remove polyflap object from the scene
		removeObject();
		//move finger to initial position
		arm->moveFingerToStartPose (context);

		{
			//learn from current sequence
			CriticalSectionWrapper csw (cs);
			updateLearners (iteration);
		}
		arm->moveFingerToStartPose(context);
		context.getMessageStream()->write(Message::LEVEL_INFO, "Done");
		//finish this iteration
		cout << "Iteration " << iteration << " completed!" << endl;
		if (universe.interrupted()) throw Interrupted();	

	}

	//move the arm to its initial position
	arm->moveArmToStartPose(context);
	
	//write obtained data into a binary file
	writeData (true);

	if (chosenAction != NULL)
		delete chosenAction;
	
}




///
///Obtain current context region pointer
///
void ActiveLearnScenario::updateCurrentRegion () {

	int regionIdx = GNGSMRegion::getSMRegion (regions, learningData.currentChunkSeq[0].featureVector);
	assert (regionIdx != -1);
	currentRegion = &regions[regionIdx];

}


//------------------------------------------------------------------------------

///
///Update learners according to a sensorimotor region splitting criterion
///
void ActiveLearnScenario::updateLearners (int iteration) {

	
	if (currentRegion->data.size() > splittingCriterion1 && startingPosition == 0) {
		writeData ();
		splitRegion (*currentRegion);
	}

	cout << "Region: " << currentRegion->index << endl;
	if (saveMDLHistory)
		currentRegion->saveMDLHistory ();

	currentRegion->cryssmex.trainInputQuantizer (iteration, learningData.currentChunkSeq, learningData.featLimits, normalization, featureSelectionMethod);
	currentRegion->cryssmex.trainOutputQuantizer (iteration, learningData.currentChunkSeq, learningData.featLimits, normalization, featureSelectionMethod);
	// wait until learning is performed
	currentRegion->cryssmex.waitForInputQuantizer ();
	if (featureSelectionMethod == _obpose || featureSelectionMethod == _obpose_direction || featureSelectionMethod == _efobpose || featureSelectionMethod == _efobpose_direction || featureSelectionMethod == _mcobpose_obpose_direction)
		currentRegion->cryssmex.waitForOutputQuantizer ();
	// currentRegion->startingPositionsHistory.push_back (startPosition);
	currentRegion->updateErrorsHistory ();
	currentRegion->updateGraphSizeHistory ();

	GNGSMRegion::RegionsMap::iterator it;
	for ( it=regions.begin() ; it != regions.end(); it++ ) {
		if (&(it->second) != currentRegion) {
			double lastEr = it->second.inputqErrorsHistory.back();
			it->second.inputqErrorsHistory.push_back(lastEr);
			lastEr = it->second.outputqErrorsHistory.back();
			it->second.outputqErrorsHistory.push_back(lastEr);
			unsigned int lastsize = it->second.inputqGraphSizeHistory.back();
			it->second.inputqGraphSizeHistory.push_back(lastsize);
			lastsize = it->second.outputqGraphSizeHistory.back();
			it->second.outputqGraphSizeHistory.push_back(lastsize);
			// it->second.startingPositionsHistory.push_back(0);
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
	
	assert (data[0].size() >= 1);
	double featVectorSize = data[0][0].featureVector.size();
	assert (featVectorSize  > 0);
	means.resize (featVectorSize, 0.0);
	variances.resize (featVectorSize, 0.0);
	int numInstances = 0;
	
	for (int i=0; i<data.size(); i++)
		for (int j=0; j<data[i].size(); j++) {
			assert (data[i][j].featureVector.size() == featVectorSize);
			numInstances++;
			for (int k=0; k<featVectorSize; k++)
				means[k] += data[i][j].featureVector[k];
		}
	for (int k=0; k<featVectorSize; k++)
		means[k] /= numInstances;
	
	for (int k=0; k<featVectorSize; k++) {
		for (int i=0; i<data.size(); i++)
			for (int j=0; j<data[i].size(); j++) {
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
double ActiveLearnScenario::evaluateMinimality (const LearningData::DataSet& firstSplittingSet, const LearningData::DataSet& secondSplittingSet, int sMContextSize) {
	//calculate variance of sequence S(t+1),...S(t+n) for each splitting set
	double varianceLastSC1stSet = variance (firstSplittingSet, sMContextSize);
	double varianceLastSC2ndSet = variance (secondSplittingSet, sMContextSize);
	return (firstSplittingSet.size()  * varianceLastSC1stSet +
		secondSplittingSet.size() * varianceLastSC2ndSet);
}


///
///partition of regions according to variance of dataset instances
///
void ActiveLearnScenario::splitRegion (GNGSMRegion& region) {
	double minimalQuantity = numeric_limits<double>::max();
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
			if (firstSplittingSetTry.size() > 0 && secondSplittingSetTry.size() > 0)
			{
				double quantityEval = evaluateMinimality (firstSplittingSetTry, secondSplittingSetTry, motorContextSize);
				if (quantityEval < minimalQuantity) {
					minimalQuantity = quantityEval;
					firstSplittingSet = firstSplittingSetTry;
					secondSplittingSet = secondSplittingSetTry;
					cuttingValue = j;
					cuttingIdx = i;
				}
			}
		}
	}
	assert (firstSplittingSet.size () > 0);
	assert (secondSplittingSet.size() > 0);
	assert (cuttingValue != -1.0);
	assert (cuttingIdx != -1);
	GNGSMRegion firstRegion (region, ++regionsCount, cuttingValue, cuttingIdx, firstSplittingSet, true);
	GNGSMRegion secondRegion (region, ++regionsCount, cuttingValue, cuttingIdx, secondSplittingSet, false);
	regions[firstRegion.index] = firstRegion;
	regions[secondRegion.index] = secondRegion;

	// create new quantizers
	GNG_Quantizer* inputQuantizer = static_cast<GNG_Quantizer*>(region.cryssmex.getInputQuantizer());
	GNG_Quantizer* outputQuantizer = static_cast<GNG_Quantizer*>(region.cryssmex.getOutputQuantizer());	
	regions[firstRegion.index].cryssmex.setInputQuantizer (*inputQuantizer);
	regions[firstRegion.index].cryssmex.setOutputQuantizer (*outputQuantizer);
	regions[secondRegion.index].cryssmex.setInputQuantizer (*inputQuantizer);
	regions[secondRegion.index].cryssmex.setOutputQuantizer (*outputQuantizer);
	regions[firstRegion.index].redirectOutputToNull ();
	regions[secondRegion.index].redirectOutputToNull ();
	// Update data set for quantizers
	regions[firstRegion.index].cryssmex.setData (firstSplittingSet, learningData.featLimits, normalization, featureSelectionMethod);
	regions[secondRegion.index].cryssmex.setData (secondSplittingSet, learningData.featLimits, normalization, featureSelectionMethod);
	// Delete dislocated nodes
	regions[firstRegion.index].cryssmex.findDislocatedNodes ();
	regions[secondRegion.index].cryssmex.findDislocatedNodes ();

	bool regiondeleted = regions.erase (region.index);
	assert (regiondeleted == 1);
	
	updateCurrentRegion ();
	cout << "region split..." << endl;


	
}


} //namespace smlearning
