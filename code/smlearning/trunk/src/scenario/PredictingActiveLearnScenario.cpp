/** @file PredictingActiveLearnScenario.cpp
 * 
 * @author	Sergio Roa (DFKI)
 *
 * @version 2.0 beta
 *
 */
#include <scenario/PredictingActiveLearnScenario.h>

using namespace golem;

namespace smlearning {

PredictingActiveLearnScenario::PredictingActiveLearnScenario(golem::Scene &scene) : Scenario (scene) {
	normalization = normalize<Real>;
	denormalization = denormalize<Real>;
	currentRegion = NULL;
}

PredictingActiveLearnScenario::~PredictingActiveLearnScenario() {
}

void PredictingActiveLearnScenario::init (boost::program_options::variables_map vm) {
	Scenario::init (vm);

	// Set feature selection method
	featureSelectionMethod = vm["featuresel"].as<feature_selection>();
	
	boost::regex regfile_re ("(.*_final)\\.reg");
	boost::cmatch matches;
	string prefix = vm["prefix"].as<string>();
	// cout << matches.size() << endl;
	path p(prefix);
	if(!exists(p)) {
		cerr<<p.leaf()<<" does not exist." << endl;
		exit (-1);
	}

	directory_iterator dir_iter (p), dir_end;
	for (;dir_iter != dir_end; ++dir_iter)
	{
		string dirstring (dir_iter->leaf().c_str());
		char *dirchar = (char *)dirstring.c_str();
		if (boost::regex_match ((const char*)dirchar, matches, regfile_re))
		{
			GNGSMRegion region;
			string regionFileName (matches[1].first, matches[1].second);
			cout << dir_iter->leaf() << endl;
			cout << regionFileName << endl;
			if (region.readData (prefix + regionFileName)) {
				cout << "region data correctly read..." << endl << "======" << endl;
			}
			else {
				cerr << "Error by readying region data..." << endl;
				exit(-1);
			}
			regions[region.index] = region;
			regions[region.index].cryssmex.setStateQuantizer (prefix + regionFileName + "_cryssmex_cvq_final.qnt");
			regions[region.index].cryssmex.setSSM (prefix + regionFileName + "_cryssmex_ssm_final.ssm");
			regions[region.index].cryssmex.averageModelVectors ();

		}
	}

	// Set data sequence
	if (vm.count("seqFile"))
	{
		if (!LearningData::read_dataset (vm["seqFile"].as<string>(), data, learningData.featLimits )) {
			cerr << "error reading data" << endl;
			exit(-1);
		}
		enumerate_labels ();
	}

}

/** \brief select a random action
*/
void PredictingActiveLearnScenario::chooseAction () {

	ActorObject::Desc tmpDesc 	= object->getDescription();
	Vec3 position 			= object->getPosition();
	Vec3 normal			= object->getNormalVec();
	Vec3 orthogonal			= object->getOrthogonalVec();
	Vec3 orientation		= object->getOrientation();

	LearningData::Chunk chunk;
	if (data.size() > 0)
	{
		int index = static_cast<int>(floor(randomG.nextUniform (0.0, Real(data.size() - 1))));
		// learningData.currentChunkSeq = data[index];
		chunk.action = data[index][0].action;
	}
	else
	{
		int startPosition = availableStartingPositions[floor(randomG.nextUniform (0.0,Real(availableStartingPositions.size())))];
		//action.pushDuration = floor (randomG.nextUniform (3.0, 6.0));
		chunk.action.pushDuration = 3.0;
		chunk.action.horizontalAngle = chooseAngle(Real(60.0), Real(120.0));
		Vec3 pos;
		pos.set (Real(position.v1), Real(position.v2), Real(position.v3));
		set_coordinates_into_target(startPosition, pos, normal, orthogonal, tmpDesc.dist, tmpDesc.side, tmpDesc.center, tmpDesc.top, tmpDesc.over);
		chunk.action.effectorPose.p = pos;
		chunk.action.efRoll = orientationTarget.v1;
		chunk.action.efPitch = orientationTarget.v2;
		chunk.action.efYaw = orientationTarget.v3;
		Vec3 centerNormalVec = computeNormalVector(pos, Vec3 (position.v1, position.v2, tmpDesc.dimensions.v2*0.5));
		Vec3 centerOrthogonalVec = computeOrthogonalVec(centerNormalVec);
		fromCartesianPose(chunk.action.endEffectorPose, pos, orientationTarget);
		setMovementAngle(chunk.action.horizontalAngle, chunk.action.endEffectorPose, desc.distance, centerNormalVec, centerOrthogonalVec);
		chunk.action.endEffectorPose.R.toEuler (chunk.action.endEfRoll, chunk.action.endEfPitch, chunk.action.endEfYaw);
	}
		
	LearningData::write_chunk_to_featvector (chunk.action.featureVector, chunk, normalization, learningData.featLimits, _end_effector_pos | _effector_pos /*| _action_params*/ );
	int regionIdx = GNGSMRegion::getSMRegion (regions, chunk.action.featureVector);
	assert (regionIdx != -1);
	currentRegion = &regions[regionIdx];
	chosenAction = chunk.action;
		
}

/** \brief calculate the start coordinates of the arm
*/
void PredictingActiveLearnScenario::calculateStartCoordinates() {

	target.pos = chosenAction.effectorPose;
	target.vel.setId(); // it doesn't move
	target.t = context.getTimer().elapsed() + arm->getDeltaAsync() + desc.descArmActor.minDuration; // i.e. the movement will last at least 5 sec

}

/** \brief Describe the experiment trajectory
*
*/
void PredictingActiveLearnScenario::initMovement() {

	duration = chosenAction.pushDuration;
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
	setMovementAngle(chosenAction.horizontalAngle, end, desc.distance, centerNormalVec, centerOrthogonalVec);
	cout << "Horizontal direction angle: " << chosenAction.horizontalAngle << " degrees" << endl;


}

void PredictingActiveLearnScenario::render () {
	CriticalSectionWrapper csw (cs);
	int seq_size = learningData.currentPredictedPfSeq.size() - 1;

	if ( seq_size < 0 || object == NULL)
		return;

	golem::BoundsRenderer boundsRenderer;

	Mat34 currentPose = learningData.currentPredictedPfSeq[seq_size];
	
	boundsRenderer.setMat(currentPose);
	boundsRenderer.setWireColour (RGBA::RED);
	boundsRenderer.renderWire (objectLocalBounds->begin(), objectLocalBounds->end());
	
}

void PredictingActiveLearnScenario::postprocess(SecTmReal elapsedTime) {
	if (bStart) {
		CriticalSectionWrapper csw(cs);
		if (object == NULL) {
			return;
		}

		LearningData::Chunk chunk;
		writeChunk (chunk);
		learningData.currentChunkSeq.push_back (chunk);

		FeatureVector inputVector;
		LearningData::load_cryssmexinput (inputVector, chunk, featureSelectionMethod, normalization, learningData.featLimits);

		pair<int,string> result = currentRegion->cryssmex.parseInput (inputVector);
		if (result.first >= 0)
		{
			FeatureVector predictedVector = currentRegion->cryssmex.getQntMvMapVector(result.first);
			if (predictedVector.size() > 0)
			{

				if (featureSelectionMethod == _obpose || featureSelectionMethod == _obpose_direction || featureSelectionMethod == _obpose_label || featureSelectionMethod == _obpose_rough_direction || featureSelectionMethod == _obpose_slide_flip_tilt || featureSelectionMethod == _mcobpose_obpose_direction || featureSelectionMethod == _mcobpose_obpose_sft ) //suitable for Mealy machines
					learningData.get_pfPose_from_cryssmexquantization (predictedVector, 0, denormalization);

				else if (featureSelectionMethod == _efobpose || featureSelectionMethod == _efobpose_direction || featureSelectionMethod == _efobpose_label || featureSelectionMethod == _efobpose_rough_direction || featureSelectionMethod == _efobpose_slide_flip_tilt) //suitable for Moore machines
					learningData.get_pfPose_from_cryssmexquantization (predictedVector, learningData.efVectorSize, denormalization);
			}
		}
		currentPredictedOutput.push_back (result.second);
	}
}

void PredictingActiveLearnScenario::enumerate_labels ()
{
	LearningData::DataSet::iterator d_iter;
	for (d_iter = data.begin(); d_iter != data.end(); d_iter++) {
		LearningData::Chunk::Seq* seq = &(*d_iter);
		if (featureSelectionMethod == _obpose_slide_flip_tilt || featureSelectionMethod == _efobpose_slide_flip_tilt || featureSelectionMethod == _mcobpose_obpose_sft )
			LearningData::label_seq (seq, _rough_direction | _slide_flip_tilt);
		else if (featureSelectionMethod == _obpose_rough_direction || featureSelectionMethod == _efobpose_rough_direction)
			LearningData::label_seq (seq, _rough_direction);
	}
}


void PredictingActiveLearnScenario::updateOutputError ()
{
	if (data.size () > 0)
	{
		if (featureSelectionMethod == _obpose_slide_flip_tilt || featureSelectionMethod == _efobpose_slide_flip_tilt || featureSelectionMethod == _mcobpose_obpose_sft)
			LearningData::label_seq (&learningData.currentChunkSeq, _rough_direction | _slide_flip_tilt);
		else if (featureSelectionMethod == _obpose_rough_direction || featureSelectionMethod == _efobpose_rough_direction)
			LearningData::label_seq (&learningData.currentChunkSeq, _rough_direction);

		if (featureSelectionMethod == _obpose_slide_flip_tilt || featureSelectionMethod == _efobpose_slide_flip_tilt || featureSelectionMethod == _mcobpose_obpose_sft || featureSelectionMethod == _obpose_rough_direction || featureSelectionMethod == _efobpose_rough_direction) {
			if (currentPredictedOutput.size () == learningData.currentChunkSeq.size ())
			{
				unsigned int false_counts = 0;
				unsigned int wrong_class = 0;
				for (unsigned int i=0; i<currentPredictedOutput.size(); i++)
				{
					stringstream labelStr;
					labelStr << learningData.currentChunkSeq[i].label;
					string _label = labelStr.str();
					double label = learningData.currentChunkSeq[i].label;
					double predictedLabel;
					try {
						predictedLabel = boost::lexical_cast<double>(currentPredictedOutput[i]);
					} catch(bad_lexical_cast&) {
						cerr << "Error converting string" << endl;
					}
					cout << "predicted: " << predictedLabel << endl;
					if (_label.compare(currentPredictedOutput[i]) != 0) {
						cout << "real: " << label << endl;
						false_counts++;
					}
					
					if (label >= -1.0 && label <= -0.34)
						if (predictedLabel > -0.34)
							wrong_class++;
					if (label >= -0.33 && label <= 0.33)
						if (predictedLabel < -0.33 || predictedLabel > 0.33)
							wrong_class++;
					if (label >= 0.34 && label <= 1.0)
						if (predictedLabel < 0.34)
							wrong_class++;
				}
				double error = (double)false_counts / (double)currentPredictedOutput.size();
				avgoutputerrors.push_back (error);
				double wrongclass_error = (double)wrong_class / (double)currentPredictedOutput.size();
				avgclassiferrors.push_back (wrongclass_error);
			}
		}
	}
	
}

void PredictingActiveLearnScenario::updateAvgError ()
{
	if (learningData.currentPredictedPfSeq.size () == learningData.currentChunkSeq.size ())
	{
		double avgerror = 0.0;
		for (unsigned int i=0; i<learningData.currentPredictedPfSeq.size(); i++)
		{
			double error = 0.0;
			error += pow(learningData.currentPredictedPfSeq[i].p.v1 - learningData.currentChunkSeq[i].object.objectPose.p.v1, 2);
			error += pow(learningData.currentPredictedPfSeq[i].p.v2 - learningData.currentChunkSeq[i].object.objectPose.p.v2, 2);
			error += pow(learningData.currentPredictedPfSeq[i].p.v3 - learningData.currentChunkSeq[i].object.objectPose.p.v3, 2);
			Real predRoll, predPitch, predYaw;
			learningData.currentPredictedPfSeq[i].R.toEuler (predRoll, predPitch, predYaw);
			error += pow(predRoll - learningData.currentChunkSeq[i].object.obRoll, 2);
			error += pow(predPitch - learningData.currentChunkSeq[i].object.obPitch, 2);
			error += pow(predYaw - learningData.currentChunkSeq[i].object.obYaw, 2);
			error = sqrt (error);
			avgerror += error;
		}
		avgerror /= learningData.currentChunkSeq.size();
		avgerrors.push_back (avgerror);
	}
}

void PredictingActiveLearnScenario::run (int argc, char* argv[]) {
	//set: random seed;init arm; get initial config
	std::cout <<"_init "<<std::endl;
	_init();
	std::cout <<"starting loop "<<std::endl;
	//start of the experiment loop
	for (int iteration = 0; iteration<numSequences; iteration++) {
		// experiment main loops
		creator.setToDefault();
		//create and setup object, compute its vectors
		std::cout <<"creating object "<<std::endl;
		object = dynamic_cast<ActorObject*>(scene.createObject(desc.descActorObject));
		object->setShape(scene,_concreteActor);
		if (iteration == 0) objectLocalBounds = object->getLocalBoundsSeq();
		//create sequence for this loop run
		learningData.currentChunkSeq.clear ();
		learningData.currentPredictedPfSeq.clear();
		currentPredictedOutput.clear();
		//select a random action
		std::cout <<"choose action "<<std::endl;
		chooseAction ();
		//compute coordinates of start position	      
		std::cout <<"calculate starting coordinates "<<std::endl;
		calculateStartCoordinates ();
		std::cout << "sending position ";
		cout << Real(target.pos.p.v1) << " " << Real(target.pos.p.v2) << " " << Real(target.pos.p.v3) << endl;
		//move the finger to the beginnnig of experiment trajectory
		arm->sendPosition(context,target , ReacPlanner::ACTION_GLOBAL);
		std::cout << "init movement"<<std::endl;
		//compute direction and other features of trajectory
		initMovement();
		std::cout << "move finger"<<std::endl;
		//move the finger along described experiment trajectory
		arm->moveFinger(context,target, bStart, duration,end);
		//turn off collision detection
		arm->setCollisionDetection(false);
		//move the finger up to avoid path finding problems 
		arm->moveFingerUp(context,target, object->getDescription().dimensions);
		//remove polyflap object from the scene
		removeObject();
		//update avg error in prediction
		updateAvgError ();
		//update error in output prediction
		updateOutputError ();
		//move finger to initial position
		arm->moveFingerToStartPose(context);
		context.getMessageStream()->write(Message::LEVEL_INFO, "Done");
		//finish this iteration
		cout << "Iteration " << iteration << " completed!" << endl;
		if (universe.interrupted()) throw Interrupted();

	}
	//move the arm to its initial position
	arm->moveArmToStartPose(context);

	double avgerror = 0.0;
	for (unsigned int i=0; i<avgerrors.size (); i++)
		avgerror += avgerrors[i];
	avgerror /= avgerrors.size();
	cout << endl << "Avg Error: " << avgerror << endl;

	if (data.size() > 0 && (featureSelectionMethod == _obpose_slide_flip_tilt || featureSelectionMethod == _efobpose_slide_flip_tilt || featureSelectionMethod == _mcobpose_obpose_sft || featureSelectionMethod == _obpose_rough_direction || featureSelectionMethod == _efobpose_rough_direction))
	{
		double outputerror_percentage = 0.0;
		for (unsigned int i=0; i<avgoutputerrors.size (); i++)
			outputerror_percentage += avgoutputerrors[i];
		outputerror_percentage /= avgoutputerrors.size ();
		cout << endl << "Avg Output error percentage: " << outputerror_percentage << endl;
		double misclass_percentage = 0.0;
		for (unsigned int i=0; i<avgclassiferrors.size (); i++)
			misclass_percentage += avgclassiferrors[i];
		misclass_percentage /= avgclassiferrors.size ();
		cout << endl << "Avg Misclassification percentage: " << misclass_percentage << endl;
	}

	
}


} //namespace smlearning
