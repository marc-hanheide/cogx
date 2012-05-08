/** @file PredictingScenario.cpp
 * 
 * @author	Sergio Roa (DFKI)
 *
 * @version 2.0 beta
 *
 */
#include <scenario/PredictingScenario.h>

using namespace golem;

namespace smlearning {

PredictingScenario::PredictingScenario(golem::Scene &scene) : Scenario (scene) {
	normalization = donotnormalize<float>;
	denormalization = donotdenormalize<float>;
}

PredictingScenario::~PredictingScenario() {
}

void PredictingScenario::init (boost::program_options::variables_map vm) {
	Scenario::init (vm);

	// Set feature selection method
	featureSelectionMethod = vm["featuresel"].as<feature_selection>();
	
	// Set Substochastic Sequential Machine
	cryssmex.setSSM (vm["ssmFile"].as<string>());

	// Set Input Quantizer
	string inputqfile;
	if (vm.count("quantizersPath"))
		inputqfile =  vm["quantizersPath"].as<string>() + "/./" + "cryssmex_inputq.qnt";
	else
		inputqfile = "cryssmex_inputq.qnt";
	cryssmex.setInputQuantizer (inputqfile);

	if (featureSelectionMethod == _obpose || featureSelectionMethod == _obpose_direction || featureSelectionMethod == _obpose_label || featureSelectionMethod == _obpose_rough_direction || featureSelectionMethod == _obpose_slide_flip_tilt) //suitable for Mealy machines
		assert (learningData.motorVectorSizeMarkov + learningData.efVectorSize == cryssmex.getInputQuantizer()->dimensionality());

	else if (featureSelectionMethod == _efobpose || featureSelectionMethod == _efobpose_direction || featureSelectionMethod == _efobpose_label || featureSelectionMethod == _efobpose_rough_direction || featureSelectionMethod == _efobpose_slide_flip_tilt) //suitable for Moore machines
		assert (learningData.motorVectorSizeMarkov == cryssmex.getInputQuantizer()->dimensionality());
	else if (featureSelectionMethod == _mcobpose_obpose_direction)
		assert (2*learningData.motorVectorSizeMarkov + learningData.pfVectorSize == cryssmex.getInputQuantizer()->dimensionality());

	// Set Output Quantizer
	if (featureSelectionMethod == _obpose || featureSelectionMethod == _obpose_direction || featureSelectionMethod == _efobpose || featureSelectionMethod == _efobpose_direction || featureSelectionMethod == _mcobpose_obpose_direction)
	{
		string outputqfile;
		if (vm.count("quantizersPath"))
			outputqfile =  vm["quantizersPath"].as<string>() + "/./" + "cryssmex_outputq.qnt";
		else
			outputqfile = "cryssmex_outputq.qnt";
		cryssmex.setOutputQuantizer (outputqfile);
		assert (learningData.pfVectorSize == cryssmex.getOutputQuantizer()->dimensionality());
		
	}

	// Set State Quantizer
	cryssmex.setStateQuantizer (vm["cvqFile"].as<string>());

	if (featureSelectionMethod == _obpose || featureSelectionMethod == _obpose_direction || featureSelectionMethod == _obpose_label || featureSelectionMethod == _obpose_rough_direction || featureSelectionMethod == _obpose_slide_flip_tilt || featureSelectionMethod == _mcobpose_obpose_direction) //suitable for Mealy machines
		assert (learningData.pfVectorSize  == cryssmex.getStateQuantizer()->dimensionality());

	else if (featureSelectionMethod == _efobpose || featureSelectionMethod == _efobpose_direction || featureSelectionMethod == _efobpose_label || featureSelectionMethod == _efobpose_rough_direction || featureSelectionMethod == _efobpose_slide_flip_tilt) //suitable for Moore machines
		assert (learningData.efVectorSize + learningData.pfVectorSize == cryssmex.getStateQuantizer()->dimensionality());
	cryssmex.averageModelVectors ();

	// Set data sequence
	if (vm.count("seqFile"))
	{
		if (!LearningData::read_dataset (vm["seqFile"].as<string>(), data, learningData.featLimits )) {
			cerr << "error reading data" << endl;
			exit(-1);
		}
		enumerate_labels ();

	}

	// Set (de)normalization functions
	if (vm.count("normalize"))
	{
		normalization = normalize<float>;
		denormalization = denormalize<float>;
	}	

}

///////// Protected //////////
/** \brief select a random action
*/
void PredictingScenario::chooseAction () {

	if (data.size() > 0)
	{
		int index = static_cast<int>(floor(randomG.nextUniform (0.0, Real(data.size() - 1))));
		// learningData.currentChunkSeq = data[index];
		seqDataset = data[index];
		/*if (learningData.currentChunkSeq.size() > 2) {
			currentSeq = LearningData::load_cryssmexinputsequence (learningData.currentChunkSeq, featureSelectionMethod, normalization, learningData.featLimits);
				
			cryssmex.parseSequence (currentSeq, predictedSeq);

			if (featureSelectionMethod == _obpose || featureSelectionMethod == _obpose_direction || featureSelectionMethod == _obpose_label || featureSelectionMethod == _obpose_rough_direction || featureSelectionMethod == _obpose_slide_flip_tilt || featureSelectionMethod == _mcobpose_obpose_direction) //suitable for Mealy machines
				learningData.get_pfSeq_from_cryssmexquantization (predictedSeq, 0, denormalization);

			else if (featureSelectionMethod == _efobpose || featureSelectionMethod == _efobpose_direction || featureSelectionMethod == _efobpose_label || featureSelectionMethod == _efobpose_rough_direction || featureSelectionMethod == _efobpose_slide_flip_tilt) //suitable for Moore machines
				learningData.get_pfSeq_from_cryssmexquantization (predictedSeq, learningData.efVectorSize, denormalization);

			
		}
		counter_sequence = -2;*/
	}
	else
	{
		// learningData.currentPredictedPfSeq.clear();
		// cryssmex.setUniformDistribution ();
		Scenario::chooseAction ();
	}
	// cryssmex.setUniformDistribution ();
}

///////// Protected //////////
/** \brief calculate the start coordinates of the arm
*/
void PredictingScenario::calculateStartCoordinates() {
	if (data.size() > 0)
	{
		// LearningData::Chunk chunk = learningData.currentChunkSeq[0];
		LearningData::Chunk chunk = seqDataset[0];
		target.pos = chunk.action.effectorPose;
		target.vel.setId(); // it doesn't move
		
		target.t = context.getTimer().elapsed() + arm->getDeltaAsync() + desc.descArmActor.minDuration; // i.e. the movement will last at least 5 sec

	}
	else
		Scenario::calculateStartCoordinates ();
}

///////// Protected //////////
/** \brief Describe the experiment trajectory
*
*/
void PredictingScenario::initMovement() {
	if (data.size() > 0)
	{
		// LearningData::Chunk chunk = learningData.currentChunkSeq[0];
		LearningData::Chunk chunk = seqDataset[0];
		duration = chunk.action.pushDuration;
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
		setMovementAngle(chunk.action.horizontalAngle, end, desc.distance, centerNormalVec, centerOrthogonalVec);
		cout << "Horizontal direction angle: " << chunk.action.horizontalAngle << " degrees" << endl;


	}
	else
		Scenario::initMovement ();
}

void PredictingScenario::render () {
	CriticalSectionWrapper csw (cs);
	unsigned int seq_size;
	// if (data.size() == 0)
		seq_size = learningData.currentPredictedPfSeq.size() - 1;
	// else
	// 	seq_size = counter_sequence;

	if ( seq_size < 0 || object == NULL || seq_size >= learningData.currentPredictedPfSeq.size())
		return;

	golem::BoundsRenderer boundsRenderer;
	vector<int> idx;
	idx.push_back(0);
	idx.push_back (seq_size);

	for (int i=1; i<2; i++) {
		Mat34 currentPose = learningData.currentPredictedPfSeq[idx[i]];
		
		boundsRenderer.setMat(currentPose);
		if (idx[i] == seq_size)
			boundsRenderer.setWireColour (RGBA::RED);
		// else if (idx[i] == 0)
		// 	boundsRenderer.setWireColour (RGBA::BLUE);			
		
		boundsRenderer.renderWire (objectLocalBounds->begin(), objectLocalBounds->end());
	}

}


void PredictingScenario::postprocess(SecTmReal elapsedTime) {
	if (bStart) {
		CriticalSectionWrapper csw(cs);
		if (object == NULL) {
			return;
		}

		// if (data.size() == 0)
		// {
			LearningData::Chunk chunk;
			writeChunk (chunk);
			learningData.currentChunkSeq.push_back (chunk);

			FeatureVector inputVector;
			LearningData::load_cryssmexinput (inputVector, chunk, featureSelectionMethod, normalization, learningData.featLimits);

			pair<int, int> result = cryssmex.parseInput (inputVector);
			if (result.first >= 0)
			{
				FeatureVector predictedVector = cryssmex.getQntMvMapVector(result.first);

				if (featureSelectionMethod == _obpose || featureSelectionMethod == _obpose_direction || featureSelectionMethod == _obpose_label || featureSelectionMethod == _obpose_rough_direction || featureSelectionMethod == _obpose_slide_flip_tilt || featureSelectionMethod == _mcobpose_obpose_direction) //suitable for Mealy machines
					learningData.get_pfPose_from_cryssmexquantization (predictedVector, 0, denormalization);

				else if (featureSelectionMethod == _efobpose || featureSelectionMethod == _efobpose_direction || featureSelectionMethod == _efobpose_label || featureSelectionMethod == _efobpose_rough_direction || featureSelectionMethod == _efobpose_slide_flip_tilt) //suitable for Moore machines
					learningData.get_pfPose_from_cryssmexquantization (predictedVector, learningData.efVectorSize, denormalization);
			}
			if (result.second >= 0)
			{
				
				currentPredictedOutput.push_back (result.second);
			}

		// }
		// else
		// {
		// 	counter_sequence++;
		// }
	}
}

void PredictingScenario::enumerate_labels ()
{
	LearningData::DataSet::iterator d_iter;
	for (d_iter = data.begin(); d_iter != data.end(); d_iter++) {
		LearningData::Chunk::Seq* seq = &(*d_iter);
		if (featureSelectionMethod == _obpose_slide_flip_tilt || featureSelectionMethod == _efobpose_slide_flip_tilt)
			LearningData::label_seq (seq, _rough_direction | _slide_flip_tilt);
		else if (featureSelectionMethod == _obpose_rough_direction || featureSelectionMethod == _efobpose_rough_direction)
			LearningData::label_seq (seq, _rough_direction);
	}	

	if (featureSelectionMethod == _obpose_slide_flip_tilt || featureSelectionMethod == _efobpose_slide_flip_tilt || featureSelectionMethod == _obpose_rough_direction || featureSelectionMethod == _efobpose_rough_direction) {
		set<string> labelsSet = LearningData::labels_enumerator (data);
		set<string>::iterator it;
		unsigned int i;
		for (i=0, it=labelsSet.begin(); it!=labelsSet.end(); it++, i++)
			output_map[*it] = i;
	}
			

}


void PredictingScenario::updateOutputError ()
{
	if (data.size () > 0)
	{
		if (featureSelectionMethod == _obpose_slide_flip_tilt || featureSelectionMethod == _efobpose_slide_flip_tilt || featureSelectionMethod == _obpose_rough_direction || featureSelectionMethod == _efobpose_rough_direction) {
			if (currentPredictedOutput.size () == learningData.currentChunkSeq.size ())
			{
				unsigned int false_counts = 0;
				for (unsigned int i=0; i<currentPredictedOutput.size(); i++)
				{
					stringstream labelStr;
					labelStr << learningData.currentChunkSeq[i].label;
					string label = labelStr.str();
					if (currentPredictedOutput[i] != output_map[label]) {
						cout << "predicted: " << currentPredictedOutput[i] << endl;
						cout << "real: " << output_map[label] << endl;
						false_counts++;
					}
				}
				double error = (double)false_counts / (double)currentPredictedOutput.size();
				avgoutputerrors.push_back (error);
			}
		}
	}
	
}

void PredictingScenario::run (int argc, char* argv[]) {
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
		//move finger to initial position
		arm->moveFingerToStartPose(context);
		context.getMessageStream()->write(Message::LEVEL_INFO, "Done");
		//finish this iteration
		cout << "Iteration " << iteration << " completed!" << endl;
		if (universe.interrupted()) throw Interrupted();

	}
	//move the arm to its initial position
	arm->moveArmToStartPose(context);

	double misclass_percentage = 0.0;
	for (unsigned int i=0; i<avgoutputerrors.size (); i++)
		misclass_percentage += avgoutputerrors[i];
	misclass_percentage /= avgoutputerrors.size ();
	cout << endl << "Avg Misclassification percentage: " << misclass_percentage << endl;
	

}

} // namespace smlearning

