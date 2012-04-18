/** @file ActiveGNGScenario.cpp
 * 
 * @author	Sergio Roa (DFKI)
 *
 * @version 2.0 beta
 *
 */
#include <scenario/ActiveGNGScenario.h>

using namespace golem;

namespace smlearning {

ActiveGNGScenario::ActiveGNGScenario(golem::Scene &scene) : Scenario (scene)
{
	normalization = donotnormalize<float>;
	denormalization = donotdenormalize<float>;
}

ActiveGNGScenario::~ActiveGNGScenario()
{
}

void ActiveGNGScenario::init (boost::program_options::variables_map vm) {
	Scenario::init (vm);

	// Set feature selection method
	string fSMethod;
	fSMethod = vm["featuresel"].as<string>();

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

	// Initialize Input Quantizer
	if (featureSelectionMethod == _obpose || featureSelectionMethod == _obpose_direction || featureSelectionMethod == _obpose_label || featureSelectionMethod == _obpose_rough_direction || featureSelectionMethod == _obpose_slide_flip_tilt) //suitable for Mealy machines
		cryssmex.initializeInputQuantizer (learningData.motorVectorSizeMarkov + learningData.efVectorSize);
	
	else if (featureSelectionMethod == _efobpose || featureSelectionMethod == _efobpose_direction || featureSelectionMethod == _efobpose_label || featureSelectionMethod == _efobpose_rough_direction || featureSelectionMethod == _efobpose_slide_flip_tilt) //suitable for Moore machines
		cryssmex.initializeInputQuantizer (learningData.motorVectorSizeMarkov);
	else if (featureSelectionMethod == _mcobpose_obpose_direction)
		cryssmex.initializeInputQuantizer (2*learningData.motorVectorSizeMarkov + learningData.pfVectorSize);

	// Initialize Output Quantizer 
	if (featureSelectionMethod == _obpose || featureSelectionMethod == _obpose_direction || featureSelectionMethod == _efobpose || featureSelectionMethod == _efobpose_direction || featureSelectionMethod == _mcobpose_obpose_direction)
		cryssmex.initializeOutputQuantizer (learningData.pfVectorSize);

	// Initialize state quantizer
	if (featureSelectionMethod == _obpose || featureSelectionMethod == _obpose_direction || featureSelectionMethod == _obpose_label || featureSelectionMethod == _obpose_rough_direction || featureSelectionMethod == _obpose_slide_flip_tilt || featureSelectionMethod == _mcobpose_obpose_direction) //suitable for Mealy machines
		cryssmex.initializeStateQuantizer (learningData.pfVectorSize);

	else if (featureSelectionMethod == _efobpose || featureSelectionMethod == _efobpose_direction || featureSelectionMethod == _efobpose_label || featureSelectionMethod == _efobpose_rough_direction || featureSelectionMethod == _efobpose_slide_flip_tilt) //suitable for Moore machines
		cryssmex.initializeStateQuantizer (learningData.efVectorSize + learningData.pfVectorSize);

	// Set (de)normalization functions
	if (vm.count("normalize"))
	{
		normalization = normalize<float>;
		denormalization = denormalize<float>;
	}

	ActiveGNG_Quantizer* inputQuantizer = static_cast<ActiveGNG_Quantizer*>(cryssmex.getInputQuantizer());
	ActiveGNG_Quantizer* outputQuantizer = static_cast<ActiveGNG_Quantizer*>(cryssmex.getOutputQuantizer());
	
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
	
	if (vm.count("mdl"))
	{
		inputQuantizer->saveMDLHistory ("mdlinput.txt");
		outputQuantizer->saveMDLHistory ("mdloutput.txt");
	}

	if (vm.count("inputout"))
		inputQuantizer->redirectOutput (vm["inputout"].as<string>());
	if (vm.count("outputout"))
		outputQuantizer->redirectOutput (vm["outputout"].as<string>());

}


// void ActiveGNGScenario::postprocess(SecTmReal elapsedTime) {
// 	if (bStart) {
// 		CriticalSectionWrapper csw(cs);
// 		if (object == NULL) {
// 			return;
// 		}

// 		LearningData::Chunk chunk;
// 		writeChunk (chunk);

// 		learningData.currentChunkSeq.push_back (chunk);

// 	}
// }

void ActiveGNGScenario::trainInputQuantizer (int iteration) {
	CriticalSectionWrapper csw(cs);

	currentInputSeq = LearningData::load_cryssmexinputsequence (learningData.currentChunkSeq, featureSelectionMethod, normalization, learningData.featLimits);
	// hacky conversion
	vector<neuralgas::Vector<double>* >* newInputSeq = new vector<neuralgas::Vector<double>* >;
	for (unsigned int i=0; i<currentInputSeq.size(); i++)
	{
		neuralgas::Vector<double> *new_item = new neuralgas::Vector<double>(currentInputSeq[i]);
		newInputSeq->push_back (new_item);
	}
	ActiveGNG_Quantizer* inputQuantizer = static_cast<ActiveGNG_Quantizer*>(cryssmex.getInputQuantizer());
	inputQuantizer->addData (newInputSeq);

	if (iteration == 0)
	{
		inputQuantizer->setRefVectors(2);
		inputQuantizer->open();
	}

	inputQuantizer->setInsertionRate (newInputSeq->size());

	for (unsigned int i=0; i < newInputSeq->size(); i++)
		delete (*newInputSeq)[i];
	newInputSeq->clear();
	delete newInputSeq;

	inputQuantizer->begin ();

}

void ActiveGNGScenario::trainOutputQuantizer (int iteration) {

	CriticalSectionWrapper csw(cs);
	if (featureSelectionMethod == _obpose || featureSelectionMethod == _obpose_direction || featureSelectionMethod == _efobpose || featureSelectionMethod == _efobpose_direction || featureSelectionMethod == _mcobpose_obpose_direction)
	{

		currentOutputSeq = LearningData::load_cryssmexoutputsequence (learningData.currentChunkSeq, featureSelectionMethod, normalization, learningData.featLimits);
		// hacky conversion
		vector<neuralgas::Vector<double>* >* newOutputSeq = new vector<neuralgas::Vector<double>* >;
		for (unsigned int i=0; i<currentOutputSeq.size(); i++)
		{
			neuralgas::Vector<double> *new_item = new neuralgas::Vector<double>(currentOutputSeq[i]);
			newOutputSeq->push_back (new_item);
		}
		ActiveGNG_Quantizer* outputQuantizer = static_cast<ActiveGNG_Quantizer*>(cryssmex.getOutputQuantizer());
		outputQuantizer->addData (newOutputSeq);

		if (iteration == 0)
		{
			outputQuantizer->setRefVectors(2);
			outputQuantizer->open();
		}

		outputQuantizer->setInsertionRate (newOutputSeq->size());

		for (unsigned int i=0; i < newOutputSeq->size(); i++)
			delete (*newOutputSeq)[i];
		newOutputSeq->clear();
		delete newOutputSeq;

		outputQuantizer->begin ();
	}

}

void ActiveGNGScenario::waitForInputQuantizer () {
	static_cast<ActiveGNG_Quantizer*>(cryssmex.getInputQuantizer())->wait ();
	static_cast<ActiveGNG_Quantizer*>(cryssmex.getInputQuantizer())->showGraph ();
}

void ActiveGNGScenario::waitForOutputQuantizer () {
	if (featureSelectionMethod == _obpose || featureSelectionMethod == _obpose_direction || featureSelectionMethod == _efobpose || featureSelectionMethod == _efobpose_direction || featureSelectionMethod == _mcobpose_obpose_direction)
	{
		static_cast<ActiveGNG_Quantizer*>(cryssmex.getOutputQuantizer())->wait ();
		static_cast<ActiveGNG_Quantizer*>(cryssmex.getOutputQuantizer())->showGraph ();
	}
}

void ActiveGNGScenario::saveInputQuantizer () {

	cryssmex.getInputQuantizer()->close ();
	save_quantizer (cryssmex.getInputQuantizer(), "cryssmex_inputq.qnt");

}

void ActiveGNGScenario::saveOutputQuantizer () {

	cryssmex.getOutputQuantizer()->close ();
	save_quantizer (cryssmex.getOutputQuantizer(), "cryssmex_outputq.qnt");

}


void ActiveGNGScenario::run (int argc, char* argv[]) {

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
		//create sequence for this loop run
		learningData.currentChunkSeq.clear ();
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
		// arm->moveFingerToStartPushing (context, target);
		std::cout << "init movement"<<std::endl;
		//compute direction and other features of trajectory
		initMovement();
		std::cout << "move finger"<<std::endl;
		//move the finger along described experiment trajectory
		arm->moveFinger(context,target, bStart, duration,end);
		//write sequence into dataset		
		data.push_back(learningData.currentChunkSeq);
		//turn off collision detection
		arm->setCollisionDetection(false);
		//move the finger up to avoid path finding problems 
		arm->moveFingerUp(context,target, object->getDescription().dimensions);
		//remove polyflap object from the scene
		removeObject();
		//move finger to initial position
		arm->moveFingerToStartPose(context);
		{
			//learn from current sequence
			CriticalSectionWrapper csw (cs);
			trainInputQuantizer (iteration);
			trainOutputQuantizer (iteration);
			// wait until learning is performed
			waitForInputQuantizer ();
			waitForOutputQuantizer ();
		}
		arm->moveFingerToStartPose(context);
		context.getMessageStream()->write(Message::LEVEL_INFO, "Done");
		//finish this iteration
		cout << "Iteration " << iteration << " completed!" << endl;
		if (universe.interrupted()) throw Interrupted();

	}
	//move the arm to its initial position
	arm->moveArmToStartPose(context);
	saveInputQuantizer ();
	saveOutputQuantizer ();
	writeData ();
}

} // namespace smlearning
