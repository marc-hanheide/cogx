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
	featureSelectionMethod = vm["featuresel"].as<feature_selection>();
	
	if (vm.count("input_quantizer"))
	{
		cryssmex.setInputQuantizer (vm["input_quantizer"].as<string>());
		if (featureSelectionMethod == _obpose || featureSelectionMethod == _obpose_direction || featureSelectionMethod == _obpose_label || featureSelectionMethod == _obpose_rough_direction || featureSelectionMethod == _obpose_slide_flip_tilt) //suitable for Mealy machines
			assert (learningData.motorVectorSizeMarkov + learningData.efVectorSize == cryssmex.getInputQuantizer()->dimensionality());

		else if (featureSelectionMethod == _efobpose || featureSelectionMethod == _efobpose_direction || featureSelectionMethod == _efobpose_label || featureSelectionMethod == _efobpose_rough_direction || featureSelectionMethod == _efobpose_slide_flip_tilt) //suitable for Moore machines
			assert (learningData.motorVectorSizeMarkov == cryssmex.getInputQuantizer()->dimensionality());
		else if (featureSelectionMethod == _mcobpose_obpose_direction)
			assert (2*learningData.motorVectorSizeMarkov + learningData.pfVectorSize == cryssmex.getInputQuantizer()->dimensionality());

	}
	else
	{
		// Initialize Input Quantizer
		if (featureSelectionMethod == _obpose || featureSelectionMethod == _obpose_direction || featureSelectionMethod == _obpose_label || featureSelectionMethod == _obpose_rough_direction || featureSelectionMethod == _obpose_slide_flip_tilt) //suitable for Mealy machines
			cryssmex.initializeInputQuantizer (learningData.motorVectorSizeMarkov + learningData.efVectorSize);
	
		else if (featureSelectionMethod == _efobpose || featureSelectionMethod == _efobpose_direction || featureSelectionMethod == _efobpose_label || featureSelectionMethod == _efobpose_rough_direction || featureSelectionMethod == _efobpose_slide_flip_tilt) //suitable for Moore machines
			cryssmex.initializeInputQuantizer (learningData.motorVectorSizeMarkov);
		else if (featureSelectionMethod == _mcobpose_obpose_direction)
			cryssmex.initializeInputQuantizer (2*learningData.motorVectorSizeMarkov + learningData.pfVectorSize);

	}
		

	if (featureSelectionMethod == _obpose || featureSelectionMethod == _obpose_direction || featureSelectionMethod == _efobpose || featureSelectionMethod == _efobpose_direction || featureSelectionMethod == _mcobpose_obpose_direction)
	{
		if (vm.count("output_quantizer"))
		{
			cryssmex.setOutputQuantizer (vm["output_quantizer"].as<string>());
			assert (learningData.pfVectorSize == cryssmex.getOutputQuantizer()->dimensionality());
			
		}
		// Initialize Output Quantizer 
		else
			cryssmex.initializeOutputQuantizer (learningData.pfVectorSize);
			

	}
	
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
	
	if (vm.count("seqFile"))
		cryssmex.setData (vm["seqFile"].as<string>(), data, learningData.featLimits, normalization, featureSelectionMethod);

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
		//move the finger to the beginning of experiment trajectory
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
			cryssmex.trainInputQuantizer (iteration, learningData.currentChunkSeq, learningData.featLimits, normalization, featureSelectionMethod);
			cryssmex.trainOutputQuantizer (iteration, learningData.currentChunkSeq, learningData.featLimits, normalization, featureSelectionMethod);
			// wait until learning is performed
			cryssmex.waitForInputQuantizer ();
			if (featureSelectionMethod == _obpose || featureSelectionMethod == _obpose_direction || featureSelectionMethod == _efobpose || featureSelectionMethod == _efobpose_direction || featureSelectionMethod == _mcobpose_obpose_direction)
				cryssmex.waitForOutputQuantizer ();
		}
		arm->moveFingerToStartPose(context);
		context.getMessageStream()->write(Message::LEVEL_INFO, "Done");
		//finish this iteration
		cout << "Iteration " << iteration << " completed!" << endl;
		if (universe.interrupted()) throw Interrupted();

	}
	//move the arm to its initial position
	arm->moveArmToStartPose(context);
	cryssmex.saveInputQuantizer ();
	cryssmex.saveOutputQuantizer ();
	writeData ();
}

} // namespace smlearning
