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

PredictingScenario::PredictingScenario(golem::Scene &scene) : Scenario (scene)
{
	ssm = 0;
	ssm_parser = 0;
	input_quantizer = 0;
	output_quantizer = 0;
}

PredictingScenario::~PredictingScenario()
{
	if (ssm != 0)
		delete ssm;
	if (ssm_parser != 0)
		delete ssm_parser;
	if (input_quantizer != 0)
		delete input_quantizer;
	if (output_quantizer != 0)
		delete output_quantizer;
}

void PredictingScenario::init (boost::program_options::variables_map vm) {
	Scenario::init (vm);

	string fSMethod;
	if (vm.count("featuresel"))
		fSMethod = vm["featuresel"].as<string>();

	if (fSMethod == "obpose")
		featureSelectionMethod = _obpose;
	else if (fSMethod == "efobpose")
		featureSelectionMethod = _efobpose;
	else if (fSMethod == "obpose_direction")
		featureSelectionMethod = _obpose_direction;
	else if (fSMethod == "efobpose_direction")
		featureSelectionMethod = _efobpose_direction;
	else if (fSMethod == "obpose_rough_direction")
		featureSelectionMethod = _obpose_rough_direction;
	else if (fSMethod == "efobpose_rough_direction")
		featureSelectionMethod = _efobpose_rough_direction;
	else if (fSMethod == "obpose_slide_flip_tilt")
		featureSelectionMethod = _obpose_slide_flip_tilt;
	else if (fSMethod == "efobpose_slide_flip_tilt")
		featureSelectionMethod = _efobpose_slide_flip_tilt;
	
	try {
		ssm = load (vm["ssmFile"].as<string>() );
		if (ssm) {
			ssm_parser = new SSM_Parser (*ssm);
		}
	}
	catch(boost::archive::archive_exception) {
		cout << vm["ssmFile"].as<string>() << " could not be loaded. File ignored.\n";
		assert(ssm==0 && ssm_parser==0);
	}
	catch(const cryssmex_exception& e) {
		cerr << "EXCEPTION: " << e.what() << endl;
		delete ssm;
		delete ssm_parser;
		ssm = 0;
		ssm_parser = 0;
	}

	try {
		string inputqfile;
		if (vm.count("quantizersPath"))
			inputqfile =  vm["quantizersPath"].as<string>() + "/./" + "cryssmex_inputq.qnt";
		else
			inputqfile = "cryssmex_inputq.qnt";
		input_quantizer = load_quantizer (inputqfile);
		if (!input_quantizer) {
			cerr << "cryssmex_input.qnt did not load correctly. Aborting.\n";
			exit(-1);
		}
		if (featureSelectionMethod == _obpose || featureSelectionMethod == _obpose_direction || featureSelectionMethod == _obpose_label || featureSelectionMethod == _obpose_rough_direction || featureSelectionMethod == _obpose_slide_flip_tilt) //suitable for Mealy machines
			assert (learningData.motorVectorSizeMarkov + learningData.efVectorSize == input_quantizer->dimensionality());

		else if (featureSelectionMethod == _efobpose || featureSelectionMethod == _efobpose_direction || featureSelectionMethod == _efobpose_label || featureSelectionMethod == _efobpose_rough_direction || featureSelectionMethod == _efobpose_slide_flip_tilt) //suitable for Moore machines
			assert (learningData.motorVectorSizeMarkov == input_quantizer->dimensionality());

	}
	catch(boost::archive::archive_exception) {
		cerr << "cryssmex_input.qnt did not load correctly. Aborting.\n";
		exit(-1);
	}
	try {
		string outputqfile;
		if (vm.count("quantizersPath"))
			outputqfile =  vm["quantizersPath"].as<string>() + "/./" + "cryssmex_outputq.qnt";
		else
			outputqfile = "cryssmex_outputq.qnt";
		output_quantizer = load_quantizer (outputqfile);
		if (!output_quantizer) {
			cerr << "cryssmex_output.qnt did not load correctly. Aborting.\n";
			exit(-1);
		}
	}
	catch(boost::archive::archive_exception) {
		cerr << "cryssmex_output.qnt did not load correctly. Aborting.\n";
		exit(-1);
	}

	if (vm.count("seqFile"))
	{
		if (!LearningData::read_dataset (vm["seqFile"].as<string>(), data, learningData.featLimits )) {
			cerr << "error reading data" << endl;
			exit(-1);
		}
	}

}

///////// Protected //////////
/** \brief select a random action
*/
void PredictingScenario::chooseAction () {

	if (data.size() > 0)
	{
		int index = static_cast<int>(floor(randomG.nextUniform (0.0, Real(data.size() - 1))));
		learningData.currentChunkSeq = data[index];
	}
	else
		Scenario::chooseAction ();
}

///////// Protected //////////
/** \brief calculate the start coordinates of the arm
*/
void PredictingScenario::calculateStartCoordinates()
{
	if (data.size() > 0)
	{
		LearningData::Chunk chunk = learningData.currentChunkSeq[0];
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
void PredictingScenario::initMovement()
{
	if (data.size() > 0)
	{
		LearningData::Chunk chunk = learningData.currentChunkSeq[0];
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
		std::cout <<"choose action "<<std::endl;
		//select a random action
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
		arm->moveFinger(context,target, bStart,duration,end);
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
	arm->moveArmToStartPose(context); 	//move the arm to its initial position 

}

} // namespace smlearning

