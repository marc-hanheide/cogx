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
}

PredictingScenario::~PredictingScenario()
{
	if (ssm != 0)
		delete ssm;
	if (ssm_parser != 0)
		delete ssm_parser;
}

void PredictingScenario::init (boost::program_options::variables_map vm) {
	Scenario::init (vm);

	string fSMethod;
	if (vm.count("featureSelectionMethod")) {
		fSMethod = vm["featureSelectionMethod"].as<string>();
	}

	if (fSMethod == "_obpose")
		featureSelectionMethod = _obpose;
	else if (fSMethod == "_efobpose")
		featureSelectionMethod = _efobpose;
	else if (fSMethod == "_obpose_direction")
		featureSelectionMethod = _obpose_direction;
	else if (fSMethod == "_efobpose_direction")
		featureSelectionMethod = _efobpose_direction;

	//TODO: load CrySSMEx machine
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


	}
}

} // namespace smlearning

