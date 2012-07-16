/** @file OnlineLearning.cpp
 * 
 * @author	Sergio Roa (DFKI)
 *
 * @version 2.0 beta
 *
 */

#include <scenario/OnlineLearningDataset.h>

namespace smlearning {

OnlineLearningDataset::OnlineLearningDataset (golem::Scene &scene) :
	ActiveLearnScenario (scene)
{
	normalization = normalize<Real>;
	denormalization = denormalize<Real>;
	chosenAction = NULL;
	saveMDLHistory = false;
	currentRegion = NULL;
}

void OnlineLearningDataset::init (boost::program_options::variables_map vm)
{
	if (!vm.count("seqFile"))
	{
		cerr<< "You need to provide a sequence data file" << endl;
		exit (-1);
	}
	
	if (!LearningData::read_dataset (vm["seqFile"].as<string>(), data, learningData.featLimits)) {
		cerr << "error reading sequence data file" << endl;
		exit (-1);
	}
	dataFileName = vm["seqFile"].as<string>();
	//Store in feature vector the motor context
	for (int i=0; i<data.size(); i++)
		for (int j=0; j<data[i].size(); j++)
			LearningData::write_chunk_to_featvector (data[i][j].featureVector, data[i][j], normalization, learningData.featLimits, _end_effector_pos | _effector_pos | _object /*| _action_params*/ );
	
	splittingCriterion1 = vm["splitting"].as<unsigned int>();
	if (splittingCriterion1 < 2)
		splittingCriterion1 = 2;
	
	cout << "splitting criterion: " << splittingCriterion1 << endl;
	
	featureSelectionMethod = vm["featuresel"].as<feature_selection>();
	
	neargreedyActionProb = 1.0;
	startingPosition = 0;

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
	
	quantizing::GNG_Quantizer* inputQuantizer = static_cast<quantizing::GNG_Quantizer*>(currentRegion->cryssmex.getInputQuantizer());
	quantizing::GNG_Quantizer* outputQuantizer = static_cast<quantizing::GNG_Quantizer*>(currentRegion->cryssmex.getOutputQuantizer());
	
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

void OnlineLearningDataset::run (int argc, char* argv[])
{
	for (int iteration = 0; iteration<data.size(); iteration++)
	{
		// set current learning item
		learningData.currentChunkSeq = data[iteration];
		// update context region
		updateCurrentRegion ();
		currentRegion->data.push_back (learningData.currentChunkSeq);
		updateLearners (iteration);
		cout << "Iteration " << iteration << " completed!" << endl;	}
}

} // namespace smlearning
