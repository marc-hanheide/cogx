/** @file offlineactivegng_experiment.cpp
 * 
 * @author	Sergio Roa (DFKI)
 *
 * @version 2.0 beta
 *
 */

#include <boost/iostreams/tee.hpp>
#include <boost/iostreams/stream.hpp>
#include <metalearning/Scenario.h>
#include <metalearning/CrySSMEx.h>

#include <boost/program_options.hpp>

#include <iostream>

namespace po = boost::program_options;

using namespace std;
using namespace neuralgas;
using namespace smlearning;


int main (int argc, char* argv[])
{
	
	po::options_description desc("Allowed parameters:");

	desc.add_options ()
		("help,h", "produce help message")
		("featuresel,f", po::value<string>()->default_value ("obpose_direction"), "Feature selection method\n(obpose|obpose_label|\nobpose_direction|obpose_slide_flip_tilt\nefobpose|efobpose_label\nefobpose_direction|efobpose_slide_flip_tilt\nmcobpose_obpose_direction)")
		("normalize,n", "normalize sequences")
		("maxepochsmdl,e", po::value<unsigned int>()->default_value (100), "Set maximum nr of epochs after mdl reduction is expected (for GNG based quantizing). This value should be proportionally changed depending on maxepochserror parameter.")
		("maxepochserror", po::value<unsigned int>()->default_value (1), "Set maximum nr of epochs after error reduction is expected (for GNG based quantizing)")
		("modelefficiency,c", po::value<double>()->default_value (5), "model efficiency constant (for GNG based quantizing) for input space")
		("output_modelefficiency", po::value<double>()->default_value(10), "model efficiency constant (for GNG based quantizing) for output space")
		("learningrate,l", po::value<double>()->default_value (0.8), "default learning rate for winner nodes (for GNG based quantizing)")
		("accuracy,y", po::value<double>()->default_value(0.001), "data accuracy constant for input space GNG quantization")
		("output_accuracy", po::value<double>()->default_value (0.0001), "data accuracy constant for output space GNG quantization")
		("input_quantizer,i", po::value<string>(), "Pretrained Input quantizer if available")
		("output_quantizer,o", po::value<string>(), "Pretrained Output quantizer if available")
		("seqFile,d", po::value<string>(), "name of file containing data sequences\n(do not type .seq extension) if available")
		("index,x", po::value<int>()->default_value(-1), "Index in data set, from which the active learning process should start")
		("inputout", po::value<string>(), "file for redirecting input space quantization output")
		("outputout", po::value<string>(), "file for redirecting output space quantization output")
		("mdl,m", "save MDL history to a file mdl.txt");

	// Declare an options description instance which will include
	// all the options
	po::options_description all("Basic usage:");
	all.add(desc);

	po::variables_map vm;
	po::store(po::command_line_parser(argc, argv).
		  options(all).run(), vm);
	po::notify(vm);


	// Set feature selection method
	string fSMethod;
	fSMethod = vm["featuresel"].as<string>();
	unsigned int featureSelectionMethod;

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

	if (vm.count("help")) {
		cout << desc << "\n";
		return 0;
	}

	ActiveCrySSMEx cryssmex;

	if (vm.count("input_quantizer"))
	{
		cryssmex.setInputQuantizer (vm["input_quantizer"].as<string>());
		if (featureSelectionMethod == _obpose || featureSelectionMethod == _obpose_direction || featureSelectionMethod == _obpose_label || featureSelectionMethod == _obpose_rough_direction || featureSelectionMethod == _obpose_slide_flip_tilt) //suitable for Mealy machines
			assert (LearningData::motorVectorSizeMarkov + LearningData::efVectorSize == cryssmex.getInputQuantizer()->dimensionality());

		else if (featureSelectionMethod == _efobpose || featureSelectionMethod == _efobpose_direction || featureSelectionMethod == _efobpose_label || featureSelectionMethod == _efobpose_rough_direction || featureSelectionMethod == _efobpose_slide_flip_tilt) //suitable for Moore machines
			assert (LearningData::motorVectorSizeMarkov == cryssmex.getInputQuantizer()->dimensionality());
		else if (featureSelectionMethod == _mcobpose_obpose_direction)
			assert (2*LearningData::motorVectorSizeMarkov + LearningData::pfVectorSize == cryssmex.getInputQuantizer()->dimensionality());

	}
	else
	{
		// Initialize Input Quantizer
		if (featureSelectionMethod == _obpose || featureSelectionMethod == _obpose_direction || featureSelectionMethod == _obpose_label || featureSelectionMethod == _obpose_rough_direction || featureSelectionMethod == _obpose_slide_flip_tilt) //suitable for Mealy machines
			cryssmex.initializeInputQuantizer (LearningData::motorVectorSizeMarkov + LearningData::efVectorSize);
	
		else if (featureSelectionMethod == _efobpose || featureSelectionMethod == _efobpose_direction || featureSelectionMethod == _efobpose_label || featureSelectionMethod == _efobpose_rough_direction || featureSelectionMethod == _efobpose_slide_flip_tilt) //suitable for Moore machines
			cryssmex.initializeInputQuantizer (LearningData::motorVectorSizeMarkov);
		else if (featureSelectionMethod == _mcobpose_obpose_direction)
			cryssmex.initializeInputQuantizer (2*LearningData::motorVectorSizeMarkov + LearningData::pfVectorSize);

	}
		

	if (featureSelectionMethod == _obpose || featureSelectionMethod == _obpose_direction || featureSelectionMethod == _efobpose || featureSelectionMethod == _efobpose_direction || featureSelectionMethod == _mcobpose_obpose_direction)
	{
		if (vm.count("output_quantizer"))
		{
			cryssmex.setOutputQuantizer (vm["output_quantizer"].as<string>());
			assert (LearningData::pfVectorSize == cryssmex.getOutputQuantizer()->dimensionality());
			
		}
		// Initialize Output Quantizer 
		else
			cryssmex.initializeOutputQuantizer (LearningData::pfVectorSize);
			

	}
	

	// Initialize state quantizer
	if (featureSelectionMethod == _obpose || featureSelectionMethod == _obpose_direction || featureSelectionMethod == _obpose_label || featureSelectionMethod == _obpose_rough_direction || featureSelectionMethod == _obpose_slide_flip_tilt || featureSelectionMethod == _mcobpose_obpose_direction) //suitable for Mealy machines
		cryssmex.initializeStateQuantizer (LearningData::pfVectorSize);

	else if (featureSelectionMethod == _efobpose || featureSelectionMethod == _efobpose_direction || featureSelectionMethod == _efobpose_label || featureSelectionMethod == _efobpose_rough_direction || featureSelectionMethod == _efobpose_slide_flip_tilt) //suitable for Moore machines
		cryssmex.initializeStateQuantizer (LearningData::efVectorSize + LearningData::pfVectorSize);

	// Set (de)normalization functions
	boost::function<float (const float&, const float&, const float&)> normalization;
	boost::function<float (const float&, const float&, const float&)> denormalization;
	normalization = donotnormalize<float>;
	denormalization = donotdenormalize<float>;

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

	LearningData::DataSet data;
	LearningData::FeaturesLimits limits;

	if (!vm.count("seqFile"))
	{
		std::cerr << "A sequences data file must be provided." << std::endl;
		return 1;
	}
	int index = vm["index"].as<int>();
			
	cryssmex.setData (vm["seqFile"].as<string>(), data, limits, normalization, featureSelectionMethod, vm["index"].as<int>());

	if (index < 0)
	{
		std::cout << "Using the whole data set for learning" << std::endl;
		index = 0;
	}


	if (vm.count("mdl"))
	{
		inputQuantizer->saveMDLHistory ("mdlinput.txt");
		outputQuantizer->saveMDLHistory ("mdloutput.txt");
	}

	if (vm.count("inputout"))
		inputQuantizer->redirectOutput (vm["inputout"].as<string>());
	if (vm.count("outputout"))
		outputQuantizer->redirectOutput (vm["outputout"].as<string>());

	for (unsigned int iter = 0, i=index; i<data.size(); i++, iter++)
	{
		cryssmex.trainInputQuantizer (iter, data[i], limits, normalization, featureSelectionMethod);
		cryssmex.trainOutputQuantizer (iter, data[i], limits, normalization, featureSelectionMethod);
		// wait until learning is performed
		cryssmex.waitForInputQuantizer ();
		if (featureSelectionMethod == _obpose || featureSelectionMethod == _obpose_direction || featureSelectionMethod == _efobpose || featureSelectionMethod == _efobpose_direction || featureSelectionMethod == _mcobpose_obpose_direction)
			cryssmex.waitForOutputQuantizer ();

	}
	cryssmex.saveInputQuantizer ();
	cryssmex.saveOutputQuantizer ();

	return 0;
		
}
