/** @file CrySSMEx.cpp
 * 
 * @author	Sergio Roa (DFKI)
 *
 * @version 2.0 beta
 *
 */
#include <metalearning/CrySSMEx.h>

namespace smlearning {

CrySSMEx::CrySSMEx ()
{
	ssm = 0;
	ssm_parser = 0;
	input_quantizer = 0;
	output_quantizer = 0;
	state_quantizer = 0;
}

CrySSMEx::~CrySSMEx ()
{
	if (ssm != 0)
		delete ssm;
	if (ssm_parser != 0)
		delete ssm_parser;
	if (input_quantizer != 0)
		delete input_quantizer;
	if (output_quantizer != 0)
		delete output_quantizer;
	if (state_quantizer != 0)
		delete state_quantizer;

}

CrySSMEx::CrySSMEx (const CrySSMEx& c)
{
	/*	if (c.ssm != 0)
	{
		if (c.ssm->is_moore ())
			ssm = new Moore_SSM (static_cast<Moore_SSM&>(*c.ssm));
	}
	else ssm = 0;
			
	c.ssm != 0 ? ssm_parser = new SSM_Parser (*ssm) : ssm_parser = 0; 
	c.input_quantizer != 0 ? input_quantizer = new GNG_Quantizer (static_cast<GNG_Quantizer&>(*c.input_quantizer)) : input_quantizer = 0;
	c.output_quantizer != 0 ? output_quantizer = new GNG_Quantizer (static_cast<GNG_Quantizer&>(*c.output_quantizer)) : output_quantizer = 0;
	c.state_quantizer != 0 ? state_quantizer = new CVQ (*c.state_quantizer) : state_quantizer = 0;*/
	*this = c;
	
}

const CrySSMEx& CrySSMEx::operator= (const CrySSMEx& c)
{
	if (c.ssm != 0)
	{
		if (c.ssm->is_moore ())
			ssm = new Moore_SSM (static_cast<Moore_SSM&>(*c.ssm));
	}
	else ssm = 0;
			
	c.ssm != 0 ? ssm_parser = new SSM_Parser (*ssm) : ssm_parser = 0; 
	c.input_quantizer != 0 ? input_quantizer = new GNG_Quantizer (static_cast<GNG_Quantizer&>(*c.input_quantizer)) : input_quantizer = 0;
	c.output_quantizer != 0 ? output_quantizer = new GNG_Quantizer (static_cast<GNG_Quantizer&>(*c.output_quantizer)) : output_quantizer = 0;
	c.state_quantizer != 0 ? state_quantizer = new CVQ (*c.state_quantizer) : state_quantizer = 0;
	
}


void CrySSMEx::setSSM (std::string ssmfile)
{
	try {
		ssm = load (ssmfile);
		if (ssm) {
			ssm_parser = new SSM_Parser (*ssm);
		}
	}
	catch(boost::archive::archive_exception) {
		std::cerr << ssmfile << " could not be loaded.\n";
		assert(ssm==0 && ssm_parser==0);
		exit (-1);
	}
	catch(const cryssmex_exception& e) {
		std::cerr << "EXCEPTION: " << e.what() << std::endl;
		delete ssm;
		delete ssm_parser;
		ssm = 0;
		ssm_parser = 0;
		exit (-1);
	}

}

void CrySSMEx::initializeInputQuantizer (unsigned int dim)
{
	input_quantizer = new GNG_Quantizer (dim);
}

void CrySSMEx::initializeOutputQuantizer (unsigned int dim)
{
	output_quantizer = new GNG_Quantizer (dim);
}

void CrySSMEx::initializeStateQuantizer (unsigned int dim)
{
	state_quantizer = new CVQ (dim);
}

void CrySSMEx::setInputQuantizer (const GNG_Quantizer& q)
{
	input_quantizer = new GNG_Quantizer (q);
}

void CrySSMEx::setOutputQuantizer (const GNG_Quantizer& q)
{
	output_quantizer = new GNG_Quantizer (q);
}

void CrySSMEx::setStateQuantizer (const CVQ& q)
{
	state_quantizer = new CVQ (q);
}

void CrySSMEx::setInputQuantizer (std::string inputqfile)
{
	try {
		input_quantizer = load_quantizer (inputqfile);
		if (!input_quantizer) {
			std::cerr << inputqfile << " did not load correctly. Aborting.\n";
			exit(-1);
		}
	}
	catch(boost::archive::archive_exception) {
		std::cerr << inputqfile << " did not load correctly. Aborting.\n";
		assert(input_quantizer == 0);
		exit(-1);
	}

}

void CrySSMEx::setOutputQuantizer (std::string outputqfile)
{
	try {
		output_quantizer = load_quantizer (outputqfile);
		if (!output_quantizer) {
			std::cerr << outputqfile << " did not load correctly. Aborting.\n";
			exit(-1);
		}
	}
	catch(boost::archive::archive_exception) {
		std::cerr << outputqfile << " did not load correctly. Aborting.\n";
		assert(output_quantizer == 0);
		exit(-1);
	}

}

void CrySSMEx::setStateQuantizer (std::string cvqfile)
{
	try {
		state_quantizer = static_cast<CVQ*>(load_quantizer (cvqfile));
		if (!state_quantizer) {
			std::cerr << cvqfile << " could not be loaded.\n";
			exit(-1);
		}
	}
	catch(boost::archive::archive_exception) {
		std::cerr << cvqfile << " could not be loaded.\n";
		assert(state_quantizer == 0);
		exit(-1);
	}

	static_cast<CVQ*>(state_quantizer)->quantization_model_vectors_map (qnt_mv_map);
	std::cout << "map size: " << qnt_mv_map.size() << std::endl;
	for (unsigned int i=0; i<qnt_mv_map.size() ; i++)
		std::cout << "map[" << i << "]: " << qnt_mv_map[i] << std::endl;

}

std::vector<double> CrySSMEx::getQntMvMapVector (unsigned int index)
{
	vector<double> dummy;
	if (qnt_mv_map[index].size () > 0)
		return qnt_mv_map[index][0].vector;
	else
		return dummy;
}


std::pair<int, int> CrySSMEx::parseInput (std::vector<double>& chunk)
{
	ssm_parser->set_state(Distribution().make_uniform(ssm->state_count()));
	unsigned int input = static_cast<GNG_Quantizer*>(input_quantizer)->quantize (chunk);
	std::cout << "parsing " << input << std::endl;
	ssm_parser->parse (input);
	std::cout << "parser output: " << ssm_parser->output()
		  << ", (Q: " << ssm_parser->state() << "), ";
	int state;
	const Distribution& _state = ssm_parser->state().distr();
	if (_state.size() == 1)
		state = _state.distr().begin()->first;
	else if (_state.size() == 0)
		state = -1;
	else
		state = _state.max_p ();
	std::cout  << "MV: " << qnt_mv_map[state] << std::endl;
	int output;
	const Distribution& _output = ssm_parser->output().distr();
	if (_output.size() == 1)
		output = _output.distr().begin()->first;
	else if (_output.size() == 0)
		output = -1;
	else
		output = _output.max_p ();
	// std::cout << "Entropy: " << ssm_parser->state().entropy() << std::endl;
	return make_pair(state,output);
}

void CrySSMEx::parseSequence (DataSequence& sequence, DataSequence& predicted_sequence)
{
	predicted_sequence.clear ();
	ssm_parser->set_state(Distribution().make_uniform(ssm->state_count()));
	for (unsigned int i=0; i<sequence.size() && !ssm_parser->state().empty(); i++)
	{
		std::pair<int, int> result = parseInput (sequence[i]);
		if (result.first >= 0)
			predicted_sequence.push_back (qnt_mv_map[result.first][0].vector);
	}
}

void CrySSMEx::setUniformDistribution ()
{
	ssm_parser->set_state(Distribution().make_uniform(ssm->state_count()));
}

void CrySSMEx::averageModelVectors ()
{
	for (unsigned int i=0; i<qnt_mv_map.size() ; i++)
	{
		if (qnt_mv_map[i].size() > 1)
		{
			Classified_Vector avg (qnt_mv_map[i][0].vector, qnt_mv_map[i][0].classification);
			for (unsigned int j=1; j<qnt_mv_map[i].size(); j++)
				avg.vector += qnt_mv_map[i][j].vector;
			avg.vector /= qnt_mv_map[i].size();
			qnt_mv_map[i].clear();
			qnt_mv_map[i].push_back (avg);
		}
		else if (qnt_mv_map[i].size() == 0)
		{
			std::cerr << "State vector has no model vectors" << std::endl;
			abort ();
		}
	}

}

void CrySSMEx::saveInputQuantizer (string filename) {

	if (input_quantizer->is_open ())
		input_quantizer->close ();
	save_quantizer (input_quantizer, filename);

}

void CrySSMEx::saveOutputQuantizer (string filename) {

	if (output_quantizer->is_open ())
		output_quantizer->close ();
	save_quantizer (output_quantizer, filename);

}

void CrySSMEx::resetLearning ()
{
	static_cast<GNG_Quantizer*>(input_quantizer)->resetLearning ();
	static_cast<GNG_Quantizer*>(output_quantizer)->resetLearning ();
}


void CrySSMEx::setData (string seqFile, LearningData::DataSet& data, LearningData::FeaturesLimits& featLimits, boost::function<float (const float&, const float&, const float&)> normalization, unsigned int featureSelectionMethod)
{
	if (!LearningData::read_dataset (seqFile, data, featLimits )) {
		cerr << "error reading data" << endl;
		exit(-1);
	}
	setData (data, featLimits, normalization, featureSelectionMethod);
}

void CrySSMEx::setData (LearningData::DataSet& data, LearningData::FeaturesLimits& featLimits, boost::function<float (const float&, const float&, const float&)> normalization, unsigned int featureSelectionMethod) {

	assert (data.size());
	// This data copying is really hacky... 
	std::vector<neuralgas::Vector<double>*>* inputData = new std::vector<neuralgas::Vector<double>*>;

	//Get input feature vectors
	for (unsigned int i=0; i<data.size(); i++)
	{
		vector<FeatureVector> inputSeq = LearningData::load_cryssmexinputsequence (data[i], featureSelectionMethod, normalization, featLimits);
		for (unsigned int j=0; j<inputSeq.size(); j++)
		{
			neuralgas::Vector<double> *new_item = new neuralgas::Vector<double>(inputSeq[j]);
			inputData->push_back (new_item);
		}
	}
	static_cast<GNG_Quantizer*>(input_quantizer)->setData (inputData);
	for(unsigned int i=0; i < inputData->size(); i++)
		delete (*inputData)[i];
	inputData->clear();
	delete inputData;

	//Get output feature vectors
	if (featureSelectionMethod == _obpose || featureSelectionMethod == _obpose_direction || featureSelectionMethod == _efobpose || featureSelectionMethod == _efobpose_direction || featureSelectionMethod == _mcobpose_obpose_direction)
	{
		std::vector<neuralgas::Vector<double>*>* outputData = new std::vector<neuralgas::Vector<double>*>;
		for (unsigned int i=0; i<data.size(); i++)
		{
			vector<FeatureVector> outputSeq = LearningData::load_cryssmexoutputsequence (data[i], featureSelectionMethod, normalization, featLimits);
			for (unsigned int j=0; j<outputSeq.size(); j++)
			{
				neuralgas::Vector<double> *new_item = new neuralgas::Vector<double>(outputSeq[j]);
				outputData->push_back (new_item);
			}
		}
		static_cast<GNG_Quantizer*>(output_quantizer)->setData (outputData);
		for (unsigned int i=0; i<outputData->size(); i++)
			delete (*outputData)[i];
		outputData->clear();
		delete outputData;
	}

	resetLearning ();
	
}

void CrySSMEx::findDislocatedNodes ()
{
	GNG_Quantizer* inputQuantizer = static_cast<GNG_Quantizer*>(input_quantizer);
	GNG_Quantizer* outputQuantizer = static_cast<GNG_Quantizer*>(output_quantizer);
	if (inputQuantizer->size())
	{
		inputQuantizer->calculateValueRange ();
		inputQuantizer->calculateMinimumDescriptionLength ();
		inputQuantizer->findDislocatedNodesStableGraph ();
	}
	if (outputQuantizer->size())
	{
		outputQuantizer->calculateValueRange ();
		outputQuantizer->calculateMinimumDescriptionLength ();
		outputQuantizer->findDislocatedNodesStableGraph ();
	}

}

void CrySSMEx::trainInputQuantizer (int iteration, LearningData::Chunk::Seq& currentChunkSeq, LearningData::FeaturesLimits& featLimits, boost::function<float (const float&, const float&, const float&)> normalization, unsigned int featureSelectionMethod) {

	vector<FeatureVector> currentInputSeq = LearningData::load_cryssmexinputsequence (currentChunkSeq, featureSelectionMethod, normalization, featLimits);
	// hacky conversion
	vector<neuralgas::Vector<double>* >* newInputSeq = new vector<neuralgas::Vector<double>* >;
	for (unsigned int i=0; i<currentInputSeq.size(); i++)
	{
		neuralgas::Vector<double> *new_item = new neuralgas::Vector<double>(currentInputSeq[i]);
		newInputSeq->push_back (new_item);
	}
	GNG_Quantizer* inputQuantizer = static_cast<GNG_Quantizer*>(input_quantizer);
	inputQuantizer->addData (newInputSeq);

	if (iteration == 0)
	{
		if (inputQuantizer->graphsize() == 0)
			inputQuantizer->setRefVectors(2);
	}
	inputQuantizer->open();
	inputQuantizer->resetLearning ();
	inputQuantizer->setInsertionRate (newInputSeq->size());

	for (unsigned int i=0; i < newInputSeq->size(); i++)
		delete (*newInputSeq)[i];
	newInputSeq->clear();
	delete newInputSeq;

	inputQuantizer->begin ();

}

void CrySSMEx::trainOutputQuantizer (int iteration, LearningData::Chunk::Seq& currentChunkSeq, LearningData::FeaturesLimits& featLimits, boost::function<float (const float&, const float&, const float&)> normalization, unsigned int featureSelectionMethod) {

	if (featureSelectionMethod == _obpose || featureSelectionMethod == _obpose_direction || featureSelectionMethod == _efobpose || featureSelectionMethod == _efobpose_direction || featureSelectionMethod == _mcobpose_obpose_direction)
	{

		vector<FeatureVector> currentOutputSeq = LearningData::load_cryssmexoutputsequence (currentChunkSeq, featureSelectionMethod, normalization, featLimits);
		// hacky conversion
		vector<neuralgas::Vector<double>* >* newOutputSeq = new vector<neuralgas::Vector<double>* >;
		for (unsigned int i=0; i<currentOutputSeq.size(); i++)
		{
			neuralgas::Vector<double> *new_item = new neuralgas::Vector<double>(currentOutputSeq[i]);
			newOutputSeq->push_back (new_item);
		}
		GNG_Quantizer* outputQuantizer = static_cast<GNG_Quantizer*>(output_quantizer);
		outputQuantizer->addData (newOutputSeq);

		if (iteration == 0)
		{
			if (outputQuantizer->graphsize() == 0)
				outputQuantizer->setRefVectors(2);
		}
		outputQuantizer->open();
		outputQuantizer->resetLearning ();

		outputQuantizer->setInsertionRate (newOutputSeq->size());

		for (unsigned int i=0; i < newOutputSeq->size(); i++)
			delete (*newOutputSeq)[i];
		newOutputSeq->clear();
		delete newOutputSeq;

		outputQuantizer->begin ();
	}

}

void CrySSMEx::waitForInputQuantizer () {
	static_cast<GNG_Quantizer*>(input_quantizer)->wait ();
	static_cast<GNG_Quantizer*>(input_quantizer)->showGraph ();
	input_quantizer->close ();

}

void CrySSMEx::waitForOutputQuantizer () {
	static_cast<GNG_Quantizer*>(output_quantizer)->wait ();
	static_cast<GNG_Quantizer*>(output_quantizer)->showGraph ();
	output_quantizer->close ();
}


void ActiveCrySSMEx::initializeInputQuantizer (unsigned int dim)
{
	input_quantizer = new ActiveGNG_Quantizer (dim);
}

void ActiveCrySSMEx::initializeOutputQuantizer (unsigned int dim)
{
	output_quantizer = new ActiveGNG_Quantizer (dim);
}

void ActiveCrySSMEx::setData (string seqFile, LearningData::DataSet& data, LearningData::FeaturesLimits& featLimits, boost::function<float (const float&, const float&, const float&)> normalization, unsigned int featureSelectionMethod, int index) {
	if (!LearningData::read_dataset (seqFile, data, featLimits )) {
		cerr << "error reading data" << endl;
		exit(-1);
	}

	assert (data.size());
	if (index < 0)
		index = data.size();
	assert (index <= data.size());
	// This data copying is really hacky... 
	std::vector<neuralgas::Vector<double>*>* inputData = new std::vector<neuralgas::Vector<double>*>;

	//Get input feature vectors
	for (unsigned int i=0; i<index; i++)
	{
		vector<FeatureVector> inputSeq = LearningData::load_cryssmexinputsequence (data[i], featureSelectionMethod, normalization, featLimits);
		for (unsigned int j=0; j<inputSeq.size(); j++)
		{
			neuralgas::Vector<double> *new_item = new neuralgas::Vector<double>(inputSeq[j]);
			inputData->push_back (new_item);
		}
	}
	static_cast<ActiveGNG_Quantizer*>(input_quantizer)->setData (inputData);
	for(unsigned int i=0; i < inputData->size(); i++)
		delete (*inputData)[i];
	inputData->clear();
	delete inputData;

	//Get output feature vectors
	if (featureSelectionMethod == _obpose || featureSelectionMethod == _obpose_direction || featureSelectionMethod == _efobpose || featureSelectionMethod == _efobpose_direction || featureSelectionMethod == _mcobpose_obpose_direction)
	{
		std::vector<neuralgas::Vector<double>*>* outputData = new std::vector<neuralgas::Vector<double>*>;
		for (unsigned int i=0; i<index; i++)
		{
			vector<FeatureVector> outputSeq = LearningData::load_cryssmexoutputsequence (data[i], featureSelectionMethod, normalization, featLimits);
			for (unsigned int j=0; j<outputSeq.size(); j++)
			{
				neuralgas::Vector<double> *new_item = new neuralgas::Vector<double>(outputSeq[j]);
				outputData->push_back (new_item);
			}
		}
		static_cast<ActiveGNG_Quantizer*>(output_quantizer)->setData (outputData);
		for (unsigned int i=0; i<outputData->size(); i++)
			delete (*outputData)[i];
		outputData->clear();
		delete outputData;
	}

	
}


void ActiveCrySSMEx::trainInputQuantizer (int iteration, LearningData::Chunk::Seq& currentChunkSeq, LearningData::FeaturesLimits& featLimits, boost::function<float (const float&, const float&, const float&)> normalization, unsigned int featureSelectionMethod) {

	vector<FeatureVector> currentInputSeq = LearningData::load_cryssmexinputsequence (currentChunkSeq, featureSelectionMethod, normalization, featLimits);
	// hacky conversion
	vector<neuralgas::Vector<double>* >* newInputSeq = new vector<neuralgas::Vector<double>* >;
	for (unsigned int i=0; i<currentInputSeq.size(); i++)
	{
		neuralgas::Vector<double> *new_item = new neuralgas::Vector<double>(currentInputSeq[i]);
		newInputSeq->push_back (new_item);
	}
	ActiveGNG_Quantizer* inputQuantizer = static_cast<ActiveGNG_Quantizer*>(input_quantizer);
	inputQuantizer->addData (newInputSeq);

	if (iteration == 0)
	{
		if (inputQuantizer->graphsize() == 0)
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

void ActiveCrySSMEx::trainOutputQuantizer (int iteration, LearningData::Chunk::Seq& currentChunkSeq, LearningData::FeaturesLimits& featLimits, boost::function<float (const float&, const float&, const float&)> normalization, unsigned int featureSelectionMethod) {

	if (featureSelectionMethod == _obpose || featureSelectionMethod == _obpose_direction || featureSelectionMethod == _efobpose || featureSelectionMethod == _efobpose_direction || featureSelectionMethod == _mcobpose_obpose_direction)
	{

		vector<FeatureVector> currentOutputSeq = LearningData::load_cryssmexoutputsequence (currentChunkSeq, featureSelectionMethod, normalization, featLimits);
		// hacky conversion
		vector<neuralgas::Vector<double>* >* newOutputSeq = new vector<neuralgas::Vector<double>* >;
		for (unsigned int i=0; i<currentOutputSeq.size(); i++)
		{
			neuralgas::Vector<double> *new_item = new neuralgas::Vector<double>(currentOutputSeq[i]);
			newOutputSeq->push_back (new_item);
		}
		ActiveGNG_Quantizer* outputQuantizer = static_cast<ActiveGNG_Quantizer*>(output_quantizer);
		outputQuantizer->addData (newOutputSeq);

		if (iteration == 0)
		{
			if (outputQuantizer->graphsize() == 0)
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

void ActiveCrySSMEx::waitForInputQuantizer () {
	static_cast<ActiveGNG_Quantizer*>(input_quantizer)->wait ();
	static_cast<ActiveGNG_Quantizer*>(input_quantizer)->showGraph ();
}

void ActiveCrySSMEx::waitForOutputQuantizer () {
	static_cast<ActiveGNG_Quantizer*>(output_quantizer)->wait ();
	static_cast<ActiveGNG_Quantizer*>(output_quantizer)->showGraph ();
}


}; // smlearning namespace
