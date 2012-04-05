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
		state_quantizer = load_quantizer (cvqfile);
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

void CrySSMEx::parseSequence (DataSequence& sequence)
{
	ssm_parser->set_state(Distribution().make_uniform(ssm->state_count()));
	for (unsigned int i=0; i<sequence.size() && !ssm_parser->state().empty(); i++)
	{
		unsigned int input = static_cast<GNG_Quantizer*>(input_quantizer)->quantize (sequence[i]);
		std::cout << "parsing " << input << std::endl;
		ssm_parser->parse (input);
		std::cout << "parser output: " << ssm_parser->output()
			  << ", (Q: " << ssm_parser->state() << "), ";
		unsigned int state;
		const Distribution& _state = ssm_parser->state().distr();
		if (_state.size() == 1)
			state = _state.distr().begin()->first;
		else
			state = _state.max_p ();
		std::cout  << "MV: " << qnt_mv_map[state] << std::endl;
		predicted_sequence.push_back (qnt_mv_map[state][0].vector);
		
	}
}

void CrySSMEx::average_model_vectors ()
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
	}

}

}; // smlearning namespace
