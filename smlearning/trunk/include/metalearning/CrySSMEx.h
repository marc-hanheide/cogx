/** @file CrySSMEx.h
 * 
 * @author	Sergio Roa (DFKI)
 *
 * @version 2.0 beta
 *
 */
#ifndef SMLEARNING_CRYSSMEX_H_
#define SMLEARNING_CRYSSMEX_H_


#include "ssm/ssm_file_export.hh"
#include "ssm/ssm_parser.hh"
#include "cryssmex/exceptions.hh"
#include "quantizing/quantizer_file_export.hh"

using namespace ssm;
using namespace cryssmex;
using namespace basic_stochastics;
using namespace naming;
using namespace quantizing;

namespace smlearning {

typedef std::vector<std::vector<double> > DataSequence;

//! \class CrySSMEx
/*! \brief Encapsulation of Crystallyzing Substochastic sequential machines extractors
 */
class CrySSMEx
{
public:
	/** default constructor */
	CrySSMEx ();
	/** default destructor */
	~CrySSMEx ();
	/** set Substochastic Sequential Machine from file */
	void setSSM (std::string ssmfile);
	/** set Input Quantizer from file */
	void setInputQuantizer (std::string inputqfile);
	/** set Output Quantizer from file */
	void setOutputQuantizer (std::string outputqfile);
	/** set State Quantizer from file */
	void setStateQuantizer (std::string cvqfile);
	/** get Substochastic Sequential Machine from file */
	SSM* getSSM () { return ssm; }
	/** get Input Quantizer from file */
	Quantizer* getInputQuantizer () { return input_quantizer; }
	/** get Output Quantizer from file */
	Quantizer* getOutputQuantizer () { return output_quantizer; }
	/** get State Quantizer from file */
	Quantizer* getStateQuantizer () { return state_quantizer; }
	/** parse an input data sequence for prediction */
	void parseSequence (DataSequence& sequence, DataSequence& predicted_sequence);
	/** parse an input chunk for prediction */
	int parseInput (std::vector<double>& chunk);
	/** average model vectors for cases with more than 1 model vector in \p qnt_mv_map */
	void average_model_vectors ();
	/** get quantization/model vectors map */
	std::vector<double> getQntMvMapVector (unsigned int index) { return qnt_mv_map[index][0].vector; }
	/** set uniform distribution to initialize parsing */
	void setUniformDistribution ();
	
private:	
	/** Substochastic sequential machine used for prediction */
	SSM* ssm;
	/** SSM parser used for prediction */
	SSM_Parser *ssm_parser;
	/** Input quantizer */
	Quantizer *input_quantizer;
	/** Output quantizer */
	Quantizer *output_quantizer;
	/** State quantizer */
	Quantizer *state_quantizer;
	/** map of quantization ids and CVQ classified model vectors */
	std::map<unsigned int, Classified_Vectors> qnt_mv_map;

};



}; // smlearning namespace

#endif // SMLEARNING_CRYSSMEX_H_
