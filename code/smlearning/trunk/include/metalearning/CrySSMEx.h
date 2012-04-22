/** @file CrySSMEx.h
 * 
 * @author	Sergio Roa (DFKI)
 *
 * @version 2.0 beta
 *
 */
#ifndef SMLEARNING_CRYSSMEX_H_
#define SMLEARNING_CRYSSMEX_H_


#include <boost/iostreams/tee.hpp>
#include <boost/iostreams/stream.hpp>
#include <metalearning/data_structs.h>
#include <quantizing/quantizer_file_export.hh>
#include <ssm/ssm_file_export.hh>
#include <ssm/ssm_parser.hh>
#include <cryssmex/exceptions.hh>
#include <boost/function.hpp>

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
	/** save input quantizer */
	void saveInputQuantizer ();
	/** save output quantizer */
	void saveOutputQuantizer ();
	/** parse an input data sequence for prediction */
	void parseSequence (DataSequence& sequence, DataSequence& predicted_sequence);
	/** parse an input chunk for prediction */
	virtual int parseInput (std::vector<double>& chunk);
	/** average model vectors for cases with more than 1 model vector in \p qnt_mv_map */
	void averageModelVectors ();
	/** get quantization/model vectors map */
	std::vector<double> getQntMvMapVector (unsigned int index) { return qnt_mv_map[index][0].vector; }
	/** set uniform distribution to initialize parsing */
	void setUniformDistribution ();
	
protected:
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

}; // CrySSMEx class

//! \class ActiveCrySSMEx
/*! \brief For dealing with active learning quantization algorithms
 */
class ActiveCrySSMEx : public CrySSMEx
{
public:
	/** initialize input quantizer */
	void initializeInputQuantizer (unsigned int dim);
	/** initialize output quantizer */
	void initializeOutputQuantizer (unsigned int dim);
	/** initialize state quantizer */
	void initializeStateQuantizer (unsigned int dim);
	/** set present data sequences */
	void setData (std::string seqFile, LearningData::DataSet& data, LearningData::FeaturesLimits& featLimits, boost::function<float (const float&, const float&, const float&)> normalization, unsigned int featureSelectionMethod, int index = -1);
	/** train input quantizer with a new data sequence */
	void trainInputQuantizer (int iteration, LearningData::Chunk::Seq& currentChunkSeq, LearningData::FeaturesLimits& featLimits, boost::function<float (const float&, const float&, const float&)> normalization, unsigned int featureSelectionMethod);
	/** train output quantizer with a new data sequence */
	void trainOutputQuantizer (int iteration, LearningData::Chunk::Seq& currentChunkSeq, LearningData::FeaturesLimits& featLimits, boost::function<float (const float&, const float&, const float&)> normalization, unsigned int featureSelectionMethod);
	/** Wait for input quantizer to finish learning */
	void waitForInputQuantizer ();
	/** Wait for output quantizer to finish learning */
	void waitForOutputQuantizer ();

}; // ActiveCrySSMEx class

}; // smlearning namespace

#endif // SMLEARNING_CRYSSMEX_H_
