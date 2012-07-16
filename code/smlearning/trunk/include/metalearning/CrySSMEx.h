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
#include <smltools/fixx11h.h>
#include <quantizing/quantizer_file_export.hh>
#include <ssm/ssm_file_export.hh>
#include <ssm/ssm_parser.hh>
#include <cryssmex/exceptions.hh>
#include <boost/function.hpp>

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
	/** copy constructor */
	CrySSMEx (const CrySSMEx& c);
	/** overloaded assignment operator */
	const CrySSMEx& operator= (const CrySSMEx& c);
	/** set Substochastic Sequential Machine from file */
	void setSSM (std::string ssmfile);
	/** initialize input quantizer */
	virtual void initializeInputQuantizer (unsigned int dim);
	/** initialize output quantizer */
	virtual void initializeOutputQuantizer (unsigned int dim);
	/** initialize state quantizer */
	void initializeStateQuantizer (unsigned int dim);
	/** set Input Quantizer from file */
	void setInputQuantizer (std::string inputqfile);
	/** set Output Quantizer from file */
	void setOutputQuantizer (std::string outputqfile);
	/** set State Quantizer from file */
	void setStateQuantizer (std::string cvqfile);
	/** set input quantizer from GNG quantizer */
	void setInputQuantizer (const quantizing::GNG_Quantizer&);
	/** set output quantizer from GNG quantizer */
	void setOutputQuantizer (const quantizing::GNG_Quantizer&);
	/** set state quantizer from CVQ quantizer */
	void setStateQuantizer (const quantizing::CVQ&);
	/** get Substochastic Sequential Machine from file */
	ssm::SSM* getSSM () { return ssm; }
	/** get Input Quantizer from file */
	quantizing::Quantizer* getInputQuantizer () { return input_quantizer; }
	/** get Output Quantizer from file */
	quantizing::Quantizer* getOutputQuantizer () { return output_quantizer; }
	/** get State Quantizer from file */
	quantizing::CVQ* getStateQuantizer () { return state_quantizer; }
	/** set present data sequences from file */
	void setData (std::string seqFile, LearningData::DataSet& data, LearningData::FeaturesLimits& featLimits, boost::function<float (const float&, const float&, const float&)> normalization, unsigned int featureSelectionMethod);
	/** set data sequences from present data set */
	void setData (LearningData::DataSet& data, LearningData::FeaturesLimits& featLimits, boost::function<float (const float&, const float&, const float&)> normalization, unsigned int featureSelectionMethod);
	/** find dislocated nodes in input and output quantizers */
	void findDislocatedNodes ();
	/** train input quantizer with a new data sequence */
	void trainInputQuantizer (int iteration, LearningData::Chunk::Seq& currentChunkSeq, LearningData::FeaturesLimits& featLimits, boost::function<float (const float&, const float&, const float&)> normalization, unsigned int featureSelectionMethod);
	/** train output quantizer with a new data sequence */
	void trainOutputQuantizer (int iteration, LearningData::Chunk::Seq& currentChunkSeq, LearningData::FeaturesLimits& featLimits, boost::function<float (const float&, const float&, const float&)> normalization, unsigned int featureSelectionMethod);
	/** Wait for input quantizer to finish learning */
	void waitForInputQuantizer ();
	/** Wait for output quantizer to finish learning */
	void waitForOutputQuantizer ();
	/** save input quantizer */
	void saveInputQuantizer (string file = "cryssmex_inputq.qnt");
	/** save output quantizer */
	void saveOutputQuantizer (string file = "cryssmex_outputq.qnt");
	/** parse an input data sequence for prediction */
	void parseSequence (DataSequence& sequence, DataSequence& predicted_sequence);
	/** parse an input chunk for prediction */
	virtual std::pair<int, string> parseInput (std::vector<double>& chunk);
	/** average model vectors for cases with more than 1 model vector in \p qnt_mv_map */
	void averageModelVectors ();
	/** get quantization/model vectors map */
	std::vector<double> getQntMvMapVector (unsigned int index);
	/** set uniform distribution to initialize parsing */
	void setUniformDistribution ();
	
protected:
	/** resetting quantizer before learning with new sequences*/
	void resetLearning ();
	/** Substochastic sequential machine used for prediction */
	ssm::SSM* ssm;
	/** SSM parser used for prediction */
	ssm::SSM_Parser *ssm_parser;
	/** Input quantizer */
	quantizing::Quantizer *input_quantizer;
	/** Output quantizer */
	quantizing::Quantizer *output_quantizer;
	/** State quantizer */
	quantizing::CVQ *state_quantizer;
	/** map of quantization ids and CVQ classified model vectors */
	std::map<unsigned int, quantizing::Classified_Vectors> qnt_mv_map;

}; // CrySSMEx class

//! \class ActiveCrySSMEx
/*! \brief For dealing with active learning quantization algorithms
 */
class ActiveCrySSMEx : public CrySSMEx
{
public:
	/** initialize input quantizer */
	virtual void initializeInputQuantizer (unsigned int dim);
	/** initialize output quantizer */
	virtual void initializeOutputQuantizer (unsigned int dim);
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
