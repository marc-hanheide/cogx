/**
 * @author Andrzej Pronobis
 *
 * Declaration of the conceptual::ChainGraphInferencer class.
 */

#ifndef CONCEPTUAL_QUERYHANDLER_H
#define CONCEPTUAL_QUERYHANDLER_H

#include <cast/architecture/ManagedComponent.hpp>
#include <ConceptualData.hpp>


namespace conceptual
{


/**
 * @author Andrzej Pronobis
 *
 * Main query handler for the Conceptual.SA
 */
class ChainGraphInferencer: public cast::ManagedComponent
{

public:

	/** Constructor. */
	ChainGraphInferencer();

	/** Destructor. */
	virtual ~ChainGraphInferencer();


protected:
	/** Called by the framework to configure the component. */
	virtual void configure(const std::map<std::string,std::string> & _config);

	/** Called by the framework after configuration, before run loop. */
	virtual void start();

	/** The main run loop. */
	virtual void runComponent();

	/** Called by the framework after the run loop finishes. */
	virtual void stop();


private:

	/** Change event. */
	void inferenceQueryAdded(const cast::cdl::WorkingMemoryChange &wmChange);

	/** Runs inference given by the query string and returns a probability distributions. */
	void runInference(std::string queryString,
			SpatialProbabilities::ProbabilityDistribution *resultDistribution);


private:

	struct Query
	{
		ConceptualData::InferenceQueryPtr queryPtr;
		cast::cdl::WorkingMemoryAddress wmAddress;
	};

	/** Recently received queries. */
	std::list<Query> _receivedQueries;

	pthread_cond_t _inferenceQueryAddedSignalCond;
	pthread_mutex_t _inferenceQueryAddedSignalMutex;


}; // class ChainGraphInferencer
} // namespace def

#endif // CONCEPTUAL_QUERYHANDLER_H



