/**
 * @author Andrzej Pronobis
 *
 * Declaration of the conceptual::ChainGraphInferencer class.
 */

#ifndef CONCEPTUAL_CHAINGRAPHINFERENCER_H
#define CONCEPTUAL_CHAINGRAPHINFERENCER_H

// CAST
#include <cast/architecture/ManagedComponent.hpp>
#include <ConceptualData.hpp>
#include <DefaultData.hpp>
// LibDAI
#include "dai/factorgraph.h"


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

	/** Change event. */
	void worldStateChanged(const cast::cdl::WorkingMemoryChange &wmChange);

	/** Updates the factor graph based on the world state if the world state changed.
	 * Returns true if the factor graph was changed. */
	bool updateFactorGraph();

	/** Performs all inferences on the factor graph. */
	void runAllInferences();

	/** Prepares inference results based on the graph in which inference was performed. */
	void prepareInferenceResult(std::string queryString,
			SpatialProbabilities::ProbabilityDistribution *resultDistribution);

	/** Reads the default knowledge factors from the Default.SA */
	void getDefaultKnowledge();


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

	pthread_mutex_t _worldStateMutex;

	/** True if the world state has changed since the last time we checked. */
	bool _worldStateChanged;

	/** Part of the local copy of the world state. */
	ConceptualData::ComaRoomInfos _worldStateRooms;

	/** Part of the local copy of the world state. */
	ConceptualData::RoomConnectivityInfos _worldStateRoomConnections;


	/** Name of the DefaultChainGraphInferencer component.  */
	std::string _defaultChainGraphInferencerName;

	/** ICE proxy to the DefaultData::ChainGraphInferencerInterface. */
	DefaultData::ChainGraphInferencerServerInterfacePrx _defaultChainGraphInferencerServerInterfacePrx;

	/** Default knowledge factors. Map from factor string to the factor itself. */
	std::map<std::string, SpatialProbabilities::ProbabilityDistribution> _defaultKnowledgeFactors;

	/** Map from variable names to DAI variables. */
	std::map<std::string, dai::Var> _variableNameToDai;


}; // class ChainGraphInferencer
} // namespace def

#endif // CONCEPTUAL_CHAINGRAPHINFERENCER_H



