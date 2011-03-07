/**
 * @author Andrzej Pronobis
 *
 * Declaration of the conceptual::Tester class.
 */

#ifndef CONCEPTUAL_TESTER_H
#define CONCEPTUAL_TESTER_H

#include <cast/architecture/ManagedComponent.hpp>
#include <ConceptualData.hpp>
#include <DefaultData.hpp>


class MainDialog;

namespace conceptual
{

/**
 * @author Andrzej Pronobis
 *
 * Tester for the Conceptual.SA. It tests Conceptual.SA
 * by using its external interfaces in a way external
 * components should use them.
 */
class Tester: public cast::ManagedComponent
{

public:
	/** Constructor. */
	Tester(): _queryHandlerAvailable(false), _chainGraphInferencerAvailable(false) {}

	/** Destructor. */
	virtual ~Tester() {}

	/** Sends a new query to the QueryHandler. */
	SpatialProbabilities::ProbabilityDistribution sendQueryHandlerQuery(const std::string &query, bool imaginary, bool factor);

	ConceptualData::VariableInfos getChainGraphVariables();
	ConceptualData::FactorInfos getChainGraphFactors();

	const DefaultData::StringSeq &getObjectCategories()
		{ return _objectCategories; }

	const DefaultData::StringSeq &getRoomCategories()
		{ return _roomCategories; }


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

	/** World state changed, infer and then update the coma room structs. */
	void worldStateChanged(const cast::cdl::WorkingMemoryChange &wmChange);


private:

	/** Id of the QueryHandler component.  */
	std::string _queryHandlerName;

	/** Id of the ChainGraphInferencer component.  */
	std::string _chainGraphInferencerName;

	/** Name of the DefaultChainGraphInferencer component.  */
	std::string _defaultChainGraphInferencerName;

	/** Set to true if the QueryHandler is available. */
	bool _queryHandlerAvailable;

	/** Set to true if the ChainGraphInferencer is available. */
	bool _chainGraphInferencerAvailable;

	/** Set to true if the ChainGraphInferencer is available. */
	bool _defaultChainGraphInferencerAvailable;

	MainDialog *_mainDialog;

	/** ICE proxy to the QueryHandlerInterface. */
	ConceptualData::QueryHandlerServerInterfacePrx _queryHandlerServerInterfacePrx;

	/** ICE proxy to the ChainGraphTestingServerInterface. */
	ConceptualData::ChainGraphTestingServerInterfacePrx _chainGraphTestingServerInterfacePrx;

	/** ICE proxy to the DefaultData::ChainGraphInferencerInterface. */
	DefaultData::ChainGraphInferencerServerInterfacePrx _defaultChainGraphInferencerServerInterfacePrx;

	/** Names of the object place property variables. */
	DefaultData::StringSeq _objectPropertyVariables;

	DefaultData::StringSeq _objectCategories;

	/** Names of all room categories. */
	DefaultData::StringSeq _roomCategories;

	/** Names of all shapes. */
	DefaultData::StringSeq _shapes;

	/** Names of all appearances. */
	DefaultData::StringSeq _appearances;



}; // class Tester
} // namespace def

#endif // CONCEPTUAL_TESTER_H



