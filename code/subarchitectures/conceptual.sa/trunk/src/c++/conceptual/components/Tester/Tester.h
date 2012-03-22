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
#include <CategoricalData.hpp>
#include <FrontierInterface.hpp>
#include <Transformation/Pose3D.hh>

class MainDialog;
class QCoreApplication;

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
	Tester():
	_queryHandlerAvailable(false),
	_placeManagerAvailable(false),
	_chainGraphInferencerAvailable(false),
	_defaultChainGraphInferencerAvailable(false),
  _mainDialog(0),
  _app(0)
	{}

	/** Destructor. */
	virtual ~Tester() {}

	/** Sends a new query to the QueryHandler. */
	ConceptualData::ProbabilityDistributions sendQueryHandlerQuery(const std::string &query, bool imaginary, bool factor);

	ConceptualData::VariableInfos getChainGraphVariables();
	ConceptualData::FactorInfos getChainGraphFactors();

	const DefaultData::StringSeq &getObjectPropertyVariables()
		{ return _objectPropertyVariables; }

	const DefaultData::StringSeq &getObjectCategories()
		{ return _objectCategories; }

	const DefaultData::StringSeq &getRoomCategories()
		{ return _roomCategories; }

	const DefaultData::StringSeq &getShapes()
		{ return _shapes; }

	const DefaultData::StringSeq &getSizes()
		{ return _sizes; }

	const DefaultData::StringSeq &getAppearances()
		{ return _appearances; }

	const DefaultData::StringSeq &getHumanAssertions()
		{ return _humanAssertions; }

	const std::vector<std::string> &getVisualizedObjects()
		{ return _visualizedObjects; }

	bool saveEvents()
		{ return _saveEvents; }

	int getCurrentPlace();

	void postNavCommand(Cure::Pose3D position, SpatialData::CommandType cmdtype);


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
	void newNavCommand(const cast::cdl::WorkingMemoryChange &wmChange);
	void newLaserScan(const cast::cdl::WorkingMemoryChange &wmChange);
	void newOdometry(const cast::cdl::WorkingMemoryChange &wmChange);
	void newImage(const cast::cdl::WorkingMemoryChange &wmChange);
	void newLaserResults(const cast::cdl::WorkingMemoryChange &wmChange);
	void newVisualResults(const cast::cdl::WorkingMemoryChange &wmChange);


private:

	/** Id of the QueryHandler component.  */
	std::string _queryHandlerName;

	std::string _placeManagerName;

	/** Id of the ChainGraphInferencer component.  */
	std::string _chainGraphInferencerName;

	/** Name of the DefaultChainGraphInferencer component.  */
	std::string _defaultChainGraphInferencerName;

	/** Set to true if the QueryHandler is available. */
	bool _queryHandlerAvailable;

	bool _placeManagerAvailable;

	/** Set to true if the ChainGraphInferencer is available. */
	bool _chainGraphInferencerAvailable;

	/** Set to true if the ChainGraphInferencer is available. */
	bool _defaultChainGraphInferencerAvailable;

	MainDialog *_mainDialog;
  QCoreApplication *_app;

	/** ICE proxy to the QueryHandlerInterface. */
	ConceptualData::QueryHandlerServerInterfacePrx _queryHandlerServerInterfacePrx;

	/** ICE proxy to the ChainGraphTestingServerInterface. */
	ConceptualData::ChainGraphTestingServerInterfacePrx _chainGraphTestingServerInterfacePrx;

	/** ICE proxy to the DefaultData::ChainGraphInferencerInterface. */
	DefaultData::ChainGraphInferencerServerInterfacePrx _defaultChainGraphInferencerServerInterfacePrx;

	FrontierInterface::PlaceInterfacePrx _placeInterfacePrx;


	/** Names of the object place property variables. */
	DefaultData::StringSeq _objectPropertyVariables;

	DefaultData::StringSeq _objectCategories;

	/** Names of all room categories. */
	DefaultData::StringSeq _roomCategories;

	/** Names of all shapes. */
	DefaultData::StringSeq _shapes;

	/** Names of all sizes. */
	DefaultData::StringSeq _sizes;

	/** Names of all appearances. */
	DefaultData::StringSeq _appearances;

	/** Names of all human assertions. */
	DefaultData::StringSeq _humanAssertions;

	/** List of objects to be shown in the visualizations. */
	std::vector<std::string> _visualizedObjects;

	/*! If true, each event will trigger saving to disk of all event history. */
	bool _saveEvents;

}; // class Tester
} // namespace def

#endif // CONCEPTUAL_TESTER_H



