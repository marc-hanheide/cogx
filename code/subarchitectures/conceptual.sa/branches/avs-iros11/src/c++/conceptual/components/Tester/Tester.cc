/**
 * @author Andrzej Pronobis
 *
 * Definition of the conceptual::Tester class.
 */

// Conceptual.SA
#include "Tester.h"
#include "MainDialog.h"
#include "SpatialProbabilities.hpp"
// CAST
#include <cast/architecture/ChangeFilterFactory.hpp>
// Qt
#include <QApplication>


/** The function called to create a new instance of our component. */
extern "C"
{
	cast::CASTComponentPtr newComponent()
	{
		return new conceptual::Tester();
	}
}

namespace conceptual
{

using namespace std;
using namespace cast;
using namespace ConceptualData;


// -------------------------------------------------------
void Tester::configure(const map<string,string> & _config)
{
	map<string,string>::const_iterator it;

	// QueryHandler name
	if((it = _config.find("--queryhandler")) != _config.end())
	{
		_queryHandlerName = it->second;
	}

	if((it = _config.find("--chaingraphinferencer")) != _config.end())
		_chainGraphInferencerName = it->second;
	if((it = _config.find("--defaultchaingraphinferencer")) != _config.end())
		_defaultChainGraphInferencerName = it->second;


	log("Configuration parameters:");
	log("-> QueryHandler name: %s", _queryHandlerName.c_str());
	log("-> ChainGraphInferencer name: %s", _chainGraphInferencerName.c_str());
	log("-> DefaultChainGraphInferencer name: %s", _defaultChainGraphInferencerName.c_str());
}


// -------------------------------------------------------
void Tester::start()
{
	try
	{
		// Get the QueryHandler interface proxy
		_queryHandlerServerInterfacePrx =
				getIceServer<ConceptualData::QueryHandlerServerInterface>(_queryHandlerName);
		_queryHandlerAvailable = true;
	}
	catch(...)
	{}

	try
	{
		// Get the ChainGraphInferencer interface proxy
		_chainGraphTestingServerInterfacePrx =
				getIceServer<ConceptualData::ChainGraphTestingServerInterface>(_chainGraphInferencerName);
		_chainGraphInferencerAvailable = true;
	}
	catch(...)
	{}

	try
	{
		// Get the DefaultChainGraphInferencer interface proxy
		_defaultChainGraphInferencerServerInterfacePrx =
				getIceServer<DefaultData::ChainGraphInferencerServerInterface>(_defaultChainGraphInferencerName);
		_defaultChainGraphInferencerAvailable = true;
	}
	catch(...)
	{}


	// Change filters
	addChangeFilter(createLocalTypeFilter<ConceptualData::WorldState>(cdl::OVERWRITE),
			new MemberFunctionChangeReceiver<Tester>(this,
					&Tester::worldStateChanged));
	addChangeFilter(createLocalTypeFilter<ConceptualData::WorldState>(cdl::ADD),
			new MemberFunctionChangeReceiver<Tester>(this,
					&Tester::worldStateChanged));
}


// -------------------------------------------------------
void Tester::runComponent()
{
	// Get default knowledge
	_objectPropertyVariables =
		_defaultChainGraphInferencerServerInterfacePrx->getObjectPropertyVariables();
	_objectCategories =
		_defaultChainGraphInferencerServerInterfacePrx->getObjectCategories();
	_roomCategories =
		_defaultChainGraphInferencerServerInterfacePrx->getRoomCategories();
	_shapes =
		_defaultChainGraphInferencerServerInterfacePrx->getShapes();
	_appearances =
		_defaultChainGraphInferencerServerInterfacePrx->getAppearances();

	QCoreApplication *app = QApplication::instance();
	if (!app)
		app = new QApplication(0,0);
	MainDialog *mainDialog = new MainDialog(this);
	_mainDialog = mainDialog;
	mainDialog->exec();
	_mainDialog=0;
	delete mainDialog;
}


// -------------------------------------------------------
void Tester::stop()
{
}


// -------------------------------------------------------
SpatialProbabilities::ProbabilityDistribution Tester::sendQueryHandlerQuery(const std::string &query, bool imaginary, bool factor)
{
	if (_queryHandlerAvailable)
	{
		if (imaginary)
			return _queryHandlerServerInterfacePrx->imaginaryQuery(query);
		else if (factor)
			return _queryHandlerServerInterfacePrx->factorQuery(query);
		else
			return _queryHandlerServerInterfacePrx->query(query);
	}
	else
		return SpatialProbabilities::ProbabilityDistribution();
}


// -------------------------------------------------------
void Tester::worldStateChanged(const cast::cdl::WorkingMemoryChange & wmChange)
{
	try
	{
		ConceptualData::WorldStatePtr worldStatePtr;
		worldStatePtr = getMemoryEntry<ConceptualData::WorldState>(wmChange.address);
		if (_mainDialog)
			_mainDialog->newWorldState(worldStatePtr);
	}
	catch(CASTException &e)
	{
		log("Exception while reading world state from the WM!");
	}
}


// -------------------------------------------------------
ConceptualData::VariableInfos Tester::getChainGraphVariables()
{
	if (_chainGraphInferencerAvailable)
	{
		return _chainGraphTestingServerInterfacePrx->getVariables();
	}
	else
		return ConceptualData::VariableInfos();
}


// -------------------------------------------------------
ConceptualData::FactorInfos Tester::getChainGraphFactors()
{
	if (_chainGraphInferencerAvailable)
	{
		return _chainGraphTestingServerInterfacePrx->getFactors();
	}
	else
		return ConceptualData::FactorInfos();
}




} // namespace def
