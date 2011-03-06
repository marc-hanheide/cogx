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

	// ChainGraphInferencer name
	if((it = _config.find("--chaingraphinferencer")) != _config.end())
	{
		_chainGraphInferencerName = it->second;
	}


	log("Configuration parameters:");
	log("-> QueryHandler name: %s", _queryHandlerName.c_str());
	log("-> ChainGraphInferencer name: %s", _chainGraphInferencerName.c_str());
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
SpatialProbabilities::ProbabilityDistribution Tester::sendQueryHandlerQuery(const std::string &query, bool imaginary)
{
	if (_queryHandlerAvailable)
	{
		if (imaginary)
			return _queryHandlerServerInterfacePrx->imaginaryQuery(query);
		else
			return _queryHandlerServerInterfacePrx->query(query);
	}
	else
		return SpatialProbabilities::ProbabilityDistribution();
}


// -------------------------------------------------------
void Tester::worldStateChanged(const cast::cdl::WorkingMemoryChange & wmChange)
{
	if (!_mainDialog)
		return;

	_mainDialog->newWorldState();

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
