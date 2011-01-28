/**
 * @author Andrzej Pronobis
 *
 * Definition of the conceptual::Tester class.
 */

// Conceptual.SA
#include "Tester.h"


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

	log("Configuration parameters:");
	log("-> QueryHandler name: %s", _queryHandlerName.c_str());
}


// -------------------------------------------------------
void Tester::start()
{
	// Get the QueryHandler interface proxy
	try
	{
		_queryHandlerServerInterfacePrx =
				getIceServer<ConceptualData::QueryHandlerServerInterface>(_queryHandlerName);
		_queryHandlerAvailable = true;
	}
	catch (...)
	{}
}


// -------------------------------------------------------
void Tester::runComponent()
{
}


// -------------------------------------------------------
void Tester::stop()
{
}


// -------------------------------------------------------
SpatialProbabilities::ProbabilityDistribution Tester::sendQueryHandlerQuery(const std::string &query)
{
	if (_queryHandlerAvailable)
		return _queryHandlerServerInterfacePrx->query(query);
	else
		return SpatialProbabilities::ProbabilityDistribution();
}



} // namespace def
