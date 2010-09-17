/**
 * @author Andrzej Pronobis
 *
 * Definition of the conceptual::QueryHandler class.
 */

// Conceptual.SA
#include "QueryHandler.h"


/** The function called to create a new instance of our component. */
extern "C"
{
	cast::CASTComponentPtr newComponent()
	{
		return new conceptual::QueryHandler();
	}
}

namespace conceptual
{

using namespace std;
using namespace ConceptualData;


// -------------------------------------------------------
void QueryHandler::configure(const map<string,string> & _config)
{
/*	map<string,string>::const_iterator it;

	// QueryHandler name
	if((it = _config.find("--test-qh")) != _config.end())
	{
		_queryHandlerName = it->second;
	}

	log("Configuration parameters:");
	log("-> Test QueryHandler: %s", (_queryHandlerName.empty())?"No":"Yes");
	log("-> QueryHandler Name: %s", _queryHandlerName.c_str());*/

	// Register the ICE Server
	ConceptualData::QueryHandlerServerInterfacePtr queryHandlerServerInterfacePtr =
			new Server(this);
	registerIceServer<ConceptualData::QueryHandlerServerInterface,
			ConceptualData::QueryHandlerServerInterface>(queryHandlerServerInterfacePtr);
}


// -------------------------------------------------------
void QueryHandler::start()
{
}


// -------------------------------------------------------
void QueryHandler::runComponent()
{
}


// -------------------------------------------------------
void QueryHandler::stop()
{
}


// -------------------------------------------------------
SpatialProbabilities::ProbabilityDistribution QueryHandler::Server::query(
		const std::string &queryStr, const Ice::Current &)
{
	SpatialProbabilities::ProbabilityDistribution d;
	return d;
}


} // namespace def
