/**
 * @author Andrzej Pronobis
 *
 * Definition of the ConceptualQueryHandler class.
 */

// Conceptual.SA
#include "ConceptualQueryHandler.h"


/** The function called to create a new instance of our component. */
extern "C"
{
	cast::CASTComponentPtr newComponent()
	{
		return new def::ConceptualQueryHandler();
	}
}

namespace def
{

using namespace std;
using namespace ConceptualData;


// -------------------------------------------------------
void ConceptualQueryHandler::configure(const map<string,string> & _config)
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
}


// -------------------------------------------------------
void ConceptualQueryHandler::start()
{
}


// -------------------------------------------------------
void ConceptualQueryHandler::runComponent()
{
}


// -------------------------------------------------------
void ConceptualQueryHandler::stop()
{
}



} // namespace def
