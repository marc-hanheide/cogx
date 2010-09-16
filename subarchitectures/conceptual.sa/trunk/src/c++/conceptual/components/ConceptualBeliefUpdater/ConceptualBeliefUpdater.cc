/**
 * @author Andrzej Pronobis
 *
 * Definition of the ConceptualBeliefUpdater class.
 */

// Conceptual.SA
#include "ConceptualBeliefUpdater.h"


/** The function called to create a new instance of our component. */
extern "C"
{
	cast::CASTComponentPtr newComponent()
	{
		return new def::ConceptualBeliefUpdater();
	}
}

namespace def
{

using namespace std;
using namespace ConceptualData;


// -------------------------------------------------------
void ConceptualBeliefUpdater::configure(const map<string,string> & _config)
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
void ConceptualBeliefUpdater::start()
{
}


// -------------------------------------------------------
void ConceptualBeliefUpdater::runComponent()
{
}


// -------------------------------------------------------
void ConceptualBeliefUpdater::stop()
{
}



} // namespace def
