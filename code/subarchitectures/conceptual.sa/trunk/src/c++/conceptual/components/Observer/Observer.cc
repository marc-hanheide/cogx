/**
 * @author Andrzej Pronobis
 *
 * Definition of the conceptual::Observer class.
 */

// Conceptual.SA
#include "Observer.h"
// System
#include "ComaData.hpp"
// CAST
#include <cast/architecture/ChangeFilterFactory.hpp>


/** The function called to create a new instance of our component. */
extern "C"
{
	cast::CASTComponentPtr newComponent()
	{
		return new conceptual::Observer();
	}
}

namespace conceptual
{

using namespace std;
using namespace cast;


// -------------------------------------------------------
void Observer::configure(const std::map<std::string,std::string> & _config)
{
/*	map<string,string>::const_iterator it;

	// QueryHandler name
	if((it = _config.find("--coma")) != _config.end())
	{
		_queryHandlerName = it->second;
	}

	log("Configuration parameters:");
	log("-> Test QueryHandler: %s", (_queryHandlerName.empty())?"No":"Yes");
	log("-> QueryHandler Name: %s", _queryHandlerName.c_str());*/

	/** Set the initial world state */
	initializeWorldState();
}


// -------------------------------------------------------
void Observer::start()
{
	// Global filter on comadata::ComaRoom
	addChangeFilter(createGlobalTypeFilter<comadata::ComaRoom>(cdl::WILDCARD),
			new MemberFunctionChangeReceiver<Observer>(this,
					&Observer::comaRoomChanged));
}


// -------------------------------------------------------
void Observer::runComponent()
{
  addInitialWorldState();
	
//    while (isRunning())
//    {
//    	sleepComponent(1000);
//    }
}


// -------------------------------------------------------
void Observer::stop()
{
}


// -------------------------------------------------------
void Observer::initializeWorldState()
{
	_worldStatePtr = new ConceptualData::WorldState();
}


// -------------------------------------------------------
void Observer::addInitialWorldState()
{
	_worldStateId = newDataID();

	// Publish the initial world state on the working memory
	addToWorkingMemory<ConceptualData::WorldState>(
			_worldStateId, _worldStatePtr);
}


// -------------------------------------------------------
void Observer::updateWorldState()
{
	overwriteWorkingMemory<ConceptualData::WorldState>(_worldStateId, _worldStatePtr);
}


// -------------------------------------------------------
void Observer::comaRoomChanged(const cast::cdl::WorkingMemoryChange & wmChange)
{
// Using the following structure of ComaRoom
//      class ComaRoom {
//    	int roomId;
//    	string seedPlaceInstance;
//    	PlaceIdSet containedPlaceIds;
//    	SpatialProbabilities::ProbabilityDistribution categories;
//    };

	// Lock the world state first
	lockEntry(_worldStateId, cdl::LOCKEDODR);

	// Decide what change has been made
	switch (wmChange.operation)
	{
	case cdl::ADD:
	{ // Room added
	        comadata::ComaRoomPtr comaRoomPtr = getMemoryEntry<comadata::ComaRoom>(wmChange.address);

		ConceptualData::ComaRoomInfo &cri = _worldStatePtr->rooms[comaRoomPtr->roomId];
		cri.wmAddress = wmChange.address;
		cri.roomId = comaRoomPtr->roomId;
		cri.containedPlaceIds = comaRoomPtr->containedPlaceIds;
		updateWorldState();
		break;
	}

	case cdl::OVERWRITE:
	{
	  	comadata::ComaRoomPtr comaRoomPtr = getMemoryEntry<comadata::ComaRoom>(wmChange.address);

		ConceptualData::ComaRoomInfo &cri = _worldStatePtr->rooms[comaRoomPtr->roomId];
		if (cri.containedPlaceIds != comaRoomPtr->containedPlaceIds)
		{
			cri.wmAddress = wmChange.address;
			cri.containedPlaceIds = comaRoomPtr->containedPlaceIds;
			updateWorldState();
		}
		break;
	}

	case cdl::DELETE:
	{
	  //		_worldStatePtr->rooms.erase(comaRoomPtr->roomId);
		updateWorldState();
		break;
	}

	default:
		break;
	} // switch

	// Unlock world state
	unlockEntry(_worldStateId);
}


} // namespace def
