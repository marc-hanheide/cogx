/**
 * @author Andrzej Pronobis
 *
 * Definition of the conceptual::ComaRoomUpdater class.
 */

// Conceptual.SA
#include "ComaRoomUpdater.h"
// System
#include "ComaData.hpp"
// CAST
#include <cast/architecture/ChangeFilterFactory.hpp>


/** The function called to create a new instance of our component. */
extern "C"
{
	cast::CASTComponentPtr newComponent()
	{
		return new conceptual::ComaRoomUpdater();
	}
}

namespace conceptual
{

using namespace std;
using namespace cast;


// -------------------------------------------------------
ComaRoomUpdater::ComaRoomUpdater() : _worldStateChanged(false)
{
	pthread_cond_init(&_worldStateChangedSignalCond, 0);
	pthread_mutex_init(&_worldStateChangedSignalMutex, 0);
}


// -------------------------------------------------------
ComaRoomUpdater::~ComaRoomUpdater()
{
	pthread_cond_destroy(&_worldStateChangedSignalCond);
	pthread_mutex_destroy(&_worldStateChangedSignalMutex);
}


// -------------------------------------------------------
void ComaRoomUpdater::configure(const map<string,string> & _config)
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
void ComaRoomUpdater::start()
{
	// Get the QueryHandler interface proxy
	_queryHandlerServerInterfacePrx =
			getIceServer<ConceptualData::QueryHandlerServerInterface>(_queryHandlerName);

	// Change filters
	addChangeFilter(createLocalTypeFilter<ConceptualData::WorldState>(cdl::OVERWRITE),
			new MemberFunctionChangeReceiver<ComaRoomUpdater>(this,
					&ComaRoomUpdater::worldStateChanged));
	addChangeFilter(createLocalTypeFilter<ConceptualData::WorldState>(cdl::ADD),
			new MemberFunctionChangeReceiver<ComaRoomUpdater>(this,
					&ComaRoomUpdater::worldStateChanged));
}


// -------------------------------------------------------
void ComaRoomUpdater::runComponent()
{
	// Run component
	while(isRunning())
	{
		// Get current time and add 1 sec
		timespec ts;
		timeval tv;
		gettimeofday(&tv, NULL);
		ts.tv_sec = tv.tv_sec + 1;
		ts.tv_nsec = 0;

		// Wait if necessary
		pthread_mutex_lock(&_worldStateChangedSignalMutex);
		if (!_worldStateChanged)
			pthread_cond_timedwait(&_worldStateChangedSignalCond, &_worldStateChangedSignalMutex, &ts);

		// Handle signal if signal arrived
		if ((!isRunning()) || (!_worldStateChanged))
			pthread_mutex_unlock(&_worldStateChangedSignalMutex);
		else
		{
			debug("Processing a world state change.");

			// Mark that we have handled the world state change
			_worldStateChanged=false;

			// Get a local copy of the roomID to coma room address mapping.
			map<int, cdl::WorkingMemoryAddress> rooms = _comaRoomIdToWmAddressMap;

			pthread_mutex_unlock(&_worldStateChangedSignalMutex);

			sched_yield();

			// Update the coma rooms
			for (map<int, cdl::WorkingMemoryAddress>::iterator it=rooms.begin(); it!=rooms.end(); it++)
			{
				// Request the inference and wait for results
				debug("Requesting inference about category of room %d.", it->first);
				stringstream varName;
				varName<<"room"<<it->first<<"_category";
				ConceptualData::ProbabilityDistributions probDists =
						_queryHandlerServerInterfacePrx->query("p("+varName.str()+")");

				sched_yield();

				debug("Updating ComaRoom ID:%d with room category information ...", it->first);
				try
				{
					lockEntry(it->second, cdl::LOCKEDODR);
					// Get the coma room
					comadata::ComaRoomPtr comaRoomPtr = getMemoryEntry<comadata::ComaRoom>(it->second);

					// Update category information
					if (probDists.size())
						comaRoomPtr->categories = probDists[0];
					else
						comaRoomPtr->categories = SpatialProbabilities::ProbabilityDistribution();

					// Update the coma room
					overwriteWorkingMemory(it->second, comaRoomPtr);
					unlockEntry(it->second);

					debug("ComaRoom ID:%d updated!", it->first);
				}
				catch(DoesNotExistOnWMException)
				{
					unlockEntry(it->second);
					debug("WARNING: Information about coma room (roomId=%d) taken from the world set is not up to date!", it->first);
				}
			}

			sched_yield();

		} // if
	} // while
} // runComponent()


// -------------------------------------------------------
void ComaRoomUpdater::stop()
{
}


// -------------------------------------------------------
void ComaRoomUpdater::worldStateChanged(const cast::cdl::WorkingMemoryChange & wmChange)
{
	// Lock the mutex to make the operations below atomic
	pthread_mutex_lock(&_worldStateChangedSignalMutex);
	_worldStateChanged=true;

	// Get the rooms information from the world state
	lockEntry(wmChange.address, cdl::LOCKEDOD);
	ConceptualData::WorldStatePtr worldStatePtr = getMemoryEntry<ConceptualData::WorldState>(wmChange.address);
	_comaRoomIdToWmAddressMap.clear();
	for(unsigned int i=0; i<worldStatePtr->rooms.size(); ++i)
		_comaRoomIdToWmAddressMap[worldStatePtr->rooms[i].roomId] = worldStatePtr->rooms[i].wmAddress;
	unlockEntry(wmChange.address);

	// Signal to make an inference and update coma room structs.
	pthread_cond_signal(&_worldStateChangedSignalCond);
	pthread_mutex_unlock(&_worldStateChangedSignalMutex);

}


} // namespace def
