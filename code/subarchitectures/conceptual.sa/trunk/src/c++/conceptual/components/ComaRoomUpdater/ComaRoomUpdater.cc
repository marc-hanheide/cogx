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
void ComaRoomUpdater::start()
{
	addChangeFilter(createLocalTypeFilter<ConceptualData::WorldState>(cdl::OVERWRITE),
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
			// Mark that we have handled the world state change
			_worldStateChanged=false;

			// Get a local copy of the roomID to coma room address mapping.
			map<int, cdl::WorkingMemoryAddress> rooms;
			for (ConceptualData::RoomSet::iterator it=_rooms.begin(); it!=_rooms.end(); it++)
				rooms[it->first] = it->second.wmAddress;

			pthread_mutex_unlock(&_worldStateChangedSignalMutex);

			sched_yield();

			// Request the inference

			// Wait for the results

			sched_yield();

			// Update the coma rooms
			for (map<int, cdl::WorkingMemoryAddress>::iterator it=rooms.begin(); it!=rooms.end(); it++)
			{
				debug("Updating ComaRoom ID:%d with room category information ...", it->first);
				try
				{
					lockEntry(it->second, cdl::LOCKEDODR);
					// Get the coma room
					comadata::ComaRoomPtr comaRoomPtr = getMemoryEntry<comadata::ComaRoom>(it->second);
					// Update category information
					stringstream varName;
					varName<<"room"<<it->first<<"_category";
					comaRoomPtr->categories.description = "p("+varName.str()+")";
					comaRoomPtr->categories.variableNameToPositionMap[varName.str()]=0;
					comaRoomPtr->categories.massFunction.clear();

					SpatialProbabilities::StringRandomVariableValuePtr rvv1Ptr = new SpatialProbabilities::StringRandomVariableValue("kitchen");
					SpatialProbabilities::JointProbabilityValue jpv1;
					jpv1.probability=0.2;
					jpv1.variableValues.push_back(rvv1Ptr);
					comaRoomPtr->categories.massFunction.push_back(jpv1);

					SpatialProbabilities::StringRandomVariableValuePtr rvv2Ptr = new SpatialProbabilities::StringRandomVariableValue("office");
					SpatialProbabilities::JointProbabilityValue jpv2;
					jpv2.probability=0.8;
					jpv2.variableValues.push_back(rvv2Ptr);
					comaRoomPtr->categories.massFunction.push_back(jpv2);

					// Update the coma room
					overwriteWorkingMemory(it->second, comaRoomPtr);
					unlockEntry(it->second);

					debug("ComaRoom ID:%d updated!", it->first);
				}
				catch(DoesNotExistOnWMException)
				{
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
	_rooms = worldStatePtr->rooms;
	unlockEntry(wmChange.address);

	// Signal to make an inference and update coma room structs.
	pthread_cond_signal(&_worldStateChangedSignalCond);
	pthread_mutex_unlock(&_worldStateChangedSignalMutex);

}


} // namespace def
