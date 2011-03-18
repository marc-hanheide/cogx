
/**
 * @author Andrzej Pronobis
 *
 * Definition of the conceptual::Observer class.
 */

// Conceptual.SA
#include "Observer.h"
// System
#include "ComaData.hpp"
#include "SpatialData.hpp"
#include "SpatialProperties.hpp"
// CAST
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/scoped_ptr.hpp>
#include <math.h>

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
Observer::Observer() : _lastWsUpdateTime(0.0)
{
	pthread_mutex_init(&_worldStateMutex, 0);
}


// -------------------------------------------------------
Observer::~Observer()
{
	pthread_mutex_destroy(&_worldStateMutex);
}


// -------------------------------------------------------
template<typename T>
static void removeDuplicates(std::vector<T>& vec)
{
	std::sort(vec.begin(), vec.end());
	vec.erase(std::unique(vec.begin(), vec.end()), vec.end());
}


// -------------------------------------------------------
void Observer::configure(const std::map<std::string,std::string> & _config)
{
	map<string,string>::const_iterator it;
	_shapeThreshold = -1.0;
	_appearanceThreshold = -1.0;
	_gatewayThreshold = -1.0;
	_betaThreshold = 0.01;

	if((it = _config.find("--shape-threshold")) != _config.end())
	{
		_shapeThreshold = boost::lexical_cast<double>(it->second);
	}
	if((it = _config.find("--appearance-threshold")) != _config.end())
	{
		_appearanceThreshold = boost::lexical_cast<double>(it->second);
	}
	if((it = _config.find("--gateway-placeholder-threshold")) != _config.end())
	{
		_gatewayThreshold = boost::lexical_cast<double>(it->second);
	}
	if((it = _config.find("--beta-threshold")) != _config.end())
	{
		_betaThreshold = boost::lexical_cast<double>(it->second);
	}
	_includePlaceholderInfo = (_config.find("--no-placeholders") == _config.end());

	if ((_shapeThreshold<0.0) || (_appearanceThreshold<0.0) || (_gatewayThreshold<0.0))
		throw CASTException(exceptionMessage(__HERE__, "Please set shape-threshold, appearance-threshold and gateway-placeholder-threshold!"));

	log("Configuration parameters:");
	log("-> Shape threshold: %f", _shapeThreshold);
	log("-> Appearances threshold: %f", _appearanceThreshold);
	log("-> Gateway placeholder threshold: %f", _gatewayThreshold);
	log("-> Include placeholder information: %s", (_includePlaceholderInfo)?"yes":"no");

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

	// Global filter on SpatialData::Place
	addChangeFilter(createGlobalTypeFilter<SpatialData::Place>(cdl::WILDCARD),
			new MemberFunctionChangeReceiver<Observer>(this,
					&Observer::placeChanged));

	// Global filter on SpatialProperties::GatewayPlaceProperty
	addChangeFilter(createGlobalTypeFilter<SpatialProperties::GatewayPlaceProperty>(cdl::WILDCARD),
			new MemberFunctionChangeReceiver<Observer>(this,
					&Observer::gatewayPlacePropertyChanged));

	// Global filter on SpatialProperties::ObjectPlaceProperty
	addChangeFilter(createGlobalTypeFilter<SpatialProperties::ObjectPlaceProperty>(cdl::WILDCARD),
			new MemberFunctionChangeReceiver<Observer>(this,
					&Observer::objectPlacePropertyChanged));

	// Global filter on SpatialProperties::RoomShapePlaceProperty
	addChangeFilter(createGlobalTypeFilter<SpatialProperties::RoomShapePlaceProperty>(cdl::WILDCARD),
			new MemberFunctionChangeReceiver<Observer>(this,
					&Observer::shapePlacePropertyChanged));

	// Global filter on SpatialProperties::RoomAppearancePlaceProperty
	addChangeFilter(createGlobalTypeFilter<SpatialProperties::RoomAppearancePlaceProperty>(cdl::WILDCARD),
			new MemberFunctionChangeReceiver<Observer>(this,
					&Observer::appearancePlacePropertyChanged));

	// Global filter on SpatialProperties::ConnectivityPathProperty
	addChangeFilter(createGlobalTypeFilter<SpatialProperties::ConnectivityPathProperty>(cdl::WILDCARD),
			new MemberFunctionChangeReceiver<Observer>(this,
					&Observer::connectivityPathPropertyChanged));

	// Global filter on SpatialProperties::ConnectivityPathProperty
	addChangeFilter(createGlobalTypeFilter<SpatialData::ObjectSearchResult>(cdl::WILDCARD),
			new MemberFunctionChangeReceiver<Observer>(this,
					&Observer::objectSearchResultChanged));

	// Global filter on SpatialProperties::GatewayPlaceholderProperty
	addChangeFilter(createGlobalTypeFilter<SpatialProperties::GatewayPlaceholderProperty>(cdl::WILDCARD),
			new MemberFunctionChangeReceiver<Observer>(this,
					&Observer::gatewayPlaceholderPropertyChanged));

}


// -------------------------------------------------------
void Observer::runComponent()
{
    while (isRunning())
    {
    	sleepComponent(100);
    	pthread_mutex_lock(&_worldStateMutex);
    	updateWorldState();
    	pthread_mutex_unlock(&_worldStateMutex);
    }
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
void Observer::updateWorldState()
{
	std::vector<ConceptualData::EventInfo> accumulatedEvents;

	// Check if we've had an update within one second. If so, do nothing
	// Get current time
	timeval now;
	gettimeofday(&now, 0);
	double curTime = now.tv_sec+ now.tv_usec/1000000.0;
	// If nothing there or not yet time, do nothing
	if ((_accumulatedEvents.empty()) || ((curTime-_lastWsUpdateTime)<1.0))
		return;
	accumulatedEvents = _accumulatedEvents;
	_accumulatedEvents.clear();
	_lastWsUpdateTime = curTime;

	// Create a new worldstate
	ConceptualData::WorldStatePtr newWorldStatePtr = new ConceptualData::WorldState();

	// -------------------------------------
	// Fill in the world state based on the cached variables
	// -------------------------------------

	// Rooms
	// Using the following structure of ComaRoom
	//      class ComaRoom {
	//    	int roomId;
	//    	string seedPlaceInstance;
	//    	PlaceIdSet containedPlaceIds;
	//    	SpatialProbabilities::ProbabilityDistribution categories;
	//    };
	map<cast::cdl::WorkingMemoryAddress, comadata::ComaRoomPtr>::iterator comaRoomIt;
	for( comaRoomIt=_comaRoomWmAddressMap.begin(); comaRoomIt!=_comaRoomWmAddressMap.end(); ++comaRoomIt)
	{
		comadata::ComaRoomPtr comaRoomPtr = comaRoomIt->second;
		ConceptualData::ComaRoomInfo cri;
		cri.wmAddress = comaRoomIt->first;
		cri.roomId = comaRoomPtr->roomId;

		// Get all the objectplaceproperties for this room
		std::vector<SpatialProperties::ObjectPlacePropertyPtr> objectPlacePropertiesForRoom;
		for(unsigned int i=0; i<comaRoomPtr->containedPlaceIds.size(); ++i)
		{
			int placeId = comaRoomPtr->containedPlaceIds[i];
			getObjectPlaceProperties(placeId, objectPlacePropertiesForRoom);
		}
		// Analyse the results of AVS
		std::vector<SpatialData::ObjectSearchResultPtr> objectSearchResults;
		getObjectSearchResults(cri.roomId, objectSearchResults);
		for(unsigned int s=0; s<objectSearchResults.size(); ++s)
		{
			// Single result for this room
			SpatialData::ObjectSearchResultPtr objectSearchResultPtr = objectSearchResults[s];
			unsigned int count = 0;
			if (objectSearchResultPtr->beta>0.0) // Only if beta > 0
			{
				// Let's see if it matches any of the actual object observations in this room
				std::vector<SpatialProperties::ObjectPlacePropertyPtr>::iterator it = objectPlacePropertiesForRoom.begin();
				while (it!=objectPlacePropertiesForRoom.end())
				{
					// Check if this object matches the result
					if ( (objectSearchResultPtr->searchedObjectCategory == (*it)->category) &&
						 (objectSearchResultPtr->relation == (*it)->relation) &&
						 (objectSearchResultPtr->supportObjectCategory == (*it)->supportObjectCategory) &&
						 (objectSearchResultPtr->supportObjectId == (*it)->supportObjectId) )
					{ // Yes, add it to our ObjectPlacePropertyInfo if it was found
						SpatialProperties::DiscreteProbabilityDistributionPtr dpdPtr =
								SpatialProperties::DiscreteProbabilityDistributionPtr::dynamicCast((*it)->distribution);
						if ( ((SpatialProperties::BinaryValuePtr::dynamicCast(dpdPtr->data[0].value)->value == true) &&
							  (dpdPtr->data[0].probability>0.5)) ||
							 ((SpatialProperties::BinaryValuePtr::dynamicCast(dpdPtr->data[1].value)->value == true) &&
													(dpdPtr->data[1].probability>0.5)) )
							count++;
					}
					it++;
				} // while
			} // if
			ConceptualData::ObjectPlacePropertyInfo oppi;
			oppi.category = objectSearchResultPtr->searchedObjectCategory;
			oppi.relation = objectSearchResultPtr->relation;
			oppi.supportObjectCategory = objectSearchResultPtr->supportObjectCategory;
			oppi.supportObjectId = objectSearchResultPtr->supportObjectId;
			oppi.count = count;
			oppi.beta = objectSearchResultPtr->beta;
			cri.objectProperties.push_back(oppi);
		}


		// Places in a room
		// Using the following structure of Place
		//	  class Place {
		//	    // Unique ID number of this Place
		//	    long id;
		//	    // Status of Place definition
		//	    PlaceStatus status;
		//	  };
		for(unsigned int i=0; i<comaRoomPtr->containedPlaceIds.size(); ++i)
		{
			int placeId = comaRoomPtr->containedPlaceIds[i];
			ConceptualData::PlaceInfo pi;
			pi.placeId = placeId;

			// Check if this is a true place (just in case, coma should do that for us already)
			if (!isTruePlace(placeId))
				continue;

			// Check if the place is not a gateway (just in case, coma should do that for us)
			if (isGatewayPlace(placeId))
				continue;

			// Get properties of the place
			// Object properties
			// Currently done on the room level ABOVE!
//			std::vector<SpatialProperties::ObjectPlacePropertyPtr> objectPlaceProperties;
//			getObjectPlaceProperties(placeId, objectPlaceProperties);
//			for (unsigned int j=0; j<objectPlaceProperties.size(); ++j)
//			{
//				SpatialProperties::ObjectPlacePropertyPtr objectPlacePropertyPtr =
//						objectPlaceProperties[j];
//				SpatialProperties::DiscreteProbabilityDistributionPtr dpdPtr =
//						SpatialProperties::DiscreteProbabilityDistributionPtr::dynamicCast(objectPlacePropertyPtr->distribution);
//
//				string objCategory = SpatialProperties::StringValuePtr::dynamicCast(dpdPtr->data[0].value)->value;
//				double objPresenceProbability = dpdPtr->data[0].probability;
//				ConceptualData::ObjectPlacePropertyInfo oppi;
//				oppi.category = objCategory;
//				oppi.count = (objPresenceProbability>0.5);
//				pi.objectProperties.push_back(oppi);
//			}

			// Shape properties
			std::vector<SpatialProperties::RoomShapePlacePropertyPtr> shapePlaceProperties;
			getShapePlaceProperties(placeId, shapePlaceProperties);
			for (unsigned int j=0; j<shapePlaceProperties.size(); ++j)
			{
				SpatialProperties::RoomShapePlacePropertyPtr shapePlacePropertyPtr =
						shapePlaceProperties[j];
				SpatialProperties::DiscreteProbabilityDistributionPtr dpdPtr =
						SpatialProperties::DiscreteProbabilityDistributionPtr::dynamicCast(shapePlacePropertyPtr->distribution);

				ConceptualData::ShapePlacePropertyInfo sppi;
				for (unsigned int k=0; k<dpdPtr->data.size(); ++k)
				{
					ConceptualData::ValuePotentialPair vpp;
					vpp.value = SpatialProperties::StringValuePtr::dynamicCast(dpdPtr->data[k].value)->value;
					vpp.potential = dpdPtr->data[k].probability;
					sppi.distribution.push_back(vpp);
				}
				pi.shapeProperties.push_back(sppi);
			}

			// Appearance properties
			std::vector<SpatialProperties::RoomAppearancePlacePropertyPtr> appearancePlaceProperties;
			getAppearancePlaceProperties(placeId, appearancePlaceProperties);
			for (unsigned int j=0; j<appearancePlaceProperties.size(); ++j)
			{
				SpatialProperties::RoomAppearancePlacePropertyPtr appearancePlacePropertyPtr =
						appearancePlaceProperties[j];
				SpatialProperties::DiscreteProbabilityDistributionPtr dpdPtr =
						SpatialProperties::DiscreteProbabilityDistributionPtr::dynamicCast(appearancePlacePropertyPtr->distribution);

				ConceptualData::AppearancePlacePropertyInfo sppi;
				for (unsigned int k=0; k<dpdPtr->data.size(); ++k)
				{
					ConceptualData::ValuePotentialPair vpp;
					vpp.value = SpatialProperties::StringValuePtr::dynamicCast(dpdPtr->data[k].value)->value;
					vpp.potential = dpdPtr->data[k].probability;
					sppi.distribution.push_back(vpp);
				}
				pi.appearanceProperties.push_back(sppi);
			}

			// Add the place info
			cri.places.push_back(pi);

			// Get room connectivity for this place
			int room1Id = comaRoomPtr->roomId;
			//debug("Getting room connectivity for place %d in room %d", placeId, room1Id);
			vector<int> connectedPlaces;
			getConnectedPlaces(placeId, &connectedPlaces);
			for(unsigned int j=0; j<connectedPlaces.size(); ++j)
			{
				// Check if the connected place is a placeholder
				if (isTruePlace(connectedPlaces[j]))
				{
					int room2Id = getRoomForPlace(connectedPlaces[j]);
					if (room2Id>=0) // We actually found a room. It's not a placeholder!
					{
						if (room1Id<room2Id)
						{
							ConceptualData::RoomConnectivityInfo rci;
							rci.room1Id=room1Id;
							rci.room2Id=room2Id;
							newWorldStatePtr->roomConnections.push_back(rci);
						}
						if (room2Id<room1Id)
						{
							ConceptualData::RoomConnectivityInfo rci;
							rci.room1Id=room2Id;
							rci.room2Id=room1Id;
							newWorldStatePtr->roomConnections.push_back(rci);
						}
					}
				}
				else
				{
					// If we include placeholders and it's a new placeholder in this room, add it to the room info
					if ( (_includePlaceholderInfo) && (isPlaceholder(connectedPlaces[j]))
							&& (!isPlaceholderPresent(cri, connectedPlaces[j])) )
					{
						// First check if there is a placeholder in the other rooms already
						// If a placeholder exists in two rooms, remove it from the room with
						// higher ID (the one created later)
						int placeholderRoomIndex = isPlaceholderInRooms(newWorldStatePtr->rooms, connectedPlaces[j]);
						if ( ((placeholderRoomIndex>=0) &&
							 (newWorldStatePtr->rooms[placeholderRoomIndex].roomId > room1Id)) ||
							 (placeholderRoomIndex<0) )
						 {
							// Create this placeholder in this room
							ConceptualData::PlaceholderInfo pli;
							pli.placeholderId = connectedPlaces[j];

							// Gateway placeholder properties
							std::vector<SpatialProperties::GatewayPlaceholderPropertyPtr> gatewayPlaceholderProperties;
							getGatewayPlaceholderProperties(pli.placeholderId, gatewayPlaceholderProperties);
							for (unsigned int l=0; l<gatewayPlaceholderProperties.size(); ++l)
							{
								SpatialProperties::GatewayPlaceholderPropertyPtr gatewayPlaceholderPropertyPtr =
										gatewayPlaceholderProperties[l];
								SpatialProperties::DiscreteProbabilityDistributionPtr dpdPtr =
										SpatialProperties::DiscreteProbabilityDistributionPtr::dynamicCast(
												gatewayPlaceholderPropertyPtr->distribution);

								ConceptualData::GatewayPlaceholderPropertyInfo gppi;
								for (unsigned int k=0; k<dpdPtr->data.size(); ++k)
								{
									if (SpatialProperties::BinaryValuePtr::dynamicCast(dpdPtr->data[k].value)->value == true)
										gppi.gatewayProbability = dpdPtr->data[k].probability;
								}
								pli.gatewayProperties.push_back(gppi);
							}

							// Add the place info
							cri.placeholders.push_back(pli);

							// If there was a placeholder in another room, remove that one
							if (placeholderRoomIndex>=0)
							{
								vector<ConceptualData::PlaceholderInfo> & placeholderInfos =
										newWorldStatePtr->rooms[placeholderRoomIndex].placeholders;
								for (unsigned int x=0; x<placeholderInfos.size(); ++x)
									if (placeholderInfos[x].placeholderId == connectedPlaces[j])
									{
										placeholderInfos.erase(placeholderInfos.begin()+x);
										break;
									}
							}

						 }
					} // if
				} // if placeholder
			} // for(unsigned int j=0; j<connectedPlaces.size(); ++j)
		} // for(unsigned int i=0; i<comaRoomPtr->containedPlaceIds.size(); ++i)

		newWorldStatePtr->rooms.push_back(cri);
	} // for( comaRoomIt=_comaRoomWmAddressMap.begin(); comaRoomIt!=_comaRoomWmAddressMap.end(); ++comaRoomIt)

	// Clean up the room connectivity info
	removeDuplicates(newWorldStatePtr->roomConnections);

	// -----------------------------------
	// Update on working memory
	// -----------------------------------
	newWorldStatePtr->lastEvents = accumulatedEvents;

	// Remember the new world state
	_worldStatePtr=newWorldStatePtr;

	if (_worldStateId.empty())
	{ // Publish the initial world state on the working memory
		log("Adding WorldState to the working memory.");
		_worldStateId = newDataID();
		addToWorkingMemory<ConceptualData::WorldState>(_worldStateId, _worldStatePtr);
	}
	else
	{ // Overwrite the world state on the WM
		log("Updating WorldState in the working memory.");
		lockEntry(_worldStateId, cdl::LOCKEDODR);
		overwriteWorkingMemory<ConceptualData::WorldState>(_worldStateId, _worldStatePtr);
		unlockEntry(_worldStateId);
	}
}


// -------------------------------------------------------
double Observer::calculateDistributionDifference(const SpatialProperties::ProbabilityDistributionPtr dist1,
		const SpatialProperties::ProbabilityDistributionPtr dist2)
{
	const SpatialProperties::ValueProbabilityPairs &d1 =
			SpatialProperties::DiscreteProbabilityDistributionPtr::dynamicCast(dist1)->data;
	const SpatialProperties::ValueProbabilityPairs &d2 =
			SpatialProperties::DiscreteProbabilityDistributionPtr::dynamicCast(dist2)->data;

	if (d1.size() != d2.size())
		return 1e6;

	double difference = 0.0;
	for (unsigned int i=0; i< d1.size(); ++i)
	{
		string v1 = SpatialProperties::StringValuePtr::dynamicCast(d1[i].value)->value;
		string v2 = SpatialProperties::StringValuePtr::dynamicCast(d2[i].value)->value;
		if (v1 != v2)
			return 1e6;
		double tmp = fabs(d1[i].probability - d2[i].probability);
		if (difference<tmp)
			difference = tmp;
	}
	return difference;
}


// -------------------------------------------------------
double Observer::calculateBinaryDistributionDifference(const SpatialProperties::ProbabilityDistributionPtr dist1,
		const SpatialProperties::ProbabilityDistributionPtr dist2)
{
	const SpatialProperties::ValueProbabilityPairs &d1 =
			SpatialProperties::DiscreteProbabilityDistributionPtr::dynamicCast(dist1)->data;
	const SpatialProperties::ValueProbabilityPairs &d2 =
			SpatialProperties::DiscreteProbabilityDistributionPtr::dynamicCast(dist2)->data;

	if (d1.size() != d2.size())
		return 1e6;

	double difference = 0.0;
	for (unsigned int i=0; i< d1.size(); ++i)
	{
		bool v1 = SpatialProperties::BinaryValuePtr::dynamicCast(d1[i].value)->value;
		bool v2 = SpatialProperties::BinaryValuePtr::dynamicCast(d2[i].value)->value;
		if (v1 != v2)
			return 1e6;
		double tmp = fabs(d1[i].probability - d2[i].probability);
		if (difference<tmp)
			difference = tmp;
	}
	return difference;
}


// -------------------------------------------------------
void Observer::comaRoomChanged(const cast::cdl::WorkingMemoryChange & wmChange)
{
	// Decide what change has been made
	switch (wmChange.operation)
	{
	case cdl::ADD:
	{ // Room added
		comadata::ComaRoomPtr comaRoomPtr;
		try
		{
			comaRoomPtr = getMemoryEntry<comadata::ComaRoom>(wmChange.address);
		}
		catch(CASTException &e)
		{
			log("Caught exception at %s. Message: %s", __HERE__, e.message.c_str());
			return;
		}

		pthread_mutex_lock(&_worldStateMutex);

		_comaRoomWmAddressMap[wmChange.address] = comaRoomPtr;
		ConceptualData::EventInfo ei;
		ei.type = ConceptualData::EventRoomAdded;
		ei.roomId = comaRoomPtr->roomId;
		ei.place1Id = -1;
		ei.place2Id = -1;
		_accumulatedEvents.push_back(ei);

		updateWorldState();

		pthread_mutex_unlock(&_worldStateMutex);
		break;
	}

	case cdl::OVERWRITE:
	{
		comadata::ComaRoomPtr comaRoomPtr;

		try
		{
			comaRoomPtr = getMemoryEntry<comadata::ComaRoom>(wmChange.address);
		}
		catch(CASTException &e)
		{
			log("Caught exception at %s. Message: %s", __HERE__, e.message.c_str());
			return;
		}

		pthread_mutex_lock(&_worldStateMutex);

		comadata::ComaRoomPtr old = _comaRoomWmAddressMap[wmChange.address];
		_comaRoomWmAddressMap[wmChange.address] = comaRoomPtr;

		// Check wmAddresss and ID
		if (old->roomId != comaRoomPtr->roomId)
			throw cast::CASTException("The mapping between ComaRoom WMAddress and ID changed!");

		// Check what has changed
		ConceptualData::EventInfo ei;
		ei.roomId = comaRoomPtr->roomId;
		ei.place2Id = -1;

		if (old->containedPlaceIds.size()<comaRoomPtr->containedPlaceIds.size())
		{
			ei.type = ConceptualData::EventRoomPlaceAdded;
			ei.place1Id = comaRoomPtr->containedPlaceIds[old->containedPlaceIds.size()];
			for (unsigned int i=0; i<old->containedPlaceIds.size(); ++i)
			{
				if (comaRoomPtr->containedPlaceIds[i]!=old->containedPlaceIds[i])
				{
					ei.place1Id = comaRoomPtr->containedPlaceIds[i];
					break;
				}
			}
			_accumulatedEvents.push_back(ei);

			updateWorldState();
		}
		else if (old->containedPlaceIds.size()>comaRoomPtr->containedPlaceIds.size())
		{
			ei.type = ConceptualData::EventRoomPlaceDeleted;
			ei.place1Id = old->containedPlaceIds[comaRoomPtr->containedPlaceIds.size()];
			for (unsigned int i=0; i<comaRoomPtr->containedPlaceIds.size(); ++i)
			{
				if (comaRoomPtr->containedPlaceIds[i]!=old->containedPlaceIds[i])
				{
					ei.place1Id = old->containedPlaceIds[i];
					break;
				}
			}
			_accumulatedEvents.push_back(ei);

			updateWorldState();
		}

		pthread_mutex_unlock(&_worldStateMutex);
		break;
	}

	case cdl::DELETE:
	{
		pthread_mutex_lock(&_worldStateMutex);

		ConceptualData::EventInfo ei;
		ei.type = ConceptualData::EventRoomDeleted;
		ei.roomId = _comaRoomWmAddressMap[wmChange.address]->roomId;
		ei.place1Id = -1;
		ei.place2Id = -1;
		_comaRoomWmAddressMap.erase(wmChange.address);
		_accumulatedEvents.push_back(ei);

		updateWorldState();

		pthread_mutex_unlock(&_worldStateMutex);
		break;
	}

	default:
		break;
	} // switch
}


// -------------------------------------------------------
void Observer::placeChanged(const cast::cdl::WorkingMemoryChange &wmChange)
{
	// Decide what change has been made
	switch (wmChange.operation)
	{
	case cdl::ADD:
	{
		SpatialData::PlacePtr placePtr;
		try
		{
			placePtr = getMemoryEntry<SpatialData::Place>(wmChange.address);
		}
		catch(CASTException &e)
		{
			log("Caught exception at %s. Message: %s", __HERE__, e.message.c_str());
			return;
		}

		pthread_mutex_lock(&_worldStateMutex);

		_placeWmAddressMap[wmChange.address] = placePtr;

		if (placePtr->status == SpatialData::PLACEHOLDER)
		{
			ConceptualData::EventInfo ei;
			ei.type = ConceptualData::EventPlaceholderAdded;
			ei.roomId = -1;
			ei.place1Id = placePtr->id;
			ei.place2Id = -1;
			_accumulatedEvents.push_back(ei);

			updateWorldState();
		}
		// updateWorldState(); // We will get that when coma room changes

		pthread_mutex_unlock(&_worldStateMutex);
		break;
	}

	case cdl::OVERWRITE:
	{
		SpatialData::PlacePtr placePtr;
		try
		{
			placePtr = getMemoryEntry<SpatialData::Place>(wmChange.address);
		}
		catch(CASTException &e)
		{
			log("Caught exception at %s. Message: %s", __HERE__, e.message.c_str());
			return;
		}

		pthread_mutex_lock(&_worldStateMutex);

		SpatialData::PlacePtr old = _placeWmAddressMap[wmChange.address];
	  	_placeWmAddressMap[wmChange.address] = placePtr;

	  	// Check wmAddresss and ID
		if (old->id != placePtr->id)
			throw cast::CASTException("The mapping between Place WMAddress and ID changed!");

		// Check what has changed
		if ((old->status != placePtr->status) )
		{
			ConceptualData::EventInfo ei;
			ei.type = (placePtr->status == SpatialData::PLACEHOLDER)?
					ConceptualData::EventPlaceholderAdded:ConceptualData::EventPlaceholderDeleted;
			ei.roomId = -1;
			ei.place1Id = placePtr->id;
			ei.place2Id = -1;
			_accumulatedEvents.push_back(ei);

			updateWorldState();
		}

		pthread_mutex_unlock(&_worldStateMutex);
		break;
	}

	case cdl::DELETE:
	{
		pthread_mutex_lock(&_worldStateMutex);

		SpatialData::PlacePtr old = _placeWmAddressMap[wmChange.address];

		if (old->status == SpatialData::PLACEHOLDER)
		{
			ConceptualData::EventInfo ei;
			ei.type = ConceptualData::EventPlaceholderDeleted;
			ei.roomId = -1;
			ei.place1Id = old->id;
			ei.place2Id = -1;
			_accumulatedEvents.push_back(ei);

			_placeWmAddressMap.erase(wmChange.address);
			updateWorldState();
		}
		// updateWorldState(); // We will get that when coma room changes

		pthread_mutex_unlock(&_worldStateMutex);
		break;
	}

	default:
		break;
	} // switch
}


// -------------------------------------------------------
void Observer::gatewayPlacePropertyChanged(const cast::cdl::WorkingMemoryChange &wmChange)
{
	// Decide what change has been made
	switch (wmChange.operation)
	{
	case cdl::ADD:
	{
		SpatialProperties::GatewayPlacePropertyPtr gatewayPlacePropertyPtr;
		try
		{
			gatewayPlacePropertyPtr =
					getMemoryEntry<SpatialProperties::GatewayPlaceProperty>(wmChange.address);
		}
		catch(CASTException &e)
		{
			log("Caught exception at %s. Message: %s", __HERE__, e.message.c_str());
			return;
		}
		pthread_mutex_lock(&_worldStateMutex);

		_gatewayPlacePropertyWmAddressMap[wmChange.address] = gatewayPlacePropertyPtr;
		ConceptualData::EventInfo ei;
		ei.type = ConceptualData::EventGatewayPlacePropertyChanged;
		ei.roomId = -1;
		ei.place1Id = gatewayPlacePropertyPtr->placeId;
		ei.place2Id = -1;
		_accumulatedEvents.push_back(ei);

		updateWorldState();

		pthread_mutex_unlock(&_worldStateMutex);
		break;
	}

	case cdl::OVERWRITE:
	{
		SpatialProperties::GatewayPlacePropertyPtr gatewayPlacePropertyPtr;
		try
		{
			gatewayPlacePropertyPtr =
					getMemoryEntry<SpatialProperties::GatewayPlaceProperty>(wmChange.address);
		}// Room added
		catch(CASTException &e)
		{
			log("Caught exception at %s. Message: %s", __HERE__, e.message.c_str());
			return;
		}

		pthread_mutex_lock(&_worldStateMutex);

		SpatialProperties::GatewayPlacePropertyPtr old = _gatewayPlacePropertyWmAddressMap[wmChange.address];
	  	_gatewayPlacePropertyWmAddressMap[wmChange.address] = gatewayPlacePropertyPtr;

	  	// Check wmAddresss and ID
		if (old->placeId != gatewayPlacePropertyPtr->placeId)
			throw cast::CASTException("The mapping between GatewayPlaceProperty WMAddress and Place ID changed!");

		ConceptualData::EventInfo ei;
		ei.type = ConceptualData::EventGatewayPlacePropertyChanged;
		ei.roomId = -1;
		ei.place1Id = gatewayPlacePropertyPtr->placeId;
		ei.place2Id = -1;
		_accumulatedEvents.push_back(ei);

		updateWorldState();

		pthread_mutex_unlock(&_worldStateMutex);
		break;
	}

	case cdl::DELETE:
	{
		pthread_mutex_lock(&_worldStateMutex);

		ConceptualData::EventInfo ei;
		ei.type = ConceptualData::EventGatewayPlacePropertyChanged;
		ei.roomId = -1;
		ei.place1Id = _gatewayPlacePropertyWmAddressMap[wmChange.address]->placeId;
		ei.place2Id = -1;
		_gatewayPlacePropertyWmAddressMap.erase(wmChange.address);
		_accumulatedEvents.push_back(ei);

		updateWorldState();

		pthread_mutex_unlock(&_worldStateMutex);
		break;
	}

	default:
		break;
	} // switch
}


// -------------------------------------------------------
void Observer::objectPlacePropertyChanged(const cast::cdl::WorkingMemoryChange &wmChange)
{
	// Decide what change has been made
	switch (wmChange.operation)
	{
	case cdl::ADD:
	{
		SpatialProperties::ObjectPlacePropertyPtr objectPlacePropertyPtr;
		try
		{
			objectPlacePropertyPtr =
					getMemoryEntry<SpatialProperties::ObjectPlaceProperty>(wmChange.address);
		}
		catch(CASTException &e)
		{
			log("Caught exception at %s. Message: %s", __HERE__, e.message.c_str());
			return;
		}
		pthread_mutex_lock(&_worldStateMutex);

		_objectPlacePropertyWmAddressMap[wmChange.address] =
				SpatialProperties::ObjectPlacePropertyPtr::dynamicCast(objectPlacePropertyPtr->ice_clone());
		ConceptualData::EventInfo ei;
		ei.type = ConceptualData::EventObjectPlacePropertyAdded;
		ei.roomId = -1;
		ei.place1Id = objectPlacePropertyPtr->placeId;
		ei.place2Id = -1;
		if (objectPlacePropertyPtr->relation==SpatialData::INROOM)
		{
			ei.propertyInfo = objectPlacePropertyPtr->category;
		}
		else
		{
			ei.propertyInfo = objectPlacePropertyPtr->category +
					((objectPlacePropertyPtr->relation==SpatialData::ON)?" on ":" in ") +
					objectPlacePropertyPtr->supportObjectCategory + "-" +
					objectPlacePropertyPtr->supportObjectId;
		}
		_accumulatedEvents.push_back(ei);

		updateWorldState();

		pthread_mutex_unlock(&_worldStateMutex);
		break;
	}

	case cdl::OVERWRITE:
	{
		SpatialProperties::ObjectPlacePropertyPtr objectPlacePropertyPtr;
		try
		{
			objectPlacePropertyPtr =
					getMemoryEntry<SpatialProperties::ObjectPlaceProperty>(wmChange.address);
		}
		catch(CASTException &e)
		{
			log("Caught exception at %s. Message: %s", __HERE__, e.message.c_str());
			return;
		}
		pthread_mutex_lock(&_worldStateMutex);

		SpatialProperties::ObjectPlacePropertyPtr old = _objectPlacePropertyWmAddressMap[wmChange.address];
	  	_objectPlacePropertyWmAddressMap[wmChange.address] =
	  			SpatialProperties::ObjectPlacePropertyPtr::dynamicCast(objectPlacePropertyPtr->ice_clone());

	  	// Check wmAddresss and ID
		if (old->placeId != objectPlacePropertyPtr->placeId)
			throw cast::CASTException("The mapping between ObjectPlaceProperty WMAddress and Place ID changed!");

		ConceptualData::EventInfo ei;
		ei.type = ConceptualData::EventObjectPlacePropertyChanged;
		ei.roomId = -1;
		ei.place1Id = objectPlacePropertyPtr->placeId;
		ei.place2Id = -1;
		if (objectPlacePropertyPtr->relation==SpatialData::INROOM)
		{
			ei.propertyInfo = objectPlacePropertyPtr->category;
		}
		else
		{
			ei.propertyInfo = objectPlacePropertyPtr->category +
					((objectPlacePropertyPtr->relation==SpatialData::ON)?" on ":" in ") +
					objectPlacePropertyPtr->supportObjectCategory + "-" +
					objectPlacePropertyPtr->supportObjectId;
		}
		_accumulatedEvents.push_back(ei);

		updateWorldState();

		pthread_mutex_unlock(&_worldStateMutex);
		break;
	}

	case cdl::DELETE:
	{
		pthread_mutex_lock(&_worldStateMutex);

		SpatialProperties::ObjectPlacePropertyPtr old = _objectPlacePropertyWmAddressMap[wmChange.address];
		ConceptualData::EventInfo ei;
		ei.type = ConceptualData::EventObjectPlacePropertyDeleted;
		ei.roomId = -1;
		ei.place1Id = old->placeId;
		ei.place2Id = -1;
		if (old->relation==SpatialData::INROOM)
		{
			ei.propertyInfo = old->category;
		}
		else
		{
			ei.propertyInfo = old->category +
					((old->relation==SpatialData::ON)?" on ":" in ") +
					old->supportObjectCategory + "-" +
					old->supportObjectId;
		}
		_objectPlacePropertyWmAddressMap.erase(wmChange.address);
		_accumulatedEvents.push_back(ei);

		updateWorldState();

		pthread_mutex_unlock(&_worldStateMutex);
		break;
	}

	default:
		break;
	} // switch
}


// -------------------------------------------------------
void Observer::objectSearchResultChanged(const cast::cdl::WorkingMemoryChange &wmChange)
{
	// Decide what change has been made
	switch (wmChange.operation)
	{
	case cdl::ADD:
	{
		SpatialData::ObjectSearchResultPtr objectSearchResultPtr;
		try
		{
			objectSearchResultPtr =
					getMemoryEntry<SpatialData::ObjectSearchResult>(wmChange.address);
		}
		catch(CASTException &e)
		{
			log("Caught exception at %s. Message: %s", __HERE__, e.message.c_str());
			return;
		}
		pthread_mutex_lock(&_worldStateMutex);

		_objectSearchResultWmAddressMap[wmChange.address] =
				SpatialData::ObjectSearchResultPtr::dynamicCast(objectSearchResultPtr->ice_clone());
		ConceptualData::EventInfo ei;
		ei.type = ConceptualData::EventObjectSearchResultAdded;
		ei.roomId = objectSearchResultPtr->roomId;
		ei.place1Id = -1;
		ei.place2Id = -1;
		if (objectSearchResultPtr->relation==SpatialData::INROOM)
		{
			ei.propertyInfo = objectSearchResultPtr->searchedObjectCategory;
		}
		else
		{
			ei.propertyInfo = objectSearchResultPtr->searchedObjectCategory +
					((objectSearchResultPtr->relation==SpatialData::ON)?" on ":" in ") +
					objectSearchResultPtr->supportObjectCategory + "-" +
					objectSearchResultPtr->supportObjectId;
		}
		_accumulatedEvents.push_back(ei);

		updateWorldState();

		pthread_mutex_unlock(&_worldStateMutex);
		break;
	}

	case cdl::OVERWRITE:
	{
		SpatialData::ObjectSearchResultPtr objectSearchResultPtr;
		try
		{
			objectSearchResultPtr =
					getMemoryEntry<SpatialData::ObjectSearchResult>(wmChange.address);
		}
		catch(CASTException &e)
		{
			log("Caught exception at %s. Message: %s", __HERE__, e.message.c_str());
			return;
		}
		pthread_mutex_lock(&_worldStateMutex);

		SpatialData::ObjectSearchResultPtr old = _objectSearchResultWmAddressMap[wmChange.address];
		_objectSearchResultWmAddressMap[wmChange.address] =
				SpatialData::ObjectSearchResultPtr::dynamicCast(objectSearchResultPtr->ice_clone());

	  	// Check wmAddresss and ID
		if (old->roomId != objectSearchResultPtr->roomId)
			throw cast::CASTException("The mapping between ObjectSearchResult WMAddress and Room ID changed!");

		// Check if something substantial changed
		if (fabs(old->beta-objectSearchResultPtr->beta)> _betaThreshold)
		{
			ConceptualData::EventInfo ei;
			ei.type = ConceptualData::EventObjectSearchResultChanged;
			ei.roomId = objectSearchResultPtr->roomId;
			ei.place1Id = -1;
			ei.place2Id = -1;
			if (objectSearchResultPtr->relation==SpatialData::INROOM)
			{
				ei.propertyInfo = objectSearchResultPtr->searchedObjectCategory;
			}
			else
			{
				ei.propertyInfo = objectSearchResultPtr->searchedObjectCategory +
						((objectSearchResultPtr->relation==SpatialData::ON)?" on ":" in ") +
						objectSearchResultPtr->supportObjectCategory + "-" +
						objectSearchResultPtr->supportObjectId;
			}
			_accumulatedEvents.push_back(ei);

			updateWorldState();
		}

		pthread_mutex_unlock(&_worldStateMutex);
		break;
	}

	case cdl::DELETE:
	{
		pthread_mutex_lock(&_worldStateMutex);

		SpatialData::ObjectSearchResultPtr old = _objectSearchResultWmAddressMap[wmChange.address];
		ConceptualData::EventInfo ei;
		ei.type = ConceptualData::EventObjectSearchResultDeleted;
		ei.roomId = old->roomId;
		ei.place1Id = -1;
		ei.place2Id = -1;
		if (old->relation==SpatialData::INROOM)
		{
			ei.propertyInfo = old->searchedObjectCategory;
		}
		else
		{
			ei.propertyInfo = old->searchedObjectCategory +
					((old->relation==SpatialData::ON)?" on ":" in ") +
					old->supportObjectCategory + "-" +
					old->supportObjectId;
		}
		_objectSearchResultWmAddressMap.erase(wmChange.address);
		_accumulatedEvents.push_back(ei);

		updateWorldState();

		pthread_mutex_unlock(&_worldStateMutex);
		break;
	}

	default:
		break;
	} // switch
}


// -------------------------------------------------------
void Observer::shapePlacePropertyChanged(const cast::cdl::WorkingMemoryChange &wmChange)
{
	// Decide what change has been made
	switch (wmChange.operation)
	{
	case cdl::ADD:
	{
		SpatialProperties::RoomShapePlacePropertyPtr shapePlacePropertyPtr;
		try
		{
			shapePlacePropertyPtr =
					getMemoryEntry<SpatialProperties::RoomShapePlaceProperty>(wmChange.address);
		}
		catch(CASTException &e)
		{
			log("Caught exception at %s. Message: %s", __HERE__, e.message.c_str());
			return;
		}
		pthread_mutex_lock(&_worldStateMutex);

		_shapePlacePropertyWmAddressMap[wmChange.address] = shapePlacePropertyPtr;

		if (isGatewayPlace(shapePlacePropertyPtr->placeId))
		{
			pthread_mutex_unlock(&_worldStateMutex);
			return;
		}

		ConceptualData::EventInfo ei;
		ei.type = ConceptualData::EventShapePlacePropertyAdded;
		ei.roomId = -1;
		ei.place1Id = shapePlacePropertyPtr->placeId;
		ei.place2Id = -1;
		_accumulatedEvents.push_back(ei);

		updateWorldState();

		pthread_mutex_unlock(&_worldStateMutex);
		break;
	}

	case cdl::OVERWRITE:
	{
		SpatialProperties::RoomShapePlacePropertyPtr shapePlacePropertyPtr;
		try
		{
			shapePlacePropertyPtr =
					getMemoryEntry<SpatialProperties::RoomShapePlaceProperty>(wmChange.address);
		}
		catch(CASTException &e)
		{
			log("Caught exception at %s. Message: %s", __HERE__, e.message.c_str());
			return;
		}
		pthread_mutex_lock(&_worldStateMutex);

		SpatialProperties::RoomShapePlacePropertyPtr old = _shapePlacePropertyWmAddressMap[wmChange.address];
	  	_shapePlacePropertyWmAddressMap[wmChange.address] = shapePlacePropertyPtr;

	  	// Check wmAddresss and ID
		if (old->placeId != shapePlacePropertyPtr->placeId)
			throw cast::CASTException("The mapping between RoomShapePlaceProperty WMAddress and Place ID changed!");

		if (isGatewayPlace(shapePlacePropertyPtr->placeId))
		{
			pthread_mutex_unlock(&_worldStateMutex);
			return;
		}

		// Check if something substantial changed
		if ( calculateDistributionDifference(old->distribution,
								shapePlacePropertyPtr->distribution) > _shapeThreshold )
		{
			ConceptualData::EventInfo ei;
			ei.type = ConceptualData::EventShapePlacePropertyChanged;
			ei.roomId = -1;
			ei.place1Id = shapePlacePropertyPtr->placeId;
			ei.place2Id = -1;
			_accumulatedEvents.push_back(ei);

			updateWorldState();
		}

		pthread_mutex_unlock(&_worldStateMutex);
		break;
	}

	case cdl::DELETE:
	{
		pthread_mutex_lock(&_worldStateMutex);

		SpatialProperties::RoomShapePlacePropertyPtr old = _shapePlacePropertyWmAddressMap[wmChange.address];
		ConceptualData::EventInfo ei;
		ei.type = ConceptualData::EventShapePlacePropertyDeleted;
		ei.roomId = -1;
		ei.place1Id = old->placeId;
		ei.place2Id = -1;
		_shapePlacePropertyWmAddressMap.erase(wmChange.address);
		_accumulatedEvents.push_back(ei);

		updateWorldState();

		pthread_mutex_unlock(&_worldStateMutex);
		break;
	}

	default:
		break;
	} // switch
}



// -------------------------------------------------------
void Observer::appearancePlacePropertyChanged(const cast::cdl::WorkingMemoryChange &wmChange)
{
	// Decide what change has been made
	switch (wmChange.operation)
	{
	case cdl::ADD:
	{
		SpatialProperties::RoomAppearancePlacePropertyPtr appearancePlacePropertyPtr;
		try
		{
			appearancePlacePropertyPtr =
					getMemoryEntry<SpatialProperties::RoomAppearancePlaceProperty>(wmChange.address);
		}
		catch(CASTException &e)
		{
			log("Caught exception at %s. Message: %s", __HERE__, e.message.c_str());
			return;
		}
		pthread_mutex_lock(&_worldStateMutex);

		_appearancePlacePropertyWmAddressMap[wmChange.address] = appearancePlacePropertyPtr;

		if (isGatewayPlace(appearancePlacePropertyPtr->placeId))
		{
			pthread_mutex_unlock(&_worldStateMutex);
			return;
		}

		ConceptualData::EventInfo ei;
		ei.type = ConceptualData::EventAppearancePlacePropertyAdded;
		ei.roomId = -1;
		ei.place1Id = appearancePlacePropertyPtr->placeId;
		ei.place2Id = -1;
		_accumulatedEvents.push_back(ei);

		updateWorldState();

		pthread_mutex_unlock(&_worldStateMutex);
		break;
	}

	case cdl::OVERWRITE:
	{
		SpatialProperties::RoomAppearancePlacePropertyPtr appearancePlacePropertyPtr;
		try
		{
			appearancePlacePropertyPtr =
					getMemoryEntry<SpatialProperties::RoomAppearancePlaceProperty>(wmChange.address);
		}
		catch(CASTException &e)
		{
			log("Caught exception at %s. Message: %s", __HERE__, e.message.c_str());
			return;
		}
		pthread_mutex_lock(&_worldStateMutex);

		SpatialProperties::RoomAppearancePlacePropertyPtr old = _appearancePlacePropertyWmAddressMap[wmChange.address];
	  	_appearancePlacePropertyWmAddressMap[wmChange.address] = appearancePlacePropertyPtr;

	  	// Check wmAddresss and ID
		if (old->placeId != appearancePlacePropertyPtr->placeId)
			throw cast::CASTException("The mapping between RoomAppearancePlaceProperty WMAddress and Place ID changed!");

		if (isGatewayPlace(appearancePlacePropertyPtr->placeId))
		{
			pthread_mutex_unlock(&_worldStateMutex);
			return;
		}

		// Check if something substantial changed
		if ( calculateDistributionDifference(old->distribution,
								appearancePlacePropertyPtr->distribution) > _appearanceThreshold )
		{
			ConceptualData::EventInfo ei;
			ei.type = ConceptualData::EventAppearancePlacePropertyChanged;
			ei.roomId = -1;
			ei.place1Id = appearancePlacePropertyPtr->placeId;
			ei.place2Id = -1;
			_accumulatedEvents.push_back(ei);

			updateWorldState();
		}

		pthread_mutex_unlock(&_worldStateMutex);
		break;
	}

	case cdl::DELETE:
	{
		pthread_mutex_lock(&_worldStateMutex);

		SpatialProperties::RoomAppearancePlacePropertyPtr old = _appearancePlacePropertyWmAddressMap[wmChange.address];
		ConceptualData::EventInfo ei;
		ei.type = ConceptualData::EventAppearancePlacePropertyDeleted;
		ei.roomId = -1;
		ei.place1Id = old->placeId;
		ei.place2Id = -1;
		_appearancePlacePropertyWmAddressMap.erase(wmChange.address);
		_accumulatedEvents.push_back(ei);

		updateWorldState();

		pthread_mutex_unlock(&_worldStateMutex);
		break;
	}

	default:
		break;
	} // switch
}


void Observer::gatewayPlaceholderPropertyChanged(const cast::cdl::WorkingMemoryChange &wmChange)
{
	// Decide what change has been made
	switch (wmChange.operation)
	{
	case cdl::ADD:
	{
		SpatialProperties::GatewayPlaceholderPropertyPtr gatewayPlaceholderPropertyPtr;
		try
		{
			gatewayPlaceholderPropertyPtr =
					getMemoryEntry<SpatialProperties::GatewayPlaceholderProperty>(wmChange.address);
		}
		catch(CASTException &e)
		{
			log("Caught exception at %s. Message: %s", __HERE__, e.message.c_str());
			return;
		}
		pthread_mutex_lock(&_worldStateMutex);

		_gatewayPlaceholderPropertyWmAddressMap[wmChange.address] = gatewayPlaceholderPropertyPtr;

		ConceptualData::EventInfo ei;
		ei.type = ConceptualData::EventGatewayPlaceholderPropertyAdded;
		ei.roomId = -1;
		ei.place1Id = gatewayPlaceholderPropertyPtr->placeId;
		ei.place2Id = -1;
		_accumulatedEvents.push_back(ei);

		updateWorldState();

		pthread_mutex_unlock(&_worldStateMutex);
		break;
	}

	case cdl::OVERWRITE:
	{
		SpatialProperties::GatewayPlaceholderPropertyPtr gatewayPlaceholderPropertyPtr;
		try
		{
			gatewayPlaceholderPropertyPtr =
					getMemoryEntry<SpatialProperties::GatewayPlaceholderProperty>(wmChange.address);
		}
		catch(CASTException &e)
		{
			log("Caught exception at %s. Message: %s", __HERE__, e.message.c_str());
			return;
		}
		pthread_mutex_lock(&_worldStateMutex);

		SpatialProperties::GatewayPlaceholderPropertyPtr old = _gatewayPlaceholderPropertyWmAddressMap[wmChange.address];
		_gatewayPlaceholderPropertyWmAddressMap[wmChange.address] = gatewayPlaceholderPropertyPtr;

	  	// Check wmAddresss and ID
		if (old->placeId != gatewayPlaceholderPropertyPtr->placeId)
			throw cast::CASTException("The mapping between GatewayPlacholderProperty WMAddress and Place ID changed!");

		// Check if something substantial changed
		if ( calculateBinaryDistributionDifference(old->distribution,
				gatewayPlaceholderPropertyPtr->distribution) > _gatewayThreshold )
		{
			ConceptualData::EventInfo ei;
			ei.type = ConceptualData::EventGatewayPlaceholderPropertyChanged;
			ei.roomId = -1;
			ei.place1Id = gatewayPlaceholderPropertyPtr->placeId;
			ei.place2Id = -1;
			_accumulatedEvents.push_back(ei);

			updateWorldState();
		}

		pthread_mutex_unlock(&_worldStateMutex);
		break;
	}

	case cdl::DELETE:
	{
		pthread_mutex_lock(&_worldStateMutex);

		SpatialProperties::GatewayPlaceholderPropertyPtr old = _gatewayPlaceholderPropertyWmAddressMap[wmChange.address];
		ConceptualData::EventInfo ei;
		ei.type = ConceptualData::EventGatewayPlaceholderPropertyDeleted;
		ei.roomId = -1;
		ei.place1Id = old->placeId;
		ei.place2Id = -1;
		_gatewayPlaceholderPropertyWmAddressMap.erase(wmChange.address);
		_accumulatedEvents.push_back(ei);

		updateWorldState();

		pthread_mutex_unlock(&_worldStateMutex);
		break;
	}

	default:
		break;
	} // switch*/
}


// -------------------------------------------------------
void Observer::connectivityPathPropertyChanged(const cast::cdl::WorkingMemoryChange &wmChange)
{
	// Decide what change has been made
	switch (wmChange.operation)
	{
	case cdl::ADD:
	{
		SpatialProperties::ConnectivityPathPropertyPtr connectivityPathPropertyPtr;
		try
		{
			connectivityPathPropertyPtr =
					getMemoryEntry<SpatialProperties::ConnectivityPathProperty>(wmChange.address);
		}
		catch(CASTException &e)
		{
			log("Caught exception at %s. Message: %s", __HERE__, e.message.c_str());
			return;
		}
		pthread_mutex_lock(&_worldStateMutex);

		_connectivityPathPropertyWmAddressMap[wmChange.address] = connectivityPathPropertyPtr;
		ConceptualData::EventInfo ei;
		ei.type = ConceptualData::EventRoomConnectivityChanged;
		ei.roomId = -1;
		ei.place1Id = connectivityPathPropertyPtr->place1Id;
		ei.place2Id = connectivityPathPropertyPtr->place2Id;
		_accumulatedEvents.push_back(ei);

		updateWorldState();

		pthread_mutex_unlock(&_worldStateMutex);
		break;
	}

	case cdl::OVERWRITE:
	{
		SpatialProperties::ConnectivityPathPropertyPtr connectivityPathPropertyPtr;
		try
		{
			connectivityPathPropertyPtr =
					getMemoryEntry<SpatialProperties::ConnectivityPathProperty>(wmChange.address);
		}
		catch(CASTException &e)
		{
			log("Caught exception at %s. Message: %s", __HERE__, e.message.c_str());
			return;
		}
		pthread_mutex_lock(&_worldStateMutex);

		SpatialProperties::ConnectivityPathPropertyPtr old = _connectivityPathPropertyWmAddressMap[wmChange.address];
	  	_connectivityPathPropertyWmAddressMap[wmChange.address] = connectivityPathPropertyPtr;

	  	// Check wmAddresss and ID
		if ( (old->place1Id != connectivityPathPropertyPtr->place1Id) ||
				(old->place2Id != connectivityPathPropertyPtr->place2Id) )
			throw cast::CASTException("The mapping between ConnectivityPathProperty WMAddress and Place ID changed!");

		ConceptualData::EventInfo ei;
		ei.type = ConceptualData::EventRoomConnectivityChanged;
		ei.roomId = -1;
		ei.place1Id = connectivityPathPropertyPtr->place1Id;
		ei.place2Id = connectivityPathPropertyPtr->place2Id;
		_accumulatedEvents.push_back(ei);

		updateWorldState();

		pthread_mutex_unlock(&_worldStateMutex);
		break;
	}

	case cdl::DELETE:
	{
		pthread_mutex_lock(&_worldStateMutex);

		SpatialProperties::ConnectivityPathPropertyPtr old = _connectivityPathPropertyWmAddressMap[wmChange.address];
		ConceptualData::EventInfo ei;
		ei.type = ConceptualData::EventRoomConnectivityChanged;
		ei.roomId = -1;
		ei.place1Id = old->place1Id;
		ei.place2Id = old->place2Id;
		_connectivityPathPropertyWmAddressMap.erase(wmChange.address);
		_accumulatedEvents.push_back(ei);

		updateWorldState();

		pthread_mutex_unlock(&_worldStateMutex);
		break;
	}

	default:
		break;
	} // switch
}


// -------------------------------------------------------
bool Observer::isTruePlace(int placeId)
{
	map<cast::cdl::WorkingMemoryAddress, SpatialData::PlacePtr>::iterator placeIt;
	for( placeIt=_placeWmAddressMap.begin(); placeIt!=_placeWmAddressMap.end(); ++placeIt)
	{
		if (placeIt->second->id == placeId)
			return (placeIt->second->status == SpatialData::TRUEPLACE);
	}

	return false; // If not found, we will say it's not a true place
}


// -------------------------------------------------------
bool Observer::isPlaceholder(int placeId)
{
	map<cast::cdl::WorkingMemoryAddress, SpatialData::PlacePtr>::iterator placeIt;
	for( placeIt=_placeWmAddressMap.begin(); placeIt!=_placeWmAddressMap.end(); ++placeIt)
	{
		if (placeIt->second->id == placeId)
			return (placeIt->second->status == SpatialData::PLACEHOLDER);
	}

	return false; // If not found, we will say it's not a placeholder
}


// -------------------------------------------------------
bool Observer::isPlaceholderPresent(const ConceptualData::ComaRoomInfo &cri, int placeholderId)
{
	for (unsigned int i=0; i<cri.placeholders.size(); ++i)
	{
		if (cri.placeholders[i].placeholderId == placeholderId)
			return true;
	}
	return false;
}


// -------------------------------------------------------
int Observer::isPlaceholderInRooms(const vector<ConceptualData::ComaRoomInfo> &cris, int placeholderId)
{
	for (unsigned int r=0; r<cris.size(); ++r)
	{
		const ConceptualData::ComaRoomInfo &cri = cris[r];
		for (unsigned int i=0; i<cri.placeholders.size(); ++i)
		{
			if (cri.placeholders[i].placeholderId == placeholderId)
				return r;
		}
	}
	return -1;
}


// -------------------------------------------------------
bool Observer::isGatewayPlace(int placeId)
{
	map<cast::cdl::WorkingMemoryAddress, SpatialProperties::GatewayPlacePropertyPtr>::iterator gatewayPlacePropertyIt;
	for( gatewayPlacePropertyIt=_gatewayPlacePropertyWmAddressMap.begin();
			gatewayPlacePropertyIt!=_gatewayPlacePropertyWmAddressMap.end(); ++gatewayPlacePropertyIt)
	{
		SpatialProperties::GatewayPlacePropertyPtr gatewayPlacePropertyPtr = gatewayPlacePropertyIt->second;
		if (gatewayPlacePropertyPtr->placeId == placeId)
		{
			if ( (gatewayPlacePropertyPtr->mapValueReliable) &&
					(SpatialProperties::BinaryValuePtr::dynamicCast(gatewayPlacePropertyPtr->mapValue)->value == true) )
				return true;
		}
	}
	return false;
}


// -------------------------------------------------------
void Observer::getObjectPlaceProperties(int placeId,
		std::vector<SpatialProperties::ObjectPlacePropertyPtr> &properties)
{
	map<cast::cdl::WorkingMemoryAddress, SpatialProperties::ObjectPlacePropertyPtr>::iterator objectPlacePropertyIt;
	for( objectPlacePropertyIt=_objectPlacePropertyWmAddressMap.begin();
			objectPlacePropertyIt!=_objectPlacePropertyWmAddressMap.end(); ++objectPlacePropertyIt)
	{
		SpatialProperties::ObjectPlacePropertyPtr objectPlacePropertyPtr = objectPlacePropertyIt->second;
		// Get only the observed properties
		if ((objectPlacePropertyPtr->placeId == placeId) && (!objectPlacePropertyPtr->inferred))
			properties.push_back(objectPlacePropertyPtr);
	}
}


// -------------------------------------------------------
void Observer::getObjectSearchResults(int roomId,
		std::vector<SpatialData::ObjectSearchResultPtr> &results)
{
	map<cast::cdl::WorkingMemoryAddress, SpatialData::ObjectSearchResultPtr>::iterator objectSearchResultIt;
	for( objectSearchResultIt=_objectSearchResultWmAddressMap.begin();
			objectSearchResultIt!=_objectSearchResultWmAddressMap.end(); ++objectSearchResultIt)
	{
		SpatialData::ObjectSearchResultPtr objectSearchResultPtr = objectSearchResultIt->second;
		// Get only the observed properties
		if (objectSearchResultPtr->roomId == roomId)
			results.push_back(objectSearchResultPtr);
	}
}


// -------------------------------------------------------
void Observer::getShapePlaceProperties(int placeId,
		std::vector<SpatialProperties::RoomShapePlacePropertyPtr> &properties)
{
	map<cast::cdl::WorkingMemoryAddress, SpatialProperties::RoomShapePlacePropertyPtr>::iterator shapePlacePropertyIt;
	for( shapePlacePropertyIt=_shapePlacePropertyWmAddressMap.begin();
			shapePlacePropertyIt!=_shapePlacePropertyWmAddressMap.end(); ++shapePlacePropertyIt)
	{
		SpatialProperties::RoomShapePlacePropertyPtr shapePlacePropertyPtr = shapePlacePropertyIt->second;
		// Get only the observed properties
		if ((shapePlacePropertyPtr->placeId == placeId) && (!shapePlacePropertyPtr->inferred))
			properties.push_back(shapePlacePropertyPtr);
	}
}


// -------------------------------------------------------
void Observer::getAppearancePlaceProperties(int placeId,
		std::vector<SpatialProperties::RoomAppearancePlacePropertyPtr> &properties)
{
	map<cast::cdl::WorkingMemoryAddress, SpatialProperties::RoomAppearancePlacePropertyPtr>::iterator appearancePlacePropertyIt;
	for( appearancePlacePropertyIt=_appearancePlacePropertyWmAddressMap.begin();
			appearancePlacePropertyIt!=_appearancePlacePropertyWmAddressMap.end(); ++appearancePlacePropertyIt)
	{
		SpatialProperties::RoomAppearancePlacePropertyPtr appearancePlacePropertyPtr = appearancePlacePropertyIt->second;
		// Get only the observed properties
		if ((appearancePlacePropertyPtr->placeId == placeId) && (!appearancePlacePropertyPtr->inferred))
			properties.push_back(appearancePlacePropertyPtr);
	}
}


// -------------------------------------------------------
void Observer::getGatewayPlaceholderProperties(int placeId,
		std::vector<SpatialProperties::GatewayPlaceholderPropertyPtr> &properties)
{
	map<cast::cdl::WorkingMemoryAddress, SpatialProperties::GatewayPlaceholderPropertyPtr>::iterator gatewayPlaceholderPropertyIt;
	for( gatewayPlaceholderPropertyIt=_gatewayPlaceholderPropertyWmAddressMap.begin();
			gatewayPlaceholderPropertyIt!=_gatewayPlaceholderPropertyWmAddressMap.end(); ++gatewayPlaceholderPropertyIt)
	{
		SpatialProperties::GatewayPlaceholderPropertyPtr gatewayPlaceholderPropertyPtr = gatewayPlaceholderPropertyIt->second;
		// Get only the observed properties
		if ((gatewayPlaceholderPropertyPtr->placeId == placeId) && (!gatewayPlaceholderPropertyPtr->inferred))
			properties.push_back(gatewayPlaceholderPropertyPtr);
	}
}



// -------------------------------------------------------
int Observer::getRoomForPlace(int placeId)
{
	map<cast::cdl::WorkingMemoryAddress, comadata::ComaRoomPtr>::iterator comaRoomIt;
	for( comaRoomIt=_comaRoomWmAddressMap.begin(); comaRoomIt!=_comaRoomWmAddressMap.end(); ++comaRoomIt)
	{
		comadata::ComaRoomPtr comaRoomPtr = comaRoomIt->second;
		for (unsigned int i=0; i<comaRoomPtr->containedPlaceIds.size(); ++i)
		{
			if (comaRoomPtr->containedPlaceIds[i] == placeId)
				return comaRoomPtr->roomId;
		}
	}
	return -1;
}


// -------------------------------------------------------
void Observer::getConnectedPlaces(int placeId, vector<int> *connectedPlaces,
		set<int> *traversedPlaces)
{
	debug("getConnectedPlaces(placeId=%d)", placeId);

	boost::scoped_ptr<set<int> > alloc_traversedPlaces;
	if (!traversedPlaces)
		alloc_traversedPlaces.reset(traversedPlaces = new set<int>());
	traversedPlaces->insert(placeId);

	map<cast::cdl::WorkingMemoryAddress,
		SpatialProperties::ConnectivityPathPropertyPtr>::iterator connectivityPathPropertyIt;
	for( connectivityPathPropertyIt=_connectivityPathPropertyWmAddressMap.begin();
			connectivityPathPropertyIt!=_connectivityPathPropertyWmAddressMap.end();
			++connectivityPathPropertyIt)
	{
		SpatialProperties::ConnectivityPathPropertyPtr connectivityPathPropertyPtr =
				connectivityPathPropertyIt->second;

		if (connectivityPathPropertyPtr->place1Id == placeId)
		{
			int p = connectivityPathPropertyPtr->place2Id;
			debug("getConnectedPlaces(placeId=%d): found connection to place %d", placeId, p);

			if (traversedPlaces->find(p)==traversedPlaces->end())
			{
				// Is p a gateway?
				if (isGatewayPlace(p))
				{ // Find connected places recursively
					getConnectedPlaces(p, connectedPlaces, traversedPlaces);
				}
				else
					connectedPlaces->push_back(p);
				traversedPlaces->insert(p);
			}
		}
		if (connectivityPathPropertyPtr->place2Id == placeId)
		{
			int p = connectivityPathPropertyPtr->place1Id;
			debug("getConnectedPlaces(placeId=%d): found connection to place %d", placeId, p);

			if (traversedPlaces->find(p)==traversedPlaces->end())
			{
				// Is p a gateway?
				if (isGatewayPlace(p))
				{ // Find connected places recursively
					getConnectedPlaces(p, connectedPlaces, traversedPlaces);
				}
				else
					connectedPlaces->push_back(p);
				traversedPlaces->insert(p);
			}
		}
	}

	debug("getConnectedPlaces(placeId=%d): Finished searching for connected places", placeId);

	// Remove duplicates and the placeId itself.
	removeDuplicates(*connectedPlaces);
	connectedPlaces->erase(std::remove(connectedPlaces->begin(), connectedPlaces->end(),
			placeId), connectedPlaces->end());
}


} // namespace def



// Not used anymore
// -------------------------------------------------------
//bool Observer::areWorldStatesDifferent(ConceptualData::WorldStatePtr ws1, ConceptualData::WorldStatePtr ws2)
//{
//	if (ws1->roomConnections != ws2->roomConnections)
//		return true;
//
//	// Analyse the room structure
//	if (ws1->rooms.size() != ws2->rooms.size())
//		return true;
//	for (unsigned int i=0; i<ws1->rooms.size(); ++i)
//	{
//		ConceptualData::ComaRoomInfo &cri1 = ws1->rooms[i];
//		ConceptualData::ComaRoomInfo &cri2 = ws2->rooms[i];
//		if (cri1.wmAddress != cri2.wmAddress)
//			return true;
//		if (cri1.roomId != cri2.roomId)
//			return true;
//		// Compare object properties
//		if (cri1.objectProperties.size()!=cri2.objectProperties.size())
//			return true;
//		for (unsigned int j=0; j<cri1.objectProperties.size(); ++j)
//		{
//			ConceptualData::ObjectPlacePropertyInfo oppi1 = cri1.objectProperties[j];
//			ConceptualData::ObjectPlacePropertyInfo oppi2 = cri2.objectProperties[j];
//			if ( (oppi1.category!=oppi2.category) ||
//				 (oppi1.count!=oppi2.count) ||
//				 (oppi1.relation!=oppi2.relation) ||
//				 (oppi1.supportObjectCategory!=oppi2.supportObjectCategory) ||
//				 (oppi1.supportObjectId != oppi2.supportObjectId) ||
//				 (fabs(oppi1.beta-oppi2.beta)> _betaThreshold) )
//				return true;
//		}
//		// Compare places
//		if (cri1.places.size() != cri2.places.size())
//			return true;
//		for (unsigned int j=0; j<cri1.places.size(); ++j)
//		{
//			ConceptualData::PlaceInfo pi1 = cri1.places[j];
//			ConceptualData::PlaceInfo pi2 = cri2.places[j];
//			if (pi1.placeId != pi2.placeId)
//				return true;
////			if (pi1.objectProperties!=pi2.objectProperties)
////				return true;
//			// Check how different shapes are
//			if (pi1.shapeProperties.size() != pi2.shapeProperties.size())
//				return true;
//			for (unsigned int k=0; k<pi1.shapeProperties.size(); ++k)
//			{
//				if ( calculateDistributionDifference(pi1.shapeProperties[k].distribution,
//						pi2.shapeProperties[k].distribution) > _shapeThreshold )
//					return true;
//			}
//			// Check how different appearances are
//			if (pi1.appearanceProperties.size() != pi2.appearanceProperties.size())
//				return true;
//			for (unsigned int k=0; k<pi1.appearanceProperties.size(); ++k)
//			{
//				if ( calculateDistributionDifference(pi1.appearanceProperties[k].distribution,
//						pi2.appearanceProperties[k].distribution) > _appearanceThreshold )
//					return true;
//			}
//		}
//		// Check placeholders
//		if (cri1.placeholders.size() != cri2.placeholders.size())
//			return true;
//		for (unsigned int j=0; j<cri1.placeholders.size(); ++j)
//		{
//			ConceptualData::PlaceholderInfo phi1 = cri1.placeholders[j];
//			ConceptualData::PlaceholderInfo phi2 = cri2.placeholders[j];
//			if (phi1.placeholderId != phi2.placeholderId)
//				return true;
//		}
//	}
//
//	return false;
//}
