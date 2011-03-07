
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
	_betaThreshold = 0.01;

	if((it = _config.find("--shape-threshold")) != _config.end())
	{
		_shapeThreshold = boost::lexical_cast<double>(it->second);
	}
	if((it = _config.find("--appearance-threshold")) != _config.end())
	{
		_appearanceThreshold = boost::lexical_cast<double>(it->second);
	}
	if((it = _config.find("--beta-threshold")) != _config.end())
	{
		_betaThreshold = boost::lexical_cast<double>(it->second);
	}
	_includePlaceholderInfo = (_config.find("--no-placeholders") == _config.end());

	if ((_shapeThreshold<0.0) || (_appearanceThreshold<0.0))
		throw CASTException(exceptionMessage(__HERE__, "Please set shape-threshold and appearance-threshold!"));

	log("Configuration parameters:");
	log("-> Shape threshold: %f", _shapeThreshold);
	log("-> Appearances threshold: %f", _appearanceThreshold);
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


}


// -------------------------------------------------------
void Observer::runComponent()
{
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
void Observer::updateWorldState()
{
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
					// If we include placeholders and it's a new placeholder, add it to the room info
					if ( (_includePlaceholderInfo) && (isPlaceholder(connectedPlaces[j]))
							&& (!isPlaceholderPresent(cri, connectedPlaces[j])) )
					{
						ConceptualData::PlaceholderInfo pli;
						pli.placeholderId = connectedPlaces[j];

						// Add the place info
						cri.placeholders.push_back(pli);

					}
				}
			} // for(unsigned int j=0; j<connectedPlaces.size(); ++j)
		} // for(unsigned int i=0; i<comaRoomPtr->containedPlaceIds.size(); ++i)

		newWorldStatePtr->rooms.push_back(cri);
	} // for( comaRoomIt=_comaRoomWmAddressMap.begin(); comaRoomIt!=_comaRoomWmAddressMap.end(); ++comaRoomIt)

	// Clean up the room connectivity info
	removeDuplicates(newWorldStatePtr->roomConnections);

	// -----------------------------------
	// Update on working memory if the new world
	// state is different than the previous one.
	// -----------------------------------
	if ( areWorldStatesDifferent(_worldStatePtr, newWorldStatePtr ) )
	{
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
			log("Updating WorldState in the working memory. Something relevant must have changed.");
			lockEntry(_worldStateId, cdl::LOCKEDODR);
			overwriteWorkingMemory<ConceptualData::WorldState>(_worldStateId, _worldStatePtr);
			unlockEntry(_worldStateId);
		}
	}
}


// -------------------------------------------------------
bool Observer::areWorldStatesDifferent(ConceptualData::WorldStatePtr ws1, ConceptualData::WorldStatePtr ws2)
{
	if (ws1->roomConnections != ws2->roomConnections)
		return true;

	// Analyse the room structure
	if (ws1->rooms.size() != ws2->rooms.size())
		return true;
	for (unsigned int i=0; i<ws1->rooms.size(); ++i)
	{
		ConceptualData::ComaRoomInfo &cri1 = ws1->rooms[i];
		ConceptualData::ComaRoomInfo &cri2 = ws2->rooms[i];
		if (cri1.wmAddress != cri2.wmAddress)
			return true;
		if (cri1.roomId != cri2.roomId)
			return true;
		// Compare object properties
		if (cri1.objectProperties.size()!=cri2.objectProperties.size())
			return true;
		for (unsigned int j=0; j<cri1.objectProperties.size(); ++j)
		{
			ConceptualData::ObjectPlacePropertyInfo oppi1 = cri1.objectProperties[j];
			ConceptualData::ObjectPlacePropertyInfo oppi2 = cri2.objectProperties[j];
			if ( (oppi1.category!=oppi2.category) ||
				 (oppi1.count!=oppi2.count) ||
				 (oppi1.relation!=oppi2.relation) ||
				 (oppi1.supportObjectCategory!=oppi2.supportObjectCategory) ||
				 (oppi1.supportObjectId != oppi2.supportObjectId) ||
				 (fabs(oppi1.beta-oppi2.beta)> _betaThreshold) )
				return true;
		}
		// Compare places
		if (cri1.places.size() != cri2.places.size())
			return true;
		for (unsigned int j=0; j<cri1.places.size(); ++j)
		{
			ConceptualData::PlaceInfo pi1 = cri1.places[j];
			ConceptualData::PlaceInfo pi2 = cri2.places[j];
			if (pi1.placeId != pi2.placeId)
				return true;
//			if (pi1.objectProperties!=pi2.objectProperties)
//				return true;
			// Check how different shapes are
			if (pi1.shapeProperties.size() != pi2.shapeProperties.size())
				return true;
			for (unsigned int k=0; k<pi1.shapeProperties.size(); ++k)
			{
				if ( calculateDistributionDifference(pi1.shapeProperties[k].distribution,
						pi2.shapeProperties[k].distribution) > _shapeThreshold )
					return true;
			}
			// Check how different appearances are
			if (pi1.appearanceProperties.size() != pi2.appearanceProperties.size())
				return true;
			for (unsigned int k=0; k<pi1.appearanceProperties.size(); ++k)
			{
				if ( calculateDistributionDifference(pi1.appearanceProperties[k].distribution,
						pi2.appearanceProperties[k].distribution) > _appearanceThreshold )
					return true;
			}
		}
		// Check placeholders
		if (cri1.placeholders.size() != cri2.placeholders.size())
			return true;
		for (unsigned int j=0; j<cri1.placeholders.size(); ++j)
		{
			ConceptualData::PlaceholderInfo phi1 = cri1.placeholders[j];
			ConceptualData::PlaceholderInfo phi2 = cri2.placeholders[j];
			if (phi1.placeholderId != phi2.placeholderId)
				return true;
		}
	}

	return false;
}


// -------------------------------------------------------
double Observer::calculateDistributionDifference(const ConceptualData::ValuePotentialPairs &dist1,
		const ConceptualData::ValuePotentialPairs &dist2)
{
	if (dist1.size() != dist2.size())
		return 1e6;

	double difference = 0.0;
	for (unsigned int i=0; i< dist1.size(); ++i)
	{
		if (dist1[i].value != dist2[i].value)
			return 1e6;
		double tmp = fabs(dist1[i].potential - dist2[i].potential);
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

		_comaRoomWmAddressMap[wmChange.address] = comaRoomPtr;
		updateWorldState();
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

		comadata::ComaRoomPtr old = _comaRoomWmAddressMap[wmChange.address];
		_comaRoomWmAddressMap[wmChange.address] = comaRoomPtr;

		// Check wmAddresss and ID
		if (old->roomId != comaRoomPtr->roomId)
			throw cast::CASTException("The mapping between ComaRoom WMAddress and ID changed!");

		updateWorldState();
		break;
	}

	case cdl::DELETE:
	{
		_comaRoomWmAddressMap.erase(wmChange.address);
		updateWorldState();
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

		_placeWmAddressMap[wmChange.address] = placePtr;
		updateWorldState();
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

		SpatialData::PlacePtr old = _placeWmAddressMap[wmChange.address];
	  	_placeWmAddressMap[wmChange.address] = placePtr;

	  	// Check wmAddresss and ID
		if (old->id != placePtr->id)
			throw cast::CASTException("The mapping between Place WMAddress and ID changed!");

		updateWorldState();
		break;
	}

	case cdl::DELETE:
	{
		_placeWmAddressMap.erase(wmChange.address);
		updateWorldState();
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

		_gatewayPlacePropertyWmAddressMap[wmChange.address] = gatewayPlacePropertyPtr;
		updateWorldState();
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

		SpatialProperties::GatewayPlacePropertyPtr old = _gatewayPlacePropertyWmAddressMap[wmChange.address];
	  	_gatewayPlacePropertyWmAddressMap[wmChange.address] = gatewayPlacePropertyPtr;

	  	// Check wmAddresss and ID
		if (old->placeId != gatewayPlacePropertyPtr->placeId)
			throw cast::CASTException("The mapping between GatewayPlaceProperty WMAddress and Place ID changed!");

		updateWorldState();
		break;
	}

	case cdl::DELETE:
	{
		_gatewayPlacePropertyWmAddressMap.erase(wmChange.address);
		updateWorldState();
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

		_objectPlacePropertyWmAddressMap[wmChange.address] = objectPlacePropertyPtr;
		updateWorldState();
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

		SpatialProperties::ObjectPlacePropertyPtr old = _objectPlacePropertyWmAddressMap[wmChange.address];
	  	_objectPlacePropertyWmAddressMap[wmChange.address] = objectPlacePropertyPtr;

	  	// Check wmAddresss and ID
		if (old->placeId != objectPlacePropertyPtr->placeId)
			throw cast::CASTException("The mapping between ObjectPlaceProperty WMAddress and Place ID changed!");

		updateWorldState();
		break;
	}

	case cdl::DELETE:
	{
		_objectPlacePropertyWmAddressMap.erase(wmChange.address);
		updateWorldState();
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

		_objectSearchResultWmAddressMap[wmChange.address] = objectSearchResultPtr;
		updateWorldState();
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

		SpatialData::ObjectSearchResultPtr old = _objectSearchResultWmAddressMap[wmChange.address];
		_objectSearchResultWmAddressMap[wmChange.address] = objectSearchResultPtr;

	  	// Check wmAddresss and ID
		if (old->roomId != objectSearchResultPtr->roomId)
			throw cast::CASTException("The mapping between ObjectSearchResult WMAddress and Room ID changed!");

		updateWorldState();
		break;
	}

	case cdl::DELETE:
	{
		_objectSearchResultWmAddressMap.erase(wmChange.address);
		updateWorldState();
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

		_shapePlacePropertyWmAddressMap[wmChange.address] = shapePlacePropertyPtr;
		updateWorldState();
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

		SpatialProperties::RoomShapePlacePropertyPtr old = _shapePlacePropertyWmAddressMap[wmChange.address];
	  	_shapePlacePropertyWmAddressMap[wmChange.address] = shapePlacePropertyPtr;

	  	// Check wmAddresss and ID
		if (old->placeId != shapePlacePropertyPtr->placeId)
			throw cast::CASTException("The mapping between RoomShapePlaceProperty WMAddress and Place ID changed!");

		updateWorldState();
		break;
	}

	case cdl::DELETE:
	{
		_objectPlacePropertyWmAddressMap.erase(wmChange.address);
		updateWorldState();
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

		_appearancePlacePropertyWmAddressMap[wmChange.address] = appearancePlacePropertyPtr;
		updateWorldState();
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

		SpatialProperties::RoomAppearancePlacePropertyPtr old = _appearancePlacePropertyWmAddressMap[wmChange.address];
	  	_appearancePlacePropertyWmAddressMap[wmChange.address] = appearancePlacePropertyPtr;

	  	// Check wmAddresss and ID
		if (old->placeId != appearancePlacePropertyPtr->placeId)
			throw cast::CASTException("The mapping between RoomAppearancePlaceProperty WMAddress and Place ID changed!");

		updateWorldState();
		break;
	}

	case cdl::DELETE:
	{
		_objectPlacePropertyWmAddressMap.erase(wmChange.address);
		updateWorldState();
		break;
	}

	default:
		break;
	} // switch
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

		_connectivityPathPropertyWmAddressMap[wmChange.address] = connectivityPathPropertyPtr;
		updateWorldState();
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

		SpatialProperties::ConnectivityPathPropertyPtr old = _connectivityPathPropertyWmAddressMap[wmChange.address];
	  	_connectivityPathPropertyWmAddressMap[wmChange.address] = connectivityPathPropertyPtr;

	  	// Check wmAddresss and ID
		if ( (old->place1Id != connectivityPathPropertyPtr->place1Id) ||
				(old->place2Id != connectivityPathPropertyPtr->place2Id) )
			throw cast::CASTException("The mapping between ConnectivityPathProperty WMAddress and Place ID changed!");

		updateWorldState();
		break;
	}

	case cdl::DELETE:
	{
		_connectivityPathPropertyWmAddressMap.erase(wmChange.address);
		updateWorldState();
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
//	debug("getConnectedPlaces(placeId=%d)", placeId);

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
//			debug("getConnectedPlaces(placeId=%d): found connection to place %d", placeId, p);

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
//			debug("getConnectedPlaces(placeId=%d): found connection to place %d", placeId, p);

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

//	debug("getConnectedPlaces(placeId=%d): Finished searching for connected places", placeId);

	// Remove duplicates and the placeId itself.
	removeDuplicates(*connectedPlaces);
	connectedPlaces->erase(std::remove(connectedPlaces->begin(), connectedPlaces->end(),
			placeId), connectedPlaces->end());
}


} // namespace def
