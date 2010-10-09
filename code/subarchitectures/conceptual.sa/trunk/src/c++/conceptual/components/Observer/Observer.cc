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

	// Global filter on SpatialProperties::ConnectivityPathProperty
	addChangeFilter(createGlobalTypeFilter<SpatialProperties::ConnectivityPathProperty>(cdl::WILDCARD),
			new MemberFunctionChangeReceiver<Observer>(this,
					&Observer::connectivityPathPropertyChanged));

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
			std::vector<SpatialProperties::ObjectPlacePropertyPtr> objectPlaceProperties;
			getObjectPlaceProperties(placeId, objectPlaceProperties);
			for (unsigned int j=0; j<objectPlaceProperties.size(); ++j)
			{
				SpatialProperties::ObjectPlacePropertyPtr objectPlacePropertyPtr =
						objectPlaceProperties[j];
				SpatialProperties::DiscreteProbabilityDistributionPtr dpdPtr =
						SpatialProperties::DiscreteProbabilityDistributionPtr::dynamicCast(objectPlacePropertyPtr->distribution);

				string objCategory = SpatialProperties::StringValuePtr::dynamicCast(dpdPtr->data[0].value)->value;
				double objPresenceProbability = dpdPtr->data[0].probability;
				ConceptualData::ObjectPlacePropertyInfo oppi;
				oppi.category = objCategory;
				oppi.present = (objPresenceProbability>0.5);
				pi.objectProperties.push_back(oppi);
			}

			cri.places.push_back(pi);
		} // for(unsigned int i=0; i<comaRoomPtr->containedPlaceIds.size(); ++i)

		newWorldStatePtr->rooms.push_back(cri);
	} // for( comaRoomIt=_comaRoomWmAddressMap.begin(); comaRoomIt!=_comaRoomWmAddressMap.end(); ++comaRoomIt)


	// -----------------------------------
	// Update on working memory if the new world
	// state is different than the previous one.
	// -----------------------------------
	if (_worldStatePtr->rooms != newWorldStatePtr->rooms)
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

// TODO: Catch all exceptions in events
// TODO: Not fail if I get delete and something already does not exist.
// TODO: Next fix the query handler
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
			log("Cought exception at %s. Message: %s", __HERE__, e.message.c_str());
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
			log("Cought exception at %s. Message: %s", __HERE__, e.message.c_str());
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
	{ // Room added
		SpatialData::PlacePtr placePtr;
		try
		{
			placePtr = getMemoryEntry<SpatialData::Place>(wmChange.address);
		}
		catch(CASTException &e)
		{
			log("Cought exception at %s. Message: %s", __HERE__, e.message.c_str());
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
			log("Cought exception at %s. Message: %s", __HERE__, e.message.c_str());
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
	{ // Room added
		SpatialProperties::GatewayPlacePropertyPtr gatewayPlacePropertyPtr;
		try
		{
			gatewayPlacePropertyPtr =
					getMemoryEntry<SpatialProperties::GatewayPlaceProperty>(wmChange.address);
		}
		catch(CASTException &e)
		{
			log("Cought exception at %s. Message: %s", __HERE__, e.message.c_str());
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
		}
		catch(CASTException &e)
		{
			log("Cought exception at %s. Message: %s", __HERE__, e.message.c_str());
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
	{ // Room added
		SpatialProperties::ObjectPlacePropertyPtr objectPlacePropertyPtr;
		try
		{
			objectPlacePropertyPtr =
					getMemoryEntry<SpatialProperties::ObjectPlaceProperty>(wmChange.address);
		}
		catch(CASTException &e)
		{
			log("Cought exception at %s. Message: %s", __HERE__, e.message.c_str());
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
			log("Cought exception at %s. Message: %s", __HERE__, e.message.c_str());
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
void Observer::connectivityPathPropertyChanged(const cast::cdl::WorkingMemoryChange &wmChange)
{
	// Decide what change has been made
	switch (wmChange.operation)
	{
	case cdl::ADD:
	{ // Room added
		SpatialProperties::ConnectivityPathPropertyPtr connectivityPathPropertyPtr;
		try
		{
			connectivityPathPropertyPtr =
					getMemoryEntry<SpatialProperties::ConnectivityPathProperty>(wmChange.address);
		}
		catch(CASTException &e)
		{
			log("Cought exception at %s. Message: %s", __HERE__, e.message.c_str());
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
			log("Cought exception at %s. Message: %s", __HERE__, e.message.c_str());
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
		if (objectPlacePropertyPtr->placeId == placeId)
			properties.push_back(objectPlacePropertyPtr);
	}
}


} // namespace def
