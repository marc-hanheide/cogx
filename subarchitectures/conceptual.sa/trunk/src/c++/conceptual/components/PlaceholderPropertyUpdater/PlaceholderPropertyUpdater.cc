/**
 * @author Andrzej Pronobis
 *
 * Definition of the conceptual::PlaceholderPropertyUpdater class.
 */

// Conceptual.SA
#include "PlaceholderPropertyUpdater.h"
// System
#include "ComaData.hpp"
#include "SpatialProbabilities.hpp"
// CAST
#include <cast/architecture/ChangeFilterFactory.hpp>

#include <algorithm>


/** The function called to create a new instance of our component. */
extern "C"
{
	cast::CASTComponentPtr newComponent()
	{
		return new conceptual::PlaceholderPropertyUpdater();
	}
}

namespace conceptual
{

using namespace std;
using namespace cast;


// -------------------------------------------------------
PlaceholderPropertyUpdater::PlaceholderPropertyUpdater() : _worldStateChanged(false)
{
	pthread_cond_init(&_worldStateChangedSignalCond, 0);
	pthread_mutex_init(&_worldStateChangedSignalMutex, 0);
}


// -------------------------------------------------------
PlaceholderPropertyUpdater::~PlaceholderPropertyUpdater()
{
	pthread_cond_destroy(&_worldStateChangedSignalCond);
	pthread_mutex_destroy(&_worldStateChangedSignalMutex);
}


// -------------------------------------------------------
void PlaceholderPropertyUpdater::configure(const map<string,string> & _config)
{
	map<string,string>::const_iterator it;

	// QueryHandler name
	if((it = _config.find("--queryhandler")) != _config.end())
		_queryHandlerName = it->second;
	if((it = _config.find("--defaultchaingraphinferencer")) != _config.end())
		_defaultChainGraphInferencerName = it->second;

	log("Configuration parameters:");
	log("-> QueryHandler name: %s", _queryHandlerName.c_str());
	log("-> DefaultChainGraphInferencer name: %s", _defaultChainGraphInferencerName.c_str());
}


// -------------------------------------------------------
void PlaceholderPropertyUpdater::start()
{
	// Get the QueryHandler interface proxy
	_queryHandlerServerInterfacePrx =
			getIceServer<ConceptualData::QueryHandlerServerInterface>(_queryHandlerName);

	// Get the DefaultChainGraphInferencer interface proxy
	_defaultChainGraphInferencerServerInterfacePrx =
			getIceServer<DefaultData::ChainGraphInferencerServerInterface>(_defaultChainGraphInferencerName);

	// Change filters
	addChangeFilter(createLocalTypeFilter<ConceptualData::WorldState>(cdl::OVERWRITE),
			new MemberFunctionChangeReceiver<PlaceholderPropertyUpdater>(this,
					&PlaceholderPropertyUpdater::worldStateChanged));
	addChangeFilter(createLocalTypeFilter<ConceptualData::WorldState>(cdl::ADD),
			new MemberFunctionChangeReceiver<PlaceholderPropertyUpdater>(this,
					&PlaceholderPropertyUpdater::worldStateChanged));
	// Global filter on SpatialData::Place
	addChangeFilter(createGlobalTypeFilter<SpatialData::Place>(cdl::WILDCARD),
			new MemberFunctionChangeReceiver<PlaceholderPropertyUpdater>(this,
					&PlaceholderPropertyUpdater::placeChanged));

}


// -------------------------------------------------------
void PlaceholderPropertyUpdater::runComponent()
{
	// Getting information about possible room categories from default.sa
	log("Obtaining default knowledge factors from DefaultChainGraphInferencer.");
	_roomCategories =
		_defaultChainGraphInferencerServerInterfacePrx->getRoomCategories();
	// Error checking
	if ( _roomCategories.empty() )
		throw CASTException("Did not receive information from Default.SA. Is everything started?");


	// Run component
	while(isRunning())
	{
		// Get current time and add 1 sec
		timespec ts;
		timeval tv;
		gettimeofday(&tv, NULL);
		ts.tv_sec = tv.tv_sec + 5; // FIXME @demmeln 12.04.2012: Hack, should be 1 second
		ts.tv_nsec = 0;

		// Wait if necessary
		pthread_mutex_lock(&_worldStateChangedSignalMutex);
		if (!_worldStateChanged)
			pthread_cond_timedwait(&_worldStateChangedSignalCond, &_worldStateChangedSignalMutex, &ts);

    // FIXME @demmeln 12.04.2012: This is a hack to make placeholders work that
    // are associated to doors.
    // 
    // Possibly uncomment this to force an update of the placeholders even
    // without conceptual world state getting an update.
    // 
    // if (!_worldStateChanged)
    // {
		// 	log("Timeout on world state change. Forcing update of placeholders [HACK].");
    //   _worldStateChanged = true;
    // }

		// Handle signal if signal arrived
		if ((!isRunning()) || (!_worldStateChanged))
			pthread_mutex_unlock(&_worldStateChangedSignalMutex);
		else
		{
			log("Processing a world state change.");

			// Mark that we have handled the world state change
			_worldStateChanged=false;

			// Get a local copy of the placeholder ids.
			vector<int> placeholderIds = _placeholderIds;

			pthread_mutex_unlock(&_worldStateChangedSignalMutex);

			sched_yield();

      // FIXME @demmeln 12.04.2012: Hack for placeholders connected to doors
      vector<int> unassociatedPlaceholderIds;
      {
        map<cdl::WorkingMemoryAddress, int>::iterator it;
        for (it = _placeWmAddressMap.begin(); it != _placeWmAddressMap.end(); ++it)
        {
          int placeholderId = it->second;
            
          if (find(placeholderIds.begin(), placeholderIds.end(), placeholderId) == placeholderIds.end())
          {
            // placeholder in WM not found in placeholderIds, which is generated from conceptual world
            // give it some default placeholder room category property.

            log("Placeholder %d not associated to a room. Giving default room category propertoies. [HACK]",
                placeholderId);
            
            // For each room category
            for (unsigned j=0; j<_roomCategories.size(); j++)
            {
              string roomCategory = _roomCategories[j];

              SpatialProbabilities::ProbabilityDistribution probDist;

              // set default value of 0.3 existance
              probDist.massFunction.resize(2);
              probDist.massFunction[0].variableValues.push_back(
                  new SpatialProbabilities::IntRandomVariableValue(0));
              probDist.massFunction[0].probability = 0.7;
              probDist.massFunction[1].variableValues.push_back(
                  new SpatialProbabilities::IntRandomVariableValue(1));
              probDist.massFunction[1].probability = 0.3;

              updateRoomCategoryPlaceholderProperty(placeholderId, roomCategory, probDist);
            }
          }
        }
      }

			// Update the properties of each placeholder
			for (unsigned i=0; i<placeholderIds.size(); i++)
			{
				int placeholderId = placeholderIds[i];

        log("Placeholder %d associated to a room. Updating its category propertoies.",
            placeholderId);

				// For each room category
				for (unsigned j=0; j<_roomCategories.size(); j++)
				{
					string roomCategory = _roomCategories[j];

					// Request the inference and wait for results
					debug("Requesting inference about possible existance of %s at/behind placeholder %d.",
							roomCategory.c_str(), placeholderId);
					stringstream varName;
					varName<<"placeholder"<<placeholderId<<"_"<<roomCategory<<"_existance";
					ConceptualData::ProbabilityDistributions probDists =
							_queryHandlerServerInterfacePrx->imaginaryQuery("p("+varName.str()+")");

					sched_yield();

					// Update the property on Spatial WM
					if (probDists.size())
						updateRoomCategoryPlaceholderProperty(placeholderId, roomCategory, probDists[0]);
				}
			}

			sched_yield();

		} // if
	} // while
} // runComponent()


// -------------------------------------------------------
void PlaceholderPropertyUpdater::stop()
{
}


// -------------------------------------------------------
void PlaceholderPropertyUpdater::worldStateChanged(const cast::cdl::WorkingMemoryChange & wmChange)
{
	// Lock the mutex to make the operations below atomic
	pthread_mutex_lock(&_worldStateChangedSignalMutex);
	_worldStateChanged=true;

	// Get the placeholder information from the world state
	lockEntry(wmChange.address, cdl::LOCKEDOD);
	ConceptualData::WorldStatePtr worldStatePtr = getMemoryEntry<ConceptualData::WorldState>(wmChange.address);
	_placeholderIds.clear();
	for(unsigned int i=0; i<worldStatePtr->rooms.size(); ++i)
	{
		ConceptualData::ComaRoomInfo &cri = worldStatePtr->rooms[i];
		for(unsigned int j=0; j<cri.placeholders.size(); ++j)
			_placeholderIds.push_back(cri.placeholders[j].placeholderId);
	}
	unlockEntry(wmChange.address);

	// Signal to make an inference and update placeholder properties.
	pthread_cond_signal(&_worldStateChangedSignalCond);
	pthread_mutex_unlock(&_worldStateChangedSignalMutex);

}


// -------------------------------------------------------
void PlaceholderPropertyUpdater::updateRoomCategoryPlaceholderProperty(int placeholderId,
		string category, const SpatialProbabilities::ProbabilityDistribution &pd)
{
	debug("Updating room category placeholder property for placeholder ID:%d and category %s ...",
			placeholderId, category.c_str());

	// Check if the property already exists on WM
	cast::cdl::WorkingMemoryAddress wmAddress =
			getRoomCategoryPlaceholderPropertyWmAddress(placeholderId, category);
	if (wmAddress.id.empty())
	{ // No property yet, create
		SpatialProperties::RoomCategoryPlaceholderPropertyPtr propertyPtr =
				new SpatialProperties::RoomCategoryPlaceholderProperty();
		propertyPtr->category = category;
		propertyPtr->placeId = placeholderId;
		propertyPtr->inferred = true;
		setRoomCategoryPlaceholderPropertyDistribution(propertyPtr, pd);

		RoomCategoryPlaceholderPropertyInfo propertyInfo;
		propertyInfo.category = category;
		propertyInfo.placeholderId = placeholderId;
		propertyInfo.wmAddress.subarchitecture = "spatial.sa";
		propertyInfo.wmAddress.id = newDataID();

		// Just before sending out the property, check if the placeholder still exists.
		// If it does not, then do not update the property
		if (placeholderExists(placeholderId))
		{
			addToWorkingMemory(propertyInfo.wmAddress, propertyPtr);
			_placeholderProperties.push_back(propertyInfo);
			debug("Room category placeholder property for placeholder ID:%d and category %s added",
					placeholderId, category.c_str());
		}
		else
			debug("Room category placeholder property for placeholder ID:%d and category %s was NOT added since the placeholder disappeared!",
					placeholderId, category.c_str());
	}
	else
	{ // Property exists already, overwrite
		try
		{
			// Just before sending out the property, check if the placeholder still exists.
			// If it does not, then do not update the property
			if (placeholderExists(placeholderId))
			{

				lockEntry(wmAddress, cdl::LOCKEDOD);
				SpatialProperties::RoomCategoryPlaceholderPropertyPtr propertyPtr =
						getMemoryEntry<SpatialProperties::RoomCategoryPlaceholderProperty>(wmAddress);
				setRoomCategoryPlaceholderPropertyDistribution(propertyPtr, pd);
				overwriteWorkingMemory(wmAddress, propertyPtr);
				unlockEntry(wmAddress);
				debug("Room category placeholder property for placeholder ID:%d and category %s updated",
						placeholderId, category.c_str());
			}
			else
				debug("Room category placeholder property for placeholder ID:%d and category %s was NOT updated since the placeholder disappeared!",
						placeholderId, category.c_str());

		}
		catch(DoesNotExistOnWMException)
		{
			unlockEntry(wmAddress);
			error("Incorrect room category placeholder property address for placeholder ID:%d and category %s! Someone else is messing with placeholder propeties!",
					placeholderId, category.c_str());
		}

	}
}


// -------------------------------------------------------
bool PlaceholderPropertyUpdater::placeholderExists(int id)
{
	pthread_mutex_lock(&_worldStateChangedSignalMutex);
	bool found = false;
	for (std::map<cast::cdl::WorkingMemoryAddress, int>::iterator it = _placeWmAddressMap.begin();
			it!=_placeWmAddressMap.end(); ++it)
	{
		if (it->second==id)
		{
			found = true;
			break;
		}
	}
	pthread_mutex_unlock(&_worldStateChangedSignalMutex);
	return found;
}


// -------------------------------------------------------
cast::cdl::WorkingMemoryAddress PlaceholderPropertyUpdater::getRoomCategoryPlaceholderPropertyWmAddress(
		int placeholderId, string category)
{
	for(list<RoomCategoryPlaceholderPropertyInfo>::iterator it = _placeholderProperties.begin();
			it!=_placeholderProperties.end(); it++)
	{
		if ((it->placeholderId == placeholderId) && (it->category == category))
			return it->wmAddress;
	}

	return cast::cdl::WorkingMemoryAddress();
}


// -------------------------------------------------------
void PlaceholderPropertyUpdater::setRoomCategoryPlaceholderPropertyDistribution(
		SpatialProperties::RoomCategoryPlaceholderPropertyPtr propertyPtr,
		const SpatialProbabilities::ProbabilityDistribution &pd)
{
	// Set the distribution
	SpatialProperties::DiscreteProbabilityDistributionPtr distributionPtr =
			new SpatialProperties::DiscreteProbabilityDistribution();
	SpatialProperties::IntegerValuePtr mapIntVal;
	double mapProb = 0;
	for (unsigned int i=0; i<pd.massFunction.size(); ++i)
	{
		double prob = pd.massFunction[i].probability;
		int value = SpatialProbabilities::IntRandomVariableValuePtr::dynamicCast(pd.massFunction[i].variableValues[0])->value;

		SpatialProperties::ValueProbabilityPair vpp;
		SpatialProperties::IntegerValuePtr intVal = new SpatialProperties::IntegerValue();
		intVal->value = value;
		vpp.value = intVal;
		vpp.probability = prob;
		distributionPtr->data.push_back(vpp);

		// Set MAP
		if (prob>mapProb)
		{
			mapProb = prob;
			mapIntVal = intVal;
		}
	}
	propertyPtr->distribution = distributionPtr;

	// Set the MAP
	propertyPtr->mapValue = mapIntVal;
	propertyPtr->mapValueReliable = true;

}


void PlaceholderPropertyUpdater::placeChanged(const cast::cdl::WorkingMemoryChange &wmChange)
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

		if (placePtr->status == SpatialData::PLACEHOLDER)
		{
			pthread_mutex_lock(&_worldStateChangedSignalMutex);
			_placeWmAddressMap[wmChange.address] = placePtr->id;
			pthread_mutex_unlock(&_worldStateChangedSignalMutex);
		}
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

		pthread_mutex_lock(&_worldStateChangedSignalMutex);

		bool wasPlaceholder = _placeWmAddressMap.find(wmChange.address)!=_placeWmAddressMap.end();
		if (wasPlaceholder)
		{
			if (_placeWmAddressMap[wmChange.address] != placePtr->id)
				throw cast::CASTException("The mapping between Place WMAddress and ID changed!");

			if (placePtr->status != SpatialData::PLACEHOLDER)
			{ // Place stopped being a placeholder
				_placeWmAddressMap.erase(wmChange.address);
			}
		}
		else
		{
			if (placePtr->status == SpatialData::PLACEHOLDER)
			{ // Place is a placeholder now
				_placeWmAddressMap[wmChange.address] = placePtr->id;
			}
		}

		pthread_mutex_unlock(&_worldStateChangedSignalMutex);
		break;
	}

	case cdl::DELETE:
	{
		pthread_mutex_lock(&_worldStateChangedSignalMutex);

		if (_placeWmAddressMap.find(wmChange.address)!=_placeWmAddressMap.end())
			_placeWmAddressMap.erase(wmChange.address);

		pthread_mutex_unlock(&_worldStateChangedSignalMutex);
		break;
	}

	default:
		break;
	} // switch
}





} // namespace def

