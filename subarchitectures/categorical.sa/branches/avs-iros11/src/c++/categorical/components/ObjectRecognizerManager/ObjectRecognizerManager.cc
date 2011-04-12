/**
 * @author Andrzej Pronobis
 *
 * Definition of the ObjectRecognizerManager class.
 */

#include "ObjectRecognizerManager.h"
// System
#include "ConceptualData.hpp"
#include "VisionData.hpp"
#include "SpatialProperties.hpp"
// CAST
#include <cast/architecture/ChangeFilterFactory.hpp>


/** The function called to create a new instance of our component. */
extern "C"
{
	cast::CASTComponentPtr newComponent()
	{
		return new categorical::ObjectRecognizerManager();
	}
}

namespace categorical
{

using namespace std;
using namespace cast;
using namespace VisionData;

// -------------------------------------------------------
ObjectRecognizerManager::ObjectRecognizerManager() : _curRoomId(0)
{
}


// -------------------------------------------------------
ObjectRecognizerManager::~ObjectRecognizerManager()
{
}


// -------------------------------------------------------
void ObjectRecognizerManager::configure(const map<string,string> & _config)
{
	map<string,string>::const_iterator it;

	if((it = _config.find("--labels")) != _config.end())
	{
		istringstream istr(it->second);
		string label;
		while(istr >> label){
			m_labels.push_back(label);
		}
	}
	if((it = _config.find("--placemanager")) != _config.end())
		_placeManagerName = it->second;

	ostringstream ostr;
	for(size_t i = 0; i < m_labels.size(); i++)
		ostr << " '" << m_labels[i] << "'";
	log("Recognizing objects: %s", ostr.str().c_str());
}


// -------------------------------------------------------
void ObjectRecognizerManager::start()
{
	addChangeFilter(createGlobalTypeFilter<VisionData::Recognizer3DCommand>(cdl::OVERWRITE),
			new MemberFunctionChangeReceiver<ObjectRecognizerManager>(this,
					&ObjectRecognizerManager::overwriteRecognizer3DCommand));
	addChangeFilter(createGlobalTypeFilter<ConceptualData::WorldState>(cdl::OVERWRITE),
			new MemberFunctionChangeReceiver<ObjectRecognizerManager>(this,
					&ObjectRecognizerManager::worldStateChanged));
	addChangeFilter(createGlobalTypeFilter<ConceptualData::WorldState>(cdl::ADD),
			new MemberFunctionChangeReceiver<ObjectRecognizerManager>(this,
					&ObjectRecognizerManager::worldStateChanged));

	try
	{
		// Get the PlaceInterface interface proxy
		_placeInterfacePrx =
				getIceServer<FrontierInterface::PlaceInterface>(_placeManagerName);
	}
	catch(...)
	{}
}


// -------------------------------------------------------
void ObjectRecognizerManager::runComponent()
{
	usleep(1000000);  // HACK: the nav visualisation might crash if we send it
	                     // object observations too soon.

	// Run component
	while(isRunning())
	{
		debug("Recognizing objects!");
		for(size_t i=0; i<m_labels.size(); i++)
		{
			addRecognizer3DCommand(m_labels[i]);
		}
		usleep(2000000);
	} // while
} // runComponent()



// -------------------------------------------------------
void ObjectRecognizerManager::stop()
{
}


void ObjectRecognizerManager::addRecognizer3DCommand(std::string label)
{
	VisionData::Recognizer3DCommandPtr rec_cmd = new VisionData::Recognizer3DCommand;
	rec_cmd->cmd = VisionData::RECOGNIZE;
	rec_cmd->label = label;
	addToWorkingMemory(newDataID(), "vision.sa", rec_cmd);
	debug("Add Recognizer3DCommand: '%s'", rec_cmd->label.c_str());
}


void ObjectRecognizerManager::overwriteRecognizer3DCommand(const cdl::WorkingMemoryChange & _wmc)
{
	VisionData::Recognizer3DCommandPtr rec_cmd = getMemoryEntry<VisionData::Recognizer3DCommand>(_wmc.address);
	if (rec_cmd->confidence>0.12)
	{
		println("Visual detection of %s %f", rec_cmd->label.c_str(), rec_cmd->confidence);
		cout << rec_cmd->label.c_str() << rec_cmd->confidence << endl;
		// Current place
		int placeId = getCurrentPlace();

		// Check if we have it
		bool exists = false;
		for (ObjectPlaceList::iterator it = _objectInPlace.begin();
				it!=_objectInPlace.end(); ++it)
		{
			if ( (it->first == rec_cmd->label) && (it->second == placeId))
			{
				exists = true;
				break;
			}
		}

		if (!exists)
		{
			println("Found new object %s!", rec_cmd->label.c_str());
			_objectInPlace.push_back(pair<string,int>(rec_cmd->label, placeId));
			addNewObject(rec_cmd->label, placeId, _curRoomId);
		}
	}
}


// -------------------------------------------------------
int ObjectRecognizerManager::getCurrentPlace()
{
	SpatialData::PlacePtr pl= _placeInterfacePrx->getCurrentPlace();
	if (pl)
		return pl->id;
	else
		return -1;
}


void ObjectRecognizerManager::addNewObject(std::string label, int curPlaceId, int curRoomId)
{
	SpatialProperties::ObjectPlacePropertyPtr opp = new SpatialProperties::ObjectPlaceProperty();
	opp->inferred = false;
	opp->category = label;
	opp->placeId = curPlaceId;
	opp->relation = SpatialData::INROOM;
	opp->supportObjectCategory = "";
	opp->supportObjectId = "";
	SpatialProperties::DiscreteProbabilityDistributionPtr pd = new SpatialProperties::DiscreteProbabilityDistribution();
	SpatialProperties::ValueProbabilityPair vap1;
		SpatialProperties::BinaryValuePtr bv1 = new SpatialProperties::BinaryValue();
		bv1->value = true;
		vap1.probability = 1.0;
		vap1.value = bv1;
	pd->data.push_back(vap1);
	SpatialProperties::ValueProbabilityPair vap2;
		SpatialProperties::BinaryValuePtr bv2 = new SpatialProperties::BinaryValue();
		bv2->value = false;
		vap2.probability = 0.0;
		vap2.value = bv2;
	pd->data.push_back(vap2);

	opp->distribution = pd;
		SpatialProperties::BinaryValuePtr bv3 = new SpatialProperties::BinaryValue();
		bv3->value = true;
	opp->mapValue = bv3;
	opp->mapValueReliable = true;

	addToWorkingMemory<SpatialProperties::ObjectPlaceProperty>(
			newDataID(), "spatial.sa", opp);



    // Check if we have such result already
	bool exists = false;
	vector< boost::shared_ptr< cast::CASTData< SpatialData::ObjectSearchResult > > > results;
	getWorkingMemoryEntries<SpatialData::ObjectSearchResult> ("spatial.sa", 0 , results);
	for (unsigned int i=0; i<results.size(); ++i)
	{
		if ((results[i]->getData()->searchedObjectCategory == label) &&
			(results[i]->getData()->supportObjectId.empty()) &&
			(results[i]->getData()->supportObjectCategory.empty()) &&
			(results[i]->getData()->relation == SpatialData::INROOM) &&
			(results[i]->getData()->roomId == curRoomId))
		{
			results[i]->getData()->beta = 1.0;
			overwriteWorkingMemory<SpatialData::ObjectSearchResult>(results[i]->getID(), "spatial.sa", results[i]->getData());
			exists=true;
			break;
		}
	}
	if (!exists)
	{
		SpatialData::ObjectSearchResultPtr osr = new SpatialData::ObjectSearchResult();
		osr->beta = 1.0;
		osr->roomId = curRoomId;
		osr->searchedObjectCategory = label;
		osr->supportObjectCategory = "";
		osr->supportObjectId = "";
		osr->relation = SpatialData::INROOM;

		addToWorkingMemory<SpatialData::ObjectSearchResult>(
				newDataID(), "spatial.sa", osr);
	}

}


// -------------------------------------------------------
void ObjectRecognizerManager::worldStateChanged(const cast::cdl::WorkingMemoryChange & wmChange)
{
	try
	{
		ConceptualData::WorldStatePtr worldStatePtr;
		worldStatePtr = getMemoryEntry<ConceptualData::WorldState>(wmChange.address);

		if (worldStatePtr)
		{
			int curPlaceId = getCurrentPlace();
			for (size_t r=0; r<worldStatePtr->rooms.size(); ++r)
			{
				ConceptualData::ComaRoomInfo &room = worldStatePtr->rooms[r];
				for (size_t p=0; p<room.places.size(); ++p)
				{
					ConceptualData::PlaceInfo &place = room.places[p];
					if (place.placeId == curPlaceId)
					{
						_curRoomId = room.roomId;
						break;
					}
				}
			} // for
		}
	}
	catch(CASTException &e)
	{
		log("Exception while reading world state from the WM!");
	}
}






} // namespace def
