/**
 * @author Andrzej Pronobis
 *
 * Definition of the conceptual::ObjectRecognizerManager class.
 */

// Conceptual.SA
#include "ObjectRecognizerManager.h"
// System
#include "VisionData.hpp"
// CAST
#include <cast/architecture/ChangeFilterFactory.hpp>


/** The function called to create a new instance of our component. */
extern "C"
{
	cast::CASTComponentPtr newComponent()
	{
		return new conceptual::ObjectRecognizerManager();
	}
}

namespace conceptual
{

using namespace std;
using namespace cast;
using namespace VisionData;

// -------------------------------------------------------
ObjectRecognizerManager::ObjectRecognizerManager()
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
	sleepComponent(1000);  // HACK: the nav visualisation might crash if we send it
	                     // object observations too soon.

	// Run component
	while(isRunning())
	{
		debug("Recognizing objects!");
		for(size_t i=0; i<m_labels.size(); i++)
		{
			addRecognizer3DCommand(m_labels[i]);
		}
		sleepComponent(2000);
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
	if (rec_cmd->confidence>0.03)
	{
		println("Found %s %f", rec_cmd->label.c_str(), rec_cmd->confidence);
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


} // namespace def
