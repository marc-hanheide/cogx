//
// = Filename
//   PlaceManager.cpp
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2009 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#include "PlaceManager.hpp"
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <Navigation/NavGraphNode.hh>

using namespace cast;
using namespace std;
using namespace boost;
using namespace spatial;

/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  cast::interfaces::CASTComponentPtr newComponent() {
    return new PlaceManager();
  }
}

PlaceManager::PlaceManager()
{
}

PlaceManager::~PlaceManager()
{
}

void 
PlaceManager::start()
{
  addChangeFilter(createLocalTypeFilter<NavData::FNode>(cdl::ADD),
		  new MemberFunctionChangeReceiver<PlaceManager>(this,
					&PlaceManager::newNavNode));
  
  addChangeFilter(createLocalTypeFilter<NavData::FNode>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<PlaceManager>(this,
					&PlaceManager::modifiedNavNode));

  addChangeFilter(createLocalTypeFilter<NavData::FNode>(cdl::DELETE),
		  new MemberFunctionChangeReceiver<PlaceManager>(this,
					&PlaceManager::deletedNavNode));
}

void 
PlaceManager::stop()
{
}

void 
PlaceManager::runComponent()
{
}

void 
PlaceManager::newNavNode(const cast::cdl::WorkingMemoryChange &objID)
{
  shared_ptr<CASTData<NavData::FNode> > oobj =
    getWorkingMemoryEntry<NavData::FNode>(objID.address);
  
  if (oobj != 0) {
    
    PlaceHolder p;
    p.m_data = new SpatialData::Place;   
    p.m_data->id = oobj->getData()->nodeId;
    p.m_data->status = SpatialData::TRUEPLACE;
    p.m_WMid = newDataID();
    log("Adding place %ld, with tag %s", p.m_data->id, p.m_WMid.c_str());
    addToWorkingMemory<SpatialData::Place>(p.m_WMid, p.m_data);

    m_Places.push_back(p);

  }

}

void 
PlaceManager::modifiedNavNode(const cast::cdl::WorkingMemoryChange &objID)
{
  shared_ptr<CASTData<NavData::FNode> > oobj =
    getWorkingMemoryEntry<NavData::FNode>(objID.address);
  
  if (oobj != 0) {

    // Look for the place in the internal vector
    for (unsigned int i = 0; i < m_Places.size(); i++) {
      if (m_Places[i].m_data->id == oobj->getData()->nodeId) {

        // Here we need to change mpore stuff, right now there is
        // nothing really that can be changed since all we have is the
        // id

        /*
        log("Modified place %ld, with tag %s", p.m_data->id, p.m_WMid.c_str());
        overwriteWorkingMemory<SpatialData::Place>(m_Places[i].m_WMid, 
                                                   m_Places[i].m_data);
        */
        return;
      }
    }
    
    // If the node is not in our 
    log("Did not find the node from before, have to assume that we did not start early enough to catch it, will treat it as new");
    newNavNode(objID);

  }
}

void 
PlaceManager::deletedNavNode(const cast::cdl::WorkingMemoryChange &objID)
{
  shared_ptr<CASTData<NavData::FNode> > oobj =
    getWorkingMemoryEntry<NavData::FNode>(objID.address);
  
  if (oobj != 0) {

    // Look for the place in the internal vector
    for (unsigned int i = 0; i < m_Places.size(); i++) {
      if (m_Places[i].m_data->id == oobj->getData()->nodeId) {

        deleteFromWorkingMemory(m_Places[i].m_WMid);

        return;
      }
    }
    
    log("WARNING: Did not find the node to delete!!!");
  }
}

