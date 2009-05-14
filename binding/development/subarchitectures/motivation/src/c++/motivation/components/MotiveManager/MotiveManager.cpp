#include "MotiveManager.hpp"

#include <cast/architecture/ChangeFilterFactory.hpp>
#include <planning/util/PlanningUtils.hpp>
#include <planning/idl/PlanningData.hh>
#include <motivation/idl/MotivationData.hh>

using namespace cast;
using namespace boost;
using namespace motivation::idl;

MotiveManager::MotiveManager(const string& _id) :
  WorkingMemoryAttachedComponent(_id),
  ManagedProcess(_id),
  m_currentMotiveID(""),
  m_resultsReceiver(NULL) {
}

void
MotiveManager::start() {
  ManagedProcess::start();

  addChangeFilter(createLocalTypeFilter<Motive>(cdl::ADD),
		  new MemberFunctionChangeReceiver<MotiveManager>(this,
								  &MotiveManager::motiveAdded));
}

void 
MotiveManager::configure(map<string,string> & _config) {

  ManagedProcess::configure(_config);

//   setBindingSubarchID(getBindingSA());
//   m_statusCache.setSubarchitectureID(getBindingSA());

//   map<string,string>::const_iterator i = _config.find("--monitor");
//   if(i != _config.end()) {
//     vector<string> monitorList;
//     SubarchitectureWorkingMemoryProtocol::tokenizeString(i->second,
// 							 monitorList,
// 							 ",");

//     for(vector<string>::const_iterator j = monitorList.begin();
// 	j != monitorList.end(); ++j) {
//       log("monitoring: %s", j->c_str());
//       m_monitoredSAs.insert(*j);
//     }
//   }


}

void 
MotiveManager::motiveAdded(const cdl::WorkingMemoryChange& _wmc) {
  debug("MotiveManager::motiveAdded");

  //load motive
  string motiveID(_wmc.m_address.m_id);
  //it's ours now
  lockEntry(motiveID,cdl::LOCKED_O);

  shared_ptr<const Motive> motive(getWorkingMemoryEntry<Motive>(motiveID)->getData());

  //keep things simple for the time being... we only have one type of motive
  assert(strcmp(motive->m_content.m_type, typeName<AchieveGoalMotive>().c_str()) == 0);

  updateMotive(motiveID, *motive, MOTIVE_QUEUED);

  m_motiveQueue.push(motiveID);
}

  /**
   * Overwrite a motive, updating its status and success fields.
   */
void 
MotiveManager::updateMotive(const std::string & _motiveID,
			    const motivation::idl::Motive & _motive,
			    const motivation::idl::MotiveStatus & _status,
			    const cast::cdl::TriBool & _succeeded) {
  //copy to overwrite
  Motive * copy = new Motive(_motive);
  copy->m_status = _status;
  copy->m_succeeded = _succeeded;
  overwriteWorkingMemory(_motiveID,copy);
}


void
MotiveManager::runComponent() {
  
  while(m_status == STATUS_RUN) {
    waitForChanges();
    lockProcess();
    //we there are things to do and we're not doing anything else
    if(!m_motiveQueue.empty() && m_currentMotiveID.empty()) {
      string nextMotiveID(m_motiveQueue.front());
      m_motiveQueue.pop();
      log("next motive: " + nextMotiveID);
      processMotive(nextMotiveID);
    }    
    unlockProcess();
  }

}


void 
MotiveManager::processMotive(const std::string & _motiveID) {
  log("MotiveManager::processMotive: " + _motiveID);
  assert(m_currentMotiveID.empty());
  try {
    shared_ptr<const Motive> motive(getWorkingMemoryEntry<Motive>(_motiveID)->getData());
    m_currentMotiveID = _motiveID;	
    updateMotive(_motiveID, *motive, MOTIVE_QUEUED);
  
    //load the planning content
    assert(strcmp(motive->m_content.m_type, typeName<AchieveGoalMotive>().c_str()) == 0);

    //get the planning goal motive
    shared_ptr<const AchieveGoalMotive> agm(getWorkingMemoryEntry<AchieveGoalMotive>(motive->m_content.m_address)->getData());

    //create a filter to listen for the results of planning
    string pprID(newDataID());
    m_resultsReceiver = new PlanningResultsReceiver(*this);
    addChangeFilter(createIDFilter(pprID, cdl::OVERWRITE), m_resultsReceiver);
    addToWorkingMemory(pprID, new PlanningProcessRequest(agm->m_ppr));    
  }
  catch(const WMException & e) {
    println("Error processing motive %s, aborting motive: %s", _motiveID.c_str(), e.what());
    m_currentMotiveID = "";
  }  
}

bool 
MotiveManager::planningRequestOverwritten(const planning::autogen::PlanningProcessRequest & _ppr) {
  if(_ppr.m_status != planning::autogen::COMPLETE) {
    log("MotiveManager::planningRequestOverwritten: planning not yet complete");
    return false;
  }
  
  log("MotiveManager::planningRequestOverwritten: planning motive complete");
  motiveProcessingComplete(_ppr.m_succeeded);
  return true;
}

void 
MotiveManager::motiveProcessingComplete(const cast::cdl::TriBool & _succeeded) {
  assert(!m_currentMotiveID.empty());
  shared_ptr<const Motive> motive(getWorkingMemoryEntry<Motive>(m_currentMotiveID)->getData());
  updateMotive(m_currentMotiveID, *motive, MOTIVE_COMPLETE, _succeeded);
  unlockEntry(m_currentMotiveID);
  m_currentMotiveID = "";
}

/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new MotiveManager(_id);
  }
}


//  LocalWords:  getData
