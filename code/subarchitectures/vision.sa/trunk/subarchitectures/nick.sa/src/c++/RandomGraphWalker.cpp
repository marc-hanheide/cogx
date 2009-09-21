#include "RandomGraphWalker.hpp"

#include <cast/architecture/ChangeFilterFactory.hpp>
#include <autogen/NavData.hpp>
#include <autogen/TTS.hpp>

#include <Ice/Ice.h>
#include <algorithm>

using namespace std;
using namespace cast;
using namespace cast::cdl;
using namespace NavData;


/**
 * The function called to create a new instance of our component.
 */
extern "C" {
  cast::CASTComponentPtr 
  newComponent() {
    return new RandomGraphWalker();
  }
}

RandomGraphWalker::RandomGraphWalker() : m_loop(true),
					 m_ptzHost("localhost")
{}



void 
RandomGraphWalker::graphAdded(const WorkingMemoryChange & _wmc) {
  NavGraphPtr graph(getMemoryEntry<NavGraph>(_wmc.address));
  log("graph added with %d nodes", graph->fNodes.size());

  //store nav sa 
  m_navSA = _wmc.address.subarchitecture;

  //just store
  m_nodesToVisit = graph->fNodes;
  m_storedNodes = graph->fNodes;
  
  //then shuffle the order in which the robot should visit them
  std::random_shuffle(m_nodesToVisit.begin(), m_nodesToVisit.end());

  //then start doing stuff
  visitNextNode();
  
}

void 
RandomGraphWalker::nodeAdded(const WorkingMemoryChange & _wmc) {
}

void 
RandomGraphWalker::connectToPTZServer() {
  Ice::Identity id;
  id.name = "PTZServer";
  id.category = "PTZServer";

  ostringstream ptzServerAddr;
  ptzServerAddr << getCommunicator()->identityToString(id)
		  << ":default -h " << m_ptzHost << " -p " << cdl::CPPSERVERPORT;

  Ice::ObjectPrx base = getCommunicator()->stringToProxy(ptzServerAddr.str());

  m_ptzServer = ptz::PTZInterfacePrx::uncheckedCast(base);
  if(!m_ptzServer) {
    println("Failed to initialise connection to PTZ server with args: %s", ptzServerAddr.str().c_str());
  }

}

void 
RandomGraphWalker::start() {
  addChangeFilter(createGlobalTypeFilter<NavData::NavGraph>(cdl::ADD),
		  new MemberFunctionChangeReceiver<RandomGraphWalker>(this,&RandomGraphWalker::graphAdded));
}


void
RandomGraphWalker::runComponent() {

  //just to test
  connectToPTZServer();    
  if(m_ptzServer) {
    ptz::PTZReading ptzr(m_ptzServer->getPose());
    println("PTZ %f, %f, %f",ptzr.pose.pan, ptzr.pose.tilt, ptzr.pose.zoom);
  }

}

void 
RandomGraphWalker::configure(const map<string,string> & _config) {
  map<string,string>::const_iterator it;

  if((it = _config.find("--ptz-host")) != _config.end()) {
    m_ptzHost = it->second;
  }
  log("using m_ptzHost=%s", m_ptzHost.c_str());
}

void
RandomGraphWalker::say(const std::string & _message) {
  TTS::SpeakPtr speak(new TTS::Speak(_message));
  addToWorkingMemory(newDataID(), speak);
}

void
RandomGraphWalker::visitNextNode() {
  //could've been another change or an empty graph, so check again
  if(m_nodesToVisit.empty()) {
    if(m_loop) {
      m_nodesToVisit = m_storedNodes;
      std::random_shuffle(m_nodesToVisit.begin(), m_nodesToVisit.end());
    }
    else {
      println("graph empty");
      say("i have been everywhere. stopping now");
      return;
    }
  }

  //get the next node to visit
  vector<FNodePtr>::iterator first(m_nodesToVisit.begin());
  FNodePtr next = *first;
  m_nodesToVisit.erase(first);
  

  println("next node: %d",next->nodeId);
  
  char buf[50];
  if(next->nodeId % 4 == 0) {
    snprintf(buf, 50, "now I'm off to node number %d", (int) next->nodeId);
  }
  else if(next->nodeId % 3 == 0) {
    snprintf(buf, 50, "going to node number %d", (int) next->nodeId);
  }
  else if(next->nodeId % 2 == 0) {
    snprintf(buf, 50, "this time I will visit node number %d", (int) next->nodeId);
  }
  else {
    snprintf(buf, 50, "o k, now for node number %d", (int) next->nodeId);
  }

  say(buf);

  
  //create a new command
  NavCommandPtr cmd = new NavCommand();
  cmd->cmd = NavData::GOTONODE;
  cmd->prio = NavData::NORMAL;
  cmd->destId.push_back(next->nodeId);

  string id(newDataID());

  //setup listener for command result
//   addChangeFilter(createAddressFilter(id, m_navSA, cdl::OVERWRITE),
// 		  new MemberFunctionChangeReceiver<RandomGraphWalker>(this, 
  //			      &RandomGraphWalker::commandOverwritten));  
  addChangeFilter(createAddressFilter(id, m_navSA, cdl::OVERWRITE),
		  new CommandListener(*this));
  
  addToWorkingMemory(id, m_navSA, cmd);
}



bool 
RandomGraphWalker::commandOverwritten(const WorkingMemoryChange & _wmc) {
    println("Node added");
  NavCommandPtr cmd(getMemoryEntry<NavCommand>(_wmc.address));
  
  if(cmd->comp == PENDING || cmd->comp == INPROGRESS) {
    println("ignoring command in progress");
    return false;
  }

  //otherwise command is over and we just don't care what happened
  if(cmd->comp == SUCCEEDED) {
    println("Yay! We got to the node we wanted to");
  }
  else {
    println("Boo! I didn't get to my goal.");
  }

  //clean up last command
  deleteFromWorkingMemory(_wmc.address);
  
  visitNextNode();
  return true;

}


