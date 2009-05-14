/*
 * Comedian example code to demonstrate CAST functionality.
 *
 * Copyright (C) 2006-2007 Nick Hawes
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include "AudienceMember.h"
#include "ComedyGoals.h"
#include "idl/ComedyEssentials.hh"

#include <cast/architecture/ChangeFilterFactory.hpp>
using namespace comedyarch::autogen;


using namespace std;
using namespace boost;
using namespace cast;


/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new AudienceMember(_id);
  }
}


AudienceMember::AudienceMember(const string &_id) : 
  WorkingMemoryAttachedComponent(_id),
  ManagedProcess(_id) {

  //  m_pCollectedIDs = new vector<string>();
  m_pProposedProcessing = new TypedDataMap();

  setReceiveXarchChangeNotifications(true);
 
}

AudienceMember::~AudienceMember() {
  delete m_pProposedProcessing;
}

void AudienceMember::start() {

  ManagedProcess::start();
  addChangeFilter(createGlobalTypeFilter<Joke>(cdl::ADD),
  		  new MemberFunctionChangeReceiver<AudienceMember>(this,
								   &AudienceMember::newJokeAdded));
}

void AudienceMember::newJokeAdded(const cdl::WorkingMemoryChange & _wmc) {

  string subarch(_wmc.m_address.m_subarchitecture);
 
  if(subarch == m_subarchitectureID) {

    string type(_wmc.m_type);
    
    // get the joke from working memory to evaluate
    
    // get the id of the changed entry
    string id(_wmc.m_address.m_id);
    
    // get the data from working memory
    shared_ptr< const CASTData<Joke> > pJokeData
      = getWorkingMemoryEntry<Joke>(id);
    
    
    println("(takes deep breath)");
    
    // get a new id for the task
    string taskID = newTaskID();
    
    // store the data we want to process for
    // later... we could just store the id in
    // working memory if we wanted to, I guess it
    // depends on storage/transport tradeoffs
    (*m_pProposedProcessing)[taskID] = pJokeData;
    
    // then ask for permission
    proposeInformationProcessingTask(taskID,
				     ComedyGoals::RESPOND_TO_JOKE_TASK);
    
  }
  else {
    log("not my business mate, but I'll have a cheeky peek into subarch: " + subarch);

    // get thoe id of the changed entry
    string id(_wmc.m_address.m_id);

    // get the data from working memory
    shared_ptr< const CASTData<Joke> > pData 
      = getWorkingMemoryEntry<Joke>(id,subarch);
      
    //println(pData->getData().m_setup);
    //println(pData->getData().m_punchline);
  }


}

void AudienceMember::workingMemoryChanged(const cdl::WorkingMemoryChangeList & _wmcl) {

  for(unsigned int i = 0; i < _wmcl.length(); i++) {
    newJokeAdded(_wmcl[i]);
  }
  

}


void AudienceMember::taskAdopted(const string &_taskID) {
    

  TypedDataMap::iterator i = m_pProposedProcessing->find(_taskID);

  // if we have stored this goal earlier
  if (i != m_pProposedProcessing->end()) {

    //get the data
    shared_ptr< const CASTData<Joke> > pJokeData = i->second;
    
    //well... let's see
    shared_ptr< const Joke> joke  = pJokeData->getData();

    //this is what I think
    string * pReaction = new string(generateAudienceReaction(*joke));

    //let's tell the world! ... this will handle the memory allocated
    //above
    //PrivilegedManagedProcess::addToWorkingMemory<string>(newDataID(),
    //ComedyOntology::REACTION_TYPE, pReaction);
   
    string id = newDataID();
    
    //println("before add");
    addToWorkingMemory(id, 
		       pReaction,
		       cdl::BLOCKING); //take a sync approach to guarantee it's there

    //may not be true as director deletes
    //assert(existsOnWorkingMemory(id));
    
    //take it out of the proposed list
    m_pProposedProcessing->erase(i);
  }
  else {
    println("oh, this is my goal, but I have no data: "
	    + _taskID);
  }

  log("task complete");
  
  // and now we're finished, tell the goal manager that the task is
  // over successfully (assuming it is... naughty!)
  taskComplete(_taskID,cdl::PROCESSING_COMPLETE_SUCCESS);
}

void AudienceMember::taskRejected(const string &_taskID) {

  println(":(");
  TypedDataMap::iterator i = m_pProposedProcessing->find(_taskID);

  // if we have stored this goal earlier
  if (i != m_pProposedProcessing->end()) {
    //remove it
    m_pProposedProcessing->erase(i);
  }
  
}

void AudienceMember::runComponent() {
  // do nothing here
}

string AudienceMember::generateAudienceReaction(const Joke & _joke) {
  return m_defaultReaction;
}


void AudienceMember::configure(map<string,string> & _config) {

  ManagedProcess::configure(_config);

  map<string,string>::const_iterator i = _config.find("--reaction");

  if(i != _config.end()) {
    m_defaultReaction = i->second;
  }
  else {
    m_defaultReaction = "BOO!";
  }
}
