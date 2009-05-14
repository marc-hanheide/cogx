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

#include "FunnyMan.h"
#include "ComedyGoals.h"
#include "idl/ComedyEssentials.hh"


/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new FunnyMan(_id);
  }
}


FunnyMan::FunnyMan(const string &_id) : 
  WorkingMemoryAttachedComponent(_id),
  ManagedProcess(_id) {

  //  m_pCollectedIDs = new vector<string>();
  m_pProposedProcessing = new TypedDataMap();
  m_pSetups = new TypedDataVector();

 
}

FunnyMan::~FunnyMan() {
  delete m_pProposedProcessing;
  delete m_pSetups;
}

void FunnyMan::start() {
  
  ManagedProcess::start();


  MemberFunctionChangeReceiver<FunnyMan> * pReceiver = 
    new MemberFunctionChangeReceiver<FunnyMan>(this,
					       &FunnyMan::newJokeAdded);
    
  //add this receiver to listen for changes
  addChangeFilter(//listen for joke types
		  //that are added
		  //that are local to this subarchitecture
		  createLocalTypeFilter<Joke>(cdl::ADD),
		  //the receiver object
		  pReceiver);  



}

void FunnyMan::newJokeAdded(const CAST::WorkingMemoryChange & _wmc) {
     // get the id of the changed entry
  string id(_wmc.m_address.m_id);

  // get the data from working memory
  CASTData<comedyarch::autogen::Joke> * pJokeData 
    = getWorkingMemoryEntry<comedyarch::autogen::Joke>(id);
  
  //now get at a reference to the actual joke
  const comedyarch::autogen::Joke jk = pJokeData->getData();
  
  string punchline(jk.m_punchline);
  
  // now see if we can provide a punchline
  if (punchline == "") {
    
    // in this case we need to propose a task to do
    // some processsing
    
    // get a new id for the task
    string taskID = newTaskID();
    //println(jk.m_setup);
    
    // store the data we want to process for
    // later... we could just store the id in
    // working memory if we wanted to, I guess it
    // depends on storage/transport tradeoffs
    (*m_pProposedProcessing)[taskID] = pJokeData;
    
    // then ask for permission
    proposeInformationProcessingTask(taskID,
				     ComedyGoals::ADD_PUNCHLINE_TASK);
    
    //then immediately retract task
    //retractInformationProcessingTask(taskID);
  }
  else {
    // nothing to do then :(
  }

  //println("going to sleep");
  //>sleep(5);
  //println("waking up");
}




void FunnyMan::taskAdopted(const string &_taskID) {
    
  TypedDataMap::iterator i = m_pProposedProcessing->find(_taskID);

  // if we have stored this goal earlier
  if (i != m_pProposedProcessing->end()) {
    // we could call quip(data) directly, but let's queue the
    // data instead... it's always good to build some suspense
    m_pSetups->push_back(i->second);
    //and take it out of the prosed list
    m_pProposedProcessing->erase(i);
  }
  else {
    println("oh, this is my goal, but I have no data: "
	    + _taskID);
  }
  
  // and now we're finished, tell the goal manager that the task is
  // over successfully (assuming it is... naughty!)
  taskComplete(_taskID,CAST::PROCESSING_COMPLETE_SUCCESS);

}

void FunnyMan::taskRejected(const string &_taskID) {

  println(":(");
  TypedDataMap::iterator i = m_pProposedProcessing->find(_taskID);
  
  // if we have stored this goal earlier
  if (i != m_pProposedProcessing->end()) {
    //remove it
    m_pProposedProcessing->erase(i);
  }
}

void FunnyMan::runComponent() {
  
  CAST::TaskDescriptionList * pTDL = new CAST::TaskDescriptionList();

  pTDL->length(1);
  (*pTDL)[0].m_taskName 
    = CORBA::string_dup(ComedyGoals::ADD_PUNCHLINE_TASK.c_str());

  registerTaskDescriptions(pTDL);

  while(m_status == STATUS_RUN) {

    //waitForNotifications();
    //log("post wait");

    
    // must check that we're still running after sleep
    if(m_status == STATUS_RUN) {

      //prevent external access
      //cout<<"locking... ";
      lockProcess();
      //cout<<"locked"<<endl;

      // check (synchronised) joke queue
      TypedDataVector::iterator i = m_pSetups->begin();

      // if(m_pSetups->empty()) {
      //log("setups empty!!");
      //}

      // see what's in there
      while (i != m_pSetups->end()) {
	
	// quick, tell the punchline
	quip(*i);

	// erase and move to the next point in the list
	i = m_pSetups->erase(i);

      }

      //sleepProcess(10000);

      //cout<<"sleeping... "<<endl;
      //sleep(20);
      //cout<<"awake..."<<endl;

      //cout<<"unlocking... ";
      unlockProcess();
      //cout<<"unlocked"<<endl;

      //sleepProcess(100);
    }

  }
}

string FunnyMan::generatePunchline(const string &_setup) {
  return "A stick!";
}

void FunnyMan::quip(CASTData<comedyarch::autogen::Joke> *_pData) {
  
  //create a new copy of the data so we can change it
  comedyarch::autogen::Joke *pJK = new comedyarch::autogen::Joke(_pData->getData());


  string setup(pJK->m_setup);
  // work out a smarty-pants punchline
  string punchline = generatePunchline(setup);

  // time it right
  println("*cough*");
  pJK->m_punchline = CORBA::string_dup(punchline.c_str());

  // now write this back into working memory, this will manage the
  // memory for us
  overwriteWorkingMemory(_pData->getID(), pJK);
  //delete data holder, this will clean up all other data
  delete _pData;
  
}
  
