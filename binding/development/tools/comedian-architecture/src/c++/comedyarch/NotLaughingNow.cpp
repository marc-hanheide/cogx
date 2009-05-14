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

#include "NotLaughingNow.h"
#include "ComedyGoals.h"
#include "idl/ComedyEssentials.hh"

#include <cast/architecture/ChangeFilterFactory.hpp>
using namespace comedyarch::autogen;

using namespace std;
using namespace boost;
using namespace cast;
using namespace comedyarch::autogen;

/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new NotLaughingNow(_id);
  }
}


NotLaughingNow::NotLaughingNow(const string &_id) : 
  WorkingMemoryAttachedComponent(_id),
  ManagedProcess(_id) {

}

NotLaughingNow::~NotLaughingNow() {
}

void NotLaughingNow::start() {
  
  ManagedProcess::start();

  MemberFunctionChangeReceiver<NotLaughingNow> * pReceiver = 
    new MemberFunctionChangeReceiver<NotLaughingNow>(this,
					       &NotLaughingNow::newJokeAdded);
  
  if(m_bLogOutput) {//lazy!
    addChangeFilter(createLocalTypeFilter<Joke>(cdl::ADD),
		    pReceiver);  
  }


}

void NotLaughingNow::newJokeAdded(const cdl::WorkingMemoryChange & _wmc) {
  println("new joke added");
  shared_ptr< const CASTData<Joke> > pJokeData = getWorkingMemoryEntry<Joke>(_wmc.m_address);
  
  //and the actual data
  shared_ptr< const Joke > pJoke = pJokeData->getData();
  
  //reset to test
  pJokeData.reset();

  //pJoke->m_setup = CORBA::string_dup("doesn't work!!!");


  cout<<endl;
  cout<<"other proc data ptr: count: "<<pJoke.use_count()<<endl;
  cout<<"other proc data ptr: setup: "<<pJoke->m_setup<<endl;
  cout<<"other proc data ptr: punchline: "<<pJoke->m_punchline<<endl;
  cout<<endl;


  sleepProcess(3000);

  cout<<endl;
  cout<<"other proc data ptr: count: "<<pJoke.use_count()<<endl;
  cout<<"other proc data ptr: setup: "<<pJoke->m_setup<<endl;
  cout<<"other proc data ptr: punchline: "<<pJoke->m_punchline<<endl;
  cout<<endl;


  sleepProcess(10000);

  cout<<endl;
  cout<<"other proc data ptr: count: "<<pJoke.use_count()<<endl;
  cout<<"other proc data ptr: setup: "<<pJoke->m_setup<<endl;
  cout<<"other proc data ptr: punchline: "<<pJoke->m_punchline<<endl;
  cout<<endl;
  



}


void NotLaughingNow::taskAdopted(const string &_taskID) {
    

}

void NotLaughingNow::taskRejected(const string &_taskID) {
}

void NotLaughingNow::runComponent() {

  if(!m_bLogOutput) {
  
  // make up a joke
  Joke *pJK = new Joke();
  pJK->m_setup = CORBA::string_dup("Are you confused?");
  pJK->m_punchline = CORBA::string_dup("Yes.");


  //store in in a shared pointer 
//   m_pJokeData = 
//     shared_ptr< const CASTData<Joke> >(new CASTData<Joke>(newDataID(),
// 						    ComedyOntology::JOKE_TYPE,
// 						    //the data object now
// 						    //owns this memory
// 						    pJK));

//   cout<<endl;
//   cout<<"data ptr: count: "<<m_pJokeData.use_count()<<endl;
//   cout<<"data ptr: setup: "<<m_pJoke->m_setup<<endl;
//   cout<<"data ptr: punchline: "<<m_pJoke->m_punchline<<endl;
//   cout<<endl;


  string id = newDataID();
  addToWorkingMemory(id, pJK);

  //must sleep to allow data to be stored
  sleepProcess(100);

  //get the data wrapper
  shared_ptr< const CASTData<Joke> > pJokeData = getWorkingMemoryEntry<Joke>(id);

  //and the actual data
  shared_ptr< const Joke > pJoke = pJokeData->getData();
  

  //reset to test
  pJokeData.reset();

  cout<<endl;
  cout<<"data ptr: count: "<<pJoke.use_count()<<endl;
  cout<<"data ptr: setup: "<<pJoke->m_setup<<endl;
  cout<<"data ptr: punchline: "<<pJoke->m_punchline<<endl;
  cout<<endl;

  sleepProcess(3000);

  cout<<endl;
  cout<<"data ptr: count: "<<pJoke.use_count()<<endl;
  cout<<"data ptr: setup: "<<pJoke->m_setup<<endl;
  cout<<"data ptr: punchline: "<<pJoke->m_punchline<<endl;
  cout<<endl;



  deleteFromWorkingMemory(id);
  //must sleep to allow data to be stored

  sleepProcess(3000);

  cout<<endl;
  cout<<"data ptr: count: "<<pJoke.use_count()<<endl;
  cout<<"data ptr: setup: "<<pJoke->m_setup<<endl;
  cout<<"data ptr: punchline: "<<pJoke->m_punchline<<endl;
  cout<<endl;

  }


//   while(m_status == STATUS_RUN) {

//     //waitForNotifications();
//     //log("post wait");

    
//     // must check that we're still running after sleep
//     if(m_status == STATUS_RUN) {

//       //prevent external access
//       //cout<<"locking... ";
//       lockProcess();
//       //cout<<"locked"<<endl;

//       //cout<<"unlocking... ";
//       unlockProcess();
//       //cout<<"unlocked"<<endl;

//       sleepProcess(1000);
//     }

//   }
}

