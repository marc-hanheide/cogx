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

#include "Director.h"
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
    return new Director(_id);
  }
}


Director::Director(const string &_id) : 
  WorkingMemoryAttachedComponent(_id),
  PrivilegedManagedProcess(_id) {
}

Director::~Director() {
}

void Director::start() {
  
  PrivilegedManagedProcess::start();

  addChangeFilter(createLocalTypeFilter<DirectorAction>(cdl::ADD),
		  new MemberFunctionChangeReceiver<Director>(this,
							     &Director::takeAction));  

  
}

void Director::takeAction(const cdl::WorkingMemoryChange & _actionChange) {
  // get the action from wm
  try {
   
    shared_ptr< const CASTData<DirectorAction> > pActionData =
      getWorkingMemoryEntry<DirectorAction>(_actionChange.m_address);

    shared_ptr<const DirectorAction> action = 
      pActionData->getData();


    switch (action->m_action) {
    case comedyarch::autogen::AskTheAudience:
      askTheAudience(action->m_address);
      break;
    case comedyarch::autogen::CheckTheReaction:
      checkTheReaction(action->m_address);
      break;
    default:
      log("Unknown action type");
      break;
    }
    
    // now delete the action itself from our sa wm    
    deleteFromWorkingMemory(string(_actionChange.m_address.m_id));
  }
  catch (CASTException & e) {
    println(e.what());
  }
  
}


void Director::askTheAudience(const cdl::WorkingMemoryAddress & _jokeAddress) {

  //assert(existsOnWorkingMemory(_jokeAddress));
        
  println("someone told a joke! ");

  // get the finished masterpiece
  shared_ptr< const CASTData<Joke> > jokeData =
    getWorkingMemoryEntry<Joke>(_jokeAddress);
  shared_ptr<const Joke> joke = jokeData->getData();



  println("let's see what the audience thinks of...");
  println("Q: " + string(joke->m_setup));
  println("A: " + string(joke->m_punchline));

  // write it to the audience subarchitecture
  addToWorkingMemory(newDataID(), "audience.subarch",
		     ///important to copy it, as it may be deleted
		     new Joke(*joke));


  // and erase it from the memory of the poor jokers
  deleteFromWorkingMemory(_jokeAddress);
}

void Director::checkTheReaction(const cdl::WorkingMemoryAddress & _reactionAddress) {


  // get the finished masterpiece
  shared_ptr< const CASTData<string> > reactionData =
    getWorkingMemoryEntry<string>(_reactionAddress);
  shared_ptr<const string> reaction = reactionData->getData();

  println("and the audience says...");
  println(*reaction);

  // and erase it from the memory of the poor jokers
  deleteFromWorkingMemory(string(_reactionAddress.m_id),
			  string(_reactionAddress.m_subarchitecture));


  if(m_testing) {
    if(*reaction == "YAY!") {
      std::exit(cdl::testing::CAST_TEST_PASS);
    }
    else {
      std::exit(cdl::testing::CAST_TEST_FAIL);
    }
  }

}

void Director::configure(map<string,string> & _config) {

  PrivilegedManagedProcess::configure(_config);

  map<string,string>::const_iterator i = _config.find("--test");

  if(i != _config.end()) {
    m_testing = true;
  }
  else {
    m_testing = false;
  }
}
