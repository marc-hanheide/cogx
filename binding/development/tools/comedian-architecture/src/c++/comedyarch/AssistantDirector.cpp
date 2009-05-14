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

#include "AssistantDirector.h"
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
    return new AssistantDirector(_id);
  }
}


AssistantDirector::AssistantDirector(const string &_id) : 
  WorkingMemoryAttachedComponent(_id),
  ManagedProcess(_id) {

}

AssistantDirector::~AssistantDirector() {
}

void AssistantDirector::start() {
  
  ManagedProcess::start();

  MemberFunctionChangeReceiver<AssistantDirector> * jokeReceiver = 
    new MemberFunctionChangeReceiver<AssistantDirector>(this,
							&AssistantDirector::handleJoke);
    
  addChangeFilter(createChangeFilter<Joke>(cdl::OVERWRITE, "", "", "stage.subarch",
					   cdl::ALL_SA),		  
		  jokeReceiver);  
  

  MemberFunctionChangeReceiver<AssistantDirector> * reactionReceiver = 
    new MemberFunctionChangeReceiver<AssistantDirector>(this,
					       &AssistantDirector::handleReaction);

  addChangeFilter(createGlobalTypeFilter<string>(cdl::ADD),
		  reactionReceiver);  


}

void AssistantDirector::handleJoke(const cdl::WorkingMemoryChange & _wmc) {
  try {
    // get the joke from working memory to see what
    // we can do

    // get the data from working memory and store it
    // with its id
    shared_ptr< const CASTData<Joke> > jokeData =
      getWorkingMemoryEntry<Joke>(_wmc.m_address);

    // because we've checked the ontological type
    // above, this should be safe
    shared_ptr<const Joke> joke = jokeData->getData();

    // now see if the joke is complete!
    if (strcmp(joke->m_setup,"") != 0
	&& strcmp(joke->m_punchline,"") != 0) {
      
      println("is that a joke I hear?!");

      // if it's tell the director to check it out
      DirectorAction * action =
	new DirectorAction();
      action->m_action = comedyarch::autogen::AskTheAudience;
      action->m_address = _wmc.m_address;
					       
      // by writing the action to working memory
      addToWorkingMemory(newDataID(),
			 action);
      
    }
  }
  catch (CASTException & e) {
    println(e.what());
  }
}

void AssistantDirector::handleReaction(const cdl::WorkingMemoryChange & _wmc) {
  try {
    println("is that a reaction I hear?!");

    // let's propose a task goal to see what the
    // audience thinks of that shall we!
    // if it's tell the director to check it out

    DirectorAction * action = 
      new DirectorAction();
    action->m_action = comedyarch::autogen::CheckTheReaction;
    action->m_address = _wmc.m_address;

    // by writing the action to working memory
    addToWorkingMemory(newDataID(),
		       action);

  }
  catch (CASTException & e) {
    println(e.what());
  }
}
