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

#include "AssistantDirector.hpp"
#include <ComedyEssentials.hpp>
#include <ChangeFilterFactory.hpp>
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
  cast::CASTComponentPtr newComponent() {
    return new AssistantDirector();
  }
}



void AssistantDirector::start() {
  
  MemberFunctionChangeReceiver<AssistantDirector> * jokeReceiver = 
    new MemberFunctionChangeReceiver<AssistantDirector>(this,
							&AssistantDirector::handleTwoLiner);
    
  addChangeFilter(createChangeFilter<TwoLiner>(cdl::OVERWRITE, "", "", "stage.subarch",
					   cdl::ALLSA),		  
		  jokeReceiver);  
  

  MemberFunctionChangeReceiver<AssistantDirector> * reactionReceiver = 
    new MemberFunctionChangeReceiver<AssistantDirector>(this,
					       &AssistantDirector::handleReaction);

  addChangeFilter(createGlobalTypeFilter<Reaction>(cdl::ADD),
		  reactionReceiver);  


}

void AssistantDirector::handleTwoLiner(const cdl::WorkingMemoryChange & _wmc) {
  try {
    // get the joke from working memory to see what
    // we can do

    assert(_wmc.operation == cdl::OVERWRITE);

    // because we've checked the ontological type
    // above, this should be safe
    TwoLinerPtr joke = getMemoryEntry<TwoLiner>(_wmc.address);


    println("is that a joke I hear?!");

    // now see if the joke is complete!
    if (!(joke->setup.empty() || joke->punchline.empty())) {
      
      println("yes...");

      // if it's tell the director to check it out
      DirectorActionPtr  action =
	new DirectorAction();
      action->action = comedyarch::autogen::AskTheAudience;
      action->address = _wmc.address;
					       
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

    DirectorActionPtr action(new DirectorAction());
    action->action = comedyarch::autogen::CheckTheReaction;
    action->address = _wmc.address;

    // by writing the action to working memory
    addToWorkingMemory(newDataID(),
		       action);

  }
  catch (CASTException & e) {
    println(e.what());
  }
}
