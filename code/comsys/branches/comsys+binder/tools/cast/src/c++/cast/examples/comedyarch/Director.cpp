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

#include "Director.hpp"

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
    return new Director();
  }
}



void Director::start() {
  

  addChangeFilter(createLocalTypeFilter<DirectorAction>(cdl::ADD),
		  new MemberFunctionChangeReceiver<Director>(this,
							     &Director::takeAction));  

  
}

void Director::takeAction(const cdl::WorkingMemoryChange & _actionChange) {
  // get the action from wm
  try {
    DirectorActionPtr action = 
      getMemoryEntry<DirectorAction>(_actionChange.address);

    switch (action->action) {
    case comedyarch::autogen::AskTheAudience:
      askTheAudience(action->address);
      break;
    case comedyarch::autogen::CheckTheReaction:
      checkTheReaction(action->address);
      break;
    default:
      log("Unknown action type");
      break;
    }
    
    // now delete the action itself from our sa wm    
    deleteFromWorkingMemory(string(_actionChange.address.id));
  }
  catch (CASTException & e) {
    println(e.what());
  }
  
}


void Director::askTheAudience(const cdl::WorkingMemoryAddress & _jokeAddress) {

  println("someone told a joke! ");

  // get the finished masterpiece
  TwoLinerPtr joke = getMemoryEntry<TwoLiner>(_jokeAddress);

  println("let's see what the audience thinks of...");
  println("Q: " + joke->setup);
  println("A: " + joke->punchline);

  // write it to the audience subarchitecture
  addToWorkingMemory(newDataID(), "audience.subarch",
		     joke);

  // and erase it from the memory of the poor jokers
  deleteFromWorkingMemory(_jokeAddress);
}

void Director::checkTheReaction(const cdl::WorkingMemoryAddress & _reactionAddress) {


  // get the finished masterpiece
  ReactionPtr reaction = getMemoryEntry<Reaction>(_reactionAddress);

  println("and the audience says...");
  println(reaction->react);

  // and erase it from the memory of the poor jokers
  deleteFromWorkingMemory(_reactionAddress);

  if(m_testing) {
    if(reaction->react == "YAY!") {
      std::exit(cdl::testing::CASTTESTPASS);
    }
    else {
      std::exit(cdl::testing::CASTTESTFAIL);
    }
  }

}

void Director::configure(const map<string,string> & _config) {

  map<string,string>::const_iterator i = _config.find("--test");

  if(i != _config.end()) {
    m_testing = true;
  }
  else {
    m_testing = false;
  }
}
