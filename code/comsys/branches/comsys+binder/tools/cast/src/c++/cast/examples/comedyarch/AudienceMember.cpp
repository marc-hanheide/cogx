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

#include "AudienceMember.hpp"

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
    return new AudienceMember();
  }
}



void AudienceMember::start() {

  addChangeFilter(createGlobalTypeFilter<TwoLiner>(cdl::ADD),
  		  new MemberFunctionChangeReceiver<AudienceMember>(this,
								   &AudienceMember::newTwoLinerAdded));
  
  //   addChangeFilter(createGlobalTypeFilter<TwoLiner>(cdl::OVERWRITE),
  //   		  new MemberFunctionChangeReceiver<AudienceMember>(this,
  // 								   &AudienceMember::newTwoLinerAdded));
}

void AudienceMember::newTwoLinerAdded(const cdl::WorkingMemoryChange & _wmc) {

  if(_wmc.address.subarchitecture == getSubarchitectureID()) {
    
    TwoLinerPtr joke  = getMemoryEntry<TwoLiner>(_wmc.address);

    //this is what I think
    ReactionPtr pReaction(new Reaction(generateAudienceReaction(*joke)));

    addToWorkingMemory(newDataID(), 
		       pReaction);
  }
  else {
    try { 
      log("I'll have a cheeky peek into subarch: " + _wmc.address.subarchitecture);


      //this will trip up the director
      //lockEntry(_wmc.address, cdl::LOCKEDOD);
      
      TwoLinerPtr joke  = getMemoryEntry<TwoLiner>(_wmc.address);
      
      log("I heard this joke:");
      log(joke->setup);
      log(joke->punchline);
    }
    catch(DoesNotExistOnWMException & e) {
      log("OK, maybe I won't");
    }
    
  }


}


void AudienceMember::runComponent() {
  // do nothing here
}

string AudienceMember::generateAudienceReaction(const TwoLiner & _joke) {
  return m_defaultReaction;
}


void AudienceMember::configure(const map<string,string> & _config) {


  map<string,string>::const_iterator i = _config.find("--reaction");

  if(i != _config.end()) {
    m_defaultReaction = i->second;
  }
  else {
    m_defaultReaction = "BOO!";
  }
}
