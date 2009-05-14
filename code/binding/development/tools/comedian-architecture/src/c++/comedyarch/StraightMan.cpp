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

#include "StraightMan.h"
#include "idl/ComedyEssentials.hh"

#include <cstdlib>

using namespace std;
using namespace cast;


/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new StraightMan(_id);
  }
}


StraightMan::StraightMan(const string &_id) : 
  WorkingMemoryAttachedComponent(_id),
  UnmanagedProcess(_id) {
}

StraightMan::~StraightMan() {
}

string StraightMan::generateSetup() {
  return "What's brown and sticky?";
}

void StraightMan::runComponent() {

  while(m_status == STATUS_RUN) {

    //sleep for a random amount of time
    sleep(2);

    
    // must check that we're still running after sleep
    if(m_status == STATUS_RUN) {

      //prevent external access
      lockProcess();
      
      log("ahem");


      //FrameworkBasics::BALTTime t = BALTTimer::getBALTTime();
      //cout<<t.m_s<<endl;
      //cout<<t.m_us<<endl;


      
      // make up a joke
      comedyarch::autogen::Joke *pJK = new comedyarch::autogen::Joke();
      pJK->m_setup = CORBA::string_dup(generateSetup().c_str());
      pJK->m_punchline = CORBA::string_dup("");
      

      // and then make it available in the s-a working memory
      // ... the sending process will manager the memory for us ;)
      addToWorkingMemory(newDataID(),pJK);
      
      unlockProcess();
      
      //return;
    }

    }
}
