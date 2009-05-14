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

#include "AudienceWorkingMemory.h"


using namespace std;
using namespace cast;

AudienceWorkingMemory::AudienceWorkingMemory(const string & _id) : 
  //InspectableComponent(_id),
  SubarchitectureWorkingMemory(_id) {
  
  // determines whether this wm should broadcast to oher
  // sub-architectures
  setSendXarchChangeNotifications(true);

}



void AudienceWorkingMemory::receivePushData(FrameworkLocalData<CASTWorkingMemoryEntry> *_pData) {


  //bool talk = (_pData->getSource() ==  "audience.member");
  bool talk = false;
  

  if(talk) {
    println("local wme received");
    println("having a sleep before doing anything");
    sleepProcess(10000);
    println("woken up");
  }

  SubarchitectureWorkingMemory::receivePushData(_pData);

  if(talk) {
    println("done stuff");
  }


}

void AudienceWorkingMemory::receivePushData(FrameworkLocalData<cdl::WorkingMemoryEntry> *_pData) {


  //bool talk = (_pData->getSource() ==  "audience.member");
  bool talk = false;
  

  if(talk) {
    println("local wme received");
    println("having a sleep before doing anything");
    //    sleepProcess(10000);
    ::sleep(10);
    println("woken up");
  }

  SubarchitectureWorkingMemory::receivePushData(_pData);

  if(talk) {
    println("done stuff");
  }


}

/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new AudienceWorkingMemory(_id);
  }
}

