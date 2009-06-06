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

#include "TestWorkingMemory.hpp"


using namespace std;

/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {

  FrameworkProcess* newComponent(const string &_id) {
    return new cast::TestWorkingMemory(_id);
  }

//   void __attribute__ ((constructor)) my_init(void);

//   void my_init(void)
//   {
//     printf("I'm in _init!\n");
//   }

 
}



namespace cast {


  TestWorkingMemory::TestWorkingMemory(const string & _id) : 
    SubarchitectureWorkingMemory(_id) {
    
    //cout<<"TestWorkingMemory::TestWorkingMemory(const string & _id)"<<endl;
    
    // determines whether this wm should broadcast to oher
    // sub-architectures
    setSendXarchChangeNotifications(true);    
  }
  
  void TestWorkingMemory::receivePushData(FrameworkLocalData<cdl::WorkingMemoryChange> *_pData) {
    if(!m_allowChanges) {
      //println("incorrectly received wm change: " + CASTUtils.toString(_data));
      println("incorrectly received wm change: ");
      std::exit(cdl::testing::CAST_TEST_FAIL);
    }    
    SubarchitectureWorkingMemory::receivePushData(_pData);
  }

  void TestWorkingMemory::configure(std::map<std::string,std::string>& _config) {
    SubarchitectureWorkingMemory::configure(_config);
    
    std::map<std::string,std::string>::iterator i = _config.find("--allow-changes");
    if(i != _config.end()) {
      if(i->second == "false") {
	m_allowChanges = false;
      }
      else {
	m_allowChanges = true;
      }
      log("allowing changes: %d",m_allowChanges);
    }
    else {
      m_allowChanges = true;
    }

  }

}


