/*
 * CAST - The CoSy Architecture Schema Toolkit
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

#include "TestTaskManager.hpp"



using namespace std;

/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new cast::TestTaskManager(_id);
  }
}

namespace cast {

  std::string TestTaskManager::TEST_TASK = "test-task";

  TestTaskManager::TestTaskManager(const string &_id)
    : //InspectableComponent(_id), 
    //CASTProcessingComponent(_id),
    WorkingMemoryAttachedComponent(_id),  
    AlwaysPositiveTaskManager(_id) {
 
  }

  TestTaskManager::~TestTaskManager() {

  }

  void TestTaskManager::taskCompleted(const std::string & _src, 
				      const cdl::TaskResult & _result) {

    
    AlwaysPositiveTaskManager::taskCompleted(_src, _result);
    
    string taskID(_result.m_id);
    log("TestTaskManager.taskCompleted(): %s %s", _src.c_str(),taskID.c_str());

    int number = taskNumber(taskID);
    
    StringMap<int>::map::iterator i = m_counts.find(_src);

    if(i != m_counts.end()) {
      if(number == (m_counts[_src] + 1)) {
	m_counts[_src] = number;
      }
      else {
	::exit(cdl::testing::CAST_TEST_FAIL);
      }
    }
    else {
      m_counts[_src] = number;
    }
  }

} //namespace cast
