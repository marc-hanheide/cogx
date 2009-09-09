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

#ifndef CAST_TEST_TASK_MANAGER_H_
#define CAST_TEST_TASK_MANAGER_H_

#include <cast/testing/AbstractTester.hpp>

#include <sstream>
#include <vector>
#include <cstdlib>

namespace cast {
  
  class TestTaskManager: public AlwaysPositiveTaskManager {
    
  public:

    static std::string TEST_TASK;

    TestTaskManager(const std::string & _id);
    ~TestTaskManager();


    static std::string numberedTask(const std::string &_src, 
				    const int & _taskNumber) {
      std::ostringstream out;
      out<<_src<<"-"<<TEST_TASK<<":"<<_taskNumber;
      return out.str();
    }


    static int  taskNumber(const std::string &_taskID) {
      
      std::vector<std::string> tokens; 
      SubarchitectureWorkingMemoryProtocol::tokenizeString(_taskID, tokens, ":");
      return std::atoi(tokens.back().c_str());
    }



  protected:
    
  /**
   * Receive a task completion event.
   * 
   * @param _src
   *            The component that has completed the task.
   * @param _pResult
   *            The completion result.
   */
  virtual void taskCompleted(const std::string & _src, 
			     const cdl::TaskResult & _result);

  private:
    StringMap<int>::map m_counts;

  };

} //namespace cast

#endif
