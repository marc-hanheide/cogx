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

#ifndef CAST_ALWAYS_POSITIVE_TASK_MANAGER_H_
#define CAST_ALWAYS_POSITIVE_TASK_MANAGER_H_

#include <cast/architecture/SubarchitectureTaskManager.hpp>


namespace cast {
  
//   typedef std::queue < cdl::InformationProcessingTask> TaskQueue;

  class AlwaysPositiveTaskManager: public SubarchitectureTaskManager {

//   public:
//     AlwaysPositiveTaskManager(const std::string & _id);
//     ~AlwaysPositiveTaskManager();

//   protected:

//     virtual void runComponent();
//     virtual void processGoalQueue();

//     virtual void taskCompleted(const std::string & _src, 
// 			       const cdl::TaskResult & _result);

//     virtual void taskProposed(const std::string & _src, 
// 			      const cdl::InformationProcessingTask & _task);

//     virtual void taskRetracted(const std::string & _src, 
// 			       const std::string  & _taskID);
  
//     virtual void taskRegistered(const std::string & _src, 
// 				const cdl::TaskDescriptionList & _taskList);


//     TaskQueue m_taskQueue;

  };

} //namespace cast

#endif
