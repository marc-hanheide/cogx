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

#include "AlwaysPositiveTaskManager.hpp"


using namespace std;
using namespace cast::cdl;

/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  cast::CASTComponentPtr
  newComponent() {
    return new cast::AlwaysPositiveTaskManager();
  }
}

// namespace cast {

//   AlwaysPositiveTaskManager::AlwaysPositiveTaskManager(const string &_id)
//     : //InspectableComponent(_id), 
//     //CASTProcessingComponent(_id),
//     WorkingMemoryAttachedComponent(_id),  
//     SubarchitectureTaskManager(_id) {
  
//     receiveNoChanges();
//   }

//   AlwaysPositiveTaskManager::~AlwaysPositiveTaskManager() {
//   }

//   void AlwaysPositiveTaskManager::processGoalQueue() {

//     while(!m_taskQueue.empty()) {

//       InformationProcessingTask task = m_taskQueue.front();
//       m_taskQueue.pop();
    
//       //send decision
//       sendTaskDecision(task,cdl::GOAL_ADOPTED);    

    
//     }
  
//   }

//   void AlwaysPositiveTaskManager::runComponent() {

//     while(m_status == STATUS_RUN) {
//       lockProcess();
//       processGoalQueue();
//       unlockProcess();
//       //sleepProcess(1);
//       waitForProposals();
//     }

//   }

//   void AlwaysPositiveTaskManager::taskCompleted(const string & _src, 
// 						const cdl::TaskResult  & _result) {  

//   }

//   void AlwaysPositiveTaskManager::taskProposed(const string & _src, 
// 					       const cdl::InformationProcessingTask & _task) {

//     log("task proposed: %s",string(_task.m_id).c_str());
//     m_taskQueue.push(_task);
//   }


//   void AlwaysPositiveTaskManager::taskRetracted(const string & _src, 
// 						const string  & _taskID)  {

//   }



//   void AlwaysPositiveTaskManager::taskRegistered(const string & _src, 
// 						 const cdl::TaskDescriptionList & _taskList) {

// //     for(unsigned int i = 0; i < _pTaskList.length(); i++) {
// //       string taskName(_taskList[i].m_taskName);
// //       //println("task registered: " + taskName);
// //     }

//   }


//} //namespace cast
