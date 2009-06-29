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

#include "ManagedComponent.hpp"
#include <sstream>
#include <exception>

using namespace std;

namespace cast {

  ManagedComponent::ManagedComponent() :
    m_notifications(0),
    //    m_pProposedTasks(new IPTaskMap()),
    m_taskCounter(0)// ,
//     m_responseRunnable(NULL)
  {
    
//     m_pTaskNotificationCondition 
//       = new omni_condition(&m_taskNotificationMutex);

  }

  ManagedComponent::~ManagedComponent() {


//     QueuedDataThread<cdl::TaskManagementResult>::cleanup(m_responseRunnable);

//     m_taskNotificationMutex.lock();
//     //dummy value to prevent assert on exit
//     m_notifications = 100;
//     m_pTaskNotificationCondition->broadcast();
//     m_taskNotificationMutex.unlock();
//     delete m_pTaskNotificationCondition;

//     //TODO might need to empty it too...
//     delete m_pProposedTasks;
  }

  void ManagedComponent::stopInternal() {
    //FrameworkComponent::

//     //stop threads
//     if(m_responseRunnable != NULL) {
//       m_responseRunnable->stop();
//     }


//     //release sleeping threads
//     m_taskNotificationMutex.lock();
//     m_pTaskNotificationCondition->broadcast();
//     m_taskNotificationMutex.unlock();
    WorkingMemoryReaderComponent::stopInternal();
    
  }

  void ManagedComponent::waitForNotifications() {

    //    m_taskNotificationMutex.lock();    
//     //if there are no outstanding notifications, then block
//     if(m_notifications == 0) {
//       //println("blocking for new");
//       m_pTaskNotificationCondition->wait();
//       //the count will have been incremented in the same action as the broadcast
//       m_notifications--;
//       assert(m_notifications >= 0);

//     }
//     //else decrement the coutn
//     else {
//       m_notifications--;
//       assert(m_notifications >= 0);
//     }
//     m_taskNotificationMutex.unlock();	            
    

  }




//   void ManagedComponent::receivePushData(FrameworkLocalData<cdl::TaskManagementResult> *_pData) {


//     //protect access to data structures
//     m_gdpMutex.lock();


//     try {


//       if(m_status == STATUS_RUN) {

// 	//check that the decision is for this component
// 	string taskID(_pData->data()->m_id);
// 	IPTaskMap::iterator i = m_pProposedTasks->find(taskID);
	
// 	//printfln("received decision!!, %s", taskID.c_str());

// 	//if the task is one we proposed
// 	if(i != m_pProposedTasks->end()) {
	  
// 	  if(m_responseRunnable == NULL) { 	    
// 	    m_responseRunnable = new GMResponseHandler(this);
// 	    m_responseRunnable->start(;
// 	  }
	
// 	  //copy ;)
// 	  //	  println("queued!!");
// 	  m_responseRunnable->queue(*_pData->data());
    
// 	} else {
// 	  //not for us! ... 
// 	  //println("shit!!");
// 	}
//       }


//     }
//     catch (exception& e)
//       {
// 	println("exception details follow...");
// 	cerr << e.what() << endl;
//       }
//     catch (...) {
//       println("exception details follow...");
//       cerr << "default exception" << endl;
//     }

//     m_gdpMutex.unlock();
  
//     // data needs to be deleted whatever happens
//     delete _pData;


//   }


  string ManagedComponent::newTaskID() {
    std::ostringstream o;
    o << "IPG:";
    o << getComponentID();
    o << ":";
    o << m_taskCounter;

    m_taskCounter++;

    return o.str();    
  }


//   void ManagedComponent::proposeInformationComponentingTask(const string &_taskID, const string &_taskName) {

//     if(m_pComponentingResultToGoalManager) {

//       logTaskProposed(_taskID,_taskName);

//       //create the goal structure
//       cdl::InformationComponentingTask *pIPT 
// 	= new cdl::InformationComponentingTask();
//       pIPT->m_id = CORBA::string_dup(_taskID.c_str());
//       pIPT->m_taskName =  CORBA::string_dup(_taskName.c_str());

//       //store it as proposed
//       (*m_pProposedTasks)[_taskID] = pIPT;

//       //COPY (not point) data into local data object... as it will
//       //get deleted by the sending component
//       FrameworkLocalData<cdl::InformationComponentingTask> *pFLD 
// 	= new FrameworkLocalData<cdl::InformationComponentingTask>(getComponentID(),*pIPT);
    
//       // push task to s-a goal manager
//       m_pProposalInputToTaskManager->push(pFLD);
      
//     }
//     else {
//       throw(SubarchitectureComponentException(__HERE__,"Push connection not to sa goal manager for proposal not set."));
//     }

//   }

//   void ManagedComponent::retractInformationComponentingTask(const string &_taskID) {

//     if(m_pRetractionInputToTaskManager) {

//       FrameworkLocalData<std::string> *pFLD 
// 	= new FrameworkLocalData<std::string>(getComponentID(),new string(_taskID));
    
//       // push task to s-a goal manager
//       m_pRetractionInputToTaskManager->push(pFLD);
      
//     }
//     else {
//       throw(SubarchitectureComponentException(__HERE__,"Push connection not to sa goal manager for retraction not set."));
//     }

//   }

//   void ManagedComponent::taskComplete(const string &_goalID, 
// 				    const cdl::TaskOutcome &_outcome) {

//     if (m_pComponentingResultToGoalManager) {

//       logTaskComplete(_goalID);

//       IPTaskMap::iterator i = m_pProposedTasks->find(_goalID);
//       if(i != m_pProposedTasks->end()) {
// 	m_pProposedTasks->erase(i);
//       }


//       cdl::TaskResult *pTR = new cdl::TaskResult();
//       pTR->m_id = CORBA::string_dup(_goalID.c_str());
//       pTR->m_outcome = _outcome;

//       //use our memory for the data, but the FLD class will delete when
//       //sending is complete
//       FrameworkLocalData<cdl::TaskResult> *pFLD 
// 	= new FrameworkLocalData<cdl::TaskResult>(getComponentID(),
// 						  pTR);
//       m_pComponentingResultToGoalManager->push(pFLD);
//     }
//     else {
//       throw(SubarchitectureComponentException(__HERE__,"Goal componenting result connection to goal manager has not been set up."));
//     }
//   }

//   void ManagedComponent::registerTaskDescriptions(cdl::TaskDescriptionList * _pTaskList) {

//     if (m_pTaskDescriptionRegistration) {
    
//       FrameworkLocalData<cdl::TaskDescriptionList> *pFLD 
// 	= new FrameworkLocalData<cdl::TaskDescriptionList>(getComponentID(),
// 							   _pTaskList);
//       m_pTaskDescriptionRegistration->push(pFLD);
//     }
//     else {
//       throw(SubarchitectureComponentException(__HERE__,"Task registration connection to goal manager has not been set up."));
//     }

//   }

//   void 
//   ManagedComponent::GMResponseHandler::nextInQueue(cdl::TaskManagementResult & _data) {
    
//     if(_data.m_decision != cdl::GOAL_WAITING) {
      
//       std::string taskID(_data.m_id);
//       m_pGDP->lockComponent();
	  
//       try {

// 	if(_data.m_decision == cdl::GOAL_ADOPTED) {
// 	  //m_pGDP->logTaskAdopted(taskID);
// 	  m_pGDP->taskAdopted(taskID);	    
// 	}
// 	else if(_data.m_decision == cdl::GOAL_REJECTED) {
// 	  m_pGDP->taskRejected(taskID);	    
// 	}	  
//       }	  
//       catch(const DoesNotExistOnWMException &e) {
// 	m_pGDP->println("ManagedComponent::taskAdopted DoesNotExistOnWMException caught");
// 	m_pGDP->println("The task involved in this error was: " + taskID);	  		
// 	m_pGDP->println("The address involved in this error was: " + 
// 			string(e.address().m_id) + " in " + 
// 			string(e.address().m_subarchitecture));	  		
// 	m_pGDP->println(e.what());
// 	std::abort();
//       }
//       catch(const CASTException &e) {
// 	m_pGDP->println("ManagedComponent::taskAdopted DoesNotExistOnWMException caught");
// 	m_pGDP->println("The task involved in this error was: " + taskID);	  		
// 	m_pGDP->println(e.what());
// 	std::abort();
//       }
//       catch(const BALTException &e) {
// 	m_pGDP->println("ManagedComponent::taskAdopted BALTException caught");
// 	m_pGDP->println("The task involved in this error was: " + taskID);	  		
// 	m_pGDP->println(e.what());
// 	std::abort();
//       }
//       catch(const CORBA::BAD_PARAM &e) {
// 	m_pGDP->println("ManagedComponent::taskAdopted CORBA::BAD_PARAM caught");		
// 	m_pGDP->println("The task involved in this error was: " + taskID);	  		
// 	m_pGDP->println(e._name());
// 	std::abort();
//       }
//       catch(const CORBA::Exception &e) {
// 	m_pGDP->println("ManagedComponent::taskAdopted CORBA::Exception caught");
// 	m_pGDP->println("The task involved in this error was: " + taskID);	  		
// 	m_pGDP->println(e._name());
// 	std::abort();
//       }
//       catch(const std::exception &e) {
// 	m_pGDP->println("ManagedComponent::taskAdopted std::exception caught");
// 	m_pGDP->println("The task involved in this error was: " + taskID);	  		
// 	m_pGDP->println(e.what());
// 	std::abort();
//       }
//       catch(...) {
// 	m_pGDP->println("ManagedComponent::taskAdopted unknown exception caught");
// 	m_pGDP->println("The task involved in this error was: " + taskID);	  		
// 	std::abort();
//       }
	    
	    
//       m_pGDP->unlockComponent();
	 
//       //wake waiting threads
//       m_pGDP->m_taskNotificationMutex.lock();
//       //increment notification count
//       m_pGDP->m_notifications++;    
//       m_pGDP->m_pTaskNotificationCondition->broadcast();
//       m_pGDP->m_taskNotificationMutex.unlock();	    	    
//     }
//   }



} //namespace cast 
