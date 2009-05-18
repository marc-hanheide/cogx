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

#include "SubarchitectureTaskManager.hpp"

using namespace std;

namespace cast {

//   SubarchitectureTaskManager::SubarchitectureTaskManager(const string &_id)
//     : //InspectableComponent(_id), 
//       //CASTProcessingComponent(_id),
//     WorkingMemoryAttachedComponent(_id),
//     WorkingMemoryReaderProcess(_id), 
//     WorkingMemoryWriterProcess(_id),
//     m_pGoalResultOutput(NULL),
//     m_tpRunnable(NULL),
//     m_trRunnable(NULL),
//     m_compRunnable(NULL),
//     m_tregRunnable(NULL)
//   {

//     m_queueBehaviour = cdl::QUEUE;
//     m_pProposalCondition = new omni_condition(&m_proposalLock);
//     m_pCompletionCondition = new omni_condition(&m_completionLock);  
    
//   }

//   SubarchitectureTaskManager::~SubarchitectureTaskManager() {

//     QueuedDataThread<FrameworkLocalData<cast::cdl::InformationProcessingTask>*>::cleanup(m_tpRunnable);
//     QueuedDataThread<FrameworkLocalData<std::string>*>::cleanup(m_trRunnable);
//     QueuedDataThread<FrameworkLocalData<cast::cdl::TaskResult>*>::cleanup(m_compRunnable);
//     QueuedDataThread<FrameworkLocalData<cast::cdl::TaskDescriptionList>*>::cleanup(m_tregRunnable);

//     m_proposalLock.lock();
//     m_pProposalCondition->broadcast();
//     m_proposalLock.unlock();

//     m_completionLock.lock();
//     m_pCompletionCondition->broadcast();
//     m_completionLock.unlock();

//     delete m_pProposalCondition;
//     delete m_pCompletionCondition;
//   }

//   void SubarchitectureTaskManager::stop() {

//     WorkingMemoryReaderProcess::stop();


//     //stop threads
//     if(m_tpRunnable) {
//       m_tpRunnable->stop();
//     }

//     if(m_trRunnable) {
//       m_trRunnable->stop();
//     }

//     if(m_compRunnable) {
//       m_compRunnable->stop();
//     }

//     if(m_tregRunnable) {
//       m_tregRunnable->stop();
//     }

//     //wake up waiting threads
//     m_proposalLock.lock();
//     m_pProposalCondition->broadcast();
//     m_proposalLock.unlock();

//     //wake up waiting threads
//     m_completionLock.lock();
//     m_pCompletionCondition->broadcast();
//     m_completionLock.unlock();

//   }

//   //connection methods

//   void SubarchitectureTaskManager::setPushConnector(const string & _connectionID, 
// 						    PushConnectorOut<cdl::TaskManagementResult> * _pOut) {
//     m_pGoalResultOutput = _pOut;
//   }
  
//   //input methods


// //   void forwardTaskProposal(void * _arg) {

// //     SubarchitectureTaskManager::TaskProposalClass *pTPC = 
// //       (SubarchitectureTaskManager::TaskProposalClass*) _arg;

// //     //locl the underlying class thread
// //     pTPC->m_pSTM->lockProcess();

// //     //pass down the source and data of the proposed goal
// //     pTPC->m_pSTM->taskProposed(pTPC->m_pData->getSource(),
// // 			       pTPC->m_pData->data());

// //     //unlock the thread
// //     pTPC->m_pSTM->unlockProcess();
  
// //     //delete input data whilst protecting proposal
// //     pTPC->m_pData->data() = NULL;

// //     delete pTPC->m_pData;

// //     pTPC->m_pData = NULL;
  
// //     //wake up waiting threads
// //     pTPC->m_pSTM->m_proposalLock.lock();
// //     pTPC->m_pSTM->m_pProposalCondition->broadcast();
// //     pTPC->m_pSTM->m_proposalLock.unlock();
  
// //     //delete helper object
// //     delete pTPC;
  
// //   }
  


//   void SubarchitectureTaskManager::receivePushData(FrameworkLocalData<cdl::InformationProcessingTask> *_pData) {

//     if(m_status == STATUS_RUN) {

//       if(m_tpRunnable == NULL) {
// 	m_tpRunnable = new TaskProposalThread(this);
// 	m_tpRunnable->start();
//       }

//       m_tpRunnable->queue(_pData);

//     }
//     else {
//       delete _pData;
//     }

//   }

// //   void forwardTaskRetraction(void * _arg) {

// //     SubarchitectureTaskManager::TaskRetractionClass *pTRC = 
// //       (SubarchitectureTaskManager::TaskRetractionClass*) _arg;

  
// //   }


//   void SubarchitectureTaskManager::receivePushData(FrameworkLocalData<string> *_pData) {
//     //println("received proposal");
  
//     if(m_status == STATUS_RUN) {

//       if(m_trRunnable == NULL) {
// 	m_trRunnable = new TaskRetractionThread(this);
// 	m_trRunnable->start();
//       }

//       m_trRunnable->queue(_pData);
//     }
//     else {
//       delete _pData;
//     }

//   }

//   void SubarchitectureTaskManager::receivePushData(FrameworkLocalData<cdl::TaskDescriptionList> *_pData) {

//     if(m_status == STATUS_RUN) {
//       if(m_tregRunnable == NULL) {
// 	m_tregRunnable = new TaskRegistrationThread(this);
// 	m_tregRunnable->start();
//       }

//       m_tregRunnable->queue(_pData);
//     }
//     else {
//       delete _pData;
//     }
//   }


//   string SubarchitectureTaskManager::toString(const cdl::TaskOutcome &_outcome) {

//     switch(_outcome) {
//     case cdl::PROCESSING_COMPLETE:  
//       return "complete";
//     case cdl::PROCESSING_COMPLETE_FAILURE:  
//       return "failure";
//     case cdl::PROCESSING_COMPLETE_SUCCESS:  
//       return "success";
//     case cdl::PROCESSING_INCOMPLETE:  
//       return "incomplete";
//     }
//     return "unknown value";

//   }

//   void SubarchitectureTaskManager::receivePushData(FrameworkLocalData<cdl::TaskResult> *_pData) {

//     if(m_status == STATUS_RUN) {

//       if(m_compRunnable == NULL) {
// 	m_compRunnable = new TaskCompletionThread(this);
// 	m_compRunnable->start();
//       }

//       m_compRunnable->queue(_pData);
//     }
//     else {
//       delete _pData;
//     }
//   }

//   void SubarchitectureTaskManager::sendTaskDecision(const cdl::InformationProcessingTask & _ipg,
// 						    const cdl::TaskManagementDecision & _decision) {


//     //create the result structure
//     cdl::TaskManagementResult * pTMR = new cdl::TaskManagementResult();
//     pTMR->m_id = CORBA::string_dup(_ipg.m_id);
//     pTMR->m_decision = _decision;
  
//     //wrap it in local data structure using memory created above
//     FrameworkLocalData<cdl::TaskManagementResult> *pFLD = 
//       new FrameworkLocalData<cdl::TaskManagementResult>(getProcessIdentifier(),
// 							pTMR);

//     if(m_pGoalResultOutput) {
//       //        println("sending push...");
//       m_pGoalResultOutput->push(pFLD);
//     }
//     else {
//       println("SubarchitectureTaskManager ERROR: result output connection not set");
//     }
//   }


//   void SubarchitectureTaskManager::workingMemoryChanged(const cdl::WorkingMemoryChangeList & _wmcl) {
//     //do nothing as default
//   }


//   void SubarchitectureTaskManager::waitForProposals() {
//     m_proposalLock.lock();
//     m_pProposalCondition->wait();
//     m_proposalLock.unlock();
//   }

//   void SubarchitectureTaskManager::waitForCompletions() {
//     m_completionLock.lock();
//     m_pCompletionCondition->wait();
//     m_completionLock.unlock();
//   }

}
