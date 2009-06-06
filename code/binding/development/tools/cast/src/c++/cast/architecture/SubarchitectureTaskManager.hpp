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

#ifndef CAST_SUBARCHITECTURE_TASK_MANAGER_H_
#define CAST_SUBARCHITECTURE_TASK_MANAGER_H_

#include <cast/slice/CDL.hpp>
#include <cast/architecture/WorkingMemoryReaderComponent.hpp>

namespace cast {


/**
 * Abstract class for a subarchitecture task manager. Currently
 * empty. If you want to use task management use the java components and contact Nick.
 *
 * @author nah
 */
class SubarchitectureTaskManager: 
  public WorkingMemoryReaderComponent, 
  public virtual interfaces::TaskManager {

 public:

  virtual void proposeTask(const ::std::string&, const ::std::string&, const ::std::string&, const ::Ice::Current&) {
    println("Task management is not (yet) implemented in C++ in CAST 2");
  }

  virtual void retractTask(const ::std::string&, const ::std::string&, const ::Ice::Current&) {
    println("Task management is not (yet) implemented in C++ in CAST 2");
  }
  
  virtual void taskComplete(const ::std::string&, const ::std::string&, ::cast::cdl::TaskOutcome, const ::Ice::Current&) {
    println("Task management is not (yet) implemented in C++ in CAST 2");  
  }

  virtual void addManagedComponent(const ::cast::interfaces::ManagedComponentPrx&, const ::Ice::Current&) {
    //println("Task management is not (yet) implemented in C++ in CAST 2");
  }



//   /**
//    * Constructor.
//    * 
//    * @param _id
//    *            Unique component id.
//    */
//   SubarchitectureTaskManager(const std::string &_id);

//   /**
//    * Empty destructor.
//    */
//   virtual ~SubarchitectureTaskManager();


//   virtual void stop();

//   /**
//    * Empty workingMemoryChanged method. Does nothing.
//    * 
//    * @see cast.architecture.abstr.WorkingMemoryReaderProcess#workingMemoryChanged(cast.corba.autogen.CAST.WorkingMemoryChange[])
//    */
//   virtual void workingMemoryChanged(const cdl::WorkingMemoryChangeList & _wmcl);


//   /**
//    * Set the push connector for sending management decisions. Only a
//    * single connector is stored, so it is assumed to be broadcast.
//    * 
//    * @param _connectionID
//    *            The id of the connector.
//    * @param _pOut
//    *            The connector.
//    */
//   virtual void setPushConnector(const std::string & _connectionID, 
// 				PushConnectorOut<cdl::TaskManagementResult> * _pOut);
  


//   /**
//    * Receive push data describing a proposed task.
//    * 
//    * @param _pData The task proposed for processing.
//    */
//   virtual void receivePushData(FrameworkLocalData<cdl::InformationProcessingTask> *_pData);


  
//   /**
//    * Receive data describing a task retraction.
//    * 
//    * @param _pData The task id for retraction.
//    */
//   virtual void receivePushData(FrameworkLocalData<std::string> *_pData);


//   /**
//    * Receive data describing the completion of a task.
//    * 
//    * @param _pData The task completion status
//    */
//   virtual void receivePushData(FrameworkLocalData<cdl::TaskResult> *_pData);

//   static std::string toString(const cdl::TaskOutcome &_outcome);

//   /**
//    * Receive push information describing tasks that can be registered
//    * with the task manager.
//    * 
//    * @param _pData Descriptions of the tasks the component can
//    * perform.
//    */
//   virtual void receivePushData(FrameworkLocalData<cdl::TaskDescriptionList> *_pData);


// //    /**
// //    * Friend function that is called to forward a task proposal event
// //    * to the subclass. This is currently coded as a separate function
// //    * to support experimenting with approaches to threading.
// //    *
// //    * @param _arg Void pointer to a TaskProposalClass object. 
// //    */
// //   friend void forwardTaskProposal(void * _arg);


// //    /**
// //    * Friend function that is called to forward a task retraction event
// //    * to the subclass. This is currently coded as a separate function
// //    * to support experimenting with approaches to threading.
// //    *
// //    * @param _arg Void pointer to a TaskRetractionClass object. 
// //    */
// //   friend void forwardTaskRetraction(void * _arg);


// //    /**
// //    * Friend function that is called to forward a task completion event
// //    * to the subclass. This is currently coded as a separate function
// //    * to support experimenting with approaches to threading.
// //    *
// //    * @param _arg Void pointer to a TaskResultClass object. 
// //    */
// //   friend void forwardTaskResult(void * _arg);

//    /**
//    * Friend function that is called to forward a task registration
//    * event to the subclass. This is currently coded as a separate
//    * function to support experimenting with approaches to threading.
//    *
//    * @param _arg Void pointer to a TaskRegistrationClass object. 
//    */
//   friend void forwardTaskRegistration(void * _arg);
  


// protected:



//   /**
//    * Receive a task registration event.
//    * 
//    * @param _src
//    *            The component registering the task.
//    * @param _pTaskList
//    *            The task descriptions.
//    */
//   virtual void taskRegistered(const std::string & _src, 
// 			      const cdl::TaskDescriptionList & _taskList) = 0;

//   /**
//    * Receive a task completion event.
//    * 
//    * @param _src
//    *            The component that has completed the task.
//    * @param _pResult
//    *            The completion result.
//    */
//   virtual void taskCompleted(const std::string & _src, 
// 			     const cdl::TaskResult & _result) = 0;


//   /**
//    * Receive a task proposal event.
//    * 
//    * @param _src
//    *            The component proposing the task.
//    * @param _pRask
//    *            The task instance information.
//    */
//   virtual void taskProposed(const std::string & _src, 
// 			    const cdl::InformationProcessingTask & _task) = 0;


//   /**
//    * Receive a task retraction event.
//    * 
//    * @param _src
//    *            The component retracting the task.
//    * @param _taskID
//    *            The task id.
//    */
//   virtual void taskRetracted(const std::string & _src, 
// 			     const std::string  & _taskID) = 0;
  
 

//   /**
//    * Send a task management decision to component in the same
//    * subarchitecture.
//    * 
//    * @param _ipg
//    *            The task that the decision is about.
//    * @param _decision
//    *            The decision that has been reached about the task.
//    */
//   virtual void sendTaskDecision(const cdl::InformationProcessingTask & _ipg,
// 				const cdl::TaskManagementDecision & _decision);


//   /**
//    * Wait until new task proposal information is received. This method
//    * causes the current thread to wait until the next exit of either
//    * taskProposed or taskRetracted.
//    */
//   void waitForProposals();
  
//   omni_mutex m_proposalLock;
//   omni_condition * m_pProposalCondition;

//   /**
//    * Wait until new task completion information is received. This
//    * method causes the current thread to wait until the next exit of
//    * taskCompleted;
//    */
//   void waitForCompletions();

//   omni_mutex m_completionLock;
//   omni_condition * m_pCompletionCondition;

//   /// The connector used to send results.
//   PushConnectorOut<cdl::TaskManagementResult> * m_pGoalResultOutput;

// private:
  
//   //thread to handle proposals
  

//   /**
//    * Struct used internally for passing data to helper functions.
//    */
//   class TaskProposalThread : public QueuedDataThread<FrameworkLocalData<cdl::InformationProcessingTask> *>  {
//   public:
//     TaskProposalThread(SubarchitectureTaskManager *_pSTM) :
//       QueuedDataThread<FrameworkLocalData<cdl::InformationProcessingTask> *>(),
//       m_pSTM(_pSTM) {
//     }
    
//   private:
//     virtual ~TaskProposalThread(){};

//   protected:
//     virtual void nextInQueue(FrameworkLocalData<cdl::InformationProcessingTask> * & _data) {
//       //locl the underlying class thread
//       m_pSTM->lockProcess();
//       //pass down the source and data of the proposed goal
//       m_pSTM->taskProposed(_data->getSource(),
// 			   _data->getData());
//       //unlock the thread
//       m_pSTM->unlockProcess();
      
//       delete _data;      
//       _data = NULL;
      
//       //wake up waiting threads
//       m_pSTM->m_proposalLock.lock();
//       m_pSTM->m_pProposalCondition->broadcast();
//       m_pSTM->m_proposalLock.unlock();
      
//     }
    
//   private:
//     SubarchitectureTaskManager * m_pSTM;
    
//   };


//   //has to be a pointer (AFAIK) as the destructor is private and the
//   //thread is cleaned up by an omniorb call
//   TaskProposalThread * m_tpRunnable;

//   /**
//    * Struct used internally for passing data to helper functions.
//    */
//   class TaskRetractionThread : public QueuedDataThread<FrameworkLocalData<std::string> *>  {
//   public:
//     TaskRetractionThread(SubarchitectureTaskManager *_pSTM) :
//       QueuedDataThread<FrameworkLocalData<std::string> *>(),
//       m_pSTM(_pSTM) {
//     }
    
//   private:
//     virtual ~TaskRetractionThread(){};
    
//   protected:
//     virtual void nextInQueue(FrameworkLocalData<std::string> * & _data) {
      
//       //locl the underlying class thread
//       m_pSTM->lockProcess();
//       //pass down the source and data of the proposed goal
//       m_pSTM->taskRetracted(_data->getSource(),
// 			    _data->getData());
//       //unlock the thread
//       m_pSTM->unlockProcess();
  
//       delete _data;
//       _data = NULL;
      
//       //wake up waiting threads
//       m_pSTM->m_proposalLock.lock();
//       m_pSTM->m_pProposalCondition->broadcast();
//       m_pSTM->m_proposalLock.unlock();
//     }
    
//   private:
//     SubarchitectureTaskManager * m_pSTM;    
//   };
  
  
//   //has to be a pointer (AFAIK) as the destructor is private and the
//   //thread is cleaned up by an omniorb call
//   TaskRetractionThread * m_trRunnable;
  













//   /**
//    * Struct used internally for passing data to helper functions.
//    */  
//   class TaskCompletionThread  : public QueuedDataThread<FrameworkLocalData<cdl::TaskResult> *>  {
//   public:
//     TaskCompletionThread(SubarchitectureTaskManager *_pSTM) :
//       QueuedDataThread<FrameworkLocalData<cdl::TaskResult> *>(),
//       m_pSTM(_pSTM) {
//     }
    
//   private:
//     virtual ~TaskCompletionThread(){};
    
//   protected:
//     virtual void nextInQueue(FrameworkLocalData<cdl::TaskResult> * & _data) { 
//     //locl the underlying class thread
//     m_pSTM->lockProcess();

//     //pass down the source and data of the proposed goal
//     m_pSTM->taskCompleted(_data->getSource(),
// 			  _data->getData());

//     //unlock the thread
//     m_pSTM->unlockProcess();
  
//     delete _data;
//     _data = NULL;

//     //wake up waiting threads
//     m_pSTM->m_completionLock.lock();
//     m_pSTM->m_pCompletionCondition->broadcast();
//     m_pSTM->m_completionLock.unlock();
//    }
    
//   private:
//     SubarchitectureTaskManager * m_pSTM;    
//   };
  
  
//   //has to be a pointer (AFAIK) as the destructor is private and the
//   //thread is cleaned up by an omniorb call
//   TaskCompletionThread * m_compRunnable;
 
//   /**
//    * Struct used internally for passing data to helper functions.
//    */
//   class TaskRegistrationThread  : public QueuedDataThread<FrameworkLocalData<cdl::TaskDescriptionList> *>  {
//   public:
//     TaskRegistrationThread(SubarchitectureTaskManager *_pSTM) :
//       QueuedDataThread<FrameworkLocalData<cdl::TaskDescriptionList> *>(),
//       m_pSTM(_pSTM) {      
//     }
    
//   private:
//     virtual ~TaskRegistrationThread(){};
    
//   protected:
//     virtual void nextInQueue(FrameworkLocalData<cdl::TaskDescriptionList> * & _data) { 
//       //locl the underlying class thread
//       m_pSTM->lockProcess();      
//       //pass down the source and data of the proposed goal
//       m_pSTM->taskRegistered(_data->getSource(),
// 			     _data->getData());
//       //unlock the thread
//       m_pSTM->unlockProcess();
      
//       delete _data;
//       _data = NULL;     
//     }
    
//   private:
//     SubarchitectureTaskManager * m_pSTM;    
//   };
  
  
//   //has to be a pointer (AFAIK) as the destructor is private and the
//   //thread is cleaned up by an omniorb call
//   TaskRegistrationThread * m_tregRunnable;

  
};

} //namespace cast

#endif
