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

#ifndef CAST_GOAL_DRIVEN_COMPONENT_H_
#define CAST_GOAL_DRIVEN_COMPONENT_H_

#include <cast/architecture/WorkingMemoryReaderComponent.hpp>
#include <cast/architecture/WorkingMemoryWriterComponent.hpp>

namespace cast {

  //typedef StringMap<cdl::InformationComponentingTask *>::map IPTaskMap;

  /**
   * An abstract class to represent a component in a subarchitecture that
   * can read and write to a working memory, and has its operations
   * controlled by a task manager.
   * 
   * @author nah
   */
  class ManagedComponent : 
    public WorkingMemoryReaderComponent,
    public virtual interfaces::ManagedComponent  {
  
  private:

    /**
     * Count of notifications that are outstanding in the "wait" cycle.
     */
    int m_notifications;

    /**
     * Task manager
     */
    interfaces::TaskManagerPrx m_taskManager;

  public:
  
    /**
     * Constructor.
     * 
     * @param _id Unique component id.
     */
    ManagedComponent();
  
    /**
     * Destructor, cleans up proposed tasks.
     */
    virtual ~ManagedComponent();
 



    virtual 
    void 
    taskDecision(const ::std::string&, ::cast::cdl::TaskManagementDecision, const ::Ice::Current&) {
      println("Task management is not (yet) implemented in C++ in CAST 2");
    }

    
    
    virtual 
    void 
    setTaskManager(const interfaces::TaskManagerPrx & _tm, 
		   const Ice::Current & _current) {
      assert(!m_taskManager);
      m_taskManager = _tm;
    }
    


  protected:

    void stopInternal();

//     /**
//      * Propose the componenting of a task.
//      * 
//      * @param _taskID
//      *            The id used to refer to this task.
//      * @param _taskName
//      *            The name of the task.
//      */
//     virtual void proposeInformationComponentingTask(const std::string &_taskID, const std::string &_taskName);

//     /**
//      * Retract the proposal of componenting a task.
//      * 
//      * @param _taskID
//      *            The id of the task.
//      */
//     virtual void retractInformationComponentingTask(const std::string &_taskID);
 
  
//     /**
//      * Receive a signal that an information componenting task has been
//      * accepted.
//      * 
//      * @param _taskID
//      */
//     virtual void taskAdopted(const std::string &_taskID) = 0;

//     /**
//      * Receive a signal that an information componenting goal has been
//      * rejected.
//      * 
//      * @param _taskID
//      */
//     virtual void taskRejected(const std::string &_taskID) = 0;
 
//     /**
//      * Signal the outcome of goal componenting to the sub-architecture
//      * goal manager.
//      * 
//      * @param _taskID
//      *            The task id.
//      * @param _outcome
//      *            The task outcome.
//      */
//     virtual void taskComplete(const std::string &_goalID, 
// 			      const cdl::TaskOutcome &_outcome);

//     /**
//      * Register a list of tasks that this component can perform. Input
//      * memory is consumed.
//      * 
//      * @param _pTaskList
//      *            A sequence of task descriptions.
//      */
//     virtual void registerTaskDescriptions(cdl::TaskDescriptionList * _pTaskList);
  

    /**
     * Generate a new unique task id.
     * 
     * @return A unique task id.
     */
    virtual std::string newTaskID();

//     /**
//      * stl map of proposed tasks
//      */
//     IPTaskMap * m_pProposedTasks;
  
    /**
     * Counter for generating task ids.
     */
    int m_taskCounter;

//     /**
//      * Mutex for controlling thread access to GDP data, not to derived
//      * class data
//      */
//     omni_mutex m_gdpMutex;


    /**
     * Put the component to sleep until a new task notification is
     * received. This method will sleep until either the taskAdopted
     * or taskRejected functions next <b>exit</b>, unless these
     * methods have exited more times than this method has been
     * called. In this case the method does not block, as the
     * assumption is that the component has to component these outstanding
     * notifications first. This latter behaviour was added to fixed
     * bug #31.
     *
     */
    void waitForNotifications();

//     omni_mutex m_taskNotificationMutex;
//     omni_condition * m_pTaskNotificationCondition;



//     /**
//      * Struct used internally for passing data to threads
//      */
//     class GMResponseHandler  : public QueuedDataThread<cdl::TaskManagementResult>  {

//     public:

//       GMResponseHandler(ManagedComponent * _pGDP) :
// 	QueuedDataThread<cdl::TaskManagementResult>(),
// 	m_pGDP(_pGDP) {
//       }
      
//     private:

//       virtual ~GMResponseHandler(){};

//     protected:

//       virtual void nextInQueue(cdl::TaskManagementResult & _data);

//     private:
//       ManagedComponent * m_pGDP; 
//     };
  
//     //has to be a pointer (AFAIK) as the destructor is private and the
//     //thread is cleaned up by an omniorb call
//     GMResponseHandler * m_responseRunnable;

  };

} //namespace cast

#endif
