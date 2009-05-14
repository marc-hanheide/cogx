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

#ifndef CAST_GOAL_DRIVEN_PROCESS_H_
#define CAST_GOAL_DRIVEN_PROCESS_H_

#include "WorkingMemoryReaderProcess.hpp"
#include "WorkingMemoryWriterProcess.hpp"

#include <cast/core/QueuedDataThread.hpp>
#include <map>

namespace cast {

  typedef StringMap<cdl::InformationProcessingTask *>::map IPTaskMap;

  /**
   * An abstract class to represent a component in a subarchitecture that
   * can read and write to a working memory, and has its operations
   * controlled by a task manager.
   * 
   * @author nah
   */
  class ManagedProcess : 
    public WorkingMemoryWriterProcess,
    public WorkingMemoryReaderProcess, 
    public PushSender<cdl::InformationProcessingTask>,
    public PushSender<std::string>,
    public PushSender<cdl::TaskDescriptionList>,
    public PushSender<cdl::TaskResult>,
    public PushReceiver<cdl::TaskManagementResult> {
  
  private:

    /**
     * @param _taskID
     */
    void logTaskAdopted(const std::string & _taskID) {
      logEvent(cdl::ui::START,
	       getProcessIdentifier(), m_subarchitectureID,
	       std::string((*m_pProposedTasks)[_taskID]->m_taskName), _taskID);
    }
  
    void logTaskComplete(const std::string & _taskID) {
      logEvent(cdl::ui::END,
	       getProcessIdentifier(), m_subarchitectureID,
	       std::string((*m_pProposedTasks)[_taskID]->m_taskName), _taskID);
    }
  
    void logTaskProposed(const std::string & _taskID, const std::string & _name) {
      logEvent(cdl::ui::PROPOSED,
	       getProcessIdentifier(), m_subarchitectureID,
	       _name, _taskID);
    }
  


    /**
     * Count of notifications that are outstanding in the "wait" cycle.
     */
    int m_notifications;


  public:
  
    /**
     * Constructor.
     * 
     * @param _id Unique component id.
     */
    ManagedProcess(const std::string &_id);
  
    /**
     * Destructor, cleans up proposed tasks.
     */
    virtual ~ManagedProcess();
  
  
    virtual void stop();


    /**
     * Set push connector to task manager for task proposition.
     * 
     * @param _connectionID
     *            The connector id.
     * @param _pOut
     *            The connector.
     */
    virtual void setPushConnector(const std::string & _connectionID, PushConnectorOut<cdl::InformationProcessingTask> * _pOut);
 


    /**
     * Set push connector to task manager for task retraction.
     * 
     * @param _connectionID
     *            The connector id.
     * @param _pOut
     *            The connector.
     */
    virtual void setPushConnector(const std::string & _connectionID, PushConnectorOut<std::string> * _pOut);
  
    /**
     * Set push connector to task manager for task results.
     * 
     * @param _connectionID
     *            The connector id.
     * @param _pOut
     *            The connector.
     */
    virtual void setPushConnector(const std::string & _connectionID, PushConnectorOut<cdl::TaskResult> * _pOut);

    /**
     * Set push connector to task manager for task registration.
     * 
     * @param _connectionID
     *            The connector id.
     * @param _pOut
     *            The connector.
     */
    virtual void setPushConnector(const std::string & _connectionID, PushConnectorOut<cdl::TaskDescriptionList> * _pOut);


    /**
     * Receives the GoalManagementResult from the sub-architecture goal
     * manager and forwards this change to the underlying class in a new
     * thread. Also locks access to the receiving thread whilst doing
     * this.
     * 
     * @param _pData Task management result.
     */
    virtual void receivePushData(FrameworkLocalData<cdl::TaskManagementResult> *_pData);


//     /**
//      * Friend function that is called to signal task adoption to a
//      * subclass.  This is currently coded as a separate function to
//      * support experimenting with approaches to threading.
//      *
//      * @param _arg Void pointer to a ManagementResultClass object. 
//      */  
//     friend void signalAdoptionGDP(void * _arg);

//     /**
//      * Friend function that is called to signal task rejection to a
//      * subclass.  This is currently coded as a separate function to
//      * support experimenting with approaches to threading.
//      *
//      * @param _arg Void pointer to a ManagementResultClass object. 
//      */
//     friend void signalRejectionGDP(void * _arg);

  protected:


    /**
     * Propose the processing of a task.
     * 
     * @param _taskID
     *            The id used to refer to this task.
     * @param _taskName
     *            The name of the task.
     */
    virtual void proposeInformationProcessingTask(const std::string &_taskID, const std::string &_taskName);

    /**
     * Retract the proposal of processing a task.
     * 
     * @param _taskID
     *            The id of the task.
     */
    virtual void retractInformationProcessingTask(const std::string &_taskID);
 
  
    /**
     * Receive a signal that an information processing task has been
     * accepted.
     * 
     * @param _taskID
     */
    virtual void taskAdopted(const std::string &_taskID) = 0;

    /**
     * Receive a signal that an information processing goal has been
     * rejected.
     * 
     * @param _taskID
     */
    virtual void taskRejected(const std::string &_taskID) = 0;
 
    /**
     * Signal the outcome of goal processing to the sub-architecture
     * goal manager.
     * 
     * @param _taskID
     *            The task id.
     * @param _outcome
     *            The task outcome.
     */
    virtual void taskComplete(const std::string &_goalID, 
			      const cdl::TaskOutcome &_outcome);

    /**
     * Register a list of tasks that this component can perform. Input
     * memory is consumed.
     * 
     * @param _pTaskList
     *            A sequence of task descriptions.
     */
    virtual void registerTaskDescriptions(cdl::TaskDescriptionList * _pTaskList);
  

    /**
     * Generate a new unique task id.
     * 
     * @return A unique task id.
     */
    virtual std::string newTaskID();


    /*
     * BALT connectors
     */
    PushConnectorOut<cdl::InformationProcessingTask> * m_pProposalInputToTaskManager;
    PushConnectorOut<std::string> * m_pRetractionInputToTaskManager;
    PushConnectorOut<cdl::TaskResult> * m_pProcessingResultToGoalManager;
    PushConnectorOut<cdl::TaskDescriptionList> * m_pTaskDescriptionRegistration;

    /**
     * stl map of proposed tasks
     */
    IPTaskMap * m_pProposedTasks;
  
    /**
     * Counter for generating task ids.
     */
    int m_taskCounter;

    /**
     * Mutex for controlling thread access to GDP data, not to derived
     * class data
     */
    omni_mutex m_gdpMutex;


    /**
     * Put the process to sleep until a new task notification is
     * received. This method will sleep until either the taskAdopted
     * or taskRejected functions next <b>exit</b>, unless these
     * methods have exited more times than this method has been
     * called. In this case the method does not block, as the
     * assumption is that the process has to process these outstanding
     * notifications first. This latter behaviour was added to fixed
     * bug #31.
     *
     */
    void waitForNotifications();

    omni_mutex m_taskNotificationMutex;
    omni_condition * m_pTaskNotificationCondition;



    /**
     * Struct used internally for passing data to threads
     */
    class GMResponseHandler  : public QueuedDataThread<cdl::TaskManagementResult>  {

    public:

      GMResponseHandler(ManagedProcess * _pGDP) :
	QueuedDataThread<cdl::TaskManagementResult>(),
	m_pGDP(_pGDP) {
      }
      
    private:

      virtual ~GMResponseHandler(){};

    protected:

      virtual void nextInQueue(cdl::TaskManagementResult & _data);

    private:
      ManagedProcess * m_pGDP; 
    };
  
    //has to be a pointer (AFAIK) as the destructor is private and the
    //thread is cleaned up by an omniorb call
    GMResponseHandler * m_responseRunnable;

  };

} //namespace cast
#endif
