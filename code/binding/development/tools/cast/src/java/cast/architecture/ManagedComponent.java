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

/**
 * 
 */
package cast.architecture;

import java.util.HashMap;

import Ice.Current;
import cast.cdl.TaskManagementDecision;
import cast.cdl.TaskOutcome;
import cast.core.Pair;
import cast.core.QueuedDataRunnable;
import cast.interfaces.TaskManagerPrx;
import cast.interfaces._ManagedComponentOperations;

/**
 * An abstract class to represent a component in a subarchitecture that can read
 * and write to a working memory, and has its operations controlled by a task
 * manager.
 * 
 * @author nah
 */
public abstract class ManagedComponent extends WorkingMemoryReaderComponent
		implements _ManagedComponentOperations {

	private static class TaskManagementResult {
		public String m_id;
		public TaskManagementDecision m_decision;

		public TaskManagementResult(String _id, TaskManagementDecision _decision) {
			m_id = _id;
			m_decision = _decision;
		}
	}

	private TaskManagerPrx m_taskManager;
	private final HashMap<String, Pair<String, String>> m_proposedTasks;

	public ManagedComponent() {
		m_proposedTasks = new HashMap<String, Pair<String, String>>();
		m_taskCounter = 0;
		m_taskNotificationLock = new Object();
		m_notifications = 0;
		m_responseRunnable = null;
	}

	public void setTaskManager(TaskManagerPrx _tm, Current __current) {
		m_taskManager = _tm;
	}

	/**
	 * Inner class used to forward the responses of the task manager on to the
	 * subclass.
	 * 
	 * @author nah
	 */
	private static class GMResponseHandler extends
			QueuedDataRunnable<TaskManagementResult> {

		private ManagedComponent m_gdp;

		/**
		 * @param _gdp
		 * @param _goalDecision
		 * @param _goalID
		 */
		public GMResponseHandler(ManagedComponent _gdp) {
			super();
			m_gdp = _gdp;
		}

		@Override
		protected void nextInQueue(TaskManagementResult _data) {
			m_gdp.lockComponent();
			m_gdp.handleTaskManagerResponse(_data.m_id, _data.m_decision);
			m_gdp.unlockComponent();
		}

	}

	private final Object m_taskNotificationLock;

	/**
	 * Count of notifications that are outstanding in the "wait" cycle.
	 */
	int m_notifications;

	/**
	 * Counter for generating task ids.
	 */
	protected int m_taskCounter;

	private GMResponseHandler m_responseRunnable;

	/**
	 * Receive a response from the task manager and call the correct
	 * result-specific method.
	 * 
	 * @param _taskID
	 *            The id of the task.
	 * @param _decision
	 *            The task manager decision.
	 */
	private void handleTaskManagerResponse(String _taskID,
			TaskManagementDecision _decision) {

		// inform sub-class of result
		if (_decision.value() == TaskManagementDecision.TaskAdopted.value()) {
			taskAdopted(_taskID);
			synchronized (m_taskNotificationLock) {
				m_notifications++;
				m_taskNotificationLock.notifyAll();
			}

		} else if (_decision.value() == TaskManagementDecision.TaskRejected.value()
				) {
			taskRejected(_taskID);
			synchronized (m_taskNotificationLock) {
				m_notifications++;
				m_taskNotificationLock.notifyAll();
			}

		} else if (_decision.value() == TaskManagementDecision.TaskWaiting.value()) {
			debug("TaskWaiting received for: " + _taskID);
		} 

	}

	/**
	 * Generate a new unique task id.
	 * 
	 * @return A unique task id.
	 */
	protected String newTaskID() {
		StringBuffer sb = new StringBuffer("task:");
		sb.append(getComponentID());
		sb.append(m_taskCounter++);
		return sb.toString();
	}

	/**
	 * Propose the processing of a task.
	 * 
	 * @param _taskID
	 *            The id used to refer to this task.
	 * @param _taskName
	 *            The name of the task.
	 */
	protected void proposeInformationProcessingTask(String _taskID,
			String _taskName) {

		assert (_taskID != null);
		assert (_taskName != null);
		assert (m_taskManager != null);

		// store it as proposed
		m_proposedTasks.put(_taskID, new Pair<String, String>(_taskID,
				_taskName));

		m_taskManager.proposeTask(getComponentID(), _taskID, _taskName);
	}

	// /**
	// * Register a list of tasks that this component can perform.
	// *
	// * @param _taskDesc
	// * An array of task descriptions.
	// * @throws SubarchitectureProcessException
	// * if connection to goal manager is not set or on comms error.
	// */
	// protected void registerTaskDescriptions(TaskDescription[] _taskDesc)
	// throws SubarchitectureProcessException {
	//
	// if (m_taskDescriptionRegistration == null) {
	// throw new SubarchitectureProcessException(
	// "Push connection not to sa goal manager not set.");
	// }
	//
	// // push registration to s-a goal manager
	// m_taskDescriptionRegistration.push(getProcessIdentifier().toString(),
	// _taskDesc);
	// }

	/**
	 * Retract the proposal of processing a task.
	 * 
	 * @param _taskID
	 *            The id of the task.
	 * @throws SubarchitectureProcessException
	 *             if connection to goal manager is not set or on comms error.
	 */
	protected void retractInformationProcessingTask(String _taskID) {
		assert (m_taskManager != null);
		m_proposedTasks.remove(_taskID);
		m_taskManager.retractTask(getComponentID(),_taskID);
	}

	/**
	 * Receive a signal that an information processing task has been accepted.
	 * 
	 * @param _taskID
	 */
	protected void taskAdopted(String _taskID) {

	}

	/**
	 * Signal the outcome of goal processing to the sub-architecture goal
	 * manager.
	 * 
	 * @param _taskID
	 *            The task id.
	 * @param _outcome
	 *            The task outcome.
	 * @throws SubarchitectureProcessException
	 *             if connection to goal manager is not set.`
	 */
	protected void taskComplete(String _taskID, TaskOutcome _outcome) {

		assert (_taskID != null);
		assert (_outcome != null);
		assert (m_taskManager != null);

		// remove the goal from the hashtable
		m_proposedTasks.remove(_taskID);

		m_taskManager.taskComplete(getComponentID(),_taskID, _outcome);
	}

	/**
	 * Receive a signal that an information processing goal has been rejected.
	 * 
	 * @param _taskID
	 */
	protected void taskRejected(String _taskID) {

	}
	

	
	//
	/**
	 * Put the process to sleep until a new task notification is received. This
	 * method will sleep until either the taskAdopted or taskRejected functions
	 * next <b>exit</b>, unless these methods have exited more times than this
	 * method has been called. In this case the method does not block, as the
	 * assumption is that the process has to process these outstanding
	 * notifications first. This latter behaviour was added to fixed bug #31.
	 * 
	 * @remark Interface change: Used to take a single argument. This was
	 *         removed to simplify behaviour.
	 */
	protected void waitForNotifications() {

		synchronized (m_taskNotificationLock) {
			if (m_notifications == 0) {
				// println("blocking for new");
				try {
					m_taskNotificationLock.wait();
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
				// the count will have been incremented in the same action as
				// the broadcast
				m_notifications--;
				assert (m_notifications >= 0);

			}
			// else decrement the cout
			else {
				m_notifications--;
				assert (m_notifications >= 0);
			}
		}

	}

	public void taskDecision(String _id, TaskManagementDecision _decision, Current __current) {

		debug("received goal management decision: " + _id);

		// if this goal was proposed by this component
		// hashtable is synchronised, so no need to protect it
		if (m_proposedTasks.containsKey(_id)) {

			if (m_responseRunnable == null) {
				m_responseRunnable = new GMResponseHandler(this);
				m_responseRunnable.start();
				new Thread(m_responseRunnable).start();
			}

			m_responseRunnable.queue(new TaskManagementResult(_id, _decision));
		}
	}

	@Override
	public void stopInternal() {

		if (m_responseRunnable != null) {
			m_responseRunnable.stop();
		}

		synchronized (m_taskNotificationLock) {
			m_taskNotificationLock.notifyAll();
		}
	}

}
