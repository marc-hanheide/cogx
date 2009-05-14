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
package cast.architecture.subarchitecture;

import java.util.Hashtable;

import balt.core.connectors.ControlledRunnable;
import balt.core.connectors.push.nonprimitive.interfaces.StringPushInterface.StringPushConnectorOut;
import balt.core.connectors.push.nonprimitive.interfaces.StringPushInterface.StringPushSender;
import cast.architecture.abstr.WorkingMemoryReaderWriterProcess;
import cast.cdl.InformationProcessingTask;
import cast.cdl.TaskDescription;
import cast.cdl.TaskManagementDecision;
import cast.cdl.TaskManagementResult;
import cast.cdl.TaskOutcome;
import cast.cdl.TaskResult;
import cast.cdl.ui.ComponentEventType;
import cast.core.CASTUtils;
import cast.core.components.QueuedDataRunnable;
import cast.core.interfaces.GMRPushInterface.GMRPushReceiver;
import cast.core.interfaces.GPRPushInterface.GPRPushConnectorOut;
import cast.core.interfaces.GPRPushInterface.GPRPushSender;
import cast.core.interfaces.IPGPushInterface.IPGPushConnectorOut;
import cast.core.interfaces.IPGPushInterface.IPGPushSender;
import cast.core.interfaces.TDLPushInterface.TDLPushConnectorOut;
import cast.core.interfaces.TDLPushInterface.TDLPushSender;

/**
 * An abstract class to represent a component in a subarchitecture that can read
 * and write to a working memory, and has its operations controlled by a task
 * manager.
 * 
 * @author nah
 */
public abstract class ManagedProcess extends WorkingMemoryReaderWriterProcess
		implements
			IPGPushSender,
			GMRPushReceiver,
			GPRPushSender,
			StringPushSender,
			TDLPushSender {

	/**
	 * Inner class used to forward the responses of the task manager on to the
	 * subclass.
	 * 
	 * @author nah
	 */
	private static class GMResponseHandler
			extends
				QueuedDataRunnable<TaskManagementResult> {

		private ManagedProcess m_gdp;

		/**
		 * @param _gdp
		 * @param _goalDecision
		 * @param _goalID
		 */
		public GMResponseHandler(ManagedProcess _gdp) {
			super();
			m_gdp = _gdp;
		}

		@Override
		protected void nextInQueue(TaskManagementResult _data) {
			m_gdp.lockProcess();
			m_gdp.handleTaskManagerResponse(_data.m_id, _data.m_decision);
			m_gdp.unlockProcess();
		}

	}

	/**
	 * Connection to the task manager.
	 */
	private GPRPushConnectorOut m_processingResultToGoalManager;

	/**
	 * Connection to the task manager.
	 */
	private TDLPushConnectorOut m_taskDescriptionRegistration;

	private Object m_taskNotificationLock;

	/**
	 * Count of notifications that are outstanding in the "wait" cycle.
	 */
	int m_notifications;

	/**
	 * Counter for generating task ids.
	 */
	protected int m_taskCounter;

	/**
	 * Connection to the task manager.
	 */
	protected IPGPushConnectorOut m_proposalInputToTaskManager;

	/**
	 * Connection to the task manager.
	 */
	protected StringPushConnectorOut m_retractionInputToTaskManager;

	/**
	 * Hashtable that stores tasks that have been proposed.
	 */
	protected Hashtable<String, InformationProcessingTask> m_proposedTasks;

	private GMResponseHandler m_responseRunnable;

	/**
	 * Constructor.
	 * 
	 * @param _id
	 *            Unique component id.
	 */
	public ManagedProcess(String _id) {
		super(_id);
		m_proposalInputToTaskManager = null;
		m_taskCounter = 0;
		m_proposedTasks = new Hashtable<String, InformationProcessingTask>();
		m_processingResultToGoalManager = null;
		m_retractionInputToTaskManager = null;
		m_taskDescriptionRegistration = null;
		m_taskNotificationLock = new Object();
		m_notifications = 0;
	}

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
		if (_decision.value() == TaskManagementDecision.GOAL_ADOPTED.value()) {
			logTaskAdopted(_taskID);
			taskAdopted(_taskID);
			synchronized (m_taskNotificationLock) {
				m_notifications++;
				m_taskNotificationLock.notifyAll();
			}

		}
		else if (_decision.value() == TaskManagementDecision.GOAL_REJECTED
				.value()) {
			taskRejected(_taskID);
			synchronized (m_taskNotificationLock) {
				m_notifications++;
				m_taskNotificationLock.notifyAll();
			}

		}
		else if (_decision.value() == TaskManagementDecision.GOAL_WAITING
				.value()) {
			// just repropose for a laugh right now...

			try {
				proposeInformationProcessingTask(_taskID, m_proposedTasks
						.get(_taskID).m_taskName);
			}
			catch (SubarchitectureProcessException e) {
				e.printStackTrace();
			}
		}
		else {
			throw new RuntimeException("Unknown task manager response");
		}

	}

	/**
	 * @param _taskID
	 */
	private void logTaskAdopted(String _taskID) {
		logEvent(ComponentEventType.START, getProcessIdentifier().toString(),
				m_subarchitectureID, m_proposedTasks.get(_taskID).m_taskName,
				_taskID);
	}

	/**
	 * @param _taskID
	 */
	private void logTaskComplete(String _taskID) {

		logEvent(ComponentEventType.END, getProcessIdentifier().toString(),
				m_subarchitectureID, m_proposedTasks.get(_taskID).m_taskName,
				_taskID);
	}

	/**
	 * @param _taskID
	 * @param _name
	 */
	private void logTaskProposed(String _taskID, String _name) {
		logEvent(ComponentEventType.PROPOSED,
				getProcessIdentifier().toString(), m_subarchitectureID, _name,
				_taskID);
	}

	/**
	 * Generate a new unique task id.
	 * 
	 * @return A unique task id.
	 */
	protected String newTaskID() {
		return "IPG:" + getProcessIdentifier() + ":" + m_taskCounter++;
	}

	/**
	 * Propose the processing of a task.
	 * 
	 * @param _taskID
	 *            The id used to refer to this task.
	 * @param _taskName
	 *            The name of the task.
	 * @throws SubarchitectureProcessException
	 *             if connection to goal manager is not set or on comms error.
	 */
	protected void proposeInformationProcessingTask(String _taskID,
			String _taskName) throws SubarchitectureProcessException {

		assert (_taskID != null);
		assert (_taskName != null);

		logTaskProposed(_taskID, _taskName);

		if (m_proposalInputToTaskManager == null) {
			throw new SubarchitectureProcessException(
					"Push connection not to sa goal manager not set.");
		}

		// create the goal structure
		InformationProcessingTask ipg = new InformationProcessingTask(_taskID,
				_taskName);

		// store it as proposed
		m_proposedTasks.put(_taskID, ipg);

		// push goal to s-a goal manager
		m_proposalInputToTaskManager.push(getProcessIdentifier().toString(),
				ipg);

		debug("proposed: " + CASTUtils.toString(ipg));
	}

	/**
	 * Register a list of tasks that this component can perform.
	 * 
	 * @param _taskDesc
	 *            An array of task descriptions.
	 * @throws SubarchitectureProcessException
	 *             if connection to goal manager is not set or on comms error.
	 */
	protected void registerTaskDescriptions(TaskDescription[] _taskDesc)
			throws SubarchitectureProcessException {

		if (m_taskDescriptionRegistration == null) {
			throw new SubarchitectureProcessException(
					"Push connection not to sa goal manager not set.");
		}

		// push registration to s-a goal manager
		m_taskDescriptionRegistration.push(getProcessIdentifier().toString(),
				_taskDesc);
	}

	/**
	 * Retract the proposal of processing a task.
	 * 
	 * @param _taskID
	 *            The id of the task.
	 * @throws SubarchitectureProcessException
	 *             if connection to goal manager is not set or on comms error.
	 */
	protected void retractInformationProcessingTask(String _taskID)
			throws SubarchitectureProcessException {

		if (m_retractionInputToTaskManager == null) {
			throw new SubarchitectureProcessException(
					"Push connection not to sa goal manager not set.");
		}

		// push retraction to s-a goal manager
		m_retractionInputToTaskManager.push(getProcessIdentifier().toString(),
				_taskID);
	}

	/**
	 * Empty runComponent method. Does nothing.
	 * 
	 * @see cast.core.components.CASTComponent#runComponent()
	 */
	protected void runComponent() {
		// don't do anything if not overridden
	}

	/**
	 * Receive a signal that an information processing task has been accepted.
	 * 
	 * @param _taskID
	 */
	protected abstract void taskAdopted(String _taskID);

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
	protected void taskComplete(String _taskID, TaskOutcome _outcome)
			throws SubarchitectureProcessException {

		assert (_taskID != null);
		assert (_outcome != null);

		logTaskComplete(_taskID);

		// remove the goal from the hashtable
		m_proposedTasks.remove(_taskID);

		if (m_processingResultToGoalManager != null) {
			m_processingResultToGoalManager.push(getProcessIdentifier()
					.toString(), new TaskResult(_taskID, _outcome));
		}
		else {
			throw new SubarchitectureProcessException(
					"Goal processing result connection to goal manager has not been set up.");
		}

	}

	/**
	 * Receive a signal that an information processing goal has been rejected.
	 * 
	 * @param _taskID
	 */
	protected abstract void taskRejected(String _taskID);

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
				}
				catch (InterruptedException e) {
					e.printStackTrace();
				}
				// the count will have been incremented in the same action as
				// the broadcast
				m_notifications--;
				assert (m_notifications >= 0);

			}
			// else decrement the coutn
			else {
				m_notifications--;
				assert (m_notifications >= 0);
			}
		}

	}

	/**
	 * Receives the GoalManagementResult from the sub-architecture goal manager
	 * and forwards this change to the underlying class in a new thread. Also
	 * locks access to the receiving thread whilst doing this.
	 * 
	 * @param _src
	 *            The component that sent the data.
	 * @param _data
	 *            Task management result.
	 * @see cast.core.interfaces.GMRPushInterface.GMRPushReceiver#receivePushData(java.lang.String,
	 *      cast.corba.autogen.CAAT.TaskManagementResult)
	 */
	public void receivePushData(String _src, TaskManagementResult _data) {

		debug("received goal management decision: " + _data.m_id);

		// if this goal was proposed by this component
		// hashtable is synchronised, so no need to protect it
		if (m_proposedTasks.containsKey(_data.m_id)) {

			if (m_responseRunnable == null) {
				m_responseRunnable = new GMResponseHandler(this);
				m_responseRunnable.start();
				new Thread(m_responseRunnable).start();
			}

			m_responseRunnable.queue(_data);

		}
	}

	/**
	 * Set push connector to task manager for task results.
	 * 
	 * @param _connectionID
	 *            The connector id.
	 * @param _out
	 *            The connector.
	 * @see cast.core.interfaces.GPRPushInterface.GPRPushSender#setPushConnector(java.lang.String,
	 *      cast.core.interfaces.GPRPushInterface.GPRPushConnectorOut)
	 */
	public void setPushConnector(String _connectionID, GPRPushConnectorOut _out) {
		m_processingResultToGoalManager = _out;
	}

	/**
	 * Set push connector to task manager for task proposition.
	 * 
	 * @param _connectionID
	 *            The connector id.
	 * @param _out
	 *            The connector.
	 * @see cast.core.interfaces.IPGPushInterface.IPGPushSender#setPushConnector(java.lang.String,
	 *      cast.core.interfaces.IPGPushInterface.IPGPushConnectorOut)
	 */
	public void setPushConnector(String _connectionID, IPGPushConnectorOut _out) {
		m_proposalInputToTaskManager = _out;
	}

	/**
	 * Set push connector to task manager for task retraction.
	 * 
	 * @param _connectionID
	 *            The connector id.
	 * @param _out
	 *            The connector.
	 * @see balt.core.connectors.push.nonprimitive.interfaces.StringPushInterface.StringPushSender#setPushConnector(java.lang.String,
	 *      balt.core.connectors.push.nonprimitive.interfaces.StringPushInterface.StringPushConnectorOut)
	 */
	public void setPushConnector(String _connectionID,
			StringPushConnectorOut _out) {
		m_retractionInputToTaskManager = _out;
	}

	/**
	 * Set push connector to task manager for task registration.
	 * 
	 * @param _connectionID
	 *            The connector id.
	 * @param _out
	 *            The connector.
	 * @see cast.core.interfaces.TDLPushInterface.TDLPushSender#setPushConnector(java.lang.String,
	 *      cast.core.interfaces.TDLPushInterface.TDLPushConnectorOut)
	 */
	public void setPushConnector(String _connectionID, TDLPushConnectorOut _out) {
		m_taskDescriptionRegistration = _out;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see caat.architecture.abstr.WorkingMemoryReaderProcess#stop()
	 */
	@Override
	public void stop() {
		super.stop();

		if (m_responseRunnable != null) {
			m_responseRunnable.stop();
		}

		synchronized (m_taskNotificationLock) {
			m_taskNotificationLock.notifyAll();
		}
	}

}
