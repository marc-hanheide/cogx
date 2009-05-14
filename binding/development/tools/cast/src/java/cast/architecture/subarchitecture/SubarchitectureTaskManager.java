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

import balt.core.connectors.push.nonprimitive.interfaces.StringPushInterface.StringPushReceiver;
import balt.core.data.Pair;
import cast.architecture.abstr.WorkingMemoryReaderWriterProcess;
import cast.cdl.InformationProcessingTask;
import cast.cdl.TaskDescription;
import cast.cdl.TaskManagementDecision;
import cast.cdl.TaskManagementResult;
import cast.cdl.TaskResult;
import cast.core.CASTUtils;
import cast.core.components.QueuedDataRunnable;
import cast.core.interfaces.GMRPushInterface.GMRPushConnectorOut;
import cast.core.interfaces.GMRPushInterface.GMRPushSender;
import cast.core.interfaces.GPRPushInterface.GPRPushReceiver;
import cast.core.interfaces.IPGPushInterface.IPGPushReceiver;
import cast.core.interfaces.TDLPushInterface.TDLPushReceiver;

/**
 * Abstract class for a subarchitecture task manager. This still uses a fairly
 * threading-heavy approach that needs to be revised.
 * 
 * @author nah
 */
public abstract class SubarchitectureTaskManager extends
		WorkingMemoryReaderWriterProcess implements IPGPushReceiver,
		GMRPushSender, GPRPushReceiver, StringPushReceiver, TDLPushReceiver {

	/**
	 * Thread that forwards task completion events to the subclass.
	 * 
	 * @author nah
	 */
	private static class TaskCompletionRunnable extends
			QueuedDataRunnable<Pair<String, TaskResult>> {

		private SubarchitectureTaskManager m_tm;

		public TaskCompletionRunnable(SubarchitectureTaskManager _tm) {
			m_tm = _tm;
		}

		@Override
		protected void nextInQueue(Pair<String, TaskResult> _data) {
			m_tm.lockProcess();
			// m_tm.logTaskCompleted();
			m_tm.taskCompleted(_data.m_first, _data.m_second);
			m_tm.unlockProcess();
			// m_tm.println("... completion done");

			// wake up waiting threads
			synchronized (m_tm.m_completionLock) {
				m_tm.m_completionLock.notifyAll();
			}
		}

		public void queueCompletion(String _src, TaskResult _data) {
			queue(new Pair<String, TaskResult>(_src, _data));
		}

	}

	/**
	 * Thread that forwards task proposal events to the subclass.
	 * 
	 * @author nah
	 */
	private static class TaskProposalRunnable extends
			QueuedDataRunnable<Pair<String, InformationProcessingTask>> {

		private SubarchitectureTaskManager m_tm;

		public TaskProposalRunnable(SubarchitectureTaskManager _tm) {
			super();
			m_tm = _tm;
		}

		@Override
		protected void nextInQueue(Pair<String, InformationProcessingTask> _data) {
			m_tm.lockProcess();
			// write to task manager
			m_tm.taskProposed(_data.m_first, _data.m_second);

			m_tm.unlockProcess();

			synchronized (m_tm.m_proposalLock) {
				m_tm.m_proposalLock.notifyAll();
			}
		}

		public void queueTask(String _src, InformationProcessingTask _task) {
			queue(new Pair<String, InformationProcessingTask>(_src, _task));
		}

	}

	/**
	 * Thread that forwards task registration events to the subclass.
	 * 
	 * @author nah
	 */
	private static class TaskRegistrationRunnable extends
			QueuedDataRunnable<Pair<String, TaskDescription[]>> {

		private SubarchitectureTaskManager m_tm;

		public TaskRegistrationRunnable(SubarchitectureTaskManager _tm) {
			m_tm = _tm;
		}

		@Override
		protected void nextInQueue(Pair<String, TaskDescription[]> _data) {
			m_tm.lockProcess();
			m_tm.taskRegistered(_data.m_first, _data.m_second);
			m_tm.unlockProcess();
		}

		public void queueRegistration(String _src, TaskDescription[] _data) {
			queue(new Pair<String, TaskDescription[]>(_src, _data));
		}

	}

	/**
	 * Thread that forwards task retraction events to the subclass.
	 * 
	 * @author nah
	 */
	private static class TaskRetractionRunnable extends
			QueuedDataRunnable<Pair<String, String>> {

		private SubarchitectureTaskManager m_tm;

		public TaskRetractionRunnable(SubarchitectureTaskManager _tm) {
			m_tm = _tm;
		}

		@Override
		protected void nextInQueue(Pair<String, String> _data) {
			m_tm.lockProcess();
			m_tm.taskRetracted(_data.m_first, _data.m_second);
			m_tm.unlockProcess();

			synchronized (m_tm.m_proposalLock) {
				m_tm.m_proposalLock.notifyAll();
			}

		}

		public void queueRetraction(String _src, String _o) {
			queue(new Pair<String, String>(_src, _o));
		}
	}

	/**
	 * Connection for goal management result.
	 */
	protected GMRPushConnectorOut m_goalResultOutput;

	protected Object m_proposalLock;

	protected Object m_completionLock;

	private TaskProposalRunnable m_tpRunnable;

	private TaskRetractionRunnable m_trRunnable;

	private TaskRegistrationRunnable m_tregRunnable;

	private TaskCompletionRunnable m_compRunnable;

	// private void logTaskProposal(String _taskID, String _taskName) {
	// logEvent(ComponentEventType.PROPOSED,
	// getProcessIdentifier().toString(), )
	// }

	// private void logTaskAdopted() {
	// synchronized (m_componentStatus) {
	// m_componentStatus.m_totalOverwrites++;
	// }
	// }
	//
	// private void logTaskCompleted() {
	// synchronized (m_componentStatus) {
	// m_componentStatus.m_totalDeletes++;
	// }
	// }

	/**
	 * Constructor.
	 * 
	 * @param _id
	 *            Unique component id.
	 */
	public SubarchitectureTaskManager(String _id) {
		super(_id);
		m_goalResultOutput = null;
		m_proposalLock = new Object();
		m_completionLock = new Object();
	}

	/**
	 * Empty runComponent method. Does nothing.
	 * 
	 * @see cast.core.components.CASTComponent#runComponent()
	 */
	@Override
	protected void runComponent() {
		// do nothing as default
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see caat.architecture.abstr.WorkingMemoryReaderProcess#stop()
	 */
	@Override
	public void stop() {
		super.stop();

		synchronized (m_proposalLock) {
			m_proposalLock.notifyAll();
		}

		synchronized (m_completionLock) {
			m_completionLock.notifyAll();
		}

		//stop threads
		
		if (m_tpRunnable != null) {
			m_tpRunnable.stop();
		}
		if (m_tregRunnable != null) {
			m_tregRunnable.stop();
		}
		if (m_trRunnable != null) {
			m_trRunnable.stop();
		}
		if (m_compRunnable != null) {
			m_compRunnable.stop();
		}
	}

	/**
	 * Send a task management decision to component in the same subarchitecture.
	 * 
	 * @param _ipg
	 *            The task that the decision is about.
	 * @param _decision
	 *            The decision that has been reached about the task.
	 */
	protected void sendTaskDecision(InformationProcessingTask _ipg,
			TaskManagementDecision _decision) {
		// create the result struct
		TaskManagementResult gmr = new TaskManagementResult(_ipg.m_id,
				_decision);

		// if (m_bLogOutput) {
		// StringBuffer sb = new StringBuffer();
		//
		// if (_decision == TaskManagementDecision.GOAL_ADOPTED) {
		// sb.append("GO");
		// }
		// else if (_decision == TaskManagementDecision.GOAL_REJECTED) {
		// sb.append("NO");
		// }
		// else if (_decision == TaskManagementDecision.GOAL_WAITING) {
		// sb.append("WT");
		// }
		//
		// sb.append(" ");
		// sb.append(NativeProcessLauncher.toString(ProcessLauncher
		// .getBALTTime()));
		// sb.append(" ");
		// sb.append("comp.name");
		// sb.append(" ");
		// sb.append(_ipg.m_id);
		// sb.append(" ");
		// sb.append(_ipg.m_taskName);
		// log(sb.toString());
		//
		// }

		// println("sending push...");
		m_goalResultOutput.push(getProcessIdentifier().toString(), gmr);
		// println("... push sent");

		debug("sent decision: " + gmr.m_id);
	}

	/**
	 * Receive a task completion event.
	 * 
	 * @param _src
	 *            The component that has completed the task.
	 * @param _data
	 *            The completion result.
	 */
	protected abstract void taskCompleted(String _src, TaskResult _data);

	/**
	 * Receive a task proposal event.
	 * 
	 * @param _src
	 *            The component proposing the task.
	 * @param _data
	 *            The task instance information.
	 */
	protected abstract void taskProposed(String _src,
			InformationProcessingTask _data);

	/**
	 * Receive a task registration event.
	 * 
	 * @param _src
	 *            The component registering the task.
	 * @param _desc
	 *            The task description.
	 */
	protected abstract void taskRegistered(String _src, TaskDescription[] _desc);

	/**
	 * Receive a task retraction event.
	 * 
	 * @param _src
	 *            The component retracting the task.
	 * @param _taskID
	 *            The task idn.
	 */
	protected abstract void taskRetracted(String _src, String _taskID);

	/**
	 * Receive push data describing a proposed task.
	 * 
	 * @param _src
	 *            The component that sent the data.
	 * @param _data
	 *            The task proposed for processing.
	 * @see caat.framework.interfaces.IPGPushInterface.IPGPushReceiver#receivePushData(java.lang.String,
	 *      cast.corba.autogen.CAAT.InformationProcessingGoal)
	 */
	public void receivePushData(String _src, InformationProcessingTask _data) {

		debug("receivePushData " + CASTUtils.toString(_data));

		if (m_status == ProcessStatus.RUN) {

			if (m_tpRunnable == null) {
				m_tpRunnable = new TaskProposalRunnable(this);
				m_tpRunnable.start();
				new Thread(m_tpRunnable).start();
			}
			m_tpRunnable.queueTask(_src, _data);
		}
	}

	/**
	 * Receive data describing a task retraction.
	 * 
	 * @param _src
	 *            The component that sent the data.
	 * @param _o
	 *            The task id for retraction.
	 * @see balt.core.connectors.push.nonprimitive.interfaces.StringPushInterface.StringPushReceiver#receivePushData(java.lang.String,
	 *      java.lang.String)
	 */
	public void receivePushData(String _src, String _o) {
		if (m_status == ProcessStatus.RUN) {
			if (m_trRunnable == null) {
				m_trRunnable = new TaskRetractionRunnable(this);
				m_trRunnable.start();
				new Thread(m_trRunnable).start();
			}
			m_trRunnable.queueRetraction(_src, _o);
		}
	}

	/**
	 * Receive push information describing tasks that can be registered with the
	 * task manager.
	 * 
	 * @param _src
	 *            The component sending the data.
	 * @param _data
	 *            Descriptions of the tasks the component can perform.
	 * @see cast.core.interfaces.TDLPushInterface.TDLPushReceiver#receivePushData(java.lang.String,
	 *      cast.corba.autogen.CAAT.TaskDescription[])
	 */
	public void receivePushData(String _src, TaskDescription[] _data) {
		if (m_status == ProcessStatus.RUN) {

			if (m_tregRunnable == null) {
				m_tregRunnable = new TaskRegistrationRunnable(this);
				m_tregRunnable.start();
				new Thread(m_tregRunnable).start();
			}
			m_tregRunnable.queueRegistration(_src, _data);
		}
	}

	/**
	 * Receive data describing the completion of a task.
	 * 
	 * @param _src
	 *            The component that sent the data.
	 * @param _data
	 *            The task completion status
	 * @see caat.framework.interfaces.GPRPushInterface.GPRPushReceiver#receivePushData(java.lang.String,
	 *      cast.corba.autogen.CAAT.GoalProcessingResult)
	 */
	public void receivePushData(String _src, TaskResult _data) {

		// println("receivePushData() TR... ");
		if (m_status == ProcessStatus.RUN) {

			if (m_compRunnable == null) {
				m_compRunnable = new TaskCompletionRunnable(this);
				m_compRunnable.start();
				new Thread(m_compRunnable).start();
			}
			m_compRunnable.queueCompletion(_src, _data);
		}
		// println("... TR done");
	}

	/**
	 * Set the push connector for sending management decisions. Only a single
	 * connector is stored, so it is assumed to be broadcast.
	 * 
	 * @param _connectionID
	 *            The id of the connector.
	 * @param _out
	 *            The connector.
	 * @see caat.framework.interfaces.GMRPushInterface.GMRPushSender#setPushConnector(java.lang.String,
	 *      caat.framework.interfaces.GMRPushInterface.GMRPushConnectorOut)
	 */
	public void setPushConnector(String _connectionID, GMRPushConnectorOut _out) {
		m_goalResultOutput = _out;
	}

	/**
	 * Create an ID for a task manager component given a subarchitecture name.
	 * 
	 * @param _subarch
	 *            The name of the subarchitecture which contains the task
	 *            manager component.
	 * @return The new id for the component.
	 */
	public static String createID(String _subarch) {
		return _subarch + ":tm";
	}

	/**
	 * Wait until new task proposal information is received. This method causes
	 * the current thread to wait until the next exit of either taskProposed or
	 * taskRetracted.
	 */
	protected void waitForProposals() {
		synchronized (m_proposalLock) {
			try {
				m_proposalLock.wait();
			} catch (InterruptedException e) {
				e.printStackTrace();
				System.exit(1);
			}
		}
	}

	/**
	 * Wait until new task completion information is received. This method
	 * causes the current thread to wait until the next exit of taskCompleted;
	 */
	protected void waitForCompletions() {
		synchronized (m_completionLock) {
			try {
				m_completionLock.wait();
			} catch (InterruptedException e) {
				e.printStackTrace();
				System.exit(1);
			}
		}
	}

}
