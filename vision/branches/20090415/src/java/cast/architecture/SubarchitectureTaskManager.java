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
import cast.interfaces.ManagedComponentPrx;
import cast.interfaces._TaskManagerOperations;

/**
 * Abstract class for a subarchitecture task manager. This still uses a fairly
 * threading-heavy approach that needs to be revised.
 * 
 * @author nah
 */
public abstract class SubarchitectureTaskManager extends
WorkingMemoryReaderComponent implements _TaskManagerOperations {


//	/**
//	 * Thread that forwards task completion events to the subclass.
//	 * 
//	 * @author nah
//	 */
//	private static class TaskCompletionRunnable extends
//			QueuedDataRunnable<Pair<String, TaskResult>> {
//
//		private SubarchitectureTaskManager m_tm;
//
//		public TaskCompletionRunnable(SubarchitectureTaskManager _tm) {
//			m_tm = _tm;
//		}
//
//		@Override
//		protected void nextInQueue(Pair<String, TaskResult> _data) {

//		}
//
//		public void queueCompletion(String _src, TaskResult _data) {
//			queue(new Pair<String, TaskResult>(_src, _data));
//		}
//
//	}
//
//	/**
//	 * Thread that forwards task proposal events to the subclass.
//	 * 
//	 * @author nah
//	 */
//	private static class TaskProposalRunnable extends
//			QueuedDataRunnable<Pair<String, InformationComponentingTask>> {
//
//		private SubarchitectureTaskManager m_tm;
//
//		public TaskProposalRunnable(SubarchitectureTaskManager _tm) {
//			super();
//			m_tm = _tm;
//		}
//
//		@Override
//		protected void nextInQueue(Pair<String, InformationComponentingTask> _data) {

//		}
//
//		public void queueTask(String _src, InformationComponentingTask _task) {
//			queue(new Pair<String, InformationComponentingTask>(_src, _task));
//		}
//
//	}
//
//	/**
//	 * Thread that forwards task registration events to the subclass.
//	 * 
//	 * @author nah
//	 */
//	private static class TaskRegistrationRunnable extends
//			QueuedDataRunnable<Pair<String, TaskDescription[]>> {
//
//		private SubarchitectureTaskManager m_tm;
//
//		public TaskRegistrationRunnable(SubarchitectureTaskManager _tm) {
//			m_tm = _tm;
//		}
//
//		@Override
//		protected void nextInQueue(Pair<String, TaskDescription[]> _data) {
//			lockComponent();
//			taskRegistered(_data.m_first, _data.m_second);
//			unlockComponent();
//		}
//
//		public void queueRegistration(String _src, TaskDescription[] _data) {
//			queue(new Pair<String, TaskDescription[]>(_src, _data));
//		}
//
//	}
//
//	/**
//	 * Thread that forwards task retraction events to the subclass.
//	 * 
//	 * @author nah
//	 */
//	private static class TaskRetractionRunnable extends
//			QueuedDataRunnable<Pair<String, String>> {
//
//		private SubarchitectureTaskManager m_tm;
//
//		public TaskRetractionRunnable(SubarchitectureTaskManager _tm) {
//			m_tm = _tm;
//		}
//
//		@Override
//		protected void nextInQueue(Pair<String, String> _data) {
//			
//
//		}
//
//		public void queueRetraction(String _src, String _o) {
//			queue(new Pair<String, String>(_src, _o));
//		}
//	}
//
	/**
	 * Connection for goal management result.
	 */

	protected Object m_proposalLock;

	protected Object m_completionLock;

	protected final HashMap<String, ManagedComponentPrx> m_components;
	
	/**
	 * Constructor.
	 * 
	 */
	public SubarchitectureTaskManager() {
		m_proposalLock = new Object();
		m_completionLock = new Object();
		m_components = new HashMap<String, ManagedComponentPrx>();
	}

	@Override
	public void stopInternal() {
		super.stopInternal();

		synchronized (m_proposalLock) {
			m_proposalLock.notifyAll();
		}

		synchronized (m_completionLock) {
			m_completionLock.notifyAll();
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
	protected void sendTaskDecision(String _component, String _taskID,
			TaskManagementDecision _decision) {
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
		// sb.append(NativeComponentLauncher.toString(ComponentLauncher
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


		ManagedComponentPrx prx = m_components.get(_component);
		assert(prx != null);
		
		prx.taskDecision(_taskID, _decision);
		
		debug("sent decision: " + _taskID);
	}

	/**
	 * Receive a task completion event.
	 * 
	 * @param _component
	 *            The component that has completed the task.
	 * @param _data
	 *            The completion result.
	 */
	protected abstract void taskCompleted(String _component, String _taskID, TaskOutcome _data);

	/**
	 * Receive a task proposal event.
	 * 
	 * @param _component
	 *            The component proposing the task.
	 * @param _data
	 *            The task instance information.
	 */
	protected abstract void taskProposed(String _component,
			String _taskID, String _taskName);

//	/**
//	 * Receive a task registration event.
//	 * 
//	 * @param _src
//	 *            The component registering the task.
//	 * @param _desc
//	 *            The task description.
//	 */
//	protected abstract void taskRegistered(String _src, TaskDescription[] _desc);

	/**
	 * Receive a task retraction event.
	 * 
	 * @param _src
	 *            The component retracting the task.
	 * @param _taskID
	 *            The task idn.
	 */
	protected abstract void taskRetracted(String _component, String _taskID);
	

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

	public void proposeTask(String _component, String _taskID,
			String _taskName, Current __current) {
		lockComponent();
		// write to task manager
		taskProposed(_component, _taskID, _taskName);
		unlockComponent();

		synchronized (m_proposalLock) {
			m_proposalLock.notifyAll();
		}
	}

	public void retractTask(String _component, String _taskID, Current __current) {
		lockComponent();
		taskRetracted(_component,_taskID);
		unlockComponent();

		synchronized (m_proposalLock) {
			m_proposalLock.notifyAll();
		}
	}

	public void taskComplete(String _component, String _taskID, TaskOutcome _outcome,
			Current __current) {	
		lockComponent();
		// logTaskCompleted();
		taskCompleted(_component, _taskID, _outcome);
		unlockComponent();
		// println("... completion done");

		// wake up waiting threads
		synchronized (m_completionLock) {
			m_completionLock.notifyAll();
		}
	}

	public void addManagedComponent(ManagedComponentPrx _comp, Current __current) {
		m_components.put(_comp.getID(), _comp);
	}

	
}
