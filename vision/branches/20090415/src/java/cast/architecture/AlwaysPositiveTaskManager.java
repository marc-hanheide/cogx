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

import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

import cast.cdl.TaskManagementDecision;
import cast.cdl.TaskOutcome;
import cast.core.Pair;

/**
 * @author nah
 */
public class AlwaysPositiveTaskManager extends SubarchitectureTaskManager {

	@Override
	protected void taskCompleted(String _component, String _taskID,
			TaskOutcome _data) {

	}

	@Override
	protected void taskProposed(String _component, String _taskID,
			String _taskName) {
		if (m_goalQueue == null) {
			m_goalQueue = new ConcurrentLinkedQueue<Pair<String, String>>();
		}
		m_goalQueue.add(new Pair<String, String>(_component, _taskID));
	}

	@Override
	protected void taskRetracted(String _component, String _taskID) {
		for (Pair<String, String> task : m_goalQueue) {
			if (task.m_second.equals(_taskID)) {
				m_goalQueue.remove(task);
				return;
			}
		}
	}

	private Queue<Pair<String, String>> m_goalQueue;

	/**
	 * @param _id
	 */
	public AlwaysPositiveTaskManager() {
		receiveNoChanges();
	}

	protected void processTaskQueue() {
		while (!m_goalQueue.isEmpty()) {
			Pair<String, String> task = m_goalQueue.poll();
			sendTaskDecision(task.m_first, task.m_second,
					TaskManagementDecision.TaskAdopted);
		}
	}
	
	@Override
	protected void stop() {
		lockComponent();
		if(m_goalQueue != null) {
			m_goalQueue.clear();
		}
		
		synchronized (m_proposalLock) {
			m_proposalLock.notifyAll();			
		}
		unlockComponent();
	}

	@Override
	public void runComponent() {
		// System.out.println("AlwaysPositiveTaskManager.runComponent()");
		while (isRunning()) {

			// println("waiting to check queue...");
			if (m_goalQueue != null) {
				lockComponent();
				processTaskQueue();
				unlockComponent();
			}

			waitForProposals();
		}

	}

	//
	//
	// /*
	// * (non-Javadoc)
	// *
	// * @see
	// caat.architecture.subarchitecture.SubarchitectureTaskManager#taskRegistered(java.lang.String,
	// * caat.corba.autogen.CAAT.TaskDescription)
	// */
	// @Override
	// protected void taskRegistered(String _src, TaskDescription[] _desc) {
	// for (int i = 0; i < _desc.length; i++) {
	// println("task registered: " + _desc[i].m_taskName);
	// }
	//
	// }

}
