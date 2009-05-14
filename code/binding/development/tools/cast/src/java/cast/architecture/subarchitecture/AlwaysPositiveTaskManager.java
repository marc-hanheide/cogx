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

import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

import cast.cdl.InformationProcessingTask;
import cast.cdl.TaskDescription;
import cast.cdl.TaskManagementDecision;
import cast.cdl.TaskResult;
import cast.cdl.WorkingMemoryChangeQueueBehaviour;
import cast.core.CASTUtils;

/**
 * @author nah
 */
public class AlwaysPositiveTaskManager extends SubarchitectureTaskManager {

	private Queue<InformationProcessingTask> m_goalQueue;

	/**
	 * @param _id
	 */
	public AlwaysPositiveTaskManager(String _id) {
		super(_id);

		// just for debugging, doesn't make a difference at run time
		m_queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;
		receiveNoChanges();

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see caat.architecture.subarchitecture.SubarchitectureTaskManager#goalProposed(java.lang.String,
	 *      caat.corba.autogen.CAAT.InformationProcessingGoal)
	 */
	@Override
	protected void taskProposed(String _src, InformationProcessingTask _data) {

		debug("taskProposed: " + _src + " " + CASTUtils.toString(_data));
		// println("adding to queue " + _data.m_id + " ...");

		if (m_goalQueue == null) {
			m_goalQueue = new ConcurrentLinkedQueue<InformationProcessingTask>();
		}

		m_goalQueue.add(_data);
		// println("... addition done");
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see caat.architecture.subarchitecture.SubarchitectureTaskManager#taskRetracted(java.lang.String,
	 *      java.lang.String)
	 */
	@Override
	protected void taskRetracted(String _src, String _taskID) {

			//nah: what's the point?
		//		if (m_retracted == null) {
//			m_retracted = new HashSet<String>();
//		}
//
//		m_retracted.add(_taskID);
	}

	protected void processGoalQueue() {
		// println("checking goals.... queue size: " +
		// m_goalQueue.size());

		// ListIterator<InformationProcessingTask> i = m_goalQueue
		// .listIterator();
		while (!m_goalQueue.isEmpty()) {

			InformationProcessingTask ipg = m_goalQueue.poll();

				sendTaskDecision(ipg, TaskManagementDecision.GOAL_ADOPTED);

		}
		// println("... goal checking done");
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see caat.core.components.CAATComponent#runComponent()
	 */
	@Override
	public void runComponent() {
		// System.out.println("AlwaysPositiveTaskManager.runComponent()");
		while (m_status == ProcessStatus.RUN) {

			// println("waiting to check queue...");
			if (m_goalQueue != null) {
				lockProcess();
				processGoalQueue();
				unlockProcess();
				// println("... queue done");
			}

			waitForProposals();
		}

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see caat.architecture.subarchitecture.SubarchitectureTaskManager#taskCompleted(java.lang.String,
	 *      caat.corba.autogen.CAAT.GoalProcessingResult)
	 */
	@Override
	protected void taskCompleted(String _src, TaskResult _data) {
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see caat.architecture.subarchitecture.SubarchitectureTaskManager#taskRegistered(java.lang.String,
	 *      caat.corba.autogen.CAAT.TaskDescription)
	 */
	@Override
	protected void taskRegistered(String _src, TaskDescription[] _desc) {
		for (int i = 0; i < _desc.length; i++) {
			println("task registered: " + _desc[i].m_taskName);
		}

	}

}
