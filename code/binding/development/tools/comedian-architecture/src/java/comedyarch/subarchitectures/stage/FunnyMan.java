/*
 * Comedian example code to demonstrate CAST functionality. Copyright
 * (C) 2006-2007 Nick Hawes This library is free software; you can
 * redistribute it and/or modify it under the terms of the GNU Lesser
 * General Public License as published by the Free Software Foundation;
 * either version 2.1 of the License, or (at your option) any later
 * version. This library is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details. You should have
 * received a copy of the GNU Lesser General Public License along with
 * this library; if not, write to the Free Software Foundation, Inc., 51
 * Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * 
 */
package comedyarch.subarchitectures.stage;

import java.util.Hashtable;
import java.util.ListIterator;
import java.util.Vector;

import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.PrivilegedManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.OperationMode;
import cast.cdl.TaskOutcome;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.data.CASTData;

import comedyarch.autogen.Joke;
import comedyarch.global.ComedyGoals;

/**
 * @author nah
 */
public class FunnyMan extends PrivilegedManagedProcess {

	// Hashtable used to record the tasks we want to carry out
	private Hashtable<String, CASTData> m_proposedProcessing;

	private Vector<CASTData> m_setups;

	/**
	 * @param _id
	 */
	public FunnyMan(String _id) {
		super(_id);
//		setOntology(ComedyOntologyFactory.getOntology());
		m_proposedProcessing = new Hashtable<String, CASTData>();
		m_setups = new Vector<CASTData>();

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.abstr.WorkingMemoryReaderProcess#start()
	 */
	@Override
	public void start() {
		super.start();

		// create a new receiver object that just calls a member
		// function from this class
		WorkingMemoryChangeReceiver receiver = new WorkingMemoryChangeReceiver() {

			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				newJokeAdded(_wmc);
			}
		};

		try {
			// add this receiver to listen for changes
			addChangeFilter(
			// listen for joke types
					// that are added
					// that are local to this subarchitecture
					// the receiver object
					ChangeFilterFactory.createLocalTypeFilter(Joke.class,
							WorkingMemoryOperation.ADD), receiver);

			// // echo receiver, just to demonstrate multiple receiver
			// // functionality
			// addChangeFilter(
			// // listen for joke types
			// ComedyOntology.JOKE_TYPE,
			// // that are added
			// WorkingMemoryOperation.ADD,
			// // that are local to this subarchitecture
			// FilterRestriction.LOCAL_SA,
			// // the receiver object
			// new WorkingMemoryChangeReceiver() {
			//
			// public void workingMemoryChanged(WorkingMemoryChange _wmc) {
			// println(CASTUtils.toString(_wmc));
			// }
			// });
			//
			// // For testing, ignore
			// addChangeFilter(ComedyOntology.JOKE_TYPE,
			// WorkingMemoryOperation.OVERWRITE, FilterRestriction.LOCAL_SA,
			// receiver);
			// addChangeFilter(ComedyOntology.JOKE_TYPE,
			// WorkingMemoryOperation.DELETE, FilterRestriction.ALL_SA,
			// receiver);
			//            
			// removeChangeFilter(ComedyOntology.JOKE_TYPE,
			// WorkingMemoryOperation.ADD, FilterRestriction.LOCAL_SA);
			// removeChangeFilter(ComedyOntology.JOKE_TYPE,
			// WorkingMemoryOperation.OVERWRITE, FilterRestriction.LOCAL_SA);
			// removeChangeFilter(ComedyOntology.JOKE_TYPE,
			// WorkingMemoryOperation.DELETE, FilterRestriction.ALL_SA);
			//   
			// addChangeFilter(ComedyOntology.JOKE_TYPE,
			// WorkingMemoryOperation.ADD, FilterRestriction.LOCAL_SA,
			// receiver);

		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.subarchitecture.ManagedProcess#taskAdopted(java.lang.String)
	 */
	@Override
	protected void taskAdopted(String _goalID) {
		// wooo! we're allowed to act

		// get the data we stored for this goal...
		CASTData<?> data = m_proposedProcessing.remove(_goalID);

		// if we have stored this goal earlier
		if (data != null) {
			// we could call quip(data) directly, but let's queue the
			// data instead... it's always good to build some suspense
			m_setups.add(data);
		} else {
			println("oh, this is my goal, but I have no data: " + _goalID);
		}

		// and now we're finished, tell the goal manager that the task
		// is over successfully (assuming it is... naughty!)
		try {
			taskComplete(_goalID, TaskOutcome.PROCESSING_COMPLETE_SUCCESS);
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		}
	}

	private String generatePunchline(String _setup) {
		return "A stick!";
	}

	/**
	 * @param _data
	 */
	private void quip(CASTData<?> _data) {
		Joke jk = (Joke) _data.getData();
		// work out a smarty-pants punchline
		String punchline = generatePunchline(jk.m_setup);
		// time it right
		log("*cough*");
		jk.m_punchline = punchline;
		// now write this back into working memory
		try {
			overwriteWorkingMemory(_data.getID(), jk,
					OperationMode.BLOCKING);

			// for testing.. should be 0
			// println("joke has been overwritten " +
			// getOverwriteCount(_data.getId()) + " times");

		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		}

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.subarchitecture.ManagedProcess#taskRejected(java.lang.String)
	 */
	@Override
	protected void taskRejected(String _goalID) {
		println(":(");
		m_proposedProcessing.remove(_goalID);
	}

	/**
	 * @param _wmc
	 * @param i
	 */
	private void newJokeAdded(WorkingMemoryChange _wmc) {
		try {

			// //for testing.. should be 0
			// println("joke has been overwritten " +
			// getOverwriteCount(_wmc.m_address) + " times");
			//            
			// get the id of the working memory entry
			String id = _wmc.m_address.m_id;

			// get the data from working memory and store it
			// with its id
			CASTData<?> jokeData = getWorkingMemoryEntry(id);

			// test
			// CASTTypedData[] data =
			// getWorkingMemoryEntries(new String[]{id});

			//                
			// CASTTypedDataWithID[] jokes =
			// getWorkingMemoryEntries(
			// _wmc[i].m_type, 0);
			//
			// CASTTypedDataWithID jokeData = jokes[0];

			// endtest

			// because we've checked the ontological type above,
			// this should be safe
			Joke jk = (Joke) jokeData.getData();

			// now see if we can provide a punchline
			if (jk.m_punchline.equals("")) {
				// in this case we need to propose a task to do
				// some processsing

				// get a new id for the task
				String taskID = newTaskID();

				// store the data we want to process for
				// later... we could just store the id in
				// working memory if we wanted to, I guess it
				// depends on storage/transport tradeoffs
				m_proposedProcessing.put(taskID, jokeData);

				// then ask for permission
				proposeInformationProcessingTask(taskID,
						ComedyGoals.ADD_PUNCHLINE_TASK);

				// then immediately retract!
				// retractInformationProcessingTask(taskID);

			} else {
				// nothing to do then :(
			}

		} catch (SubarchitectureProcessException e) {
			println(e.getLocalizedMessage());
			e.printStackTrace();
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.components.CASTComponent#runComponent()
	 */
	@Override
	public void runComponent() {
		// try {

		// TaskDescription desc = new
		// TaskDescription(ComedyGoals.ADD_PUNCHLINE_TASK);
		// registerTaskDescriptions(new TaskDescription[]{desc});

		while (m_status == ProcessStatus.RUN) {
			// do nothing for a while
			// Thread.sleep(1000);
			waitForNotifications();

			// lock from external access
			lockProcess();

			// must check we're still running after sleep!
			if (m_status == ProcessStatus.RUN) {

				// check (synchronised) joke queue
				ListIterator<CASTData> i = m_setups.listIterator();

				// see what's in there
				while (i.hasNext()) {

					// quick, tell the punchline
					quip(i.next());

					// remove data from queue
					i.remove();

				}
			}

			// sleepProcess(15000);

			// let other stuff happen if necessary
			unlockProcess();
		}
	}

}
