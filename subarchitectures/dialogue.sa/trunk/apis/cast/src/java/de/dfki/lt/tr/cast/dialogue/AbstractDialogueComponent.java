// =================================================================
// Copyright (C) 2009-2010 Geert-Jan M. Kruijff (gj@dfki.de)
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2.1 of
// the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
// =================================================================

// =================================================================
// PACKAGE DEFINITION
package de.dfki.lt.tr.cast.dialogue;

//=================================================================
// IMPORTS

// CAST
import java.util.Hashtable;
import java.util.ListIterator;
import java.util.Vector;

import cast.SubarchitectureComponentException;
import cast.architecture.*;
import cast.cdl.TaskOutcome;
import cast.cdl.WorkingMemoryChange;
import cast.core.CASTData;

// Dialogue API CAST
import de.dfki.lt.tr.cast.ProcessingData;
import de.dfki.lt.tr.dialogue.util.DialogueException;

/**
 * Implements all the basic CAST methods for task handling, which are
 * independent of the data structures handled by a component. The class
 * defines a <tt>runComponent</tt> method that calls <tt>executeTask</tt>
 * on a <tt>ProcessingData</tt> object. From the viewpoint of task execution,
 * any class extending this component just needs to implement this (abstract)
 * execution method. 
 * 
 * @author Geert-Jan M. Kruijff 
 * @version 100618
 * @started 100618
 */
abstract public class AbstractDialogueComponent 
extends ManagedComponent
{

	// Hashtable used to record the tasks we want to carry out. For each
	// taskID we store a Vector with the data it is to work on
	protected Hashtable<String, ProcessingData> m_proposedProcessing = new Hashtable<String, ProcessingData>();
	
	// Hashtable linking data IDs to goal IDs
	protected Hashtable<String, String> m_dataToProcessingGoalMap = new Hashtable<String, String>();
	
	// Hashtable linking task IDs to task types
	protected Hashtable<String, String> m_taskToTaskTypeMap = new Hashtable<String, String>();
	
	// Vector with objects to be processed,
	// can be ComSys:PhonString,...
	protected Vector<ProcessingData> m_dataObjects = new Vector<ProcessingData>();
	
	// Counter for ProcessingData identifiers
	protected int pdIdCounter = 0;	
	
	// Counter for identifiers
	protected int idCounter = 0;
	
	@Override
	protected void taskAdopted(String _goalID) {
		// get the data we stored for this goal
		ProcessingData pd = m_proposedProcessing.remove(_goalID);
		if (pd != null) {
			// add the data item to the data objects queue
			m_dataObjects.addElement(pd);
			// get the identifier of the processing data object
			String pdID = pd.getID();
			// link the data ID to the goal ID, for future reference
			// on task completion (done in runComponent)
			m_dataToProcessingGoalMap.put(pdID, _goalID);
		}
		else {
			log("ERROR: Goal without data: " + _goalID);
		} // end if..else
	} // end taskAdopted
	
	/**
	 * The method <i>taskRejected</i> removes a rejected task from the
	 * list of proposed processing tasks.
	 * 
	 * @see cast.architecture.subarchitecture.ManagedProcess#taskRejected(java.lang.String)
	 */
	@Override
	protected void taskRejected(String _goalID) {
		log("WARNING: The goal with ID [" + _goalID
			+ "] has been rejected.");
		m_proposedProcessing.remove(_goalID);
	} // end taskRejected	

    @Override
	public void start() 
	{
		super.start();
	}
	
    /**
     * Produces a new identifier, starting with the given prefix
     * @param   prefix 	The prefix to be used
     * @return	String	A new (unique) string identifier
     */
	
	protected String newId(String prefix) {
		String result = prefix + idCounter;
		idCounter++;
		return result;
	} // end newProcessingDataId
	
	/**
	 * Produces a new identifier for processing data.
	 * @return String	A new identifier of the form pdINT
	 */
	
	protected String newProcessingDataId() {
		String result = "pd" + pdIdCounter;
		pdIdCounter++;
		return result;
	} // end newProcessingDataId
	
	
	/**
	 * The method <i>handleWorkingMemoryChange</i> retrieves a CASTData object from the working memory, 
	 * and stores it in the <tt>m_proposedProcessing</tt> table with a processing task. 
	 * 
	 * @param _wmc The working memory change -- a recently added CASTData object
	 * @param taskGoal A goal listed in the DialogueGoals class
	 * @see de.dfki.lt.tr.cast.dialogue.DialogueGoals
	 */
	private void handleWorkingMemoryChange(WorkingMemoryChange _wmc, String taskGoal) {
		try {
			// get the id of the working memory entry
			String id = _wmc.address.id;
			// get the data from working memory and store it with its id
			CASTData data = getWorkingMemoryEntry(id);
			// get a new id for the task
			String taskID = newTaskID();
			// store the data we want to process for later
			ProcessingData pd = new ProcessingData(newProcessingDataId());
			pd.add(data);
			m_proposedProcessing.put(taskID, pd);
			// store the goal with its information
			proposeInformationProcessingTask(taskID, taskGoal);
		}
		catch (SubarchitectureComponentException e) {
			e.printStackTrace();
		} // end try..catch
	} // end handleWorkingMemoryChange		
	
	abstract public void executeTask (ProcessingData data)
	throws DialogueException;
	
	@Override
	public void runComponent () 
	{
        try {
            while (this.isRunning()) {
                lockComponent();
                ListIterator<ProcessingData> i = m_dataObjects.listIterator();
                while (i.hasNext()) {
                    ProcessingData data = i.next();
                    String dataID = data.getID();
                    String taskID = m_dataToProcessingGoalMap
                        .get(dataID);
                    log("For data [" + dataID + "/" + data.getTypes()
                        + "] do [" + taskID + "]");
                    if (data != null) {
                        try {
                            executeTask(data);
                                taskComplete(
                                    taskID,
                                    TaskOutcome.ProcessingCompleteSuccess);
                        }
                        catch (DialogueException e) {
                            log("Exception while executing a task: "
                                + e.getMessage());
                            e.printStackTrace();
                            taskComplete(
                                    taskID,
                                    TaskOutcome.ProcessingCompleteFailure);
                        } 					
                    } else {
                        log("Nothing to process: data null");
                    } // end
                    i.remove();
                } // end while
                // Free the process
                unlockComponent();
                sleepComponent(20);
                waitForNotifications();
            } // end while running
        }
        catch (Exception e) {
            e.printStackTrace();
        } // end try..catch
	}
	
} // end class
