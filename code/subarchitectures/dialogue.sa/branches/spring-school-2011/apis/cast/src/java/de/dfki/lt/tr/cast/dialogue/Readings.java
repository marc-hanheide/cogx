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
import cast.architecture.*;
import cast.cdl.*;
import cast.core.CASTData;
import cast.core.CASTUtils;
import cast.AlreadyExistsOnWMException;
import cast.SubarchitectureComponentException;

// Dialogue API CAST
import de.dfki.lt.tr.cast.ProcessingData;

// Dialogue API slice
import de.dfki.lt.tr.dialogue.slice.lf.*;
import de.dfki.lt.tr.dialogue.slice.ref.*;

// Dialogue API
import de.dfki.lt.tr.dialogue.util.*;
import de.dfki.lt.tr.dialogue.ref.ReferentialReadings;

// Java
import java.util.Hashtable;
import java.util.Iterator;
import java.util.ListIterator;
import java.util.Vector;


/**
 * The <tt>Readings</tt> class implements a CAST component around 
 * a referential readings engine <tt>ReferentialReadings</tt>. The 
 * engine uses a collection of factories to produce sets of readings
 * for a given logical form. Factories are registered for specific
 * ontological sorts of nominals in a logical form. Accordingly, 
 * for a given nominal of the applicable sort, a factory then generates
 * possible restrictive and attributive readings. These readings are
 * provided as a <tt>RefReadings</tt> data structure. 
 * 
 * <h4>CAST-file compponent configuration options </h4>
 * <ul>
 * <li> 
 * 
 * </ul>
 * 
 * @see de.dfki.lt.tr.dialogue.ref.ReferentialReadings
 * @see de.dfki.lt.tr.dialogue.slice.ref.RefReadings
 * @see de.dfki.lt.tr.dialogue.slice.ref.RefReading
 * @author  Geert-Jan M. Kruijff (gj@dfki.de)
 * @version 100611
 */

public class Readings 
		extends ManagedComponent
{

	ReferentialReadings readingsConstructor; 	


	// Hashtable used to record the tasks we want to carry out. For each
	// taskID we store a Vector with the data it is to work on
	private Hashtable<String, ProcessingData> m_proposedProcessing = new Hashtable<String, ProcessingData>();
	
	// Hashtable linking data IDs to goal IDs
	private Hashtable<String, String> m_dataToProcessingGoalMap = new Hashtable<String, String>();
	
	// Hashtable linking task IDs to task types
	private Hashtable<String, String> m_taskToTaskTypeMap = new Hashtable<String, String>();
	
	// Vector with objects to be processed,
	// can be ComSys:PhonString,...
	private Vector<ProcessingData> m_dataObjects = new Vector<ProcessingData>();
	
	// Counter for ProcessingData identifiers
	private int pdIdCounter;	
	
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

	
	
	/** 
	The start method registers a change listener for LogicalForm objects. 
	 */ 
	
	@Override
	public void start() {
		super.start();
		init();
		// Change filters for caches
		addChangeFilter(
						ChangeFilterFactory.createLocalTypeFilter(LogicalForm.class,  WorkingMemoryOperation.ADD),
						new WorkingMemoryChangeReceiver() {
						public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleLogicalForm(_wmc);
						}
						});
	} // end start
	
	public void init () { 
		readingsConstructor = new ReferentialReadings();
	} // end method init
	
	
	private void handleLogicalForm (WorkingMemoryChange _wmc) {
		try {
			// get the id of the working memory entry
			String id = _wmc.address.id;
			// get the data from working memory
			CASTData lfWM = getWorkingMemoryEntry(id);
			LogicalForm lf = (LogicalForm) lfWM.getData();	
	
			log("Starting task to determine the referential readings for the given LF");
			// Create an id
			String taskID = newTaskID();
			// store the data we want to process for later
			ProcessingData pd = new ProcessingData(newProcessingDataId());
			pd.add(lfWM);
			m_proposedProcessing.put(taskID, pd);
			// set up the goal
			String taskGoal;
			taskGoal = DialogueGoals.REFERENTIALREADINGS_TASK;
			// then ask for permission
			proposeInformationProcessingTask(taskID, taskGoal);
			// store the id with the task type
			m_taskToTaskTypeMap.put(taskID, taskGoal);
			
		} catch (SubarchitectureComponentException e) {
			e.printStackTrace();
		} // end try..catch		
	} // end method
	
	
	/**
	 * Executes the task of generating readings for a given logical form. The readings 
	 * are added to working memory as a <tt>RefReadings</tt> object. 
	 * 
	 * @param pd	A processing data object that contains the LogicalForm to be processed
	 * @param readingsTask	The task description 
	 * @throws DialogueException	Thrown if the processing data does not contain a LogicalForm
	 * @throws AlreadyExistsOnWMException	Thrown by CAST
	 */
	
	public void executeReadingsTask(ProcessingData pd, String readingsTask)
	throws DialogueException, AlreadyExistsOnWMException 
	{
		log("Starting to create dialogue move");
		// Get the cache 
		CASTData data = pd.getByType(CASTUtils.typeName(LogicalForm.class));
		if (data != null) {
				RefReadings readings = readingsConstructor.constructReadings((LogicalForm)data.getData());
				assert readings != null; 
				addToWorkingMemory(newDataID(),	readings);
				log("referential readings successfully added to working memory");
			
		} else {
			throw new DialogueException(
									  "Error: referential readings task on illegal processing data type"
									  + pd.getTypes());
		} // end if..else type-check on data item
		
	} // end executeReadingTask
	
	
	/**
	 * Main processing loop called by CAST. The loop operates on REFERENTIALREADINGS_TASK tasks. 
	 */
	
	@Override
	public void runComponent() {
		try {
			log("Entering loop checking for data in referential readings component");
			while (this.isRunning()) {
				// lock from external access
				lockComponent();
				// check (synchronised) processing data objects queue
				ListIterator<ProcessingData> i = m_dataObjects.listIterator();
				while (i.hasNext()) {
					ProcessingData data = i.next();
					String dataID = data.getID();
					String taskID = m_dataToProcessingGoalMap.get(dataID);
					String taskType = (String) m_taskToTaskTypeMap.get(taskID);
					log("For data [" + dataID + "/" + data.getTypes()+ "] do [" + taskID + "/" + taskType + "]");
					if (taskType != null && data != null) {
						// execution tasks throw comsys exceptions
						try {
							if (taskType
								.equals(DialogueGoals.REFERENTIALREADINGS_TASK)) {
								executeReadingsTask(data, taskType);
							}
						}
						catch (DialogueException e) {
							log("Exception while executing a task in dialogue interpretation: "
								+ e.getMessage());
							// inform the goal manager that the task has
							// been completed, but unsuccessfully
							// we may want to make this more specific
							//try {
							taskComplete(
										 taskID,
										 TaskOutcome.ProcessingCompleteFailure);
						} // end try..catch for processing exceptions
					}
					else {
						log("Nothing to process: taskType / data null");
					} // end
					// Clean up!
					// no matter what happened, remove the data item
					// from the queue
					i.remove();
					m_taskToTaskTypeMap.remove(taskID);
				} // end while
				// Free the process
				unlockComponent();
                sleepComponent(20);
				// wait for new tasks!
				waitForNotifications();
			} // end while running
		}
		catch (Exception e) {
			e.printStackTrace();
		} // end try..catch
	} // end method runComponent

	private String newProcessingDataId() {
		String result = "pd" + pdIdCounter;
		pdIdCounter++;
		return result;
	} // end newProcessingDataId
	
	

} // end class

