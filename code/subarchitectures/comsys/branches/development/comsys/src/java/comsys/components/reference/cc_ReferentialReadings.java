//
//  cc_ReferentialReadings.java
//  
//
//  Created by Geert-Jan Kruijff on 9/21/09.
//  Copyright 2009 __MyCompanyName__. All rights reserved.
//

package comsys.components.reference;

//-----------------------------------------------------------------
// CAST IMPORTS
//-----------------------------------------------------------------
import cast.architecture.*;
import cast.cdl.*;
import cast.core.CASTData;
import cast.core.CASTUtils;
import cast.AlreadyExistsOnWMException;
import cast.SubarchitectureComponentException;

//-----------------------------------------------------------------
// COMSYS IMPORTS
//-----------------------------------------------------------------
import comsys.arch.*;
import comsys.datastructs.SelectedLogicalForm;
import comsys.datastructs.comsysEssentials.*;
import comsys.processing.reference.ReferentialReadings;
import comsys.utils.datastructs.*;


//-----------------------------------------------------------------
// JAVA IMPORTS
//-----------------------------------------------------------------
import java.util.Hashtable;
import java.util.Iterator;
import java.util.ListIterator;
import java.util.Vector;

//-----------------------------------------------------------------
// LOGICAL FORM IMPORTS
//-----------------------------------------------------------------
import comsys.datastructs.lf.*;




public class cc_ReferentialReadings 
		extends ManagedComponent
{

	ReferentialReadings readingsConstructor; 	


	// Hashtable used to record the tasks we want to carry out. For each
	// taskID we store a Vector with the data it is to work on
	private Hashtable<String, ProcessingData> m_proposedProcessing;
	
	// Hashtable linking data IDs to goal IDs
	private Hashtable<String, String> m_dataToProcessingGoalMap;
	
	// Hashtable linking task IDs to task types
	private Hashtable<String, String> m_taskToTaskTypeMap;
	
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
						ChangeFilterFactory.createLocalTypeFilter(SelectedLogicalForm.class,  WorkingMemoryOperation.ADD),
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
			taskGoal = ComsysGoals.REFERENTIALREADINGS_TASK;
			// then ask for permission
			proposeInformationProcessingTask(taskID, taskGoal);
			// store the id with the task type
			m_taskToTaskTypeMap.put(taskID, taskGoal);
			
		} catch (SubarchitectureComponentException e) {
			e.printStackTrace();
		} // end try..catch		
	} // end method
	
	
	public void executeReadingsTask(ProcessingData pd, String readingsTask)
	throws ComsysException, AlreadyExistsOnWMException 
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
			throw new ComsysException(
									  "Error: referential readings task on illegal processing data type"
									  + pd.getTypes());
		} // end if..else type-check on data item
		
	} // end executeReadingTask
	
	
	
	
	
	
	
	
	
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
								.equals(ComsysGoals.REFERENTIALREADINGS_TASK)) {
								executeReadingsTask(data, taskType);
							}
						}
						catch (ComsysException e) {
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

