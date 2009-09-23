//
//  cc_ReferentialBindings.java
//  
//
//  Created by Geert-Jan Kruijff on 9/22/09.
//  Copyright 2009 __MyCompanyName__. All rights reserved.
//

package comsys.components.reference;

//-----------------------------------------------------------------
// BINDER IMPORTS
//-----------------------------------------------------------------

import binder.abstr.AbstractBindingPredictor;
import binder.autogen.core.Union;
import binder.autogen.specialentities.PhantomProxy;

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
import comsys.datastructs.comsysEssentials.*;
import comsys.datastructs.lf.*;
import comsys.processing.reference.ProxyResults;
import comsys.processing.reference.ReferentialReadings;
import comsys.processing.reference.RestrictorProxyConstruction;
import comsys.utils.datastructs.*;

import comsys.lf.utils.ArrayIterator; 
import comsys.lf.utils.LFUtils; 

//-----------------------------------------------------------------
// JAVA IMPORTS
//-----------------------------------------------------------------
import java.util.Hashtable;
import java.util.Iterator;
import java.util.ListIterator;
import java.util.Vector;




public class cc_ReferentialBindings 
		extends AbstractBindingPredictor
{

	
	// Hashtable used to record the tasks we want to carry out. For each
	// taskID we store a Vector with the data it is to work on
	private Hashtable<String, ProcessingData> m_proposedProcessing;
	
	// Hashtable linking data IDs to goal IDs
	private Hashtable<String, String> m_dataToProcessingGoalMap;
	
	// Hashtable linking task IDs to task types
	private Hashtable<String, String> m_taskToTaskTypeMap;
	
	// Vector with objects to be processed,
	// can be ComSys:PhonString,...
	private Vector<ProcessingData> m_dataObjects;	
	
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
	 The start method registers a change listener for RefReadings objects. 
	 */ 
	
	@Override
	public void start() {
		super.start();
		init();
		// Change filters for caches
		addChangeFilter(
						ChangeFilterFactory.createLocalTypeFilter(RefReadings.class,  WorkingMemoryOperation.ADD),
						new WorkingMemoryChangeReceiver() {
						public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleRefReadings(_wmc);
						}
						});
	} // end start
	
	
	public void init () { 

	} // end method init
	
	private void handleRefReadings (WorkingMemoryChange _wmc) {
		try {
			// get the id of the working memory entry
			String id = _wmc.address.id;
			// get the data from working memory
			CASTData rrWM = getWorkingMemoryEntry(id);
			RefReadings lf = (RefReadings) rrWM.getData();	
			
			log("Starting task to determine the referential bindings for the available restrictive readings");
			// Create an id
			String taskID = newTaskID();
			// store the data we want to process for later
			ProcessingData pd = new ProcessingData(newProcessingDataId());
			pd.add(rrWM);
			m_proposedProcessing.put(taskID, pd);
			// set up the goal
			String taskGoal;
			taskGoal = ComsysGoals.REFERENTIALBINDINGS_TASK;
			// then ask for permission
			proposeInformationProcessingTask(taskID, taskGoal);
			// store the id with the task type
			m_taskToTaskTypeMap.put(taskID, taskGoal);
			
		} catch (SubarchitectureComponentException e) {
			e.printStackTrace();
		} // end try..catch		
	} // end method
	
	
	public void executeBindingsTask(ProcessingData pd, String readingsTask)
	throws ComsysException, AlreadyExistsOnWMException 
	{
		log("Starting to create bindings for readings");
		// boot up the factory
		RestrictorProxyConstruction factory = new RestrictorProxyConstruction();
		// Get the cache 
		CASTData data = pd.getByType(CASTUtils.typeName(RefReadings.class));
		if (data != null) {
			// get the readings
			RefReadings readings = (RefReadings)data.getData();
			LogicalForm lf = readings.lform; 
			assert readings != null; 
			Vector<BoundReadings> boundReadings = new Vector();
			// get the restrictive readings
			for (ArrayIterator readingsIter = new ArrayIterator(readings.refRdngs); readingsIter.hasNext(); ) {
				RefReading reading = (RefReading)readingsIter.next();
				Vector<RefBinding> refBindings = new Vector();
				if (reading.restrictiveTrees != null && reading.restrictiveTrees.length > 0) { 
					for (ArrayIterator idsIter = new ArrayIterator(reading.restrictiveTrees); idsIter.hasNext(); ) { 
						String restrTreeRoot = (String)idsIter.next();
						LogicalForm restrLF = LFUtils.lfConstructSubtree(LFUtils.lfGetNominal(lf,restrTreeRoot),lf);
						ProxyResults prxResults = factory.constructProxy(restrLF);
						for (Iterator<PhantomProxy> phantIter = prxResults.getProxies(); phantIter.hasNext(); ) {
							PhantomProxy phant = phantIter.next();
							// get the unions, delete phantom afterwards
							Vector<Union> unions = getPredictedUnions(phant,true); 
							// create the anchorings
							Vector<Anchor> anchors  = new Vector();
							for (Iterator<Union> unionsIter = unions.iterator(); unionsIter.hasNext(); ) { 
								Union union = unionsIter.next();
								Anchor anchor = createAnchorFromUnion(union);
								anchors.add(anchor);
							} // end for over possible unions
							// create the reference binding, add it to the vector of bindings for this reading
							Anchor[] antecedents = new Anchor[anchors.size()];
							antecedents = (Anchor[])anchors.toArray(antecedents);
							RefBinding binding = new RefBinding();
							binding.id = "refbinding";
							binding.nomVar = restrTreeRoot;
							binding.antecedents = antecedents; 
							refBindings.add(binding);
						} // end for over phantom proxies
					} // end for over ids
				} // end if.. check whether we have restrictors
				// create the bindings object for this reading, and store it on the vector for all readings
				ReadingBindings readingBindings = new ReadingBindings();
				readingBindings.id = "readingbindings";
				RefBinding[] bindings = new RefBinding[refBindings.size()];
				bindings = (RefBinding[])refBindings.toArray(bindings);
				readingBindings.bindings = bindings;
				boundReadings.add(readingBindings);
			} // end for over readings
			// create the BoundReadings object
			BoundReadings bindings = new BoundReadings();
			bindings.id = "bindings";
			bindings.lform = lf;
			ReadingBindings[] rbindings = new ReadingBindings[boundReadings.size()];
			rbindings = (ReadingBindings[])boundReadings.toArray(rbindings);
			bindings.bindings = rbindings;
			// add bindings object to working memory
			addToWorkingMemory(newDataID(),	bindings);
			log("restrictive referential bindings successfully added to working memory");
		} else {
			throw new ComsysException(
									  "Error: referential bindings task on illegal processing data type"
									  + pd.getTypes());
		} // end if..else type-check on data item
		
	} // end executeBindingsTask
	
	
	@Override
	public void runComponent() {
		try {
			log("Entering loop checking for data in referential bindings component");
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
								.equals(ComsysGoals.REFERENTIALBINDINGS_TASK)) {
								executeBindingsTask(data, taskType);
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
	
	
	private Anchor createAnchorFromUnion (Union union) { 
		Anchor anchor = new Anchor();
		anchor.entityID = union.entityID;
		anchor.probExists = union.probExists; 
		return anchor; 
	} 
	
} // end class

