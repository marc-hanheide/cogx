//=================================================================
//Copyright (C) 2006 Geert-Jan M. Kruijff (gj@dfki.de)

//This library is free software; you can redistribute it and/or
//modify it under the terms of the GNU Lesser General Public License
//as published by the Free Software Foundation; either version 2.1 of
//the License, or (at your option) any later version.

//This library is distributed in the hope that it will be useful, but
//WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
//Lesser General Public License for more details.

//You should have received a copy of the GNU Lesser General Public
//License along with this program; if not, write to the Free Software
//Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
//02111-1307, USA.
//=================================================================

//=================================================================
//PACKAGE DEFINITION
//=================================================================

package comsys.components.dialogue;

//=================================================================
//IMPORTS
//=================================================================

//-----------------------------------------------------------------
//JAVA IMPORTS
//-----------------------------------------------------------------

import java.util.*;

import comsys.datastructs.comsysEssentials.*;
import comsys.arch.*;
import comsys.utils.*;
import comsys.processing.dialogue.*;
import comsys.processing.parse.ActiveIncrCCGParser;

import comsys.datastructs.lf.*;
import comsys.lf.utils.ArrayIterator;
import comsys.lf.utils.LFUtils;

import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.architecture.ManagedComponent;
import cast.SubarchitectureComponentException;
import cast.cdl.*;
import cast.core.CASTData;
import cast.core.CASTUtils;

//=================================================================
//CLASS DOCUMENTATION
//=================================================================

/**
 * Discourse Referent Bindings
 */

public class cc_DiscRefBindingsComponent extends ManagedComponent {

	// =================================================================
	// CLASS-GLOBAL DATA STRUCTURES
	// =================================================================

	// ----------------------------------------------------------------
	// INFORMATION PROCESSING DATA STRUCTURES
	// ----------------------------------------------------------------

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

	// ----------------------------------------------------------------
	// DATA STRUCTURES FOR THE DISCOURSE REFERENTS BINDING
	// ----------------------------------------------------------------

	// Dialogue context model
	private SDRS dialctxt;

	// Counter for DiscRefBindings identifiers
	private int bindingsIdCounter;

	// Counter for LFBinding identifiers
	private int lfBindingIdCounter;

	// Counter for ProcessingData identifiers
	private int pdIdCounter;
	
	// cache not yet commited to the SDRS
	private Cache uncommittedCache = new Cache();

	// ----------------------------------------------------------------
	// DIALOGUE INTERPRETATION ENGINES
	// ----------------------------------------------------------------

	// Engine for contextual reference resolution
	ContextualReferenceResolver ctxtRefResolver;


	// =================================================================
	// CONSTRUCTOR METHODS
	// =================================================================

	/**
	 * The unary constructor
	 * 
	 * @param _id
	 */
/**	public cc_DiscRefBindingsComponent() {

	//	init();
	} // constructor/1 */

	/**
	 * The method <i>init</i> initializes the global data structures,
	 * and sets the ontology for this component.
	 * 
	 * @see #configure
	 */

	private void init() {
		log("Initializing discRefBindings component");
		// nah: making all the comsys queue changes... don't want to
		// miss a thing
		m_queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;

		// general information processing structures
		m_proposedProcessing = new Hashtable<String, ProcessingData>();
		m_dataToProcessingGoalMap = new Hashtable<String, String>();
		m_taskToTaskTypeMap = new Hashtable<String, String>();
		m_dataObjects = new Vector<ProcessingData>();
		pdIdCounter = 0;
		// dialogue interpretation data structures
		bindingsIdCounter = 0;
		lfBindingIdCounter = 0;

	} // init


	// =================================================================
	// TASK METHODS
	// =================================================================

	/**
	 * The method <i>taskAdopted</i> processes a dialogue-level
	 * interpretation task once the task manager has informed the
	 * component it can run. The method pushes the processing data for a
	 * given task onto the m_dataObjects queue, so that the
	 * runComponent() method can spot something needs to be done. The
	 * method does not distinguish between different types of tasks.
	 * <p>
	 * This method does <b>not</b> inform the task manager, whether the
	 * task has been succesfully completed. This notification happens in
	 * the <i>runComponent</i> method.
	 * 
	 * @see #runComponent
	 * @see cast.architecture.subarchitecture.ManagedProcess#taskAdopted(java.lang.String)
	 */
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
	public void start() {

		super.start();

		init();
		
		try {


			// the SDRS structure (created and updated by DialInt) is necessary to 
			// initialize the context reference resolver.
			
			// Since the structure may not have been added yet, we use a change filter
			addChangeFilter(
					ChangeFilterFactory.createLocalTypeFilter(SDRS.class,  WorkingMemoryOperation.ADD),
					new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					addedSDRS(_wmc);
				}
			});
			
			// In case the data is already there
			CASTData[] Data = getWorkingMemoryEntries(SDRS.class, 0);
			if (Data.length ==1) {
				SDRS sdrs = (SDRS) Data[0].getData() ;
				ctxtRefResolver = new ContextualReferenceResolver(sdrs);
				log("SDRS structure successfully retrieved");
			}
			else {
				log("No SDRS structure found in working memory yet");
			}
			
			addChangeFilter(
					ChangeFilterFactory.createLocalTypeFilter(PackedLFs.class,  WorkingMemoryOperation.ADD),
					new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					addedPackedLF(_wmc);
				}
			});
			addChangeFilter(
					ChangeFilterFactory.createLocalTypeFilter(PackedLFs.class,  WorkingMemoryOperation.OVERWRITE),
					new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
			addedPackedLF(_wmc);
				}
			});

		}

		catch (SubarchitectureComponentException e) {
			e.printStackTrace();
		}

	} // end start

	
	


	/**
	 * The SDRS structure is necessary to initialize the context
	 * reference resolver
	 * @param _wmc
	 */
	private void addedSDRS(WorkingMemoryChange _wmc) {

		try {
			// get the id of the working memory entry
			String id = _wmc.address.id;
			// get the data from working memory and store it
			// with its id
			CASTData plfWM = getWorkingMemoryEntry(id);
			SDRS sdrs = (SDRS) plfWM.getData();
			
			ctxtRefResolver = new ContextualReferenceResolver(sdrs);
			ctxtRefResolver.setLogging(m_bLogOutput);
			log("SDRS structure successfully retrieved");
		}
		catch (SubarchitectureComponentException e) {
			e.printStackTrace();
		} // end try..catch
	}
	

	/**
	 * @param _wmc
	 * @param i
	 */
	private void addedPackedLF(WorkingMemoryChange _wmc) {
		// if we have a logical form collection string,
		try {
			// get the id of the working memory entry
			String id = _wmc.address.id;
			// get the data from working memory and store it
			// with its id
			CASTData plfWM = getWorkingMemoryEntry(id);
			PackedLFs plf = (PackedLFs) plfWM.getData();
			
			if (plf.type.equals("interpretation")) {
			log("[DiscRefBinding] Handling PLF with finalized flag ["+plf.finalized+"]");

			//if (plf.finalized < ActiveIncrCCGParser.FINAL_PRUNING) { 
				String taskID = newTaskID();
				// store the data we want to process for later
				ProcessingData pd = new ProcessingData(
						newProcessingDataId());
				pd.add(plfWM);
				m_proposedProcessing.put(taskID, pd);
				// set up the goal
				String taskGoal;
				taskGoal = ComsysGoals.DISCREFBINDING_TASK;
				// then ask for permission
				proposeInformationProcessingTask(taskID, taskGoal);
				// store the id with the task type
				m_taskToTaskTypeMap.put(taskID, taskGoal);
			//} // end if .. check whether still to do discref binding
		}
		}
		catch (SubarchitectureComponentException e) {
			e.printStackTrace();
		} // end try..catch
	}

	/**
	 * The method <i>executeDiscRefBindingTask</i> processes an
	 * LFCollection and proposes, for each logical form in the
	 * collection, a set of possible bindings to (new) discourse
	 * referents for the variables in the collection.
	 * <h4>Notes</h4>
	 * <b>1.</b> To get a properly incremental binding procedure, we
	 * need to retrieve previously established bindings, and extend
	 * those -- where necessary, as some potential bindings may have
	 * become obsolete if the corresponding logical form has been
	 * pruned. This relies on two important factors: the possibility to
	 * retrieve the previous version of the bindings for an
	 * LFCollection, and to ensure that identifiers for logical forms
	 * are persistent across versions of LFCollections.
	 * 
	 * @param data
	 *            The Comsys:LFCollection data item with LFs
	 * @param bindingTask
	 *            The exact task to be performed
	 */

	public void executeDiscRefBindingTask(ProcessingData pd,
			String bindingTask)
	throws ComsysException {
		
		CASTData data = pd
			.getByType(CASTUtils.typeName(PackedLFs.class));
		if (data != null) {
			// get the actual collection
			PackedLFs plf = (PackedLFs) data.getData();

			Cache drBindings = constructBindings(plf);
			
			if (drBindings != null) {
			// Characterize the cache explicitly as discourse referent bindings
			drBindings.cacheType = CASTUtils.typeName(Cache.class);

			// now try to add the resulting discourse referent bindings
			// entry to working memory
			if (drBindings.content1 != null && drBindings.content1.length>0 ) {
				
				uncommittedCache = drBindings;
				log("Number of discourse referent bindings for PackedLF ["
						+ plf.packedLF.packedLFId
						+ "]: "
						+ java.lang.reflect.Array
						.getLength(drBindings.content1));
				// next, push the result into working memory
				log("Adding DiscRefBindings to working memory");
				
				try {
					CASTData[] Data = getWorkingMemoryEntries(SDRS.class, 0);
					SDRS newSDRS = (SDRS) Data[0].getData() ;
					
					// addToWorkingMemory(newDataID(),	ComsysOntology.CACHE_TYPE, drBindings);
					addToWorkingMemory(newDataID(),	drBindings); // refactored, data type determined from provided object
				
					if (plf.finalized == ActiveIncrCCGParser.FINAL_PARSE) {
						uncommittedCache = new Cache();
					}
					
					log("DiscRefBindings successfully added to working memory");
					
				}
				catch (Exception e) {
					e.printStackTrace();
					throw new ComsysException(e.getMessage());
				} // end try..catch overwriting working memory
			}
			else {
				log("No referents bound for PackedLF");
			}
		}
	}
		else {
			throw new ComsysException(
					"Error: dialogue interpretation task on illegal processing data type"
					+ pd.getTypes());
		} // end if..else type-check on data item
		
	} // end executeDiscRefBindingTask



	// =================================================================
	// COMPUTATION METHODS
	// =================================================================

	/** Returns a new identifier for a DiscRefBindings object */

	private String newDRBindingsId() {
		String result = "" + bindingsIdCounter + "";
		bindingsIdCounter++;
		return result;
	} // end newDRBindingsId


	/** Returns a new identifier for a ProcessingData object */

	private String newProcessingDataId() {
		String result = "pd" + pdIdCounter;
		pdIdCounter++;
		return result;
	} // end newProcessingDataId



	/**
	 * The method <i>constructLFBindings</i> constructs the discourse
	 * referent bindings, for a given packed logical form object, These
	 * bindings may be either new discourse referents, or anchorings to
	 * already existing discourse referents.
	 * <p>
	 * For each variable in the logical form, a Binding object is
	 * construed. These Binding objects are then gathered in an
	 * LFBinding object, which is returned (with a unique identifier).
	 * 
	 * @param lf
	 *            The logical form (ParsingLF object) to be bound
	 * @return LFBindings The object with the discourse referent
	 *         bindings
	 */

	public Cache constructBindings(PackedLFs plf) {

		// transfer into the namespace of the context model
		// BoundPLF boundlf = dialctxt.lfNamespaceTransfer(plf);
		// Next we need to resolve contextual references

		try {
			// We first update the context model
			CASTData[] Data = getWorkingMemoryEntries(SDRS.class, 0);
			
			if (Data.length > 0 ) {
			SDRS newSDRS = (SDRS) Data[0].getData() ;
			ctxtRefResolver.setContextModel(newSDRS);
			if (SDRSUtils.getLastFormula(newSDRS) !=null && SDRSUtils.getLastFormula(newSDRS).caches != null) 
				log("number of caches in last formula SDRS:" + (SDRSUtils.getLastFormula(newSDRS).caches.length));
			ctxtRefResolver.setUncommittedCache(uncommittedCache);
			// And start the resolution 
			Cache bindings = ctxtRefResolver.resolve(plf);
			log("discourse referents resolution done");
			return bindings;
			}
			else {
				log("WARNING, no SDRS structure in working memory. Aborting. ");
				return null;
			}
		}
		catch (Exception e) {
			e.printStackTrace();
			return null;
		} // end try..catch overwriting working memory

		/**  if (plf.finalized == ActiveIncrCCGParser.FINAL_PRUNING) {
        	// TEMPORARY: here we update the discourse model
        	try {
        		CASTData[] Data = getWorkingMemoryEntries(ComsysOntology.SDRS_TYPE, 0);
        		SDRS sdrs = (SDRS) Data[0].getData() ;
        		SDRSFormula form = new SDRSFormula();
        		form.label = SDRSUtils.generateLabel();
        		form.type = new SDRSType();
        		form.type.plf(plf);
        		SDRSUtils.addCacheToFormula(form, bindings);
        		sdrs = SDRSUtils.addFormula(sdrs, form);
        		overwriteWorkingMemory(Data[0].getID(),ComsysOntology.SDRS_TYPE, sdrs);
        		log("SDRS in working memory OVERWRITTEN");
        	}
        	catch (Exception e) {
        		e.printStackTrace();
        	} // end try..catch overwriting working memory
        }
		 */
	} // end constructLFBindings



	// =================================================================
	// RUN METHODS
	// =================================================================

	/**
	 * The method <i>runComponent</i> cycles over the queue with
	 * processing data objects (for which tasks have been proposed),
	 * checking whether tasks can be executed. If there is a processing
	 * data object on the queue, the appropriate task is executed, and
	 * the task manager is informed of task completion (including
	 * success / failure).
	 * 
	 * @see cast.core.components.CASTComponent#runComponent()
	 */
	@Override
	public void runComponent() {
		try {
			log("Entering loop checking for data in dialint component");
			while (this.isRunning()) {
				// lock from external access
				lockComponent();
				// check (synchronised) processing data objects queue
				ListIterator<ProcessingData> i = m_dataObjects
				.listIterator();
				// for the data on the queue, tasks have been approved,
				// so
				// iterate over data objects, and execute the
				// appropriate tasks.
				// we retrieve the task to be executed based on the type
				// of the approved
				// task, linked thru taskID/dataID.
				while (i.hasNext()) {
					// get the data from the queue
					ProcessingData data = i.next();
					// check its ID, get the corresponding task ID, and
					// the task type
					String dataID = data.getID();
					String taskID = m_dataToProcessingGoalMap
					.get(dataID);
					String taskType = (String) m_taskToTaskTypeMap
					.get(taskID);
					log("For data [" + dataID + "/" + data.getTypes()
							+ "] do [" + taskID + "/" + taskType + "]");
					// make sure we know what to do, and have something
					// to do it with
					if (taskType != null && data != null) {
						// execution tasks throw comsys exceptions
						try {
							if (taskType
									.equals(ComsysGoals.DISCREFBINDING_TASK)) {
								executeDiscRefBindingTask(data,
										taskType);
							}
							else {
								log("Unknown task type to process in Comsys:DialInt component");
							} // end if..else check for task type
							// inform the goal manager that the task has
							// been completed, but unsuccessfully
								taskComplete(
										taskID,
										TaskOutcome.ProcessingCompleteSuccess);
						}
						catch (ComsysException e) {
							log("Exception while executing a task in dialogue interpretation: "
									+ e.getMessage());
							// inform the goal manager that the task has
							// been completed, but unsuccessfully
							// we may want to make this more specific
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
				// waitForNotifications(m_dataObjects);
				waitForNotifications();

			} // end while running
		}
		catch (Exception e) {
			e.printStackTrace();
		} // end try..catch
	} // end runComponent

	/**
	 * The <i>configure</i> method overrides the method from
	 * CASTProcessingComponent (though calls the super methods to ensure
	 * any higher-level configuration is done), and looks for a
	 * command-line argument <tt>--digrammar</tt> specifying the
	 * grammar networks for dialogue interpretation. The grammar is
	 * loaded in this method.
	 * 
	 * @see cast.core.components.CASTProcessingComponent#configure(java.util.Properties)
	 */
	public void configure(Properties _config) {
//		_config.list(System.out);
		if (_config.containsKey("--log")) {
			String logFlag = _config.getProperty("--log");
			if (logFlag.equals("") | logFlag.equals("true")) {
				m_bLogOutput = true;
			}
			else {
				m_bLogOutput = false;
			} // end if..else
		} // end if..else check for command-line arguments

	} // end configure

} // end class

