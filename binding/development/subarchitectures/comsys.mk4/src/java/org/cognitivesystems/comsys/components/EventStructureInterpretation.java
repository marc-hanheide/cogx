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

package org.cognitivesystems.comsys.components;

//=================================================================
//IMPORTS
//=================================================================

//-----------------------------------------------------------------
// CAST IMPORTS
//-----------------------------------------------------------------
import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.ManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.*;
import cast.core.data.CASTData;

//-----------------------------------------------------------------
// COMSYS IMPORTS
//-----------------------------------------------------------------
import org.cognitivesystems.comsys.autogen.ComsysEssentials.*;
import org.cognitivesystems.comsys.data.*;
import org.cognitivesystems.comsys.general.ComsysException;
import org.cognitivesystems.comsys.general.ComsysUtils;
import org.cognitivesystems.comsys.general.SDRSUtils;
import org.cognitivesystems.comsys.ontology.ComsysGoals;

import org.cognitivesystems.comsys.processing.*;
import org.cognitivesystems.comsys.processing.eventstructurefactories.ActionMotionGo;
import org.cognitivesystems.comsys.processing.eventstructurefactories.ActionMotionMoveObject;
import org.cognitivesystems.comsys.processing.eventstructurefactories.ActionMotionMovePerson;
import org.cognitivesystems.comsys.processing.eventstructurefactories.ActionMotionPut;
import org.cognitivesystems.comsys.processing.eventstructurefactories.ActionMotionTake;



//-----------------------------------------------------------------
// JAVA IMPORTS
//-----------------------------------------------------------------
import java.util.Hashtable;
import java.util.Iterator;
import java.util.ListIterator;
import java.util.Properties;
import java.util.TreeMap;
import java.util.Vector;

//-----------------------------------------------------------------
// LOGICAL FORM IMPORTS
//-----------------------------------------------------------------
import org.cognitivesystems.repr.lf.autogen.LFEssentials.LFNominal;
import org.cognitivesystems.repr.lf.autogen.LFEssentials.LogicalForm;
import org.cognitivesystems.repr.lf.utils.ArrayIterator;
import org.cognitivesystems.repr.lf.utils.LFUtils;
import org.cognitivesystems.repr.lf.autogen.LFPacking.PackedNominal;
import cast.core.CASTUtils
;
//=================================================================
//CLASS DOCUMENTATION
//=================================================================

/**
    The class <b>EventStructureInterpretation</b> provides an event 
	structure interpretation, consisting of one or more event nuclei 
	and their tempero-aspectual relationships, for a given packed 
	logical form. 
	
	The component listens for discourse	referent bindings (caches) 
	being written to working memory. The interpretation is currently 
	only provided if we are dealing with caches for 
	completed packed logical forms (no incremental interpretation, 
	though there is a flag <tt>--incrementalInterpretation</tt> which 
	takes values "true, false, 0, 1, 2" to set this feature).  
	With the set of referent bindings, the component then uses the applicable event structure
	factories to create the event structure for the packed logical form. 
 */

public class EventStructureInterpretation 
	extends ManagedProcess 
{

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
	// DATA STRUCTURES FOR EVENT STRUCTURE INTERPRETATION 
	// ----------------------------------------------------------------

	// Dialogue context model
	private SDRS dialctxt;

	// Counter for ProcessingData identifiers
	private int pdIdCounter;
	
	/** The flag to set whether interpretation should be incremental, or only done on complete PLFs.
		This flag can be set using the boolean command-line option "--incrementalInterpretation". 
	*/
	private boolean incrementalInterpretation;
	
	/** The flag which sets which completeness level a parse should have to be considered "complete". 
		This allows for a more flexible "non-incremental" binding interaction -- either after parsing
		is finished, or only after pruning is completely done. By default this level is set to 2 (
		complete after pruning). This flag can be set as numerical value to the 
		"--incrementalInterpretation" option (instead of "false", use "1" or "2"). 
	*/ 
	private int	plfCompletenessLevel; 

	/** The hashtable "factories" stores the registered event structure factories by type (key). Returned
		for a key is another hashtable, indexed by proposition. If a factory does not specify a proposition
		as filter, then it is added as "noProp"; the restriction is that at most one such factory can be 
		registered for a given sort. 
	*/
	private Hashtable factories; 

	/** constant */ 
	private final String NOPROP = "noProp";

	/** Counter for event nucleus identifiers */ 
	private int nucleusIdCounter;
	
	
	/** List of processed packed logical forms (to avoid double work), stored by Id, with their results.
		The results are (currently) a Vector of event nuclei that have been produced. 
	*/
	Hashtable plfEventStructures; 
	
	
	/** the full spatio-temporal representaiton */
	SpatioTemporalRepresentation spatioTemporalRep;
	String strId ;
	
	/** Set of words (actually, propositions) triggering specific inter-nucleus relations */
	Hashtable<String,InterNucleusRelation> interNucleusRelationTriggers ;


	// last processed nucleus
	Nucleus lastProcessedNucleus;
	// =================================================================
	// CONSTRUCTOR METHODS
	// =================================================================

	/**
	 * The unary constructor
	 * 
	 * @param _id
	 */
	public EventStructureInterpretation(String _id) {
		super(_id);
		init();
	} // constructor/1

	/**
	 * The method <i>init</i> initializes the global data structures,
	 * and sets the ontology for this component.
	 * 
	 * @see #configure
	 */

	private void init() {
		log("Initializing event structure interpretation component");
		// nah: making all the comsys queue changes... don't want to
		// miss a thing
		m_queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;

		// general information processing structures
		m_proposedProcessing = new Hashtable<String, ProcessingData>();
		m_dataToProcessingGoalMap = new Hashtable<String, String>();
		m_taskToTaskTypeMap = new Hashtable<String, String>();
		m_dataObjects = new Vector<ProcessingData>();
		pdIdCounter = 0;
		
		// event structure interpretation data structures
		incrementalInterpretation = false;
		plfCompletenessLevel = 1;
		factories = new Hashtable();
		nucleusIdCounter = 0;
		plfEventStructures = new Hashtable();
		
		lastProcessedNucleus = new Nucleus();
		

		// Register the available factories
		try { 
			this.registerFactory (new ActionMotionGo()); 
			this.registerFactory (new ActionMotionMoveObject()); 			
			this.registerFactory (new ActionMotionMovePerson()); 				
			this.registerFactory (new ActionMotionPut()); 
			this.registerFactory (new ActionMotionTake()); 		
		} catch (ComsysException e) { 
			System.err.println(e.getMessage());
			System.exit(0);
		} // end try.. catch
		
		// adding words (actually, propositions) triggering specific inter-nucleus relations 
		interNucleusRelationTriggers = new Hashtable<String,InterNucleusRelation>();
		InterNucleusRelation inr1 = new InterNucleusRelation();
		inr1.mode = "PREPARATION";
		interNucleusRelationTriggers.put("now", inr1);
		interNucleusRelationTriggers.put("then", inr1);
	} // init

	// =================================================================
	// ACCESSOR METHODS
	// =================================================================

	/** 
		The method <i>registerFactory</i> takes an event nucleus
		factory, and registers it with the class. A factory is registered
		on the basis of the ontological sort and the proposition that 
		it uses to determine event structure for a given verbal predicate. 

		@param EventNucleusFactory	factory The factory to be registered
		@throws ComsysException		Thrown if there is a problem registering
	*/ 

	public void registerFactory (EventNucleusFactory factory) 
		throws ComsysException 
	{ 
		String sort = factory.getSort();
		String prop = factory.getProposition();
		if (prop.equals("")) { prop = NOPROP; }
		Hashtable sortFactories = new Hashtable();
		Vector propInterpretations = new Vector();
		if (factories.containsKey(sort)) { 
			sortFactories = (Hashtable) factories.get(sort);
		} // end if.. check for presence of other factories
		if (sortFactories.containsKey(prop)) { 
			if (prop.equals(NOPROP)) { 
				throw new ComsysException("Cannot store factory for ["+sort+"/"+prop+"] -- already present");
			} else { 
				propInterpretations = (Vector) sortFactories.get(prop); 
				propInterpretations.addElement(factory);
				// Update the tables with stored factories
				sortFactories.put(prop,propInterpretations);
				factories.put(sort,sortFactories);
				System.out.println ("[REGISTER FACTORY] Updated stored factories for ["+sort+"/"+prop+"]");
			} // end if..else check for doubles in NOPROP
		} else { 
			propInterpretations.addElement(factory);
			// Update the tables with stored factories
			sortFactories.put(prop,propInterpretations);
			factories.put(sort,sortFactories);			
			System.out.println ("[REGISTER FACTORY] Added factory for ["+sort+"/"+prop+"]");			
		} // end if.. else check for storing
	} // end registerFactory


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


	/**		
		The start method registers ADD- and OVERWRITE listeners for PACKEDLFs and CACHEs. 
	*/ 

	@Override
	public void start() {
		super.start();

		spatioTemporalRep = new SpatioTemporalRepresentation();
		spatioTemporalRep.nuclei = new Nucleus[0];
		spatioTemporalRep.interNucleusRelations = new InterNucleusRelation[0];
        strId = newDataID();
        try {
            // addToWorkingMemory(strId, ComsysOntology.SPATIOTEMPORALREPRESENTATION_TYPE, spatioTemporalRep);
			addToWorkingMemory(strId, spatioTemporalRep); // refactored, data type determined from provided object
        }
        catch (Exception e) {
            e.printStackTrace();
        } // end try..catch overwriting working memory

		try {
			// Change filters for caches
			addChangeFilter(
					ChangeFilterFactory.createLocalTypeFilter(Cache.class,  WorkingMemoryOperation.ADD),
					new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					handleCache(_wmc);
				}
			});
			addChangeFilter(
					ChangeFilterFactory.createLocalTypeFilter(Cache.class,  WorkingMemoryOperation.OVERWRITE),
					new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					handleCache(_wmc);
				}
			});			
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		} // end try..catch
	} // end start

	/**
		The method <i>handleCache</i> retrieves a cache, and then checks whether it 
		is a cache for discourse referent bindings. 
		If it is, the method checks whether the packed logical form stored in the cache 
		has the required level of completeness. 
		If so, the method proposes a task for creating event structures. 
	
		@param _wmc	The working memory change

	 */
	private void handleCache(WorkingMemoryChange _wmc) {
		try {
			// get the id of the working memory entry
			String id = _wmc.m_address.m_id;
			// get the data from working memory
			CASTData cacheWM = new CASTData(id,
					getWorkingMemoryEntry(id));
			Cache cache = (Cache) cacheWM.getData();
			log("Checking a cache written to WM: ["+cache.cacheType+"]");			 
			// Check what kind of type we are dealing with, only deal with disc refs
			if (cache.cacheType.indexOf(CASTUtils.typeName(Cache.class)) != -1) { 
				// Check that we have a packed logical form that is complete
				PackedLFs plf = cache.plf; 
				// check the completeness level of the plf
				if (plfEventStructures.containsKey(plf.id)) { 
					// do nothing
				} else if (plf.finalized >= 1) { 
					log("Starting task to create event nucleus for a complete packed logical form ["+plf.id+"] ["+plfEventStructures.containsKey(plf.id)+"]");
					// Create an id
					String taskID = newTaskID();
					// store the data we want to process for later
					ProcessingData pd = new ProcessingData(newProcessingDataId());
					pd.add(cacheWM);
					m_proposedProcessing.put(taskID, pd);
					// set up the goal
					String taskGoal;
					taskGoal = ComsysGoals.EVENSTRUCTUREINTERPRETATION_TASK;
					// then ask for permission
					proposeInformationProcessingTask(taskID, taskGoal);
					// store the id with the task type
					m_taskToTaskTypeMap.put(taskID, taskGoal);
				} // end if.. check for completeness
			} // end if.. check for right cache type
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		} // end try..catch
	} // end handleCache
	
	// =================================================================
	// EXECUTION METHODS
	// =================================================================


	/**
		The method <i>executeInterpretatioTask</i> cycles over the nominals in the packed logical 
		form stored in the cache, and applies event structure factories if possible to create an 
		event structure for the PLF. 
	 */

	public void executeInterpretationTask(ProcessingData pd, String interpretationTask)
		throws ComsysException 
	{
		log("Starting to create event nucleus");
		// Get the cache 
		CASTData data = pd.getByType(CASTUtils.typeName(Cache.class));
		if (data != null) {
			// List of created event structure interpretation results
			Vector evIntResults = new Vector();
			// List of created inter-nucleus relation
			Vector<InterNucleusRelation> inrResults = new Vector<InterNucleusRelation>();
			// Get the cache, and the packed logical forms
			Cache drBindings = (Cache) data.getData();
			PackedLFs plf = drBindings.plf;
			log("Creating an event structure interpretation for PLF ["+plf.id+"], seen before: ["+plfEventStructures.containsKey(plf.id)+"]");
			if (!plfEventStructures.containsKey(plf.id)) { 
				// Create a map with packed nominals, for faster access
				TreeMap packedNoms = LFUtils.createNomTreeMap(plf.packedLF);
				// Applicable factory; object initialized once, then reset to null
				Vector propInterpretations = null;
				// Cycle over the packed nominals in the stored plf
				Iterator pnomsIter = packedNoms.keySet().iterator();
				while (pnomsIter.hasNext()) { 
					String nv = (String) pnomsIter.next();
					PackedNominal nom = (PackedNominal) packedNoms.get(nv);
					// Get the sort and the proposition of the nominal
					String sort = LFUtils.getPackedNominalSort(nom);
					String prop = nom.prop.prop;
					// Check whether there is a factory for the nominal 
					if (factories.containsKey(sort)) {
						log("Found a factory list for sort ["+sort+"] of nomvar ["+nom.nomVar+"]");
						Hashtable sortFactories = (Hashtable) factories.get(sort);
						
						if (sortFactories.containsKey(prop)) { 
							propInterpretations = (Vector) sortFactories.get(prop);
						} else if (sortFactories.containsKey(NOPROP)) { 
							propInterpretations = (Vector) sortFactories.get(NOPROP);
						} else { 
							propInterpretations = null ;
						} // end if.. else check whether applicable factory
						// If we have a factory, apply it
						if (propInterpretations != null) { 
							log("There is an applicable factory for the proposition ["+prop+"]");
							// Cycle over the factories in the interpretations, until we have a result
							boolean foundResult = false; 
							Iterator propIntIter = propInterpretations.iterator();
							// Just create a nucleus ID once
							String nucleusId = newNucleusId(); 
							while (propIntIter.hasNext() && !foundResult) { 
								// get the factory
								EventNucleusFactory factory = (EventNucleusFactory) propIntIter.next();
								// apply the factory
								factory.setDiscRefCache(drBindings); 
								EventNucleusResults evStruct = factory.produceEventNucleus(nom.nomVar, nucleusId, packedNoms);
								if (evStruct != null) { 
									foundResult = true;
									evStruct.eventNucleus.nucleusId = nucleusId;
									evIntResults.addElement(evStruct);
																		
								} // end if.. check for non-null										
							} // end while over factories
							// Reset 
							propInterpretations = null;
						} // end check for factories modeling different propositional interpretations
					} // end if.. check for factory
					
					// checking if an inter-nucleus relation is to be triggered
					if (interNucleusRelationTriggers.containsKey(nom.prop.prop)) {
						inrResults.add(interNucleusRelationTriggers.get(nom.prop.prop));
					}
				} // end while over packed nominals
				// Cycle over the interpretations; if there are more, we need to introduce relations between the nuclei
				log("Number of event nucleus interpretations found: ["+evIntResults.size()+"]");
				
				Iterator evIntIter = evIntResults.iterator();
				while (evIntIter.hasNext()) { 
					EventNucleusResults evInt = (EventNucleusResults) evIntIter.next();
					// Get the nucleus, and store it
					Nucleus nucleus = evInt.eventNucleus; 
					nucleus.plfId = plf.id;
					nucleus.discRefs = drBindings;
					
					// adding the result to the spatio-temporal representation
					int nucleiSize = 	spatioTemporalRep.nuclei.length ;
					spatioTemporalRep.nuclei = (Nucleus[]) LFUtils.resizeArray
					(spatioTemporalRep.nuclei, nucleiSize +1);
					spatioTemporalRep.nuclei[nucleiSize] = evInt.eventNucleus ;
					
					Iterator<InterNucleusRelation> inrIter = inrResults.iterator();
					while (inrIter.hasNext()) {
						
						InterNucleusRelation inr = new InterNucleusRelation();
						inr.mode = inrIter.next().mode;
						inr.targetId = nucleus.nucleusId;
						inr.sourceId = lastProcessedNucleus.nucleusId;			
											
						int inrSize = 	spatioTemporalRep.interNucleusRelations.length ;
						spatioTemporalRep.interNucleusRelations = (InterNucleusRelation[]) LFUtils.resizeArray
						(spatioTemporalRep.interNucleusRelations, inrSize +1);
						spatioTemporalRep.interNucleusRelations[inrSize] = inr ;
					}
					try {
						// addToWorkingMemory(newDataID(),	ComsysOntology.NUCLEUS_TYPE, nucleus);
						addToWorkingMemory(newDataID(),	nucleus); // refactored, data type established from provided object 
    					overwriteWorkingMemory(strId,spatioTemporalRep, OperationMode.BLOCKING);
						log("Stored nucleus and spatio-temporal representation on working memory"); 
					} catch (Exception e) {
						e.printStackTrace();
						throw new ComsysException(e.getMessage());
					} // end try..catch overwriting working memory
					
					lastProcessedNucleus = nucleus;
										
				} // end while
				plfEventStructures.put(plf.id,evIntResults);
				
			} else {
				log("Already built an event structure for plf ["+plf.id+"]");
			} // end if..else check for re-occurrence
		} else {
			throw new ComsysException(
					"Error: event structure interpretation task on illegal processing data type"
					+ pd.getTypes());
		} // end if..else type-check on data item
		
	} // end executeInterpretationTask



	// =================================================================
	// COMPUTATION METHODS
	// =================================================================

	/** Returns a new identifier for a nucleus object */

	private String newNucleusId() {
		String result = "nucleus" + nucleusIdCounter;
		nucleusIdCounter++;
		return result;
	} // end newNucleusDataId

	/** Returns a new identifier for a ProcessingData object */

	private String newProcessingDataId() {
		String result = "pd" + pdIdCounter;
		pdIdCounter++;
		return result;
	} // end newProcessingDataId




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
			log("Entering loop checking for data in event structure interpretation component");
			while (m_status == ProcessStatus.RUN) {
				// lock from external access
				lockProcess();
				// check (synchronised) processing data objects queue
				ListIterator<ProcessingData> i = m_dataObjects
				.listIterator();
				// for the data on the queue, tasks have been approved,
				// so iterate over data objects, and execute the
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
									.equals(ComsysGoals.EVENSTRUCTUREINTERPRETATION_TASK)) {
								executeInterpretationTask(data,
										taskType);
							}
							else {
								log("Unknown task type to process in Comsys:EventStructureInterpretation component");
							} // end if..else check for task type
							// inform the goal manager that the task has
							// been completed, but unsuccessfully
							try {
								taskComplete(
										taskID,
										TaskOutcome.PROCESSING_COMPLETE_SUCCESS);
							}
							catch (SubarchitectureProcessException e) {
								e.printStackTrace();
							} // end try..catch
						}
						catch (ComsysException e) {
							log("Exception while executing a task in dialogue interpretation: "
									+ e.getMessage());
							// inform the goal manager that the task has
							// been completed, but unsuccessfully
							// we may want to make this more specific
							try {
								taskComplete(
										taskID,
										TaskOutcome.PROCESSING_COMPLETE_FAILURE);
							}
							catch (SubarchitectureProcessException ex) {
								ex.printStackTrace();
							} // end try..catch
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
				unlockProcess();
				
                sleepProcess(20);

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
	@Override
	public void configure(Properties _config) {
//		_config.list(System.out);
		super.configure(_config);
	
		if (_config.containsKey("--incrementalInterpretation")) { 
			String incrValue = _config.getProperty("--incrementalInterpretation");
			if (incrValue.equals("false")) { 
				incrementalInterpretation = false;
			} else if (incrValue.equals("0") || incrValue.equals("true")) {
				// currently, no way of doing incremental interpretation
				// incrementalInterpretation = true;
			} else if (incrValue.equals("1")) { 
				incrementalInterpretation = false;
				plfCompletenessLevel = 1;
			} else if (incrValue.equals("2")) { 
				incrementalInterpretation = false;
				plfCompletenessLevel = 2;			
			} else {
				// incrementalInterpretation = true;
			} // end 
		} // end if.. check for incremental binding 		
		

	} // end configure

} // end class

