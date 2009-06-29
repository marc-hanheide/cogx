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
// CAST IMPORTS
//-----------------------------------------------------------------
import cast.architecture.*;
import cast.cdl.*;
import cast.core.CASTData;
import cast.core.CASTUtils;
import cast.SubarchitectureComponentException;
//-----------------------------------------------------------------
// COMSYS IMPORTS
//-----------------------------------------------------------------
import comsys.datastructs.comsysEssentials.*;
import comsys.arch.*;
import comsys.utils.SDRSUtils;
import comsys.utils.datastructs.*;
import comsys.processing.dialogue.DialogueMoveFactory;
import comsys.processing.dialogue.dialoguemovefactories.*;
import comsys.processing.parse.*;

//-----------------------------------------------------------------
// JAVA IMPORTS
//-----------------------------------------------------------------
import java.util.Hashtable;
import java.util.Iterator;
import java.util.ListIterator;
import java.util.Properties;
import java.util.TreeMap;
import java.util.Vector;
import java.util.List;
import java.util.ArrayList;

//-----------------------------------------------------------------
// LOGICAL FORM IMPORTS
//-----------------------------------------------------------------
import comsys.datastructs.lf.*;
import comsys.lf.utils.*;

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

public class cc_DialogueMove 
	extends ManagedComponent 
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
	// DATA STRUCTURES FOR DIALOGUE MOVE INTERPRETATION 
	// ----------------------------------------------------------------

	// Dialogue context model
	private SDRS dialctxt;

	// Counter for ProcessingData identifiers
	private int pdIdCounter;
	

	/** The hashtable "factories" stores the registered dialogue move factories by type (key). Returned
		for a key is another hashtable, indexed by proposition. If a factory does not specify a proposition
		as filter, then it is added as "noProp"; the restriction is that at most one such factory can be 
		registered for a given sort. 
	*/
	private Hashtable factories; 

	/** constant */ 
	private final String NOPROP = "noProp";

	/** Counter for event nucleus identifiers */ 
	private int nucleusIdCounter;
	
	
	DecisionTree decisionTree ;
	Hashtable<String,DialogueMoveFactory> dialogueMoves;
	List<String> whWordsList ;
	
	public static final String[] decisionsList = {"sort", "mood", "polarity", "wh-restrictor", "tense", "prop"};
	
	
	/** List of processed packed logical forms (to avoid double work), stored by Id, with their results.
		The results are (currently) a Vector of event nuclei that have been produced. 
	*/
	Hashtable plfEventStructures; 


	// =================================================================
	// CONSTRUCTOR METHODS
	// =================================================================

	/**
	 * The unary constructor
	 * 
	 * @param _id
	 */
/**	public cc_DialogueMove(String _id) {
		init();
	} // constructor/1 */ 

	/**
	 * The method <i>init</i> initializes the global data structures,
	 * and sets the ontology for this component.
	 * 
	 * @see #configure
	 */

	private void init() {
		log("Initializing dialogue move interpretation component");
		// nah: making all the comsys queue changes... don't want to
		// miss a thing
		m_queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;

		// general information processing structures
		m_proposedProcessing = new Hashtable<String, ProcessingData>();
		m_dataToProcessingGoalMap = new Hashtable<String, String>();
		m_taskToTaskTypeMap = new Hashtable<String, String>();
		m_dataObjects = new Vector<ProcessingData>();
		pdIdCounter = 0;
		
		factories = new Hashtable();
		nucleusIdCounter = 0;
		plfEventStructures = new Hashtable();

		decisionTree = new DecisionTree(decisionsList);
		dialogueMoves = new Hashtable<String,DialogueMoveFactory>();
		whWordsList = new ArrayList<String>();
		
		whWordsList.add("what");
		whWordsList.add("which");
		whWordsList.add("where");
		whWordsList.add("who");
		
		// Register the available factories
		 try { 
			this.registerFactory (new Assert()); 
			this.registerFactory (new ActionDirective()); 
			this.registerFactory (new Accept()); 
			this.registerFactory (new Opening()); 
			this.registerFactory (new Closing()); 
			this.registerFactory (new QuestionW()); 
			this.registerFactory (new QuestionYN()); 
			this.registerFactory (new Reject()); 

		 } catch (ComsysException e) { 
			System.err.println(e.getMessage());
			System.exit(0);
		 } // end try.. catch
		 
	//	String fileName = "graphs/decisionTree";
	//	decisionTree.DecisionTreeToGraph(fileName);
	//	 System.out.println("Decision tree successfully written to " + fileName + ".png");
	
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

	public void registerFactory (DialogueMoveFactory factory) 
		throws ComsysException 
	{ 
		List<AbstractFeatureValue> avm = factory.getProperties();
		String label = factory.getLabel();
		decisionTree.addDialogueMove(label, avm) ;
		dialogueMoves.put(factory.getLabel(), factory);
		System.out.println("Dialogue move " + factory.getLabel() + " successfully registered");
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
		init();
//		try {
			// Change filters for caches
			addChangeFilter(
					ChangeFilterFactory.createLocalTypeFilter(PackedLFs.class,  WorkingMemoryOperation.ADD),
					new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					handlePackedLFs(_wmc);
				}
			});
			addChangeFilter(
					ChangeFilterFactory.createLocalTypeFilter(PackedLFs.class,  WorkingMemoryOperation.OVERWRITE),
					new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					handlePackedLFs(_wmc);
				}
			});			
//		} catch (SubarchitectureComponentException e) {
//			e.printStackTrace();
//		} // end try..catch
	} // end start
	// =================================================================
	// EXECUTION METHODS
	// =================================================================

	private void handlePackedLFs(WorkingMemoryChange _wmc) {
		try {
			// get the id of the working memory entry
			String id = _wmc.address.id;
			// get the data from working memory
			CASTData plfWM = getWorkingMemoryEntry(id);
			PackedLFs plf = (PackedLFs) plfWM.getData();
			
			if (plf.type.equals("interpretation")) {
			
	//		LFUtils.plfToGraph(plf.packedLF, "TEST1");
			
			log("Handling the packedLF]");	
			if (plf.finalized == ActiveIncrCCGParser.FINAL_PARSE) {

				log("Starting task to determine the dialogue move for the given packed LF");
				// Create an id
				String taskID = newTaskID();
				// store the data we want to process for later
				ProcessingData pd = new ProcessingData(newProcessingDataId());
				pd.add(plfWM);
				m_proposedProcessing.put(taskID, pd);
				// set up the goal
				String taskGoal;
				taskGoal = ComsysGoals.DIALOGUEMOVEINTERPRETATION_TASK;
				// then ask for permission
				proposeInformationProcessingTask(taskID, taskGoal);
				// store the id with the task type
				m_taskToTaskTypeMap.put(taskID, taskGoal);
			}
			}
				// Check what kind of type we are dealing with, only deal with disc refs
		} catch (SubarchitectureComponentException e) {
			e.printStackTrace();
		} // end try..catch
	} // end handleCache
	

	/**
		The method <i>executeInterpretatioTask</i> cycles over the nominals in the packed logical 
		form stored in the cache, and applies event structure factories if possible to create an 
		event structure for the PLF. 
	 */

	public void executeInterpretationTask(ProcessingData pd, String interpretationTask)
	throws ComsysException 
	{
		log("Starting to create dialogue move");
		// Get the cache 
		CASTData data = pd.getByType(CASTUtils.typeName(PackedLFs.class));
		if (data != null) {
			Vector evIntResults = new Vector();
			PackedLFs plf = (PackedLFs) data.getData();

			List<AbstractFeatureValue> avm = extractFeatures(plf);
			
			String dialogueMove =  decisionTree.getDialogueMove(avm);
			log("Dialogue move determined for the utterance: " + dialogueMove);

			if (!dialogueMove.equals("none")) {
				log("Adding dialogue move to working memory");

				try {
					CASTData[] Data = getWorkingMemoryEntries(SDRS.class, 0);
					String SDRSFormulaId1;
					String SDRSFormulaId2;
					
					if (Data.length > 0 ) {
					SDRS newSDRS = (SDRS) Data[0].getData() ;

					// before adding the cache to the working memory, we have to wait for the dialogue
					// interpretation component to have integrated the packed logical form in the 
					// dialogue model (otherwise the cache is useless)
					
					SDRSFormula lastFormula = SDRSUtils.getLastFormula(newSDRS);
					if (lastFormula == null) {
						SDRSFormulaId1 = "start";
						SDRSFormulaId2 = SDRSUtils.getNextLabel();						
					}
					else if (SDRSUtils.getFormulaType(lastFormula).type.equals(SDRSUtils.PLF_TYPE) &&
							lastFormula.type.plf.id.equals(plf.id)) {
						SDRSFormulaId1 = lastFormula.tprec;
						SDRSFormulaId2 = lastFormula.label;
					}
					else {
						SDRSFormulaId1 = lastFormula.label;
						SDRSFormulaId2 = SDRSUtils.getNextLabel();				
					}
					}
					
					else {
						log("no SDRS structure found, adding a bare dialogue move type");
						SDRSFormulaId1 = "";
						SDRSFormulaId2 = "";
					}
					DialogueMoveFactory factory = dialogueMoves.get(dialogueMove);
					
					if (factory != null) {
						DialogueMove dm = factory.produceDialogueMove(SDRSFormulaId1, SDRSFormulaId2, this.newDialogueMoveId());
						// addToWorkingMemory(newDataID(),	ComsysOntology.DIALOGUEMOVE_TYPE, dm);
						addToWorkingMemory(newDataID(),	dm); // refactored, data type determined from provided object
						log("dialogue move successfully added to working memory");
					}
					
				}
				catch (Exception e) {
					e.printStackTrace();
				}

			}

		} else {
			throw new ComsysException(
					"Error: dialogue move interpretation task on illegal processing data type"
					+ pd.getTypes());
		} // end if..else type-check on data item

	} // end executeInterpretationTask



	private List<AbstractFeatureValue> extractFeatures (PackedLFs plf) {
		List<AbstractFeatureValue> avm = new ArrayList<AbstractFeatureValue> ();
		Iterator<PackedNominal> roots = LFUtils.plfGetRoots(plf.packedLF);
		
		PackingNode packingNode = LFUtils.plfGetPackingNode(plf.packedLF, plf.packedLF.root);
		
		while (roots.hasNext()) {
			PackedNominal root = roots.next();
			if (!root.prop.prop.equals("list") && !root.prop.prop.equals("context")) {  // we want to get read of "fake" roots, like connectives
				if (root.packedSorts.length == 1) {
					avm.add(new FeatureValue("sort", root.packedSorts[0].sort));
				}
				for (int i=0; i< root.feats.length ; i++) {
					avm.add(new FeatureValue(root.feats[i].feat, root.feats[i].value));
				}
				
				avm.add(new FeatureValue("prop", root.prop.prop));
				
				for (int i=0; i < root.rels.length ; i++) {
					if (root.rels[i].mode.equals("Wh-Restr")) {
						PackedNominal dep = LFUtils.plfGetPackedNominal(plf.packedLF, root.rels[i].dep);
						if (dep != null) {
							if (whWordsList.contains(dep.prop.prop)) {
								avm.add(new FeatureValue("wh-restrictor", "yes"));
							}
						}
					}
				}
				PackingEdge pEdge = LFUtils.plfGetPackingEdge(packingNode, "Wh-Restr");
				for (int i = 0; pEdge != null && i < pEdge.targets.length ; i++) {

					PackingNodeTarget target = pEdge.targets[i];
					PackingNode pNode = LFUtils.plfGetPackingNode(plf.packedLF, target.pnId);
					PackedNominal nom = LFUtils.plfGetPackedNominal(pNode, pNode.root);

					if (whWordsList.contains(nom.prop.prop)) {
						avm.add(new FeatureValue("wh-restrictor", "yes"));
					}
				}
			}
			else if (root.rels.length > 0){
				PackedNominal dep1 = LFUtils.plfGetPackedNominal(
						packingNode,  root.rels[0].dep);
				if (dep1.packedSorts.length == 1) {
					avm.add(new FeatureValue("sort", dep1.packedSorts[0].sort));
				}
				avm.add(new FeatureValue("prop", dep1.prop.prop));
			}
		}
		// handling the absence of the wh-restrictor
		Iterator<AbstractFeatureValue> it = avm.iterator();
		boolean foundInterrogativeMood = false;
		boolean foundWhRestrictor = false;
		while (it.hasNext()) {
			AbstractFeatureValue fv = it.next();
			if (fv.getFeat().equals("mood") && fv.getValue().equals("int")) {
				foundInterrogativeMood = true;
			}
			if (fv.getFeat().equals("wh-restrictor") && fv.getValue().equals("yes")) {
				foundWhRestrictor = true;
			}
		}
		if (foundInterrogativeMood && !foundWhRestrictor) {
			avm.add(new FeatureValue("wh-restrictor", "no"));
		}
		log("features found: " + avm.toString());
		return avm;
	}
	// =================================================================
	// COMPUTATION METHODS
	// =================================================================

	/** Returns a new identifier for a nucleus object */

	private String newDialogueMoveId() {
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
			log("Entering loop checking for data in dialogue move interpretation component");
			while (this.isRunning()) {
				// lock from external access
				lockComponent();
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
									.equals(ComsysGoals.DIALOGUEMOVEINTERPRETATION_TASK)) {
								executeInterpretationTask(data,
										taskType);
							}
							else {
								log("Unknown task type to process in Comsys:DialogueMoveInterpretation component");
							} // end if..else check for task type
							// inform the goal manager that the task has
							// been completed, but unsuccessfully
							//try {
								taskComplete(
										taskID,
										TaskOutcome.ProcessingCompleteSuccess);
							//}
							//catch (SubarchitectureComponentException e) {
							//	e.printStackTrace();
							//} // end try..catch
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
							//}
							//catch (SubarchitectureComponentException ex) {
							//	ex.printStackTrace();
							//} // end try..catch
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
				//waitForNotifications(m_dataObjects);
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
//		super.configure(_config);
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

