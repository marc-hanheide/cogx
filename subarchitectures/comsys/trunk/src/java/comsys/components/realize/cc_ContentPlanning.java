// =================================================================
// Copyright (C) 2007-2009 Geert-Jan M. Kruijff (gj@dfki.de)
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
// =================================================================

package comsys.components.realize;

// =================================================================
// IMPORTS
// =================================================================

// -----------------------------------------------------------------
// BINDING IMPORTS
// -----------------------------------------------------------------
// import BindingData.BindingProxy;

// -----------------------------------------------------------------
// CAST IMPORTS
// -----------------------------------------------------------------

import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.architecture.ManagedComponent;
import cast.SubarchitectureComponentException;
import cast.AlreadyExistsOnWMException;
import cast.cdl.*;
import cast.core.CASTData;
import cast.core.CASTUtils;

// -----------------------------------------------------------------
// LF IMPORTS
// -----------------------------------------------------------------
import comsys.lf.utils.LFUtils;

// -----------------------------------------------------------------
// COMSYS IMPORTS
// -----------------------------------------------------------------
import comsys.datastructs.comsysEssentials.*;
import comsys.datastructs.lf.*;
import comsys.arch.ProcessingData;
import comsys.arch.ComsysException;
import comsys.arch.ComsysGoals;
import comsys.utils.ComsysUtils;
import comsys.processing.uttplan.GREHandler;
import comsys.processing.uttplan.UtterancePlanner;
import comsys.processing.uttplan.UtterancePlanningGrammar;

// -----------------------------------------------------------------
// JAVA IMPORTS
// -----------------------------------------------------------------
import java.io.*;
import java.net.*;
import java.util.*;

// =================================================================
// CLASS DOCUMENTATION
// =================================================================


public class cc_ContentPlanning 
	extends ManagedComponent 
	implements GREHandler 	{

		
		// =================================================================
		// CLASS-GLOBAL DATA STRUCTURES
		// =================================================================		
		
		/* The grammar file with the OpenCCG grammar used for realization **/
		String grammarfile;
		
		/* Process identifier counter, used for proposed tasks **/
		int pdIdCounter;

		// Hashtable used to record the tasks we want to carry out
		private Hashtable<String, ProcessingData> m_proposedProcessing;
		
		// Hashtable linking data IDs to goal IDs
		private Hashtable<String, String> m_dataToProcessingGoalMap;
		
		// Vector with objects to be processed, can be ComSys:PhonString,...
		private Vector<ProcessingData> m_dataObjects;		
		
		UtterancePlanner planner; 
		
		String reduxfile; 
		
		private Vector modelRedux; 
		
		/* Name of the relation under the root of the planned LF, pointing to the content for the utterance **/
		
		String contentBody;
		
		// =================================================================
		// CONSTRUCTOR METHODS
		// =================================================================		
				
		/**
		 * The method <i>init</i> initializes the global data structures.
		 * The variable <tt>grammarFile</tt> is set in the <i>configure</tt> method.
		 * 
		 * @see #configure
		 */
		
		private void init () { 
			pdIdCounter = 0;
			UtterancePlanningGrammar upg = new UtterancePlanningGrammar();
			try { 
				upg.read(grammarfile);
			} catch (Exception e) { 
				log("error while reading UPG file: "+e.getMessage());
			} 
			// Initialize the planner with the grammar and
			// register this component as handler for referring expression generation			
			planner = new UtterancePlanner(this);
			planner.setGrammar(upg);
			// Handle model reduction
			modelRedux = new Vector();
			if (reduxfile != null) { 
				log("Loading a model redux file from: ["+reduxfile+"]");
				loadModelRedux(reduxfile);
			} // end if.. 			
			// Make all the comsys queue changes to avoid missing updates
			m_queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;
			// Data processing
			m_proposedProcessing = new Hashtable<String, ProcessingData>();
			m_dataToProcessingGoalMap = new Hashtable<String, String>();
			m_dataObjects = new Vector<ProcessingData>();			
		} // end init
		

		private void loadModelRedux (String filename) { 
			Scanner s = null;
			try {
				s = new Scanner(new BufferedReader(new FileReader(filename)));
				while (s.hasNext()) { 
					String redux = s.next();
					log("Redux: ["+redux+"]");
				modelRedux.add(redux); } 
			} catch (IOException e) { 
				log("Error while loading model reduction file:\n"+e.getMessage());
			} finally {
				if (s != null) { s.close(); } 
			} // end try..finally
		} // and loadModelReduction		
		
		// =================================================================
		// TASK HANDLING METHODS
		// =================================================================	
		
		/**
		 * The method <i>taskAdopted</i> processes a parsing task once the
		 * task manager has informed the parser component it can run. 
		 * This method does <b>not</b> inform the task manager, whether the
		 * task has been succesfully completed. This notification happens in
		 * the <i>runComponent</i> method, which executes the task (run on its 
		 * associated data). 
		 * 
		 * @param _goalID The identifier of the task that has been adopted
		 * 
		 * @see #runComponent
		 * @see cast.architecture.subarchitecture.ManagedProcess#taskAdopted(java.lang.String)
		 */
		@Override
		protected void taskAdopted(String _goalID) {
			// get the data we stored for this goal
			ProcessingData data = m_proposedProcessing.remove(_goalID);
			// if we have stored this goal earlier
			if (data != null) {
				// add the data item to the data objects queue
				m_dataObjects.addElement(data);
				// get the identifier of the CAST data type
				String dataID = data.getID();
				// link the data ID to the goal ID, for future reference
				// on task completion (done in runComponent)
				m_dataToProcessingGoalMap.put(dataID, _goalID);
			}
			else {
				log("ERROR: Goal without data: " + _goalID);
			} // end if..else
		} // end taskAdopted
		
		/**
		 * The method <i>taskRejected</i> prints out a log message that the proposed task 
		 * has been rejected, and removes the goal from the queue of proposed tasks (by ID). 
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
		 * The method "start" adds a working memory change filter on PhonString and PackedLFs objects. If an object has been added to working memory, 
		 * the method <i>handleWorkingMemoryChange</i> is being called. <p>
		 * Furthermore, the method registers change listeners for all the data types that have been registered for the active processes run within the component. These change
		 * listeners all call the method <i>handleActiveDataWorkingMemoryChange</i>. 
		 * 
		 * @see #handleWorkingMemoryChange(WorkingMemoryChange _wmc)
		 * @see #handleActiveDataWorkingMemoryChange(WorkingMemoryChange _wmc)
		 * @see cast.architecture.abstr.WorkingMemoryReaderProcess#workingMemoryChanged(cast.corba.autogen.SubArchitecture.WorkingMemoryChange[])
		 */
		@Override
		public void start() {
			super.start();
			// now call the initialization method for the object
			init();
			// now do the rest
			// register change filters for ProductionLF, which triggers realization
			addChangeFilter(
							ChangeFilterFactory.createLocalTypeFilter(ContentPlanningGoal.class,  WorkingMemoryOperation.ADD),
							new WorkingMemoryChangeReceiver() {
							public void workingMemoryChanged(WorkingMemoryChange _wmc) {
							handleWorkingMemoryChange(_wmc);
							}
							});
		}// end start
		
		
		/**
		 * The method <i>handleWorkingMemoryChange</i> retrieves a CASTData object from the working memory, 
		 * and stores it in the <tt>m_proposedProcessing</tt> table with a parsing task. The CASTData object
		 * can be either a PhonString object (beginning a new utterance processing task) or a PackedLFs object
		 * (continuing an already ongoing processing task).  
		 * 
		 * If one of these objects turns out to provide contextual information used in a registered active
		 * subprocess, then this information needs to be passed on <b>before</b> any further tasks are being
		 * proposed. This is checked here. The change is passed on to the method <i>handleActiveDataWorkingMemoryChange</i>,
		 * and carried out (waiting till the called update methods return) to ensure that all active processes 
		 * are up-to-date before any tasks methods are called. 
		 * 
		 * @param _wmc The working memory change -- a recently added CASTData object
		 */
		private void handleWorkingMemoryChange(WorkingMemoryChange _wmc) {
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
				// set the goal
				String taskGoal = ComsysGoals.CONTENTPLANNING_TASK;
				// store the goal with its information
				proposeInformationProcessingTask(taskID, taskGoal);
			}
			catch (SubarchitectureComponentException e) {
				e.printStackTrace();
			} // end try..catch
		} // end handleWorkingMemoryChange			
		
		/** Returns a new identifier for a ProcessingData object 
		 @return String The new (unique) identifier
		 */
		
		private String newProcessingDataId() {
			String result = "pd" + pdIdCounter;
			pdIdCounter++;
			return result;
		} // end newProcessingDataId		
		

		// =================================================================
		// COMPUTATION METHODS
		// =================================================================	
		
		/** The method <i>executeRealizationTask</i> triggers a realization step,
		 producing String given a ProductionLF object. 
		 
		 @param pd The processing data to be used in parsing
		 @throws ComsysException Thrown if there is no data to parse
		 */ 
		
		public void executeContentPlanningTask(ProcessingData pd)
		throws ComsysException {
			Vector pdTypes = pd.getTypes();
			if (pdTypes.contains(CASTUtils.typeName(ContentPlanningGoal.class))) {
				CASTData data = pd
                .getByType(CASTUtils.typeName(ContentPlanningGoal.class));
				String dataType = data.getType();
				log("Execute content planning task on data item [" + dataType + "]");
				if (data != null) {
					// Get the input object
					ContentPlanningGoal contentPlanningGoal = (ContentPlanningGoal) data.getData();
					// Get the logical form
					LogicalForm goalLF = contentPlanningGoal.lform;
					log("Now try to start content planning with the goal LF "+LFUtils.lfToString(goalLF));
					// Create the logical form for the goal
					LogicalForm contentPlanLF = planner.plan(goalLF);
					// Apply model reduction
					LogicalForm reduxLF = applyModelReduction(contentPlanLF);

					/** 
					// Retrieve the content subtree
					LFRelation content = LFUtils.lfNominalGetRelation(reduxLF.root,contentBody);
					String contentRoot = content.dep;
					LFNominal rootNom  = LFUtils.lfGetNominal(reduxLF,contentRoot);
					LogicalForm planLF = LFUtils.lfConstructSubtree(rootNom,reduxLF);
					*/ 
					
					// Send the content plan to WM for realization
					log("Sending content plan LF to WM:\n"+LFUtils.lfToString(reduxLF));
					try { 
						ProductionLF plf = new ProductionLF();
						plf.lform = reduxLF;
						addToWorkingMemory(newDataID(), plf); 
					} catch (AlreadyExistsOnWMException ioe) { 
						log("ERROR: "+ioe.getMessage());
					}
				} // end if
			} // end if.. check for data presence	
		} // end executeRealizationTask

		
		
		private LogicalForm applyModelReduction (LogicalForm lf) { 
			// log("Root nominal: \n"+LFUtils.lfNominalToString(lf.root));
			// Create a new result
			LogicalForm result = LFUtils.newLogicalForm();
			result.noms = null;
			// Cycle over the nominals in the given lf
			for (Iterator<LFNominal> nomsIter = LFUtils.lfGetNominals(lf); nomsIter.hasNext(); ) { 
				// Get the next nominal
				LFNominal nom = nomsIter.next();
				// log("Reducing nominal: \n"+LFUtils.lfNominalToString(nom));
				
				LFNominal newNom = LFUtils.lfNominalClone(nom);
				// Reset the features in the cloned nominal
				newNom.feats = new comsys.datastructs.lf.Feature[0];
				// Cycle over the features in the nominal
				for (Iterator<comsys.datastructs.lf.Feature> featsIter = LFUtils.lfNominalGetFeatures(nom); featsIter.hasNext(); ) { 
					comsys.datastructs.lf.Feature feat = featsIter.next();
					if (modelRedux.contains(feat.feat)) { 
						// do sweet bugger all, this is one to delete
						// log("Removing feature from nominal: ["+feat.feat+"]");
					} else { 
						// log("Adding feature from nominal: ["+feat.feat+"]");
						newNom.feats = LFUtils.lfNominalAddFeature(newNom,feat);
					} // end if.. check whether to delete
				} // end for over features
				// Finally, add the updated nominal
				// log("Reduced nominal: \n"+LFUtils.lfNominalToString(newNom));
				result.noms = LFUtils.lfAddNominal(result,newNom);
				// log("Updated reduced LF: "+LFUtils.lfToString(result));
			} // end for over nominals
			// log("End for over nominals for applying model reduction");
			result.noms = LFUtils.lfRemoveNominal(result.noms,"");
			LFNominal resultRoot = LFUtils.lfGetNominal(result,lf.root.nomVar);
			result.root = resultRoot;
			log("Final reduced LF: "+LFUtils.lfToString(result));
			return result;
		} // end applyModelReduction
		
		
		/**
			The method <i>produceRFX</i> provides the main interface between the utterance planner, 
			and the distributed components which implement a variety of RFX algorithms. The method 
			writes a struct out to working memory, and waits until an RFX component overwrites that
			struct with a resulting logical form. 
		 
			@param	referent	The referent for which a referring expression should be produced
			@returns LogicalForm The content for the referring expression
		*/ 

		public String produceRFX (Object referent) { 
			log("****************** produceRFX("+referent+") called");
			String rfxLF = new String();
			// Get the referent
			String referentProxy = (String) referent;
			// Create the struct
			GRETask greTask = new GRETask();
			
			// THE NEXT BIT WILL INEVITABLY CRASH; NOT CLEAR HOW TO SOLVE THIS RIGHT NOW .. 
			
			greTask.intendedReferentProxyID = "NOT YET INITIALIZED"; //new WorkingMemoryPointer(CASTUtils.typeName(BindingProxy.class), new WorkingMemoryAddress (referentProxy, "unknown.sa"));
			
			
			
			greTask.resultLF = "";
			greTask.done = false;
			// Set the flag that we are not done yet
			boolean done = false;
			// Add the struct to working memory, make it blocking
			try { 
				String greTaskId = newDataID();
				addToWorkingMemory(greTaskId, greTask); 
				// Try to retrieve the struct, do that till it's done
				while (!done) { 
					lockEntry(greTaskId, WorkingMemoryPermissions.LOCKEDODR);
					CASTData data = getWorkingMemoryEntry(greTaskId);
					greTask = (GRETask) data.getData();
					done = greTask.done;
					unlockEntry(greTaskId);
					sleepComponent(20);
				} // end 	
				// Get the logical form
				rfxLF = greTask.resultLF;
			}  catch (AlreadyExistsOnWMException ioe) { 
				log("ERROR: "+ioe.getMessage());
			} catch (SubarchitectureComponentException se) { 
				log("ERROR: "+se.getMessage());						
			}
			log("Obtained the following logical form with content for a referring expression: ["+rfxLF+"]");
			// transform the logical form if we have an entity root
			LogicalForm	rfx = LFUtils.convertFromString(rfxLF); 
			if (rfx.root.sort.equals("entity")) { 
				if (LFUtils.lfNominalHasRelation(rfx.root,"Owner")) { 
					// Get the Owner dependent nominal 
					LFRelation ownerRel = LFUtils.lfNominalGetRelation(rfx.root,"Owner");
					LFNominal ownerNom = LFUtils.lfGetNominal(rfx,ownerRel.dep);
					// Check whether the nominal has a Name feature
					if (LFUtils.lfNominalHasFeature(ownerNom,"Name")) { 
						String ownerName = LFUtils.lfNominalGetFeature(ownerNom,"Name"); 
						LFNominal compactedOwner = LFUtils.newLFNominal();
						compactedOwner.sort = "person";
						Proposition prop = LFUtils.newProposition();
						prop.prop = ownerName; 
						compactedOwner.prop = prop;
						compactedOwner.nomVar = ownerNom.nomVar;
						rfx.noms = LFUtils.lfRemoveNominal(rfx.noms,ownerNom.nomVar);
						rfx.noms = LFUtils.lfAddNominal(rfx.noms,compactedOwner);
					} // end if..check for name
				} // end if.. check for Owner
			} // 
			rfxLF = LFUtils.lfToString(rfx);
			log("Returning the following LF to the utterance planner: ["+rfxLF+"]");
			// Return the logical form 
			return rfxLF; 
		} // end produceRFX

		
		// =================================================================
		// RUN METHODS
		// =================================================================	
		
		/**
		 * The method <i>runComponent</i> cycles over the queues with data
		 * items (for which tasks have been proposed), checking whether
		 * tasks can be executed. If there is a data item on a queue, the
		 * appropriate task is executed, and the task manager is informed of
		 * task completion (including success / failure).
		 * 
		 * @see cast.core.components.CASTComponent#runComponent()
		 */
		public void runComponent() {
			try {
				log("Entering loop checking for data in utterance realization component");
				while (this.isRunning()) {
					// lock from external access
					lockComponent();
					// check (synchronised) data objects queue
					ListIterator<ProcessingData> i = m_dataObjects
                    .listIterator();
					// for the data on the queue, tasks have been approved, so
					// iterate over data objects, and execute the tasks.
					
					while (i.hasNext()) {
						// get the data from the queue
						ProcessingData data = i.next();
						// check its ID, get the corresponding task ID
						String dataID = data.getID();
						String taskID = m_dataToProcessingGoalMap
                        .get(dataID);
						log("For data [" + dataID + "/" + data.getTypes()
							+ "] do [" + taskID + "]");
						// make sure we know what to do, and have something
						// to do it with
						if (data != null) {
							// execution tasks throw comsys exceptions
							try {
								executeContentPlanningTask(data);
								// inform the goal manager that the task has
								// been completed
								taskComplete(
											 taskID,
											 TaskOutcome.ProcessingCompleteSuccess);
							}
							catch (ComsysException e) {
								log("Exception while executing a task in utterance realization: "
									+ e.getMessage());
								e.printStackTrace();
								// inform the goal manager that the task has
								// been completed, but unsuccessfully
								// we may want to make this more specific
								taskComplete(
											 taskID,
											 TaskOutcome.ProcessingCompleteFailure);
							} // end try..catch for processing exceptions
						} else {
							log("Nothing to process in utterance realization: data null");
						} // end
						// Clean up!
						// no matter what happened, remove the data item
						// from the queue
						i.remove();
					} // end while
					// Free the process
					unlockComponent();
					sleepComponent(20);
					// wait for new tasks!
					//waitForNotifications();
				} // end while running
			}
			catch (Exception e) {
				e.printStackTrace();
			} // end try..catch
		} // end runComponent
		

		// =================================================================
		// CONFIGURATION METHODS
		// =================================================================			
		
		/**
		 * The method <i>configure</i> processes the command-line options provided to the class. 
		 * The following options are processed: 
		 * <ul>
		 * <li><tt>--ccg [file]</tt>: the file name of the grammar to be used, sets the <tt>grammarFile</tt> variable</li> 
		 * </ul>
		 * 
		 * @param _config The properties table
		 */ 
		
		@Override
	    public void configure(Map<String, String> _config) {
			//_config.list(System.out);
			// super.configure(_config);
			String parserArg = "";
			if (_config.containsKey("--grammar")) {
				grammarfile = _config.get("--grammar");
			} else if (_config.containsKey("-grammar")) {
				grammarfile = _config.get("-grammar");
			} else {
				grammarfile = "./subarchitectures/comsys.mk4/grammars/contentPlanning/grammar.xml";
			} // end if..else check for command-line argument
			
			if (_config.containsKey("--redux")) { 
				reduxfile = _config.get("--redux");
			} else { 
				reduxfile = null;
			}
			if (_config.containsKey("--contentBody")) { 
				contentBody = _config.get("--contentBody");
			} else { 
				contentBody = "Content";
			}			
		
		} // end configure
		
		
		
		
		
} // end class ContentPlanning
