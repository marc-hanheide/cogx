// =================================================================
// Copyright (C) 2007-2010 Geert-Jan M. Kruijff (gj@dfki.de)
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

// =================================================================
// IMPORTS

// CAST 

import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.architecture.ManagedComponent;
import cast.SubarchitectureComponentException;
import cast.AlreadyExistsOnWMException;
import cast.cdl.*;
import cast.core.CASTData;
import cast.core.CASTUtils;

// Dialogue API CAST extension 
import de.dfki.lt.tr.cast.ProcessingData;
import de.dfki.lt.tr.cast.dialogue.DialogueGoals;


// Dialogue API slice
import de.dfki.lt.tr.dialogue.interpret.IntentionManagementConstants;
import de.dfki.lt.tr.dialogue.slice.asr.PhonString;
import de.dfki.lt.tr.dialogue.slice.discourse.DialogueMove;
import de.dfki.lt.tr.dialogue.slice.lf.*;
import de.dfki.lt.tr.dialogue.slice.parse.*;
import de.dfki.lt.tr.dialogue.slice.produce.*;
import de.dfki.lt.tr.dialogue.slice.ref.NominalReference;
import de.dfki.lt.tr.dialogue.slice.synthesize.*;



// Dialogue API 
import de.dfki.lt.tr.dialogue.parse.IncrCCGStringParser;
import de.dfki.lt.tr.dialogue.util.LFUtils;
import de.dfki.lt.tr.dialogue.util.DialogueException;
import de.dfki.lt.tr.dialogue.util.LFPacking;

// OPENCCG IMPORTS
import opennlp.ccg.lexicon.*;
import opennlp.ccg.grammar.*;
import opennlp.ccg.parse.*;
import opennlp.ccg.util.*;
import opennlp.ccg.synsem.*;
import opennlp.ccg.realize.*;
import opennlp.ccg.hylo.*;
import opennlp.ccg.unify.*;
import opennlp.ccg.ngrams.*;
import opennlp.ccg.test.*;

// -----------------------------------------------------------------
// JDOM IMPORTS
// -----------------------------------------------------------------
import org.jdom.*;
import org.jdom.output.XMLOutputter;

// -----------------------------------------------------------------
// JAVA IMPORTS
// -----------------------------------------------------------------
import java.io.*;
import java.net.*;
import java.util.*;

// =================================================================
// CLASS DOCUMENTATION
// =================================================================

/**
 * The class <tt>CCGRealizer</tt> implements a CAST managed component
 * that takes a <tt>ProductionLF</tt> as input, and realizes a surface
 * form for that logical form using a (statistical) CCG realizer. This
 * surface form is written out to working memory as a <tt>SpokenOutputItem</tt>. 
 * The (packed) logical form for this surface form can also be provided to 
 * working memory, as a <tt>PackedLFs</tt> structure. 
 * 
 * <h4>CAST-file compponent configuration options </h4>
 * <ul>
 * <li> <tt>--ccg FILENAME</tt>: specifies the location of the CCG grammar (grammar.xml file) to be used (REQUIRED)
 * <li> <tt>--contentBody LABEL</tt>: specifies the LABEL of the relation under the root of a logical form, under which 
 * the content to be realized can be found (DEFAULT: "Content")
 * <li> <tt>--cannedText LABEL</tt>: specifies the LABEL of the relation under the root of a logical form, under which 
 * cannedText to be synthesized directly can be found (DEFAULT: "CannedText")
 * <li> <tt>--ngrams FILENAME</tt>: specifies the location of the Ngrams corpus to be used for the statistical realizer (OPTIONAL)
 * <li> <tt>--packedlf BOOL</tt>: specifies whether to output a (packed) logical form of the produced surface form to working memory (OPTIONAL, DEFAULT:false)
 * </ul> 
 *
 * @author	Geert-Jan M. Kruijff (gj@dfki.de)
 * @version 101007 
 */



public class CCGRealizer 
	extends ManagedComponent {

		
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
		
		/* The OpenCCG realizer **/
		Realizer realizer; 
		NgramScorer ngramScorer; 
		
		/* Relation for the content body */
		String contentBody = "";
		
		/* Relation for the canned text */
		String cannedText  = "";
		
		// Boolean flag to determine whether or not to write out a packed LF 
		// of the generated surface form 
		boolean writeOutPackedLF;
		
		String ngramCorpus = "";

		public final String UNABLE_TO_REALISE = "I am sorry I am lost for words on this one";
		
		
		// =================================================================
		// CONSTRUCTOR METHODS
		// =================================================================		
		
		/**
		 * The unary constructor. The initialization is called after the start() method. 
		 * 
		 * @param _id
		 */
		public CCGRealizer (String _id) {
		} // constructor/1

		public CCGRealizer() {
		}

		/**
		 * The method <i>init</i> initializes the global data structures.
		 * The variable <tt>grammarFile</tt> is set in the <i>configure</tt> method.
		 * 
		 * @see #configure
		 */
		
		private void init () { 
			pdIdCounter = 0;
			try { 
				// load grammar                                                                                     
				URL grammarURL = new File(grammarfile).toURL();
				System.out.println("Loading grammar from URL: " + grammarURL);
				Grammar grammar = new Grammar(grammarURL);
				realizer = new Realizer(grammar);
				ngramScorer = null;
			} catch (IOException ioe) { 
				log("ERROR: "+ioe.getMessage());				
			} // end try..catch
			// Make all the comsys queue changes to avoid missing updates
			m_queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;
			// Data processing
			m_proposedProcessing = new Hashtable<String, ProcessingData>();
			m_dataToProcessingGoalMap = new Hashtable<String, String>();
			m_dataObjects = new Vector<ProcessingData>();			
			ngramScorer = null;
			// by default do not write out the packed lf
			writeOutPackedLF = false;
		} // end init
		

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
					ChangeFilterFactory.createLocalTypeFilter(ProductionLF.class,  WorkingMemoryOperation.ADD),
					new WorkingMemoryChangeReceiver() {
						@Override
						public void workingMemoryChanged(WorkingMemoryChange _wmc) {
							handleWorkingMemoryChange(_wmc);
						}
					});
			
			if (!ngramCorpus.equals("")) {
				String[] targets = loadCorpus(ngramCorpus);
			    ngramScorer = new NgramPrecisionModel(targets);
			}
			
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
				String taskGoal = DialogueGoals.REALIZATION_TASK;
				// store the goal with its information
				proposeInformationProcessingTask(taskID, taskGoal);
			}
			catch (SubarchitectureComponentException e) {
				e.printStackTrace();
			} // end try..catch
		} // end handleWorkingMemoryChange			
		
		/** 
		 * Returns a new identifier for a ProcessingData object 
		 * @return String The new (unique) identifier
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
		
		public void executeRealizationTask(ProcessingData pd)
		throws DialogueException {
			Vector pdTypes = pd.getTypes();
			if (pdTypes.contains(CASTUtils.typeName(ProductionLF.class))) {
				CASTData data = pd.getByType(CASTUtils.typeName(ProductionLF.class));
				String dataType = data.getType();
				log("Execute realization task on data item [" + dataType + "]");
				if (data != null) {
					// Get the input object
					ProductionLF productionLF = (ProductionLF) data.getData();
					// Get the logical form
					LogicalForm logicalForm = productionLF.lform;
					// Initialize the return string
					String realString = UNABLE_TO_REALISE;
					NominalReference nr = null;
					LogicalForm planLF = null;
					// Check whether there is canned text OR a content body
					String canned = LFUtils.lfNominalGetFeature(logicalForm.root, cannedText);
					log("Canned text feature value: "+canned);
					if (!canned.equals("")) 
					{
						realString = canned.replaceAll("_"," ");		
					} else { 
						// Retrieve the content subtree
						LFRelation content = LFUtils.lfNominalGetRelation(logicalForm.root,contentBody);
						String contentRoot = "";
						if (content != null) {
							contentRoot = content.dep;
						} else {
							contentRoot = logicalForm.root.nomVar;
						} 
						LFNominal contentRootNom  = LFUtils.lfGetNominal(logicalForm,contentRoot);
						planLF = LFUtils.lfConstructSubtree(contentRootNom,logicalForm);					
						// Translate the planLF logical form into XML for OpenCCG
						LF lf = LFUtils.convertToLF(planLF);
						// Realize the XML-based logical form
						// Initialize the realizer with the CCG grammar, if there is one specified 					

						if (ngramScorer == null) { 
							realizer.realize(lf);
						} else {
							realizer.realize(lf,ngramScorer);
						} 
						// Get the best string out of the results
						opennlp.ccg.realize.Chart chart = realizer.getChart();
						String bestRealization = "";
						List bestEdges = chart.bestEdges();
						for (Iterator beIter = bestEdges.iterator(); beIter.hasNext(); ) {
							opennlp.ccg.realize.Edge edge = (opennlp.ccg.realize.Edge) beIter.next(); 
							// Sign sign = edge.getSign();
							bestRealization = bestRealization+edge.toString()+"\n";
						} // end for over best edges
						if (bestEdges.size() > 0) {
							log("Best realization: "+bestEdges.get(0));
						}
						int start = bestRealization.indexOf("]");
						int end   = bestRealization.indexOf(":-");
						if (start != -1 && end != -1) { 
							realString = bestRealization.substring(start+1,end);
						}
						if (!realString.equals(UNABLE_TO_REALISE)) {
							// assume that realisation worked out
							nr = productionLF.topic;
						}
					}
					// Forward the string to WM, to be synthesized
					try { 
						log("Sending on for synthesis: ["+realString+"]");
						SpokenOutputItem spoi = new SpokenOutputItem();
						spoi.phonString = realString;

						DialogueMove dm = new DialogueMove(IntentionManagementConstants.thisAgent, planLF, nr);

						addToWorkingMemory(newDataID(), spoi);
						addToWorkingMemory(newDataID(), dm);

					} catch (AlreadyExistsOnWMException ioe) { 
						log("ERROR: "+ioe.getMessage());
					} catch (SubarchitectureComponentException se) { 
						log("ERROR: "+se.getMessage());						
					}
					// if opted for, write out the packed lf of the surface form
					if (writeOutPackedLF && planLF != null) 
					{
						addNewPLFToWorkingMemory(planLF, realString);
					}
				} // end if
			} // end if.. check for data presence	
		} // end executeRealizationTask

		private void addNewPLFToWorkingMemory (LogicalForm planLF, String realization) {
			PackedLFs plf = new PackedLFs();
			plf.id = newDataID();			
			LFPacking packingTool = new LFPacking();
			packingTool.incrementUttNumber();
			LogicalForm[] lfs = new LogicalForm[1];
			lfs[0] = planLF;
			plf.packedLF = packingTool.packLogicalForms(lfs);
			plf.phonStringLFPairs[0].phonStr = new PhonString();
			plf.phonStringLFPairs[0].phonStr.wordSequence = realization;
			plf.type = "production";
			plf.finalized = IncrCCGStringParser.FINAL_PARSE;
			try {
				addToWorkingMemory(newDataID(), plf);	
				log("Packed logical form containing the planned LF added to working memory");
			}
			catch (Exception e) {
				e.printStackTrace();
			}
		} // end addNewPLFToWorkingMemory
		
		
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
		@Override
		public void runComponent() {
			try {
				log("Entering loop checking for data in utterance realization component");
				while (this.isRunning()) {
					// lock from external access
					lockComponent();
					// check (synchronised) data objects queue
					ListIterator<ProcessingData> i = m_dataObjects.listIterator();
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
								executeRealizationTask(data);
								// inform the goal manager that the task has
								// been completed
								taskComplete(
										taskID,
										TaskOutcome.ProcessingCompleteSuccess);

							}
							catch (DialogueException e) {
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
					waitForNotifications();
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
			if (_config.containsKey("--ccg")) {
				grammarfile = _config.get("--ccg");
			} else if (_config.containsKey("-ccg")) {
				grammarfile = _config.get("-ccg");
			} else {
				grammarfile = "./subarchitectures/comsys.mk4/grammars/openccg/moloko.v5/grammar.xml";
			} // end if..else check for command-line argument
			
			// Set the relation label for content body
			if (_config.containsKey("--contentBody")) { 
				contentBody = _config.get("--contentBody");
			} else { 
				contentBody = "Content";
			}				

			// Set the label for canned text
			if (_config.containsKey("--cannedText")) { 
				cannedText = _config.get("--cannedText");
			} else { 
				cannedText = "CannedText";
			}	
			
			if (_config.containsKey("--ngrams")) { 
				ngramCorpus = _config.get("--ngrams");
			} 	
			if (_config.containsKey("--packedlf")) 
			{
				String writeOutBool = _config.get("--packedlf");
				if (writeOutBool.startsWith("true")) 
				{
					writeOutPackedLF = true;
				}
				else
				{
					writeOutPackedLF = false;
				}
			}	
				
		} // end configure
		
		private String[] loadCorpus (String filename) { 
			log("Loading ngram corpus from file ["+filename+"]");
			Scanner s = null;
			Vector<String> targets = new Vector<String>();
			try {
				s = new Scanner(new BufferedReader(new FileReader(filename)));
				while (s.hasNext()) { 
					String redux = s.next();
				targets.add(redux); } 
			} catch (IOException e) { 
				System.out.println("Error while loading model reduction file:\n"+e.getMessage());
			} finally {
				if (s != null) { s.close(); } 
			} // end try..finally
			String[] result = new String[targets.size()];
			int i = 0;
			for (String target : targets) { 
				result[i] = target;
				i++;
			} // end for
			log("The corpus contains ["+i+"] items");
			return result;
		} // end loadCorpus		
		
		
		
} // end class UtteranceRealization
