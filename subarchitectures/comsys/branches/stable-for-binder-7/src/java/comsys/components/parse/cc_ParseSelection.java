// =================================================================
// Copyright (C) 2007-2008 Geert-Jan M. Kruijff (gj@dfki.de)
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

package comsys.components.parse;

// =================================================================
// IMPORTS
// =================================================================

// -----------------------------------------------------------------
// CAST IMPORTS
// -----------------------------------------------------------------

import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.architecture.ManagedComponent;
import cast.SubarchitectureComponentException;
import cast.cdl.*;
import cast.core.CASTData;
import cast.core.CASTUtils;
import comsys.lf.utils.LFUtils;
import comsys.processing.parse.ActiveIncrCCGParser;
import comsys.processing.parseselection.Decoder;
import comsys.processing.parseselection.LearningUtils;
import comsys.processing.parseselection.ParameterVector;
import comsys.utils.ComsysUtils;

// -----------------------------------------------------------------
// COMSYS IMPORTS
// -----------------------------------------------------------------

import comsys.datastructs.comsysEssentials.PackedLFs;
import comsys.datastructs.comsysEssentials.PhonString;
import comsys.datastructs.comsysEssentials.SelectedLogicalForm;
import comsys.datastructs.lf.LogicalForm;
import comsys.arch.ComsysException;
import comsys.arch.ComsysGoals;
import comsys.arch.ProcessingData;

// -----------------------------------------------------------------
// JAVA IMPORTS
// -----------------------------------------------------------------
import java.util.Hashtable;
import java.util.Iterator;
import java.util.ListIterator;
import java.util.Map;
import java.util.Properties;
import java.util.TreeSet;
import java.util.Vector;
import java.util.Enumeration;

// -----------------------------------------------------------------
// REPRESENTATION IMPORTS
// -----------------------------------------------------------------

// =================================================================
// CLASS DOCUMENTATION
// =================================================================

/**
The class <b>ParseSelection</b> ... 

@version 080822 
@started 080822
@author Pierre Lison (pierre.lison@dfki.de)
*/ 


public class cc_ParseSelection 
	extends ManagedComponent {

	   // ----------------------------------------------------------------
    // INFORMATION PROCESSING DATA STRUCTURES
    // ----------------------------------------------------------------

    // Hashtable used to record the tasks we want to carry out. For each
    // taskID we store a Vector with the data it is to work on
    private Hashtable<String, ProcessingData> m_proposedProcessing;

    // Hashtable linking data IDs to goal IDs
    private Hashtable<String, String> m_dataToProcessingGoalMap;

    // Hashtable linking task IDs to task types
    protected Hashtable<String, String> m_taskToTaskTypeMap;
    
    // Vector with objects to be processed,
    // can be ComSys:PackedLFs,...
    private Vector<ProcessingData> m_dataObjects;
    
    // Processing Data id counter
    private int pdIdCounter;
    
    private ParameterVector params;
    private Decoder decoder;
    
    
    // =================================================================
    // CONSTRUCTOR METHODS
    // =================================================================

    /**
     * The unary constructor. The initialization is called after the start() method. 
     * 
     * @param _id
     */
  /**  public ParseSelection () {
        super(_id);
    } // constructor/1 */

	/**
     * The method <i>init</i> initializes the global data structures,
     * and sets the ontology for this component. The variable
     * <tt>grammarFile</tt> is set in the <i>configure</tt> method.
     * 
     * @see #configure
	*/

	private void init () { 
        // Component
        log("Initializing parse selection component");
         // Make all the comsys queue changes to avoid missing updates
        m_queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;

        // Data processing
        m_proposedProcessing = new Hashtable<String, ProcessingData>();
        m_dataToProcessingGoalMap = new Hashtable<String, String>();
        m_dataObjects = new Vector<ProcessingData>();
        m_taskToTaskTypeMap = new Hashtable<String, String>();

	} // end init
		
	
	// =================================================================
    // CONFIGURATION METHODS
    // =================================================================	
	
	
	/**
	* The method <i>configure</i> processes the command-line options provided to the class. 
	* The following options are processed: 
	* <ul>
	* <li><tt>--grammar [file]</tt>: the file name of the grammar to be used, sets the <tt>grammarFile</tt> variable</li> 
	* </ul>
	* 
	* @param _config The properties table
	*/ 
	
    @Override
    public void configure(Map<String,String> _config) {
   //     _config.list(System.out);
        super.configure(_config);
        
        if (_config.containsKey("--parameterfile")) {
           String parameterfile  = _config.get("--parameterfile");
           Vector<String> lines = LearningUtils.readFile(parameterfile);
           params = LearningUtils.buildParameterVector(lines);
       //    log("Parameter vector: " + params.toString());
           decoder = new Decoder(params);
        }  
        
        if (_config.containsKey("--asr-correction")) {
            String asrcorrectionStr  = _config.get("--asr-correction");
            boolean asrcorrection = (new Boolean(asrcorrectionStr)).booleanValue();
            opennlp.ccg.grammar.AbstractRule.asrcorrectionRules = asrcorrection;
         }  

        if (_config.containsKey("--disfluency-correction")) {
            String disflcorrectionStr  = _config.get("--disfluency-correction");
            boolean disflcorrection = (new Boolean(disflcorrectionStr)).booleanValue();
            opennlp.ccg.grammar.AbstractRule.disfluencycorrectionRules = disflcorrection;
         }  
 
        
        if (_config.containsKey("--discourse-level-composition")) {
            String disclevelcompStr  = _config.get("--discourse-level-composition");
            boolean disclevelcomp = (new Boolean(disclevelcompStr)).booleanValue();
            opennlp.ccg.grammar.AbstractRule.disclevelcompositionRules = disclevelcomp;
         }  
 		
	} // end configure

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
			// register change filters
				try {
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
				catch (Exception e) {
					e.printStackTrace();
				}
				
    }// end start
	
    /**
     * @param _wmc
     * @param i
     */
    private void addedPackedLF(WorkingMemoryChange _wmc) {
    	
    	log("PackedLF received");
        // if we have a logical form collection string,
        try {
            // get the id of the working memory entry
            String id = _wmc.address.id;
            // get the data from working memory and store it
            // with its id
            CASTData PlfWM = getWorkingMemoryEntry(id);
            PackedLFs plf = (PackedLFs) PlfWM.getData();

            if (plf.type.equals("interpretation") && 
            		plf.finalized == ActiveIncrCCGParser.FINAL_PARSE) {
            	log("final PLF retrieved, inserting new parse selection task");
            	// Propose a new task
            	String taskID = newTaskID();
            	// store the data we want to process for later
            	ProcessingData pd = new ProcessingData(newProcessingDataId());
            	pd.add(PlfWM);
            	m_proposedProcessing.put(taskID, pd);
            	// set up the goal
            	String taskGoal;
            	taskGoal = ComsysGoals.PARSESELECTION_TASK;
            	// then ask for permission
            	proposeInformationProcessingTask(taskID, taskGoal);
            	// store the id with the task type
            	m_taskToTaskTypeMap.put(taskID, taskGoal);       
            }
        }
        catch (SubarchitectureComponentException e) {
        	e.printStackTrace();
        } // end try..catch

    } 
    
	
	/** Returns a new identifier for a ProcessingData object 
		@return String The new (unique) identifier
	*/

    private String newProcessingDataId() {
        String result = "pd" + pdIdCounter;
        pdIdCounter++;
        return result;
    } // end newProcessingDataId
	
	
	// =================================================================
    // EXECUTION METHODS
    // =================================================================	
	
	public static String recogString;
	public static double lastscore = 0.0;
	
	/** The method <i>executeParseTask</i> triggers a parsing step, in 
		analyzing a given PhonString object. 
	
		@param pd The processing data to be used in parsing
		@throws ComsysException Thrown if there is no data to parse
	*/ 
		
    public void executeParseSelectionTask(ProcessingData pd, String task)
            throws ComsysException {
   // 	log("starting executeParseSelectionTask...");
    	try {	
            CASTData plfWM = pd.getByType(CASTUtils.typeName(PackedLFs.class));
            if (plfWM != null) {
            	PackedLFs plf = (PackedLFs) plfWM.getData();
                log("PackedLFs correctly retrieved, start decoding...");
                if (decoder != null) {
                	
                	decoder.logging = true;
                	LogicalForm lf = decoder.getBestParse(plf);
                	decoder.logging = false;
                	
                	log("Decoding successful");
                	String bestLFStr = LFUtils.lfToString(lf);
                	
                	log("==> Best logical form according to current model: " + bestLFStr);
                	if (bestLFStr == null) {
                //		LFUtils.plfToGraph(plf.packedLF, "parsesel", true);
                   		recogString = "";
                		addToWorkingMemory(newDataID(),lf);
                	}
                	else {
                	String scoreStr = new Double(decoder.getLastMaxScore()).toString();
                	if (scoreStr.length() >10)
                		scoreStr = scoreStr.substring(0,10);
                	log("==> Decoding score: " + scoreStr);
                	
                	PhonString phon = ComsysUtils.getPhonStringFromPair(plf, lf.logicalFormId);
                	log("==> Phonological string: \"" + phon.wordSequence +"\"");
                	recogString += decoder.recogError;
                	lastscore = decoder.getLastMaxScore();
                	
                	SelectedLogicalForm selectedLF = convertToSelectedLogicalForm(lf);
                	
                	selectedLF.score = lastscore;
                	selectedLF.phon = phon;
                	selectedLF.scoreOfSecondBest = decoder.getLast2ndMaxScore();
                
                	String id = newDataID();
                	addToWorkingMemory(id,selectedLF);
                	log("ID: "  + id);
                	}
                }
                else {
                	log("WARNING: parameter vector is not specified, cannot decode");
                }
            }
        }
        catch (Exception e) {
        	e.printStackTrace();
        }

    } // end executeParseSelectionTask
	
	
    private SelectedLogicalForm convertToSelectedLogicalForm (LogicalForm lf) {
    	SelectedLogicalForm selectedLF = new SelectedLogicalForm();

    	selectedLF.logicalFormId = lf.logicalFormId;
    	selectedLF.noms = lf.noms;
    	selectedLF.root = lf.root;
    	selectedLF.stringPos = lf.stringPos;
    	selectedLF.preferenceScore = lf.preferenceScore;
    	
    	return selectedLF;
    }
		
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

            log("Run parse selection...");

            lockComponent();
            sleepComponent(100); // plison - give enough time for other
                                // Componentes to start before sending
                                // anything
            unlockComponent();

            while (this.isRunning()) {

                    lockComponent();
                    
                    ListIterator<ProcessingData> i = m_dataObjects
                        .listIterator();

                    while (i.hasNext()) {

                        // get the data from the queue
                        ProcessingData data = i.next();

                        String dataID = data.getID();
                        String taskID = m_dataToProcessingGoalMap
                            .get(dataID);
                        String taskType = (String) m_taskToTaskTypeMap
                            .get(taskID);

                        if (taskType != null && data != null) {

                            try {
                                if (taskType
                                    .equals(ComsysGoals.PARSESELECTION_TASK)) {
                                	executeParseSelectionTask(data, taskType);
                                }
                                else {
                                    log("Unknown task type to Component in Comsys:parseSelection component");
                                } // end if..else check for task type
                                // inform the goal manager that the task
                                // has
                                // been completed, but unsuccessfully
                                try {
                                    taskComplete(
                                        taskID,
                                        TaskOutcome.ProcessingCompleteSuccess);
                                } 
                                catch (Exception e) {
                                    e.printStackTrace();
                                } // end try..catch
                            }
                            catch (ComsysException e) {
                                log("Exception while executing a task in speech synthesis: "
                                    + e.getMessage());
                                // inform the goal manager that the task
                                // has
                                // been completed, but unsuccessfully
                                // we may want to make this more
                                // specific
                                try {
                                    taskComplete(
                                        taskID,
                                        TaskOutcome.ProcessingCompleteFailure);
                                }
                                catch (Exception ex) {
                                    ex.printStackTrace();
                                } // end try..catch
                            } // end try..catch for Processing
                                // exceptions
                        }
                        else {
                            log("Nothing to Component: taskType / data null");
                        } // end
                        // Clean up!
                        // no matter what happened, remove the data item
                        // from the queue
                        i.remove();
                        m_taskToTaskTypeMap.remove(taskID);
                    } // end while
                    // Free the Component
                    unlockComponent();

                    // nah... put in a tiny sleep so it doesn't use 100%
                    // of
                    // the CPU!
                    sleepComponent(20);

       

            } // end while running
        }
        catch (Exception e) {
            e.printStackTrace();
        } // end try..catch
    } // end runComponent
	


} // end class
