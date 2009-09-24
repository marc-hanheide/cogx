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
import cast.AlreadyExistsOnWMException;
import cast.cdl.*;
import cast.core.CASTData;
import cast.core.CASTUtils;

// -----------------------------------------------------------------
// COMSYS IMPORTS
// -----------------------------------------------------------------

import comsys.datastructs.comsysEssentials.*;
import comsys.processing.parse.PackedLFParseResults;
import comsys.arch.ProcessingData;


// import comsys.processing.asr.WordRecognitionLattice;
import comsys.arch.ComsysException;
import comsys.arch.ComsysGoals;
import comsys.processing.parse.ActiveCCGLexicon;
import comsys.processing.parse.ActiveWordLatticeParser;
import comsys.utils.ComsysUtils;

// -----------------------------------------------------------------
// INTERCONNECTIVITY IMPORTS
// -----------------------------------------------------------------
import interconnectivity.processing.AttentionMechanismPipeline;
import interconnectivity.processing.ContextActiveProcess;


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
// OPENCCG IMPORTS
// -----------------------------------------------------------------
import opennlp.ccg.parse.FrontierCatFilter;
import opennlp.ccg.parse.CategoryChartScorer;
import opennlp.ccg.parse.UnrestrictedBeamWidthFunction;
import opennlp.ccg.parse.ParseException;


// -----------------------------------------------------------------
// REPRESENTATION IMPORTS
// -----------------------------------------------------------------
import comsys.datastructs.lf.* ;
import comsys.lf.utils.LFPacking ;
import comsys.lf.utils.LFUtils ;
import java.util.Arrays;
// =================================================================
// CLASS DOCUMENTATION
// =================================================================

/**
The class <b>cc_Parser</b> ... 


<h4>Control over component processing tasks</h4>

<h4>Handling active processes</h4>

An <i>active process</i> is a process which has a selectional attention 
mechanism that is triggered by one or more types of data appearing on 
the subarchitecture's working memory. Crucial for active processes is that
any new data is first used to update the process its internal representations, 
before any tasks on these representations are triggered. 
<p>
Upon initialization of an active process, the component stores the individual 
processes in a vector <tt>activeProcesses</tt> to enable iteration over the 
processes, and stores the mapping from data types to (vectors of) active processes 
in the hashtable <tt>activeDataMap</tt>. These types are, in turn, used to 
register change filters with the working memory.
<p>
Working memory data is processed in two methods. The method <i>handleWorkingMemoryChange</i>, 
proposes tasks for the data types on which processes in this component operate. The method
<i>handleActiveDataWorkingMemoryChange</i> processes data relevant to active
processes, providing them to the active processes for which these types have been registered.
Because there may be an overlap between the sets of data types handled in these methods, 
we need to ensure that any active data type is first forwarded to the active process to 
update any internal representations, before tasks are being proposed. We therefore check 
whether a task-relevant data type is also active, and first update active processes accordingly. 
Because the change filters are threaded methods, we cannot rely on the handling methods always
being called in the right order directly by the change filters. The class therefore maintains
a list (TreeSet) of working memory change IDs <tt>processedActiveData</tt> to check which changes have 
already been handled. This way we can ensure consistency, and avoid double processing of a single 
working memory change. 
 


@version 090520
@started 070813
@author Geert-Jan M. Kruijff (gj@dfki.de)

*/ 


public class cc_Parser
	extends ManagedComponent {

    // =================================================================
    // GLOBAL DATA STRUCTURES
    // =================================================================

    // Hashtable used to record the tasks we want to carry out
    private Hashtable<String, ProcessingData> m_proposedProcessing;

    // Hashtable linking data IDs to goal IDs
    private Hashtable<String, String> m_dataToProcessingGoalMap;

    // Vector with objects to be processed, can be ComSys:PhonString,...
    private Vector<ProcessingData> m_dataObjects;

	// Vector with active processes 
	private Vector<ContextActiveProcess> activeProcesses; 
	
	// TreeSet with the identifiers of already processed active data types
	private TreeSet processedActiveData;
	
	// Hashtable mapping data types to (vectors of) active processes
	private Hashtable<String,Vector> activeDataMap;

    // Processing Data id counter
    private int pdIdCounter;

	// Filename of the utterance grammar to be used 
	public String grammarFile; 

	// The grammatical inference engine
	public ActiveWordLatticeParser parser;
	
	// The grammar access
	ActiveCCGLexicon grammar; 

	// Map with (unique) identifiers of PackedLogicalForm objects to PackedLFs in which they are stored
	Hashtable<String,String> plfToPackedLFsId;

	// Whether to merge nominals with same propositional content
	// but different ontological sort
	boolean mergeSorts = false ;
	boolean mergeSortsForEmptyProps = false ;
	
	// Whether to generate DOT graphs
	boolean generateLFGraphs = false ;
	boolean generatePackedGraph = false ;
	
	// whether to rename nominal identifiers
	boolean renameNominalIds = false ;
	
	// the subdirectory in which to store the DOT and PNG files
	String graphsDir = "graphs";

    String asr_subarch;

    // =================================================================
    // CONSTRUCTOR METHODS
    // =================================================================

    /**
     * The unary constructor. The initialization is called after the start() method. 
     * 
     * @param _id
     */
    public cc_Parser () {

    } // constructor/1

	/**
     * The method <i>init</i> initializes the global data structures,
     * and sets the ontology for this component. The variable
     * <tt>grammarFile</tt> is set in the <i>configure</tt> method.
     * 
     * @see #configure
	*/

	public void init () { 
        // Component
        log("Initializing utterance interpretation component");
         // Make all the comsys queue changes to avoid missing updates
        m_queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;

        // Data processing
        m_proposedProcessing = new Hashtable<String, ProcessingData>();
        m_dataToProcessingGoalMap = new Hashtable<String, String>();
        m_dataObjects = new Vector<ProcessingData>();
		pdIdCounter = 0;
		plfToPackedLFsId = new Hashtable<String,String>();
		
		try { 
			activeProcesses = new Vector<ContextActiveProcess>();
			activeDataMap	= new Hashtable<String,Vector>();
			processedActiveData = new TreeSet();
		
			// Parsing processes: set up the grammar
			grammar = new ActiveCCGLexicon();
			if (m_bLogOutput) { 
				//grammar.setLogLevel(grammar.LOG_TRUE);
			} else {
				//grammar.setLogLevel(grammar.LOG_FALSE);			
			} 
			grammar.setGrammar(grammarFile);
			// initialize the grammar with an empty attention mechanism pipeline, 
			// i.e no filtering (required as per ContextActiveProcess)
			grammar.registerAttentionMechanism(new AttentionMechanismPipeline());
			// add this active process to the list of active processes
			activeProcesses.addElement(grammar);
			// add the data types of this process to the active data map. 
			this.addActiveDataTypes(grammar);
			
			// Parsing processes: set up the actual parser, connect it to the grammar
			parser	= new ActiveWordLatticeParser();
			if (m_bLogOutput) { 
				parser.setLogLevel(parser.LOG_TRUE);
			} else {
				parser.setLogLevel(parser.LOG_FALSE);			
			} 
			// Register the grammar: this initializes the connection with the lexicon, 
			// and creates the ccg parser with the corresponding openccg grammar
			parser.registerGrammarAccess(grammar); 
			// Register the component for callback on CAST functionality
			parser.registerComponent(this);
			// initialize the parser with an empty attention mechanism pipeline, 
			// i.e no filtering (required as per ContextActiveProcess)
			parser.registerAttentionMechanism(new AttentionMechanismPipeline());
			// initialize the parser with a category scorer on sign hashes, as post-parsing step pruning
			CategoryChartScorer cshs = new CategoryChartScorer();
			// cshs.setLogging(m_bLogOutput);
			cshs.setBeamWidthFunction(new UnrestrictedBeamWidthFunction());
			parser.registerChartScorer(cshs);
			// register a frontier filter that looks at categories, at the moment all pass so single-word parsing
			FrontierCatFilter cfilter = new FrontierCatFilter();
			// cfilter.setLogging(m_bLogOutput);
			parser.registerFrontierFilter(cfilter);
			// add this active process to the list of active processes
			activeProcesses.addElement(parser);			
			// add the data types of this process to the active data map. 
			this.addActiveDataTypes(parser);			
		} 
		catch (ComsysException e) { 
			log("Fatal error: "+e.getMessage());
			e.printStackTrace();
			System.exit(0);
		} // end try..catch for setting up parser and grammar
	} // end init
	
	/**
		The method <i>addActiveDataTypes</i> adds a data types for the given active 
		process	to the <tt>activeDataMap</tt> hashtable. 
			
		@param type		The data type
		@param process	The active process (as object)
		@see   #addActiveDataType(String,ContextActiveDataProcess)
	*/ 
	
	private void addActiveDataTypes (ContextActiveProcess process) { 
		Iterator adtsIter = process.getContextDataTypes();
		while (adtsIter.hasNext()) { 
			String dataType = (String) adtsIter.next();
			addActiveDataType(dataType,process);
		} // end while over context data types
	} // end addActiveDataTypes
	
	
	/**
		The method <i>addActiveDataType</i> adds a data type and the active process
		to the <tt>activeDataMap</tt> hashtable. This table contains, for each data
		type, a vector of active processes that have declared this data type as
		context data.  
	
		@param type		The data type
		@param process	The active process (as object)
	*/ 
	
	private void addActiveDataType (String type, ContextActiveProcess process) { 
		log("type added: " +  type.toString());
		if (activeDataMap.containsKey(type)) { 
			Vector processes = (Vector) activeDataMap.get(type);
			processes.addElement(process);
			activeDataMap.put(type,processes);			
		} else { 
			Vector processes = new Vector();
			processes.addElement(process);
			activeDataMap.put(type,processes);
		} // end if..else check whether type has already been declared for other processes
	} // end addActiveDataType
	
	
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
	
    
    public void configure(Map<String, String> _config) {
     //   _config.list(System.out);
     //   configure(_config);
        String parserArg = "";
        if (_config.containsKey("--grammar")) {
            grammarFile = _config.get("--grammar");
        }
        else {
            grammarFile = "./grammars/openccg/moloko.v4/grammar.xml";
        } // end if..else check for command-line argument
		
		
	       // merging sorts option
        if (_config.containsKey("--mergeSorts")
            || _config.containsKey("-m")) {
            log("merging ontological sorts");
            mergeSorts = true;
        }

      // merging sorts option
        if (_config.containsKey("--asr_subarch")) {
            asr_subarch = _config.get("--asr_subarch");
           log("using the subarchitecture " + asr_subarch);
        }
        
        // merging sorts option for empty sorts option
        if (_config.containsKey("--mergeSortsForEmptyProps")
            || _config.containsKey("-m2")) {
            log("merging ontological sorts for empty propositions");
            mergeSortsForEmptyProps = true;
        }
        
        // generate a PNG file (using DOT) for each logical form
        if (_config.containsKey("--generateLFGraphs")
            || _config.containsKey("-glfg")) {
            log("generate DOT graphs representing the logical forms");
            generateLFGraphs = true;
        }
        
        // generate a PNG file (using DOT) for the packed logical form
        if (_config.containsKey("--generatePackedGraph")
            || _config.containsKey("-gpg")) {
            log("generate a DOT graph representing the packed logical form");
            generatePackedGraph = true;
        }
        
        // rename the nominal identifiers
        if (_config.containsKey("--renameNominalIds")
            || _config.containsKey("-rename")) {
            log("rename the nominal Identifiers");
            renameNominalIds = true;
        }
        
        if (_config.containsKey("--graphsDir")) {
            graphsDir = _config.get("--graphsDir");
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
		// now do the rest
        //try {

        	
			// register change filters for the active data types
			for (Iterator adtsIter = activeDataMap.keySet().iterator(); adtsIter.hasNext(); ) { 
				String adtsType = (String) adtsIter.next();
				try {
				addChangeFilter(
						ChangeFilterFactory.createLocalTypeFilter((Class<Ice.Object>)Class.forName(adtsType.replace("::", ".").substring(1)),  WorkingMemoryOperation.ADD),
								new WorkingMemoryChangeReceiver() {
									public void workingMemoryChanged(WorkingMemoryChange _wmc) {
										handleActiveDataWorkingMemoryChange(_wmc);
									}
								});		
				}
				catch (Exception e) {
					e.printStackTrace();
				}
			} // end for over unique active data types

			// register change filters for PhonString, which triggers parsing
            addChangeFilter(
            		ChangeFilterFactory.createGlobalTypeFilter(PhonString.class,  WorkingMemoryOperation.ADD),
                new WorkingMemoryChangeReceiver() {
                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                    	handleWorkingMemoryChange(_wmc, true);
                    }
                });
            
			// register change filters for WordRecognitionLattices, which triggers parsing
      /**      addChangeFilter(
            		ChangeFilterFactory.createGlobalTypeFilter((Class<Ice.Object>)WordRecognitionLattice.class,  WorkingMemoryOperation.ADD),
                new WorkingMemoryChangeReceiver() {
                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                    	log("Word recognition lattice successfully received!");
                    	handleWorkingMemoryChange(_wmc, true);
                    }
                }); */

			// register change filters for PackedLFs, which triggers continuation of parsing				
            addChangeFilter(
            		ChangeFilterFactory.createLocalTypeFilter(PackedLFs.class,  WorkingMemoryOperation.ADD),
                new WorkingMemoryChangeReceiver() {
                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                         handleWorkingMemoryChange(_wmc, false);
                    }
                }); 
				
			// register change filters for PackedLFs, which triggers continuation or finalization of parsing				
            addChangeFilter(
            		ChangeFilterFactory.createLocalTypeFilter(PackedLFs.class,  WorkingMemoryOperation.OVERWRITE),
                new WorkingMemoryChangeReceiver() {
                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                         handleWorkingMemoryChange(_wmc, false);
                    }
                });				
				
				
        //}
        //catch (SubarchitectureComponentException e) {
        //    e.printStackTrace();
        //} // end try..catch
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
    private void handleWorkingMemoryChange(WorkingMemoryChange _wmc, boolean isPhonString) {
        try {
            // get the id of the working memory entry
            String id = _wmc.address.id;
            // get the data from working memory and store it with its id
            
            CASTData data;
            if (isPhonString) {
                if (asr_subarch != null) {
            	data = getWorkingMemoryEntry(id, asr_subarch);
                }
                else {
                    data = getWorkingMemoryEntry(id);
                    }
            }
            else {
            	data = getWorkingMemoryEntry(id);
            }
            
			// check whether this is data used by active processes; 
			// if so, then first call the method for handling active data
            
			if (activeDataMap.containsKey(data.getType())) { 
				handleActiveDataWorkingMemoryChange(_wmc);
			} // end if.. check whether this is active data
            // get a new id for the task
            String taskID = newTaskID();
            // store the data we want to process for later
            ProcessingData pd = new ProcessingData(
                newProcessingDataId());
            pd.add(data);
            m_proposedProcessing.put(taskID, pd);
			// set the goal
            String taskGoal = ComsysGoals.INCREMENTAL_PARSING_STEP_TASK;
            System.out.println("ID: " + taskID);
            System.out.println("goal: " + taskGoal);
			// store the goal with its information
            proposeInformationProcessingTask(taskID, taskGoal);
        }
        catch (SubarchitectureComponentException e) {
            e.printStackTrace();
        } // end try..catch
    } // end handleWorkingMemoryChange	
	
    
    
	/**
		The method <i>handleActiveDataWorkingMemoryChange</i> takes a working memory
		change with an active data type, and informs all the active processes about
		the new information so that they can update their internal representations. 
	
		@param _wmc The working memory change with the active data. 
	*/ 
	
	private void handleActiveDataWorkingMemoryChange(WorkingMemoryChange _wmc) { 
        try {
            // get the id of the working memory entry
            String id = _wmc.address.id;
			// check whether the change has already been processed
			// (needed because both handling*WMC methods can be called concurrently)

			if (!processedActiveData.contains(id)) { 
				// get the data from working memory and store it with its id
				CASTData data = getWorkingMemoryEntry(id);			
				// get the relevant active processes
				Vector processes = (Vector) activeDataMap.get(data.getType()); 
				// iterate over the processes, informing each
				for (Iterator prcsIter = processes.iterator(); prcsIter.hasNext(); ) { 
					ContextActiveProcess process = (ContextActiveProcess) prcsIter.next();
					process.updateContextData(data);
				} // end for over processes
				// finally, add the data id to the treeset, stating it has been handled
				processedActiveData.add(id);
			} // end if check whether wmc has already been handled
        }
        catch (SubarchitectureComponentException e) {
            e.printStackTrace();
        } // end try..catch		
	} // end handleActiveDataWorkingMemoryChange
	
	
	/** Returns a new identifier for a ProcessingData object 
		@return String The new (unique) identifier
	*/

    private String newProcessingDataId() {
        String result = "pd" + pdIdCounter;
        pdIdCounter++;
        return result;
    } // end newProcessingDataId
	
	
	/** 
		Stores the given packed logical form. The method retrieves the 
		PackedLFs object from working memory, updates the plf object in 
		it, and then stores the PackedLFs object again, with updated
		finalized flag.
	*/ 
	
	public void storePackedLF (PackedLogicalForm plf, int finalizedFlag) { 
		log("Storing pruned packed logical form on working memory");
		if (plfToPackedLFsId.containsKey(plf.packedLFId)) { 
			// get the packedlf id = wmc address 
			String packedLFId = plfToPackedLFsId.get(plf.packedLFId);
			try { 
				// try to get the packed lf from WM
				CASTData data = getWorkingMemoryEntry(packedLFId);			
				String dataType = data.getType();
				if (dataType.equals(CASTUtils.typeName(PackedLFs.class))) {
					// get the object
					PackedLFs packedLFs = (PackedLFs) data.getData();
					// set the plf
					packedLFs.packedLF = plf;
					// set the finalized flag
					packedLFs.finalized = finalizedFlag;
					// store the object
					//overwriteWorkingMemory(data.getID(),
					//		packedLFs,
					//		OperationMode.BLOCKING);
					overwriteWorkingMemory(data.getID(),
										   packedLFs);
					
					log("Successfully stored packed logical form on working memory");
				} else { 
					System.err.println("[ERROR:UtteranceInterpretation] Retrieved data does not contain PackedLFs ["+packedLFId+"]");				
				} // end if..else check for correct data type
			}	catch (SubarchitectureComponentException e) {
				System.err.println("[ERROR:UtteranceInterpretation] Could not retrieve PackedLFs ["+packedLFId+"]");
				e.printStackTrace();
			} // end try..catch		
	
		} else {
			System.err.println("[WARNING:UtteranceInterpretation] No PackedLFs ID found for PLF ["+plf.packedLFId+"]");
		} // end if..else
	} // end 
	
	
	
	// =================================================================
    // EXECUTION METHODS
    // =================================================================	
	
	public PackedLFs lastAddedPLf ;
	
	/** The method <i>executeParseTask</i> triggers a parsing step, in 
		analyzing a given PhonString object. 
	
		@param pd The processing data to be used in parsing
		@throws ComsysException Thrown if there is no data to parse
	*/ 
		
    public void executeParseTask(ProcessingData pd)
            throws ComsysException, ParseException {
        Vector<String> pdTypes = pd.getTypes();
           
        if (pdTypes.contains(CASTUtils.typeName(PhonString.class))) {
            CASTData data = pd
                .getByType(CASTUtils.typeName(PhonString.class));
            String dataType = data.getType();
            log("Execute parse task on data item [" + dataType + "]");
            if (data != null) {
			
				log("Now try to start parsing with the phon string");
                // get the input string
                PhonString phonString = (PhonString) data.getData();
				// let the parser start the incremental analysis
                
                if (phonString.wordSequence.length() != 0) {
                	PackedLFParseResults results = (PackedLFParseResults) parser.incrParse(phonString);
                	// put the results on the working memory
                	// now write the results to the WM, as a PackedLFs 
                						
					PhonStringLFPair[] pairs = ComsysUtils.convertPhonString2LFPairs(results);
					
                	PackedLFs plf = new PackedLFs(
                			newDataID(),
                			pairs, 
                			new PhonString[0],
                			results.stringPos, 
                			results.plf,
                			results.finalized, 
                			"interpretation", 
                			new NonStandardRulesAppliedForLF[0]);
                	
                	log("Add the newly formed PackedLF to working memory, finalized is ["+results.finalized+"] ");
					
					// Update the maps 
					String packedLFsId = plf.id;
					String plfId = "";
					if (results != null) { 
						if (results.plf != null) { 
							plfId = results.plf.packedLFId;
						} else { 
							throw new ParseException("Cannot parse this thing");
						} 
					} else { 
						throw new ParseException("Cannot parse this thing");
					} 	

                	try {
                		// addToWorkingMemory(packedLFsId,	ComsysOntology.PACKEDLFS_TYPE,	plf);
						addToWorkingMemory(packedLFsId,	plf); // refactored, data type determined from provided object
						lastAddedPLf = plf;
						plfToPackedLFsId.put(plfId,packedLFsId);
                	}
                	catch (Exception e) {
                		e.printStackTrace();
                		throw new ComsysException(e.getMessage());
                	} // end try..catch overwriting working memory
                }
            }
            else {
                throw new ComsysException(
                    "Error: parsing task on empty data for type ["
                        + dataType + "]");
            } // end if..else check for data object
        } 
        
       /** 
        else if (pdTypes.contains(CASTUtils.typeName(WordRecognitionLattice.class))) {
            CASTData data = pd
                .getByType(CASTUtils.typeName(WordRecognitionLattice.class));
            String dataType = data.getType();
            log("Execute parse task on data item [" + dataType + "]");
            if (data != null) {
			
				log("Now try to start parsing with the word recognition lattice");
                // get the input string
				WordRecognitionLattice lattice = (WordRecognitionLattice) data.getData();
				// let the parser start the incremental analysis
                
                if (lattice != null && lattice.getMaximumLength() > 0 ) {
                	PackedLFParseResults results = (PackedLFParseResults) parser.parse(lattice);
                	// put the results on the working memory
                	// now write the results to the WM, as a PackedLFs 
                	
					NonStandardRulesAppliedForLF[] nonStandardRulesForLf = 
						ComsysUtils.convertNonStandardRulesDataStructs(results);
					PhonString[] nonParsablePhonStrings = new PhonString[results.nonParsablePhonStrings.size()];
					int i = 0;
					for (Enumeration<PhonString> g = results.nonParsablePhonStrings.elements(); g.hasMoreElements();) {
						nonParsablePhonStrings[i] = g.nextElement();
						i++;
					}
					PhonStringLFPair[] pairs = ComsysUtils.convertPhonString2LFPairs(results);

                	PackedLFs plf = new PackedLFs(
                			newDataID(),
                			pairs, 
                			nonParsablePhonStrings,
                			results.stringPos, 
                			results.plf,
                			results.finalized, 
                			"interpretation", 
                			nonStandardRulesForLf);
                	
                	log("Add the newly formed PackedLF to working memory, finalized is ["+results.finalized+"] ");
					
					// Update the maps 
					String packedLFsId = plf.id;
					String plfId = "";
					if (results != null) { 
						if (results.plf != null) { 
							plfId = results.plf.packedLFId;
						} else { 
							throw new ParseException("Cannot parse this thing");
						} 
					} else { 
						throw new ParseException("Cannot parse this thing");
					} 	

                	try {
                		// addToWorkingMemory(packedLFsId,	ComsysOntology.PACKEDLFS_TYPE,	plf);
						if (getSubarchitectureID() != null) addToWorkingMemory(packedLFsId,	plf); // refactored, data type determined from provided object
						plfToPackedLFsId.put(plfId,packedLFsId);
						lastAddedPLf = plf;
						log("Insertion of PLF into working memory successful");
                	}
                	catch (Exception e) {
                		e.printStackTrace();
                		throw new ComsysException(e.getMessage());
                	} // end try..catch overwriting working memory
                }
            }
            else {
                throw new ComsysException(
                    "Error: parsing task on empty data for type ["
                        + dataType + "]");
            } // end if..else check for data object
        } 
        
        */
        
        else if (pdTypes.contains(CASTUtils.typeName(PackedLFs.class))) { 
			CASTData data = pd
                .getByType(CASTUtils.typeName(PackedLFs.class)); 
            String dataType = data.getType();
            log("Execute parse task on data item [" + dataType + "]"); 
            if (data != null) {
				// get the packed logical forms
				PackedLFs plfs = (PackedLFs) data.getData();
                // get the input string
                PhonString phonString = plfs.phonStringLFPairs[0].phonStr;
				// the active process has already been updated with the PackedLFs 
				// (if registered) through handling the working memory changes 
				// trigger the parser to continue the incremental analysis
				// -- question is of course whether we SHOULD continue at all! 
				// at the same time, we may want to do one final round of pruning
				// after parsing has finished, to prune the final set of analyses. 
				log("Check whether the parsing has finished: ["+plfs.finalized+"]");
				if (plfs.finalized < parser.FINAL_PARSE) { 
					log("Now try to continue parsing with the phon string");
					PackedLFParseResults results = (PackedLFParseResults) parser.incrParse(phonString);
					// now write the results to the WM, as a PackedLFs 
					if (results.plf != null) {
				
						PhonStringLFPair[] pairs = ComsysUtils.convertPhonString2LFPairs(results);
						
	                	PackedLFs plf = new PackedLFs(
	                			data.getID(),
	                			pairs, 
	                			new PhonString[0],
	                			results.stringPos, 
	                			results.plf,
	                			results.finalized, 
	                			"interpretation", 
	                			new NonStandardRulesAppliedForLF[0]);					
	                	
	                	log("Updating PackedLF in working memory, finalized flag is ["+results.finalized+"]");
					try {
						//overwriteWorkingMemory(data.getID(),
						//	plf, OperationMode.BLOCKING);
						// in CAST v2, BLOCKING seems to be gone .. .
						overwriteWorkingMemory(data.getID(),plf);						
						
						lastAddedPLf = plf;
						plfToPackedLFsId.put(results.plf.packedLFId,data.getID());							
					}
					catch (Exception e) {
						e.printStackTrace();
						throw new ComsysException(e.getMessage());
					} // end try..catch overwriting working memory
					} 
					else {
						log("Packed logical form is empty --> not inserting into working memory");
						throw new ParseException("Packed logical form is empty --> not inserting into working memory");
					}
				} else { 
					log("PackedLFs provide a complete, and finally pruned, interpretation -- no need to parse");
				} // end if..else check whether to parse
			}
            else {
                throw new ComsysException(
                    "Error: parsing task on empty data for type ["
                        + dataType + "]");
            } // end if..else check for data object
        }  else {
            throw new ComsysException(
                "Error: parsing task on illegal data type ["
                    + pd.getTypes() + "]");
        } // end if..else check for data types
    } // end executeParseTask
	
	
		
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
            log("Entering loop checking for data in utterance interpretation component");
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
                            executeParseTask(data);
                            // inform the goal manager that the task has
                            // been completed
                            //try {
                                taskComplete(
                                    taskID,
                                    TaskOutcome.ProcessingCompleteSuccess);
                            //}
                            //catch (SubarchitectureComponentException e) {
                            //    e.printStackTrace();
                            //} // end try..catch
                        }
                        catch (ComsysException e) {
                            log("Exception while executing a task in utterance interpretation: "
                                + e.getMessage());
                            e.printStackTrace();
                            // inform the goal manager that the task has
                            // been completed, but unsuccessfully
                            // we may want to make this more specific
                            //try {
                                taskComplete(
                                    taskID,
                                    TaskOutcome.ProcessingCompleteFailure);
                            //}
                            //catch (SubarchitectureComponentException ex) {
                            //    ex.printStackTrace();
                            //} // end try..catch
                        } 					
						catch (ParseException pe) {
							log("Parsing exception while executing a task in utterance interpretation: "
								+ pe.getMessage());

							// log(">>>>>> SAY SOMETHING NICE HERE <<<<<<<");
							try {
								// Send feedback to be realized
								LogicalForm comGoal = LFUtils.convertFromString("@d1:dvp(cg ^ <FeedBackSource>parser ^ <Polarity>negative ^ <Content>(f1:feedback ))");
								ContentPlanningGoal cpg = new ContentPlanningGoal();
								cpg.lform = comGoal;
								addToWorkingMemory(newDataID(), cpg); 
								// Indicate that the task could not be completed
								taskComplete(
											 taskID,
											 TaskOutcome.ProcessingCompleteFailure);
							}
							catch (AlreadyExistsOnWMException ioe) { 
								log("ERROR: "+ioe.getMessage());
							}
							catch (SubarchitectureComponentException ex) {
								ex.printStackTrace();
							} // end try..catch
						} // end try..catch for processing exceptions
					
                    }
					
                    else {
                        log("Nothing to process: data null");
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
                // waitForNotifications(m_dataObjects);
				waitForNotifications();
				
            } // end while running
        }
        catch (Exception e) {
            e.printStackTrace();
        } // end try..catch
    } // end runComponent
	
	
	


} // end class
