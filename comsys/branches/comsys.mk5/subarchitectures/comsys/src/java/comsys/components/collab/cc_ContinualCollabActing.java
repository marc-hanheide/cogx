package comsys.components.collab;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.*;

import comsys.arch.ComsysException;
import comsys.arch.ComsysGoals;
import comsys.arch.ProcessingData;
import comsys.datastructs.comsysEssentials.PackedLFs;
import comsys.datastructs.SelectedLogicalForm;

import comsys.lf.utils.LFUtils;

import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.architecture.ManagedComponent;
import cast.SubarchitectureComponentException;
import cast.cdl.*;
import cast.core.CASTData;
import cast.core.CASTUtils;

import Abducer.*;
import comsys.processing.collab.MercuryUtils;
import comsys.processing.collab.AbdUtils;

public class cc_ContinualCollabActing extends ManagedComponent {

	// =================================================================
	// CLASS-INTERNAL GLOBAL VARIABLES
	// =================================================================

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

    // Identifiers for ProcessData objects
    private int pdIdCounter;
	
    private Ice.Communicator ic;
    private AbducerServerPrx abducer;

    private String rulesFilename = "/Users/sandius/Work/comsys.mk5/subarchitectures/comsys/abduction/rules.txt";
    private String factsFilename = "/Users/sandius/Work/comsys.mk5/subarchitectures/comsys/abduction/facts.txt";
    
    // =================================================================
    // CONSTRUCTOR METHODS
    // =================================================================

    private void init() {
        log("initializing continual collaborative acting component");

        // general information processing structures
        m_proposedProcessing = new Hashtable<String, ProcessingData>();
        m_dataToProcessingGoalMap = new Hashtable<String, String>();
        m_taskToTaskTypeMap = new Hashtable<String, String>();
        m_dataObjects = new Vector<ProcessingData>();
        pdIdCounter = 0;

        m_queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;

    	// connect to the server
        log("connecting to the server");
        try { 
            ic = Ice.Util.initialize(); 
            Ice.ObjectPrx base = ic.stringToProxy("AbducerServer:default -p 10000"); 
            abducer = AbducerServerPrxHelper.checkedCast(base); 

            if (abducer == null)
            	throw new Error("Invalid proxy"); 

            abducer.clearFacts();
            abducer.loadFactsFromFile(factsFilename);
            abducer.clearRules();
            abducer.loadRulesFromFile(rulesFilename);
        }
        catch (Ice.LocalException e) { 
            e.printStackTrace(); 
        }
        catch (Exception e) { 
            System.err.println(e.getMessage()); 
        }
    }
    
	public void start() {
		super.start();
		// now call the initialization method for the object
		init();
		// now do the rest
		// register change filters for ProductionLF, which triggers realization
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(SelectedLogicalForm.class,  WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
//						System.err.println("slf");
						handleWorkingMemoryChange(_wmc);
					}
				});
	}
    
    // =================================================================
    // CAST TASK METHODS
    // =================================================================

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
	}

	protected void taskRejected(String _goalID) {
		log("WARNING: The goal with ID [" + _goalID
			+ "] has been rejected.");
		m_proposedProcessing.remove(_goalID);
	}
    
    // =================================================================
    // CAST WORKING MEMORY MONITORING
    // =================================================================

	private void handleWorkingMemoryChange(WorkingMemoryChange _wmc) {
		log("Got a WM change");
		try {
			String id = _wmc.address.id;
			CASTData data = getWorkingMemoryEntry(id);
			String taskID = newTaskID();
			ProcessingData pd = new ProcessingData(newProcessingDataId());
			pd.add(data);
			m_proposedProcessing.put(taskID, pd);
			String taskGoal = ComsysGoals.CCA_UNDERSTAND_TASK;
			proposeInformationProcessingTask(taskID, taskGoal);
        	m_taskToTaskTypeMap.put(taskID, taskGoal);       
		}
		catch (SubarchitectureComponentException e) {
			e.printStackTrace();
		}
	}			
	
	private String newProcessingDataId() {
		String result = "pd" + pdIdCounter;
		pdIdCounter++;
		return result;
	}
	
    // =================================================================
    // CAST RUN COMPONENT
    // =================================================================

    public void runComponent() {
        try {	                
            log("Entering loop checking for data in continual collab acting component");
            while (this.isRunning()) {
                lockComponent();
                ListIterator<ProcessingData> i = m_dataObjects.listIterator();

                while (i.hasNext()) {

                    ProcessingData data = i.next();
                    String dataID = data.getID();
                    String taskID = m_dataToProcessingGoalMap.get(dataID);
                    String taskType = (String) m_taskToTaskTypeMap.get(taskID);

                    if (taskType != null && data != null) {
                        try {
                            if (taskType.equals(ComsysGoals.CCA_UNDERSTAND_TASK)) {
                                executeEventInterpretTask(data);
                            }
                            else {
                                log("Unknown task type to process in Comsys:continualCollabActing component");
                            }                                
                            taskComplete(taskID, TaskOutcome.ProcessingCompleteSuccess);
                        }
                        catch (ComsysException e) {
                            log("Exception while executing a task in cont. collab acting: " + e.getMessage());
                                taskComplete(taskID, TaskOutcome.ProcessingCompleteFailure);
                        }
                    }
                    else {
                        log("Nothing to process: taskType / data null");
                    }
                    i.remove();
                    m_taskToTaskTypeMap.remove(taskID);
                }
                unlockComponent();
                sleepComponent(20);
            }
        }
        catch (Exception e) {
            e.printStackTrace();
        }
        
        // close the connection to the server
        if (ic != null) { 
        	try { 
        		ic.destroy(); 
        	}
        	catch (Exception e) { 
        		System.err.println(e.getMessage()); 
        	} 
        }
    }

    // =================================================================
    // COMPUTATION METHODS
    // =================================================================

    private void executeEventInterpretTask(ProcessingData pd) throws ComsysException {
    	log("interpreting an event");
    	try {	
            CASTData slfWM = pd.getByType(CASTUtils.typeName(SelectedLogicalForm.class));
            if (slfWM != null) {

            	SelectedLogicalForm slf = (SelectedLogicalForm) slfWM.getData();
            	AbdUtils.addLFAsExplicitFacts(abducer, slf.lf);
            	
            	Abducer.UnsolvedQuery goal = new Abducer.UnsolvedQuery();
            	goal.mark = Abducer.Marking.Unsolved;
            	goal.body = new ModalisedFormula();
            	goal.body.m = new Modality[] {AbdUtils.modEvent()};
            	goal.body.p = AbdUtils.predicate("uttered", new Term[] {
            				AbdUtils.term("h"),
            				AbdUtils.term(slf.lf.root.nomVar)
            			});
            	goal.isConst = true;
            	goal.costFunction = "";
            	goal.constCost = 50.0f;

            	Abducer.MarkedQuery[] goals = new Abducer.MarkedQuery[] {goal};

            	log("proving: " + MercuryUtils.modalisedFormulaToString(goals[0].body));

//           	goal.body.termString = LFUtils.lfToMercString(slf.lf);
            	
            	ProveResult result = abducer.prove(goals);
//            	ProofResult result = abducer.proveGoal("e(now) : uttered(h, " + LFUtils.lfToMercString(slf.lf) + ").");
            	if (result == Abducer.ProveResult.SUCCESS) {
            		log("seems we've got a proof");
            		AbductiveProof p = abducer.getBestProof();

            		String logString = "proof: body = [\n";
            		for (int i = 0; i < p.body.length; i++) {
            			logString += MercuryUtils.modalisedFormulaToString(p.body[i].body);
            			if (i < p.body.length-1) { logString += ",\n"; }
            		}
            		logString += "\n]";
            		log(logString);

            	}
            }
    	}
        catch (Exception e) {
        	e.printStackTrace();
        }
    }

 /*
  * BROKEN
    private void select(ProcessingData pd) throws ComsysException {
        CASTData pWM = pd.getByType(CASTUtils.typeName(autogen.Abducer.AbductiveProof.class)); 
        if (pWM != null) {
        	autogen.Abducer.AbductiveProof p = (autogen.Abducer.AbductiveProof) pWM.getData();
        	autogen.Abducer.RevGoal rg = srv.reverseGoal(p);
        	log(tg.toString());
        }
    }
*/    
    
    // =================================================================
    // CAST CONFIGURATION METHODS
    // =================================================================

    public void configure(Properties _config) {
		if (_config.containsKey("--facts")) {
			factsFilename = _config.getProperty("--facts");
		}
		
		if (_config.containsKey("--rules")) {
			rulesFilename = _config.getProperty("--rules");
		}
	}
    
}