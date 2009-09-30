package comsys.components.cca;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.*;

import comsys.arch.ComsysException;
import comsys.arch.ComsysGoals;
import comsys.arch.ProcessingData;
import comsys.datastructs.comsysEssentials.PackedLFs;
import comsys.datastructs.comsysEssentials.BoundReadings;

import comsys.lf.utils.LFUtils;

import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.architecture.ManagedComponent;
import cast.SubarchitectureComponentException;
import cast.cdl.*;
import cast.core.CASTData;
import cast.core.CASTUtils;

import Abducer.*;
import comsys.processing.cca.ContinualCollaborativeActivity;
import comsys.processing.cca.MercuryUtils;
import comsys.processing.cca.AbducerUtils;

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
	

	// Main engine handling the processing for the component
	ContinualCollaborativeActivity ccaEngine = null; 
	
	private String rulesFileName = null;
	private String factsFileName = null;
    
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

		// Initialize the CCA engine
		ccaEngine = new ContinualCollaborativeActivity();
		// if needed, set facts/rules-filenames
		// initialize the abduction engine
		if (factsFileName != null) ccaEngine.setFactsFileName(factsFileName);
		if (rulesFileName != null) ccaEngine.setRulesFileName(rulesFileName);
		ccaEngine.initAbducer();
		
    } // end init
    
    
	public void start() {
		super.start();
		// now call the initialization method for the object
		init();
		// now do the rest
		// register change filters for ProductionLF, which triggers realization
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(BoundReadings.class,  WorkingMemoryOperation.ADD),
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
            log("Entering loop checking for data in continual collab component");
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
    }

    // =================================================================
    // COMPUTATION METHODS
    // =================================================================

    private void executeEventInterpretTask(ProcessingData pd) throws ComsysException {
    	log("interpreting an event");
    	try {	
            CASTData brsWM = pd.getByType(CASTUtils.typeName(BoundReadings.class));
            if (brsWM != null) {
				// get the data
            	BoundReadings boundReadings = (BoundReadings) brsWM.getData();
            	
				// construct the abductive proof
            	ccaEngine.addFactualContext(boundReadings.lform);
            	ccaEngine.addAnchoringContext(boundReadings);
				MarkedQuery[] proof = ccaEngine.constructProof(ContinualCollaborativeActivity.UNDERSTAND, boundReadings.lform);

				// print the proof ... 
				if (proof != null) { 	
            		String logString = "proof: body = [\n";
            		for (int i = 0; i < proof.length; i++) {
            			logString += MercuryUtils.modalisedFormulaToString(proof[i].body);
            			if (i < proof.length-1) { logString += ",\n"; }
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
    
    // =================================================================
    // CAST CONFIGURATION METHODS
    // =================================================================

    @Override
    public void configure(Map<String, String> _config) {
		if (_config.containsKey("--facts")) {
			log("have configured facts");
			factsFileName = _config.get("--facts");
		}
		
		if (_config.containsKey("--rules")) {
			log("have configured rules");
			rulesFileName = _config.get("--rules");
		}
	}
    
}
