package comsys.components.cca;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.*;

import comsys.arch.ComsysException;
import comsys.arch.ComsysGoals;
import comsys.arch.ProcessingData;
import comsys.datastructs.comsysEssentials.*;

import comsys.lf.utils.LFUtils;
import comsys.datastructs.lf.LogicalForm;

import cast.architecture.*;
import cast.SubarchitectureComponentException;
import cast.cdl.*;
import cast.core.CASTData;
import cast.core.CASTUtils;

import Abducer.*;
import comsys.processing.cca.ContinualCollaborativeActivity;
import comsys.processing.cca.MercuryUtils;
import comsys.processing.cca.AbducerUtils;
import comsys.processing.cca.StackUtils;
import comsys.processing.cca.ProofUtils;
import comsys.processing.cca.PrettyPrinting;

import beliefmodels.adl.*;
import beliefmodels.domainmodel.cogx.*;

import binder.abstr.BeliefModelInterface;

public class cc_ContinualCollabActing extends BeliefModelInterface {

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
	
	private String understandRulesFileName = null;
	private String understandFactsFileName = null;
	private String generateRulesFileName = null;
	private String generateFactsFileName = null;
    
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
		if (understandFactsFileName != null) ccaEngine.setUnderstandFactsFileName(understandFactsFileName);
		if (understandRulesFileName != null) ccaEngine.setUnderstandRulesFileName(understandRulesFileName);
		if (generateFactsFileName != null) ccaEngine.setGenerateFactsFileName(generateFactsFileName);
		if (generateRulesFileName != null) ccaEngine.setGenerateRulesFileName(generateRulesFileName);
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
						handleBoundReadings(_wmc);
					}
				});
		
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(ClarificationRequest.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleClarificationRequest(_wmc);
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

	private void handleBoundReadings(WorkingMemoryChange _wmc) {
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
	
	private void handleClarificationRequest(WorkingMemoryChange _wmc) {
		log("Got a WM change");
		try {
			String id = _wmc.address.id;
			CASTData data = getWorkingMemoryEntry(id);
			String taskID = newTaskID();
			ProcessingData pd = new ProcessingData(newProcessingDataId());
			pd.add(data);
			m_proposedProcessing.put(taskID, pd);
			String taskGoal = ComsysGoals.CCA_CLARIFICATION_REQUEST_TASK;
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
                            else if (taskType.equals(ComsysGoals.CCA_CLARIFICATION_REQUEST_TASK)) {
                            	executeClarificationRequestTask(data);
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
				MarkedQuery[] proof = ccaEngine.understandProof(ContinualCollaborativeActivity.UNDERSTAND,
						AbducerUtils.term(boundReadings.lform.root.nomVar));

				// print the proof ... 
				if (proof != null) { 	
            		log("Found an abductive proof:\n" + PrettyPrinting.proofToString(proof));
     		
            		log("starting verifiable update");
            		// TODO: where do I get the belief model?
            		Belief[] contextUpdates = ccaEngine.verifiableUpdate(proof, getCurrentBeliefModel());

            		if (contextUpdates.length == 0) {
            			log("no belief updates...");
            		}
            		else {
                		String ls = "updates = {\n";
                		for (int i = 0; i < contextUpdates.length; i++) {
                			ls += "  " + PrettyPrinting.beliefToString(contextUpdates[i]);
                			ls += (i < contextUpdates.length-1) ? ",\n" : "\n";
                			
                			Belief[] related = getBeliefsByUnionEntityId(referringUnionId(contextUpdates[i])).toArray(new Belief[] {});
                			
                			boolean found = false;
                			for (int j = 0; j < related.length; j++) {
                				if (related[j].ags.equals(contextUpdates[i].ags)) {
                					mergeFormulaIntoBelief(related[j], (SuperFormula) contextUpdates[i].phi);
                					found = true;
                				}
                			}
                			if (!found) {
                				contextUpdates[i].id = "update-" + referringUnionId(contextUpdates[i]);
                				addNewBelief(contextUpdates[i]);
                			}
                		}
                		ls += "}";
                		log(ls);
            		}
            		
            		ccaEngine.abducer.clearKFacts();
            		syncWithBeliefModel(getCurrentBeliefModel());
            		
        			log("looking for linguistic feedback");
        			ModalisedFormula[] assumptions = ProofUtils.proofToFacts(ProofUtils.filterAssumed(proof));
        			
        			// filter out just the event modality
        			ArrayList<ModalisedFormula> list = new ArrayList<ModalisedFormula>();
        			for (int i = 0; i < assumptions.length; i++) {
        				if (assumptions[i].m.length == 1 && assumptions[i].m[0] instanceof EventModality) {
        					list.add(assumptions[i]);
        				}
        			}

        			// generate feedback
            		MarkedQuery[] feedbackProof = ccaEngine.generateProof(list.toArray(new ModalisedFormula[0]));
            		if (feedbackProof != null) {
            			log("Production proof:\n" + PrettyPrinting.proofToString(feedbackProof));
            			if (toBeRealised(feedbackProof)) {
            				log("feedback to be realised");
            				
                			LogicalForm prodLF = AbducerUtils.factsToLogicalForm(ProofUtils.proofToFacts(feedbackProof), "dummy1");
                			log("will realise this proto-LF: " + LFUtils.lfToString(prodLF));
                			ContentPlanningGoal cpg = new ContentPlanningGoal();
                			cpg.cpgid = newDataID();
                			cpg.lform = prodLF;
                			addToWorkingMemory(newDataID(), cpg);
            			}
            			else {
            				log("no feedback for realisation");
            			}
            		}
            		
            		// TODO: update the belief model
            		log("update the belief model");            		
            	}
            }
    	}
        catch (Exception e) {
        	e.printStackTrace();
        }
    }

    public static String referringUnionId(Belief b) {
    	if (b.phi instanceof ComplexFormula) {
			
			for (int i = 0; i < ((ComplexFormula)b.phi).formulae.length ; i++) {
				SuperFormula formula = ((ComplexFormula)b.phi).formulae[i];
				
				if (formula instanceof UnionRefProperty) {
					return ((UnionRefProperty)formula).unionRef;
				}
			}
		}
		return null;
    }
    
    public Abducer.Predicate formulaToPredicate(String unionId, SuperFormula cf) {
    	Abducer.Predicate pred = new Abducer.Predicate();
    	pred.args = new Abducer.Term[2];
    	pred.args[0] = AbducerUtils.term("form-" + unionId);
    	
    	if (cf instanceof ObjectTypeProperty) {
    		pred.predSym = "objecttype";
    		pred.args[1] = AbducerUtils.term( ((ObjectTypeProperty)cf).typeValue.toString() );
    		return pred;
    	}

    	return null;
    }
    
	public void syncWithBeliefModel(BeliefModel model) {
		log("syncing with the belief model");
		for (int i = 0 ; i < model.k.length; i++) {
			Belief b = null;
			try {
				b = getBelief(model.k[i]);
			}
			catch (Exception e) {
				e.printStackTrace();
			}
			log("got a belief, id=" + model.k[i]);
			Modality[] mod = new Modality[] { AbducerUtils.kModality(b.ags) };
			String unionId = referringUnionId(b);

			log("inspecting feats");

			for (int j = 0; j < ((ComplexFormula)b.phi).formulae.length ; j++) {
				SuperFormula formula = ((ComplexFormula)b.phi).formulae[j];

				Predicate pred = formulaToPredicate(unionId, formula);
				ModalisedFormula mf = AbducerUtils.modalisedFormula(mod, pred);

				if (pred != null) {
					log("adding " + MercuryUtils.modalisedFormulaToString(mf));
					ccaEngine.abducer.addFact(mf);
				}
			}
		}
	}
	    
    private void executeClarificationRequestTask(ProcessingData pd) throws ComsysException {
    	log("interpreting an event");
    	try {	
            CASTData crWM = pd.getByType(CASTUtils.typeName(ClarificationRequest.class));
            if (crWM != null) {
				// get the data
            	ClarificationRequest cr = (ClarificationRequest) crWM.getData();
            	
				// construct the abductive proof

            	ccaEngine.addCRContext(cr);
				MarkedQuery[] proof = ccaEngine.understandProof(ContinualCollaborativeActivity.CLARIFY,
						AbducerUtils.term(cr.id));

				// print the proof ... 
				if (proof != null) { 	
            		log("Found an abductive proof:\n" + PrettyPrinting.proofToString(proof));
     		
            		log("starting verifiable update");
            		// TODO: where do I get the belief model?
            		Belief[] contextUpdates = ccaEngine.verifiableUpdate(proof, null);

            		String ls = "updates = {\n";
            		for (int i = 0; i < contextUpdates.length; i++) {
            			ls += "  " + PrettyPrinting.beliefToString(contextUpdates[i]);
            			ls += (i < contextUpdates.length-1) ? ",\n" : "\n";
            		}
            		ls += "}";
            		log(ls);
            		
            		if (contextUpdates.length == 0) {
            			log("no belief updates");
            		}
            		
        			log("looking for linguistic feedback");
        			ModalisedFormula[] assumptions = ProofUtils.proofToFacts(ProofUtils.filterAssumed(proof));
        			
        			// filter out just the event modality
        			ArrayList<ModalisedFormula> list = new ArrayList<ModalisedFormula>();
        			for (int i = 0; i < assumptions.length; i++) {
        				if (assumptions[i].m.length == 1 && assumptions[i].m[0] instanceof EventModality) {
        					list.add(assumptions[i]);
        				}
        			}

        			// generate feedback
            		MarkedQuery[] feedbackProof = ccaEngine.generateProof(list.toArray(new ModalisedFormula[0]));
            		if (feedbackProof != null) {
            			log("Production proof:\n" + PrettyPrinting.proofToString(feedbackProof));
            			if (toBeRealised(feedbackProof)) {
            				log("feedback to be realised");
            				
                			LogicalForm prodLF = AbducerUtils.factsToLogicalForm(ProofUtils.proofToFacts(feedbackProof), "dummy1");
                			log("will realise this proto-LF: " + LFUtils.lfToString(prodLF));
                			ContentPlanningGoal cpg = new ContentPlanningGoal();
                			cpg.cpgid = newDataID();
                			cpg.lform = prodLF;
                			addToWorkingMemory(newDataID(), cpg);
            			}
            			else {
            				log("no feedback for realisation");
            			}
            		}

            		// TODO: update the belief model
            		log("update the belief model");
            	}
            }
    	}
        catch (Exception e) {
        	e.printStackTrace();
        }
    }

    /**
     * Return true iff the proof assumes realisation of an utterance that can be extracted from the proof.
     * @param proof the proof
     * @return true if the proof assumes its realisation
     */
    private boolean toBeRealised(MarkedQuery[] proof) {
    	ModalisedFormula[] assumed = ProofUtils.proofToFacts(ProofUtils.filterAssumed(proof));
    	for (int i = 0; i < assumed.length; i++) {
    		if (assumed[i].m.length == 1 && assumed[i].m[0] instanceof Abducer.EventModality
    				&& assumed[i].p.predSym.equals("produce") && ProofUtils.termToString(assumed[i].p.args[0]).equals("r")) {
    			return true;
    		}
    	}
    	return false;
    }
    
    // =================================================================
    // CAST CONFIGURATION METHODS
    // =================================================================

    @Override
    public void configure(Map<String, String> _config) {
		if (_config.containsKey("--understandFacts")) {
			understandFactsFileName = _config.get("--understandFacts");
		}
		if (_config.containsKey("--generateFacts")) {
			generateFactsFileName = _config.get("--generateFacts");
		}
		
		if (_config.containsKey("--understandRules")) {
			understandRulesFileName = _config.get("--understandRules");
		}
		if (_config.containsKey("--generateRules")) {
			generateRulesFileName = _config.get("--generateRules");
		}
	}
    
}
