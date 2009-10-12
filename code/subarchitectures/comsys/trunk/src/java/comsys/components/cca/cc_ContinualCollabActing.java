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
import cast.AlreadyExistsOnWMException;
import cast.SubarchitectureComponentException;
import cast.cdl.*;
import cast.core.CASTData;
import cast.core.CASTUtils;

import Abducer.*;
import comsys.processing.cca.ContextUpdate;
import comsys.processing.cca.ContinualCollaborativeActivity;
import comsys.processing.cca.MercuryUtils;
import comsys.processing.cca.AbducerUtils;
import comsys.processing.cca.StackUtils;
import comsys.processing.cca.ProofUtils;
import comsys.processing.cca.PrettyPrinting;
import comsys.processing.cca.Counter;
import comsys.processing.cca.abduction.BeliefModelSynchronization;
import comsys.processing.reference.belieffactories.AbstractBeliefFactory;

import beliefmodels.adl.*;
import beliefmodels.clarification.*;
import beliefmodels.domainmodel.cogx.*;

import binder.abstr.BeliefModelInterface;
import binder.components.Binder;
import binder.utils.BeliefModelUtils;

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

	private Counter counter = null;
	
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

        counter = new Counter("cca");
        
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
				ChangeFilterFactory.createLocalTypeFilter(beliefmodels.clarification.ClarificationRequest.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleClarificationRequest(_wmc);
					}
				});
		
		addChangeFilter(
				ChangeFilterFactory.createGlobalTypeFilter(beliefmodels.domainmodel.cogx.GroundedBelief.class, WorkingMemoryOperation.OVERWRITE),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleGroundedBelief(_wmc);
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
		log("Got a WM change: bound readings");
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
		log("Got a WM change: clarification request");
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

	private void handleGroundedBelief(WorkingMemoryChange _wmc) {
		log("Got a WM change: grounded belief");
		try {
			String id = _wmc.address.id;
			CASTData data = getWorkingMemoryEntry(id, Binder.BINDER_SA);
			String taskID = newTaskID();
			ProcessingData pd = new ProcessingData(newProcessingDataId());
			pd.add(data);
			m_proposedProcessing.put(taskID, pd);
			String taskGoal = ComsysGoals.CCA_VERIFICATION_TASK;
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
                        	if (taskType.equals(ComsysGoals.CCA_UNDERSTAND_TASK)
                        			|| taskType.equals(ComsysGoals.CCA_CLARIFICATION_REQUEST_TASK)) {
                        	
                        		ContextUpdate cu = null;
	                            if (taskType.equals(ComsysGoals.CCA_UNDERSTAND_TASK)) {
	                            	CASTData brsWM = data.getByType(CASTUtils.typeName(BoundReadings.class));
	                            	if (brsWM != null) {
	                            		cu = understandBoundReadings((BoundReadings) brsWM.getData());
	                            	}
	                            }
	                            else if (taskType.equals(ComsysGoals.CCA_CLARIFICATION_REQUEST_TASK)) {
	                            	CASTData crWM = data.getByType(CASTUtils.typeName(ClarificationRequest.class));
	                            	if (crWM != null) {
	                            		cu = understandClarifRequest((ClarificationRequest) crWM.getData());
	                            	}
	                            }
	                            updateContext(cu);
	                            actPublicly(selectAction(cu));
                        	}
                        	else if (taskType.equals(ComsysGoals.CCA_VERIFICATION_TASK)) {
                        		ContextUpdate cu = null;
                            	CASTData gbWM = data.getByType(CASTUtils.typeName(GroundedBelief.class));
                            	if (gbWM != null) {
                            		cu = processGroundedBelief((GroundedBelief) gbWM.getData());
                            	}
                            	updateContext(cu);
                            	actPublicly(selectAction(cu));
                        	}
                            else {
                                log("Unknown task type to process in Comsys:continualCollabActing component");
                            }                                
                            taskComplete(taskID, TaskOutcome.ProcessingCompleteSuccess);
                        }
                        catch (Exception e) {
                        	log("Exception while executing a task in cont. collab acting: " + e.getMessage());
                        	taskComplete(taskID, TaskOutcome.ProcessingCompleteFailure);
                        	e.printStackTrace();
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

    
    private ContextUpdate understandBoundReadings(BoundReadings boundReadings) {
    	log("got bound readings");

    	ccaEngine.addFactualContext(boundReadings.lform);
    	ccaEngine.addAnchoringContext(boundReadings);
		MarkedQuery[] proof = ccaEngine.understandProof(ContinualCollaborativeActivity.UNDERSTAND,
				AbducerUtils.term(boundReadings.lform.root.nomVar));

		if (proof != null) {
    		log("abductive proof found:\n" + PrettyPrinting.proofToString(proof));
    		return new ContextUpdate(proof);
		}
		else {
			log("no proof found");
			return understandingFailed();
		}
    }
    
    private ContextUpdate understandClarifRequest(ClarificationRequest cr) {
    	log("got clarif request");
    	ccaEngine.addCRContext(cr);
		MarkedQuery[] proof = ccaEngine.understandProof(ContinualCollaborativeActivity.CLARIFY,
				AbducerUtils.term(cr.id));

		if (proof != null) {
    		log("abductive proof found:\n" + PrettyPrinting.proofToString(proof));
    		return new ContextUpdate(proof);
		}
		else {
			log("no proof found");
			return new ContextUpdate();
		}
    }
    
    private ContextUpdate processGroundedBelief(GroundedBelief gb) {
    	ContextUpdate cu = new ContextUpdate();
    	cu.proof = new MarkedQuery[] { };
    	cu.intention = AbducerUtils.predicate("have_sensed", new Term[] { AbducerUtils.term(gb.grounding.modality) });
    	return cu;
    }
    
    private void updateContext(ContextUpdate cu) {
    	log("updating context");
		Belief[] beliefUpdates = ccaEngine.verifiableUpdate(cu.proof, getCurrentBeliefModel());
		
		if (beliefUpdates.length == 0) {
			log("no belief updates...");
		}
		else {
			log("starting belief model update, " + beliefUpdates.length + " beliefs total");
    		
			for (int i = 0; i < beliefUpdates.length; i++) {
				log("  update #" + i + ": " + PrettyPrinting.beliefToString(beliefUpdates[i]));

				Belief[] related = getBeliefsByUnionEntityId(referringUnionId(beliefUpdates[i])).toArray(new Belief[] {});
				
				boolean found = false;
				for (int j = 0; j < related.length; j++) {
					if (related[j].ags.equals(beliefUpdates[i].ags)) {
						mergeFormulaIntoBelief(related[j], (SuperFormula) beliefUpdates[i].phi);
						found = true;
					}
				}
				if (!found) {
					beliefUpdates[i].id = counter.inc("ub-" + referringUnionId(beliefUpdates[i]));
					addNewBelief(beliefUpdates[i]);
				}
			}
			log("done with belief model update");
		}
		
		log("syncing abducer with current belief model");
		ccaEngine.abducer.clearFactsByModality(ModalityType.K);
		syncWithBeliefModel(getCurrentBeliefModel());
    }

    private Predicate selectAction(ContextUpdate cu) {
    	return cu.intention;
    }

    private void actPublicly(Predicate action) {
    	if (action != null) {
	    	log("initiating public acting");
	    	ModalisedFormula actionMF = AbducerUtils.modalisedFormula(new Modality[] {AbducerUtils.eventModality()}, action);
			MarkedQuery[] actionProof = ccaEngine.generateProof(new ModalisedFormula[] {actionMF});
			
			if (actionProof != null) {
				log("Action proof:\n" + PrettyPrinting.proofToString(actionProof));
				
				// TODO: do this in a less hardcoded way
				if (toBeRealised(actionProof)) {				
	    			LogicalForm prodLF = AbducerUtils.factsToLogicalForm(ProofUtils.proofToFacts(actionProof), "dummy1");
	    			log("will realise this proto-LF: " + LFUtils.lfToString(prodLF));
	    			realizeLogicalForm(prodLF);
				}
				else {
					log("no action found in the action intention proof");
				}
			}
			else {
				log("public acting: no proof found");
			}
    	}
    	else {
    		log("no public action intention selected");
    	}
    }
    
    public ContextUpdate understandingFailed() {
    	ContextUpdate cu = new ContextUpdate();
    	cu.proof = new MarkedQuery[0];
    	cu.intention = AbducerUtils.predicate("not_understood", new Term[] { AbducerUtils.term("r") });
    	return cu;
    }
    
    public void realizeLogicalForm(LogicalForm lf) {
		ContentPlanningGoal cpg = new ContentPlanningGoal();
		cpg.cpgid = newDataID();
		cpg.lform = lf;
		try {
			addToWorkingMemory(newDataID(), cpg);
		} catch (AlreadyExistsOnWMException e) {
			e.printStackTrace();
		}
    }
        
	public void syncWithBeliefModel(BeliefModel model) {
		try {
			log("syncing with the belief model");
		
			Vector<Belief> vectKBeliefs = new Vector<Belief>();
			for (int i = 0; i < model.k.length; i++) {
				vectKBeliefs.add(getBelief(model.k[i]));
			}
		
			BeliefModelSynchronization.sync(ccaEngine.abducer, vectKBeliefs.toArray(new Belief[] {}));
		}
		catch (Exception e) {
			e.printStackTrace();
		}
/*
		Map<AgentStatus, Predicate> toAdd = new HashMap<AgentStatus, Predicate>();
		
		for (int i = 0 ; i < model.k.length; i++) {
			Belief b = null;
			try {
				b = getBelief(model.k[i]);
			}
			catch (Exception e) {
				e.printStackTrace();
			}

			//log("got a belief, id=" + model.k[i]);
			Modality[] mod = new Modality[] { AbducerUtils.kModality(b.ags) };
			String unionId = referringUnionId(b);

			//log("inspecting feats");

			for (int j = 0; j < ((ComplexFormula)b.phi).formulae.length ; j++) {
				SuperFormula formula = ((ComplexFormula)b.phi).formulae[j];

				Predicate pred = formulaToPredicate(unionId, formula);
				ModalisedFormula mf = AbducerUtils.modalisedFormula(mod, pred);

				if (pred != null) {
					log("adding " + MercuryUtils.modalisedFormulaToString(mf) + " ... from " + model.k[i]);
					ccaEngine.abducer.addFact(mf);
				}
			}
		}
		log("sync done");
*/
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
