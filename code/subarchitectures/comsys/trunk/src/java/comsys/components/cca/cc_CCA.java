package comsys.components.cca;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
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
import cast.UnknownSubarchitectureException;
import cast.cdl.*;
import cast.core.CASTData;
import cast.core.CASTUtils;

import Abducer.*;
import comsys.processing.cca.BeliefUtils;
import comsys.processing.cca.ContextUpdate;
import comsys.processing.cca.ContinualCollaborativeActivity;
import comsys.processing.cca.MercuryUtils;
import comsys.processing.cca.AbducerUtils;
import comsys.processing.cca.ProofStack;
import comsys.processing.cca.PrettyPrinting;
import comsys.processing.cca.Counter;
import comsys.processing.cca.abduction.BeliefModelSynchronization;
import comsys.processing.cca.abduction.ModalityFactory;
import comsys.processing.cca.abduction.PredicateFactory;
import comsys.processing.cca.abduction.ProofUtils;
import comsys.processing.reference.belieffactories.AbstractBeliefFactory;

import beliefmodels.adl.*;
import beliefmodels.adl.Agent;
import beliefmodels.clarification.*;
import beliefmodels.domainmodel.cogx.*;

import binder.abstr.BeliefModelInterface;
import binder.components.Binder;
import binder.utils.BeliefModelUtils;

public class cc_CCA extends BeliefModelInterface {

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
	
	private String rulesetFilename = null;

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

		if (rulesetFilename != null) {
			try {
				BufferedReader f = new BufferedReader(new FileReader(rulesetFilename));
				String file = null;
				while ((file = f.readLine()) != null) {
					log("adding file " + file);
					ccaEngine.addFileToLoad(file);
				}
				f.close();
			}
			catch (FileNotFoundException e) {
				log("ruleset filename not found");
			}
			catch (IOException e) {
				log("I/O exception while reading files from list");
				e.printStackTrace();
			}
		}
		else {
			log("no ruleset to read");
		}
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
	                            actPublicly(selectAction(updateContext(cu)));
                        	}
                        	else if (taskType.equals(ComsysGoals.CCA_VERIFICATION_TASK)) {
                        		ContextUpdate cu = null;
                            	CASTData gbWM = data.getByType(CASTUtils.typeName(GroundedBelief.class));
                            	if (gbWM != null) {
                            		cu = processGroundedBelief((GroundedBelief) gbWM.getData());
                            	}
                            	if (cu != null) {
                            		actPublicly(selectAction(updateContext(cu)));
                            	}
                            	else {
                            		log("context update is null in GroundedBelief handle");
                            	}
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
				PredicateFactory.term(boundReadings.lform.root.nomVar));

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
				PredicateFactory.term(cr.id));

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
    	//cu.proof = new MarkedQuery[] { };
    	if (ccaEngine.stack.beliefPresent(gb.id)) {
        	ContextUpdate cu = new ContextUpdate();
    		cu.intention = PredicateFactory.predicate("grounding", new Term[] {
	    			PredicateFactory.term(gb.grounding.modality), // source modality
	    			//PredicateFactory.term(gb.id), // belief id
	    			PredicateFactory.term(gb.grounding.gstatus.toString())  // grounding status
	    		});
	    	return cu;
    	}
    	else {
    		return null;
    	}
    }
    
    private ContextUpdate updateContext(ContextUpdate cu) {
    	log("updating context");
    	log("context update: " + cu.toString());
    	log("stack before update");
    	ccaEngine.printStack();

    	String[] nonAssertedIds = beliefsWithNoAssertions(getCurrentBeliefModel());
    	for (int i = 0; i < nonAssertedIds.length; i++) {
    		ccaEngine.stack.removeBeliefFromBlocks(nonAssertedIds[i]);
    	}
    	
		cu = verifiableUpdate(cu, getCurrentBeliefModel());
		log("stack after update");
    	ccaEngine.printStack();
		
    	Belief[] beliefUpdates = cu.beliefs;

		if (beliefUpdates.length == 0) {
			log("no belief updates...");
		}
		else {
			log("starting belief model update, " + beliefUpdates.length + " beliefs total");

			for (int i = 0; i < beliefUpdates.length; i++) {
				log("  update #" + i + ": " + BeliefModelUtils.getBeliefPrettyPrint(beliefUpdates[i], 1));

				try {
					
					if (existsOnWorkingMemory(beliefUpdates[i].id, Binder.BINDER_SA)) {
						updateExistingBelief(beliefUpdates[i]);
					}
					else {
						addNewBelief(beliefUpdates[i]);
					}
					
				} catch (UnknownSubarchitectureException e) {
					e.printStackTrace();
				}
			}
			log("done with belief model update");
		}
		
		log("syncing abducer with current belief model");
		ccaEngine.abducer.clearFactsByModality(ModalityType.K);
		syncWithBeliefModel(getCurrentBeliefModel());
		
		return cu;
    }

    private Predicate selectAction(ContextUpdate cu) {
    	return cu.intention;
    }

    private void actPublicly(Predicate actionPredicate) {
    	if (actionPredicate != null) {
	    	log("initiating public acting");
	    	Modality[] actionModality = new Modality[] { ModalityFactory.generationModality(), ModalityFactory.intentionModality() };
	    	ModalisedFormula actionMF = AbducerUtils.modalisedFormula(actionModality, actionPredicate);
			MarkedQuery[] actionProof = ccaEngine.generateProof(new ModalisedFormula[] {actionMF});
			
			if (actionProof != null) {
				log("Action proof:\n" + PrettyPrinting.proofToString(actionProof));
				
				// TODO: do this in a less hardcoded way
				if (toBeRealised(actionProof)) {
					ModalisedFormula[] fs = ProofUtils.proofToFacts(actionProof);
					Vector<String> roots = new Vector<String>();
					for (int i = 0; i < fs.length; i++) {
						if (fs[i].p.predSym.equals("sort") && ProofUtils.termToString(fs[i].p.args[1]).equals("dvp")) {
							roots.add(ProofUtils.termToString(fs[i].p.args[0]));
						}
					}
					log(roots.size() + " proto-LFs for realisation");
					for (int i = 0; i < roots.size(); i++) {
		    			LogicalForm prodLF = AbducerUtils.factsToLogicalForm(ProofUtils.proofToFacts(actionProof), roots.elementAt(i));
		    			log("will realise this proto-LF: " + LFUtils.lfToString(prodLF));
		    			realizeLogicalForm(prodLF);
					}
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
    	//cu.proof = new MarkedQuery[0];
    	cu.intention = PredicateFactory.predicate("not_understood", new Term[] { PredicateFactory.term("r") });
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
	/**
	 * Perform verifiable update.
	 *
	 * TODO: return <i>grounded</i> beliefs?
	 *
	 * @param proof the proof to be considered
	 * @param model current belief model
	 * @return beliefs that are consistent with the belief model, and with which this model needs to be updated
	 */
	public ContextUpdate verifiableUpdate(ContextUpdate cu, BeliefModel model) {
		ProofBlock pi = ProofStack.construct(cu, new HashMap<String, Belief>());
//		List<Belief> consistentUpdates = new ArrayList<Belief>();

		log("new proofblock: " + PrettyPrinting.proofBlockToString(pi));

		if (ccaEngine.stack.isEmpty()) {
			// no outstanding assertions
			if (pi.assertedBeliefIds.length != 0) {
				// we've got new assertions
				ccaEngine.stack.push(pi);
				return cu;
			}
			else {
				// we don't have any assertions at all
				// -> no stack manipulation
				return cu;
			}
		}
		else {
			// there are some unverified assertions on the stack
			
			// the operation may be:
			// * verification -- remove from the stack, change the belief's agent status etc
			// * falsification -- change the formula, put on top of the stack (at the end)
			// * neither of these -- change the formula appropriately, but leave it where it was

			Belief[] updates = new Belief[0];
			
			if (cu.intention.predSym.equals("grounding")) {
				String modality = ((FunctionTerm) cu.intention.args[0]).functor;
				String result = ((FunctionTerm) cu.intention.args[1]).functor;

				int idxValueNeed = ccaEngine.stack.findTopmostBlockByIntention("need_get_value");
				int idxValueVerify = ccaEngine.stack.findTopmostBlockByIntention("need_verify_hypothesis");
				
				ProofBlock related = null;
				if (idxValueNeed < idxValueVerify) {
					// "yes" as a response to the robot's "what color is the box?"
					related = ccaEngine.stack.retrieveBlockByIntention("need_get_value");
					cu.intention = related.intention;
					cu.intention.args[3] = PredicateFactory.term("repeated");
					cu.intention.predSym = cu.intention.predSym;
					updates = new Belief[0];
					ccaEngine.stack.push(related);
				}
				else {
					// expected response
					related = ccaEngine.stack.retrieveBlockByIntention("need_verify_hypothesis");
					
					Agent speaker = new Agent("human");
					
					if (related != null && result.equals("assertionVerified")) {
						updates = new Belief[related.assertedBeliefIds.length];
						for (int i = 0; i < related.assertedBeliefIds.length; i++) {
							Belief b = getBelief(related.assertedBeliefIds[i]);
	
							if (b.ags instanceof PrivateAgentStatus) {
								b.ags = BeliefUtils.attribute((PrivateAgentStatus) b.ags, speaker);
							}
							if (b.ags instanceof AttributedAgentStatus) {
								b.ags = BeliefUtils.raise((AttributedAgentStatus) b.ags);
							}
							if (b.ags instanceof MutualAgentStatus) {
								b.ags = BeliefUtils.addToGroup((MutualAgentStatus) b.ags, speaker);
							}
	
							b.phi = BeliefUtils.changeAssertionsToPropositions((SuperFormula) b.phi, 1.0f);
							updates[i] = b;
						}
					}
					else if (related != null && result.equals("assertionFalsified")) {
						
						if (pi.assertedBeliefIds.length == 0) {
							// no reason, just flip the polarities
							updates = new Belief[related.assertedBeliefIds.length];
							for (int i = 0; i < related.assertedBeliefIds.length; i++) {
								Belief b = getBelief(related.assertedBeliefIds[i]);
	
								if (b.ags instanceof PrivateAgentStatus) {
									b.ags = BeliefUtils.attribute((PrivateAgentStatus) b.ags, speaker);
								}
	
								b.phi = BeliefUtils.swapPolarityOfAssertions((SuperFormula) b.phi);
								updates[i] = b;
							}
						}
						else {
							// with reason, overwrite the belief formula
							// TODO: consistency
							assert related.assertedBeliefIds.length == 1;
							updates = new Belief[1];
							
							Belief b = getBelief(related.assertedBeliefIds[0]);
	
							if (b.ags instanceof PrivateAgentStatus) {
								b.ags = BeliefUtils.attribute((PrivateAgentStatus) b.ags, speaker);
							}
	
							b.phi = cu.beliefs[0].phi;
							updates[0] = b;
						}
					}
					// push it back, it's still asserted
					ccaEngine.stack.push(related);
				}
				cu.beliefs = updates;
			}

			// assert_prop(h, 0:G, color)
			else if (cu.intention.predSym.equals("assert_prop")) {
				
				// need_get_value(vision, 0:G, color)

				int idxValueNeed = ccaEngine.stack.findTopmostBlockByIntention("need_get_value");
				int idxValueVerify = ccaEngine.stack.findTopmostBlockByIntention("need_verify_hypothesis");

				ProofBlock related = null;
				if (idxValueNeed < idxValueVerify) {
					
					related = ccaEngine.stack.retrieveBlockByIntention("need_get_value");
					
					Agent speaker = new Agent("human");
	
					if (related != null) {
						
						assert related.assertedBeliefIds.length == 1;
		
						updates = new Belief[1];
						
						Belief b = getBelief(related.assertedBeliefIds[0]);
						
						if (b.ags instanceof PrivateAgentStatus) {
							b.ags = BeliefUtils.attribute((PrivateAgentStatus) b.ags, speaker);
						}
	
						b.phi = cu.beliefs[0].phi;
						updates[0] = b;
						cu.beliefs = updates;
	
						// push back as we still haven't grounded it
						ccaEngine.stack.push(related);
					}
					else {
						// related == null, i.e. no explicit info request from the robot
						// (this doesn't exclude verification requests from the robot!)
						ccaEngine.stack.push(pi);
					}
				}
				else {
					// overwrite
					related = ccaEngine.stack.retrieveBlockByIntention("need_verify_hypothesis");

					Agent speaker = new Agent("human");
					
					assert related.assertedBeliefIds.length == 1;
					updates = new Belief[1];
					
					Belief b = getBelief(related.assertedBeliefIds[0]);

					if (b.ags instanceof PrivateAgentStatus) {
						b.ags = BeliefUtils.attribute((PrivateAgentStatus) b.ags, speaker);
					}

					b.phi = cu.beliefs[0].phi;
					updates[0] = b;

					cu.beliefs = updates;
					ccaEngine.stack.push(related);
				}
			}
			
			return cu;
		}
	}
		
/*
		boolean verified = true;

		for (int i = 0; i < piPrime.assertedBeliefs.length; i++) {
			switch (VerifiableUpdate.consistent(model, piPrime.assertedBeliefs[i], pi.assertedBeliefs)) {  // XXX !!!
			
				case Consistent:
					// assertion verified -> change its continual status to "proposition"
					for (int j = 0; j < pi.assertedBeliefs.length; j++) {
						consistentUpdates.add(pi.assertedBeliefs[j]);
					}
					break;
				
				case Inconsistent:
					// assertion falsified
					verified = false;
					// TODO: look here
					break;					
			}
		}
		
		stack.push(pi);
		if (verified == false) {
			stack.push(piPrime);
		}
*/

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
	}
	
	public String[] beliefsWithNoAssertions(BeliefModel model) {
		Vector<String> closed = new Vector<String>();
		for (int i = 0; i < model.k.length; i++) {
			Belief b = getBelief(model.k[i]);
			if (!BeliefUtils.formulaHasAssertions((SuperFormula) b.phi)) {
				closed.add(b.id);
			}
		}
		return closed.toArray(new String[]{});
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
    	if (_config.containsKey("--ruleset")) {
    		rulesetFilename = _config.get("--ruleset");
    	}
	}
    
}
