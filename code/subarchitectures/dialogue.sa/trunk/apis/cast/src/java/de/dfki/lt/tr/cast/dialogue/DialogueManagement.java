package de.dfki.lt.tr.cast.dialogue;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import org.junit.Before;

import cast.AlreadyExistsOnWMException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues;
import de.dfki.lt.tr.beliefs.slice.epstatus.AttributedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.events.Event;
import de.dfki.lt.tr.beliefs.slice.intentions.CommunicativeIntention;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.components.DialogueManager;
import de.dfki.lt.tr.dialmanagement.data.PolicyAction;
import de.dfki.lt.tr.dialmanagement.utils.EpistemicObjectUtils;
import de.dfki.lt.tr.dialmanagement.utils.FormulaUtils;
import de.dfki.lt.tr.dialmanagement.utils.PolicyReader;
import de.dfki.lt.tr.dialogue.interpret.IntentionManagementConstants;
import java.util.HashMap;



/**
 * CAST wrapper for the dialogue manager.  Listens to new intentions and events inserted onto the Working Memory,
 * and compute the next appropriate action, if any is available.  
 * 
 * @author Pierre Lison
 * @version 08/07/2010
 *
 */
public class DialogueManagement extends ManagedComponent {

	// the dialogue manager
	DialogueManager manager;
	
	// default parameters for the dialogue manager
	String policyFile = "subarchitectures/dialogue.sa/config/policies/yr2/testpolicy.txt";
	String actionsFile = "subarchitectures/dialogue.sa/config/policies/yr2/testaction.txt";
	String observationsFile = "subarchitectures/dialogue.sa/config/policies/yr2/testobservation.txt";
	
	
	/**
	 * Construct a new dialogue manager, with default values
	 * 
	 */
	public DialogueManagement() {
		try {
		manager = new DialogueManager(PolicyReader.constructPolicy(policyFile, observationsFile, actionsFile));
		}
		catch (DialogueException e) {
			e.printStackTrace();
		} 
	} 
	
	
	/**
	 * Configuration method.  If the parameters --policy, --actions and --observations are given, 
	 * use their values as parameters to the FSA-based dialogue manager.  If these values
	 * are not given, the default ones are used
	 * 
	 */
	@Override
	public void configure(Map<String, String> _config) {
		if ((_config.containsKey("--policy")) && 
			(_config.containsKey("--actions")) && 
			(_config.containsKey("--observations"))) {
				try {
					log("Provided parameters: policy=" + _config.get("--policy") + ", actions=" + _config.get("--actions") + ", observations=" + _config.get("--observations"));
					policyFile = _config.get("--policy");
					observationsFile = _config.get("--observations") ;
					actionsFile = _config.get("--actions");
					manager = new DialogueManager(PolicyReader.constructPolicy(policyFile,observationsFile, actionsFile));
				} catch (DialogueException e) {
					log(e.getMessage());
					e.printStackTrace();
				} 	
			}
	}
	
	
	/*
	 * Starting up the CAST component by registering filters on the insertion of new intentions and events. 
	 * 
	 * @see cast.architecture.abstr.WorkingMemoryReaderProcess#start()
	 */
	@Override
	public void start() {

		//try {
		
		
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(CommunicativeIntention.class,  WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {

					public void workingMemoryChanged(
							WorkingMemoryChange _wmc) {
						try {
							CommunicativeIntention initIntention = getMemoryEntry(_wmc.address, CommunicativeIntention.class);
							if (initIntention.estatus instanceof AttributedEpistemicStatus) {
								newIntentionReceived(initIntention);
							}
						} catch (DoesNotExistOnWMException e) {
							e.printStackTrace();
						} catch (UnknownSubarchitectureException e) {
							e.printStackTrace();
						}
					}
				});
		
		/**
			addChangeFilter(
					ChangeFilterFactory.createLocalTypeFilter(Intention.class,  WorkingMemoryOperation.ADD),
					new WorkingMemoryChangeReceiver() {

						public void workingMemoryChanged(
								WorkingMemoryChange _wmc) {
							try {
								Intention initIntention = getMemoryEntry(_wmc.address, Intention.class);
								if (initIntention.estatus instanceof AttributedEpistemicStatus) {
									newIntentionReceived(initIntention);
								}
							} catch (DoesNotExistOnWMException e) {
								e.printStackTrace();
							} catch (UnknownSubarchitectureException e) {
								e.printStackTrace();
							}
						}
					}); */
			 
	
			addChangeFilter(
					ChangeFilterFactory.createLocalTypeFilter(Event.class,  WorkingMemoryOperation.ADD),
					new WorkingMemoryChangeReceiver() {

						public void workingMemoryChanged(
								WorkingMemoryChange _wmc) {
							try {
								newEventReceived(getMemoryEntry(_wmc.address, Event.class));
							} catch (DoesNotExistOnWMException e) {
								e.printStackTrace();
							} catch (UnknownSubarchitectureException e) {
								e.printStackTrace();
							}
						}
					});

	}
	
	

	   
	/**
	 * If a new intention is added, triggers the dialogue manager to determine the next appropriate
	 * action, if any is available.  If an intention action is returned, create a new private intention
	 * and inserts it into the working memory
	 * 
	 * @param intention the (attributed) intention received as observation
	 * 
	 */
	public void newIntentionReceived (CommunicativeIntention intention) {
		try {
			
			CommunicativeIntention augmentedIntention = createAugmentedIntention(intention);
			
			String formAsString = FormulaUtils.getString(augmentedIntention.content.get(0).postconditions);
			debug("augmented intention: " + formAsString);	

			PolicyAction action = manager.nextAction(augmentedIntention);
			
			debug(action);
			log("action chosen: " + action.toString());
			
			if (!action.isVoid()) {
				CommunicativeIntention response = 
					EpistemicObjectUtils.createSimplePrivateCommunicativeIntention((action).getContent(), 1.0f);
				addToWorkingMemory(newDataID(), response);
				log("new private intention successfully added to working memory");
		/**		if (manager.isFinished()) {
					log("restarting the policy...");
					manager = new DialogueManager(PolicyReader.constructPolicy(policyFile,observationsFile, actionsFile));
				} */
			}
		
		} catch (Exception e) {
			log(e.getMessage());
			e.printStackTrace();
		}
	}
	
	
	
	@Override
	public void run () {
	}
	
  
	   
	/**
	 * If a new event is added, triggers the dialogue manager to determine the next appropriate
	 * action, if any is available.  If an intention action is returned, create a new private intention
	 * and inserts it into the working memory
	 * 
	 * @param event the event received as observation
	 * 
	 */
	public void newEventReceived (Event event) {
		try {
			PolicyAction action = manager.nextAction(event);
			if (!action.isVoid()) {
				CommunicativeIntention response = 
					EpistemicObjectUtils.createSimplePrivateCommunicativeIntention((action).getContent(), 1.0f);
				addToWorkingMemory(newDataID(), response);
			}
		
		} catch (DialogueException e) {
			log(e.getMessage());
			e.printStackTrace();
		} catch (AlreadyExistsOnWMException e) {
			e.printStackTrace();
		}
	}
	
	
	
	
	/**
	 * Create a new intention based on the content of the init intention, by
	 * replace pointers by the content of the pointed epistemic object
	 * 
	 * @param initIntention
	 * @return
	 * @throws DialogueException
	 * @throws DoesNotExistOnWMException
	 */
	public CommunicativeIntention createAugmentedIntention (CommunicativeIntention initIntention) 
	throws DialogueException, DoesNotExistOnWMException {

		if (initIntention!=null && initIntention.content.size() > 0)  {

			if (initIntention.content.get(0).postconditions instanceof ModalFormula &&
					((ModalFormula)initIntention.content.get(0).postconditions).op.equals(IntentionManagementConstants.beliefLinkModality)) {

				String beliefID = FormulaUtils.getString(
						((ModalFormula)initIntention.content.get(0).postconditions).form);

				if (existsOnWorkingMemory(beliefID)) {
					dBelief b = getMemoryEntry(beliefID, dBelief.class);

					debug("type of distrib: " + b.content.getClass().getCanonicalName());

					if (b.content instanceof BasicProbDistribution &&
							((BasicProbDistribution)b.content).values instanceof FormulaValues) {

//						List<FormulaProbPair> newPairs = new LinkedList<FormulaProbPair>();
						HashMap<dFormula,Float> mapPairs = new HashMap<dFormula,Float>();

						for (FormulaProbPair pair : ((FormulaValues)((BasicProbDistribution)b.content).values).values) {
							FormulaProbPair newPair = new FormulaProbPair(new ModalFormula(0, IntentionManagementConstants.beliefLinkModality, pair.val), pair.prob);
//							newPairs.add(newPair);
							mapPairs.put(newPair.val, newPair.prob);
						}

						CommunicativeIntention forgedIntention = EpistemicObjectUtils.createAttributedCommunicativeIntention(mapPairs);

						return forgedIntention;
					}
				}
			}
		}
		return initIntention;
	}
	
}
