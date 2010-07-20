package de.dfki.lt.tr.cast.dialogue;

import java.util.Map;

import cast.AlreadyExistsOnWMException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import de.dfki.lt.tr.beliefs.slice.events.Event;
import de.dfki.lt.tr.beliefs.slice.intentions.Intention;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.components.DialogueManager;
import de.dfki.lt.tr.dialmanagement.data.actions.AbstractAction;
import de.dfki.lt.tr.dialmanagement.data.actions.IntentionAction;
import de.dfki.lt.tr.dialmanagement.utils.EpistemicObjectUtils;
import de.dfki.lt.tr.dialmanagement.utils.PolicyReader;



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
	final String policyFile = "config/policies/minipolicy.txt";
	final String actionsFile = "config/policies/miniaction.txt";
	final String observationsFile = "config/policies/miniobservation.txt";
	
	
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
					manager = new DialogueManager(PolicyReader.constructPolicy(_config.get("--policy"), _config.get("--actions"), _config.get("--observations")));
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
					ChangeFilterFactory.createLocalTypeFilter(Intention.class,  WorkingMemoryOperation.ADD),
					new WorkingMemoryChangeReceiver() {

						public void workingMemoryChanged(
								WorkingMemoryChange _wmc) {
							try {
								newIntentionReceived(getMemoryEntry(_wmc.address, Intention.class));
							} catch (DoesNotExistOnWMException e) {
								e.printStackTrace();
							} catch (UnknownSubarchitectureException e) {
								e.printStackTrace();
							}
						}
					});
			
			
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
	public void newIntentionReceived (Intention intention) {
		try {
			AbstractAction action = manager.nextAction(intention);
			if (action instanceof IntentionAction) {
				Intention response = 
					EpistemicObjectUtils.createSimplePrivateIntention(((IntentionAction)action).getFormula(), 1.0f);
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
	 * If a new event is added, triggers the dialogue manager to determine the next appropriate
	 * action, if any is available.  If an intention action is returned, create a new private intention
	 * and inserts it into the working memory
	 * 
	 * @param event the event received as observation
	 * 
	 */
	public void newEventReceived (Event event) {
		try {
			AbstractAction action = manager.nextAction(event);
			if (action instanceof IntentionAction) {
				Intention response = 
					EpistemicObjectUtils.createSimplePrivateIntention(((IntentionAction)action).getFormula(), 1.0f);
				addToWorkingMemory(newDataID(), response);
			}
		
		} catch (DialogueException e) {
			log(e.getMessage());
			e.printStackTrace();
		} catch (AlreadyExistsOnWMException e) {
			e.printStackTrace();
		}
	}
}
