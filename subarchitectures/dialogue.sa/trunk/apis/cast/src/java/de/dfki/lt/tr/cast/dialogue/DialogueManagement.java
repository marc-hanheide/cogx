package de.dfki.lt.tr.cast.dialogue;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;


import cast.AlreadyExistsOnWMException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import de.dfki.lt.tr.beliefs.slice.epstatus.AttributedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.PrivateEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.events.Event;
import de.dfki.lt.tr.beliefs.slice.intentions.CommunicativeIntention;
import de.dfki.lt.tr.beliefs.slice.intentions.Intention;
import de.dfki.lt.tr.beliefs.slice.intentions.IntentionalContent;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ComplexFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.components.DialogueManager;
import de.dfki.lt.tr.dialmanagement.data.policies.PolicyAction;
import de.dfki.lt.tr.dialmanagement.utils.EpistemicObjectUtils;
import de.dfki.lt.tr.dialmanagement.utils.FormulaUtils;
import de.dfki.lt.tr.dialmanagement.utils.XMLPolicyReader;



/**
 * CAST wrapper for the dialogue manager.  Listens to new intentions and events inserted onto the Working Memory,
 * and compute the next appropriate action, if any is available.
 * 
 *  TODO: also listen (and take into account) private intentions which are externally provided
 *  TODO: take preconditions into account
 * 
 * @author Pierre Lison
 * @version 08/07/2010
 *
 */
public class DialogueManagement extends ManagedComponent {

	// the dialogue manager
	DialogueManager manager;

	// default parameters for the dialogue manager
	String policyFile = "subarchitectures/dialogue.sa/config/policies/yr2/basicforwardpolicy.xml";


	/**
	 * Construct a new dialogue manager, with default values
	 * 
	 */
	public DialogueManagement() {
		try {
			manager = new DialogueManager(XMLPolicyReader.constructPolicy(policyFile));
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
		if ((_config.containsKey("--policy"))) {
			try {
				log("Dialogue manager using the following policy: " + _config.get("--policy"));
				policyFile = _config.get("--policy");
				manager = new DialogueManager(XMLPolicyReader.constructPolicy(policyFile));
			} catch (DialogueException e) {
				log(e.getMessage());
				e.printStackTrace();
			} 	
		}
	}



	@Override
	public void start() {

		// communicative intentions
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(CommunicativeIntention.class,  WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {

					public void workingMemoryChanged(
							WorkingMemoryChange _wmc) {
						try {
							CommunicativeIntention initIntention = getMemoryEntry(_wmc.address, CommunicativeIntention.class);
							if (initIntention.intent.estatus instanceof AttributedEpistemicStatus) {
								newIntentionReceived(initIntention);
							}
						} catch (DoesNotExistOnWMException e) {
							e.printStackTrace();
						} catch (UnknownSubarchitectureException e) {
							e.printStackTrace();
						}
					}
				});


		// (non-communicative) intentions
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(Intention.class,  WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {

					public void workingMemoryChanged(
							WorkingMemoryChange _wmc) {
						try {
							Intention initIntention = getMemoryEntry(_wmc.address, Intention.class);
							if (initIntention.estatus instanceof PrivateEpistemicStatus) {
								addToWorkingMemory(newDataID(), new CommunicativeIntention(initIntention));
							}
						} catch (DoesNotExistOnWMException e) {
							e.printStackTrace();
						} catch (UnknownSubarchitectureException e) {
							e.printStackTrace();
						}
						catch (AlreadyExistsOnWMException e) {
							e.printStackTrace();
						} 
					}
				}); 


		// events
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
	 * If a new intention is added, triggers the dialogue manager to determine 
	 * the next appropriate action, if any is available.  
	 * 
	 * If an intention action is returned, create a new private intention and 
	 * inserts it into the working memory
	 * 
	 * @param intention the (attributed) intention received as observation
	 * 
	 */
	public void newIntentionReceived (CommunicativeIntention cintention) {
		try {

			// create an "augmented" intention based on the received one
			if (includesQuestion(cintention.intent)) {
				cintention = expandCommunicativeIntention(cintention);
			String formAsString = FormulaUtils.getString(cintention.intent.content.get(0).postconditions);
			log("expanded postcondition: " + formAsString);	
			}

			// running the dialogue manager to select the next action
			PolicyAction action = manager.nextAction(cintention);
			log("action chosen: " + action.toString());

			// if the action is not void or ill-formed, adds the new intention to the WM
			if (!action.isVoid() && !action.toString().contains("%")) {
			
				// if it is a communicative intention
				if (action.getType() == PolicyAction.COMMUNICATIVE_INTENTION) {
		
					log("reacting with a communicative response...");
					IntentionalContent content = EpistemicObjectUtils.createIntentionalContent(action.getContent(), 
							EpistemicObjectUtils.robotAgent , 1.0f);
					
					Intention response = new Intention (
							cintention.intent.frame, EpistemicObjectUtils.privateStatus, 
							cintention.intent.id, Arrays.asList(content));

					CommunicativeIntention cresponse = new CommunicativeIntention (response);
					addToWorkingMemory(newDataID(), cresponse);
					log("communicative intention successfully added to working memory");
				}

				// if it is an attributed intention to forward
				else if (action.getType() == PolicyAction.ATTRIBUTED_INTENTION) {
					
					log("forwarding the communicative intention beyond dialogue.sa...");								
					IntentionalContent content = EpistemicObjectUtils.createIntentionalContent(action.getContent(), 
							cintention.intent.content.get(0).agents, 1.0f);
					
					Intention response = new Intention (cintention.intent.frame, cintention.intent.estatus, 
							cintention.intent.id, Arrays.asList(content));
					
					addToWorkingMemory(cintention.intent.id, response);
					log("attributed intention successfully added to working memory");
				}
			}
			else {
				log("dialogue manager returned a void or ill-formed action, not doing anything");
			}

		} catch (Exception e) {
			log(e.getMessage());
			e.printStackTrace();
		}
	}

	
	
	/**
	 * Returns true if the intention contains the predicate "question-answered", false otherwise
	 * 
	 * @param intent the intention
	 * @return true if the predicate is contained, false otherwise
	 */
	private boolean includesQuestion (Intention intent)  {
		
		for (IntentionalContent content : intent.content) {
			if (FormulaUtils.getString(content.postconditions).contains("question-answered")) {
				return true;
			}
		}
		return false;
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

			// running the dialogue manager to select the next action
			PolicyAction action = manager.nextAction(event);
			log("action chosen: " + action.toString());

			// if the action is not void, adds the new intention to the WM
			
			if (action.getType() == PolicyAction.COMMUNICATIVE_INTENTION) {
				
				log("reacting with a communicative response...");
				IntentionalContent content = EpistemicObjectUtils.createIntentionalContent(action.getContent(), 
						EpistemicObjectUtils.robotAgent , 1.0f);
				
				Intention response = new Intention (
						event.frame, EpistemicObjectUtils.privateStatus, 
						event.id, Arrays.asList(content));

				CommunicativeIntention cresponse = new CommunicativeIntention (response);
				addToWorkingMemory(newDataID(), cresponse);
				log("communicative intention successfully added to working memory");
			}

		} catch (DialogueException e) {
			log(e.getMessage());
			e.printStackTrace();
		} catch (AlreadyExistsOnWMException e) {
			e.printStackTrace();
		}
	}

	

	
	
	/**
	 * Expand the intentional content -- i.e. if the formula of the postcondition contains
	 * a pointer formula, replace this pointer by the value of the epistemic object
	 * being pointed at
	 * 
	 * @param initCI the initial communicative intention
	 * @return the expanded intentional content (as a copy)
	 * 
	 * @throws DialogueException
	 * @throws DoesNotExistOnWMException
	 * @throws UnknownSubarchitectureException
	 */
	private  CommunicativeIntention expandCommunicativeIntention  (CommunicativeIntention initCI) 
	throws DialogueException, DoesNotExistOnWMException, UnknownSubarchitectureException  {
		
		CommunicativeIntention newCI = EpistemicObjectUtils.copy (initCI);
		debug("communicative intention " + initCI.intent.id + " successfully copied");
		
		for (IntentionalContent alternativeContent : newCI.intent.content) {			
			alternativeContent.postconditions = expandFormula(alternativeContent.postconditions);		
			debug("expanded postcondition: " + FormulaUtils.getString(alternativeContent.postconditions));
		}
			
		return newCI;
	}


	
	/**
	 * Expand the given formula (if the formula contains a pointer, replaces it by
	 * the content of the epistemic object being pointed at)
	 * 
	 * @param form the formula to expand
	 * @return the expanded formula
	 * 
	 * @throws UnknownSubarchitectureException
	 * @throws DoesNotExistOnWMException
	 * @throws DialogueException
	 */
	private dFormula expandFormula (dFormula form)
		throws UnknownSubarchitectureException, DoesNotExistOnWMException, DialogueException {
		
		if (form instanceof ComplexFormula) {
			List<dFormula> newSubFormulae = new LinkedList<dFormula>();
			for (dFormula existingSubFormula : ((ComplexFormula)form).forms) {
				newSubFormulae.add(expandFormula(existingSubFormula));
			}
			return new ComplexFormula(0, newSubFormulae, ((ComplexFormula)form).op);
		}
		
		else if (form instanceof ModalFormula) {
			return new ModalFormula(0, ((ModalFormula)form).op, expandFormula(((ModalFormula)form).form));
		}
		
		else if (form instanceof PointerFormula) {
			debug("found pointer: " + FormulaUtils.getString(form));
			WorkingMemoryAddress WMPointer= ((PointerFormula)form).pointer;
			if (WMPointer != null && existsOnWorkingMemory(WMPointer)) {			
				try {
				dBelief b = getMemoryEntry(WMPointer, dBelief.class);
				ComplexFormula expandedFormula =  EpistemicObjectUtils.getBeliefContent(b);
				expandedFormula.forms.add(new ModalFormula(0, "ref", form));
				return expandedFormula;
				}
				catch (ClassCastException e) {
					e.printStackTrace();
				}
			}
		}
		
		return form;
	}
}
