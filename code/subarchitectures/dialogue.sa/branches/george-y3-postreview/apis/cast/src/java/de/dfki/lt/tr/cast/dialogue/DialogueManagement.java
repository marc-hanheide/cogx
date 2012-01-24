package de.dfki.lt.tr.cast.dialogue;

import java.util.Arrays;
import java.util.HashMap;
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
import de.dfki.lt.tr.beliefs.slice.logicalcontent.BinaryOp;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ComplexFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.components.DialogueManager;
import de.dfki.lt.tr.dialmanagement.data.actions.AbstractAction;
import de.dfki.lt.tr.dialmanagement.data.actions.EventAction;
import de.dfki.lt.tr.dialmanagement.data.actions.IntentionAction;
import de.dfki.lt.tr.dialmanagement.data.actions.RemoveVariableAction;
import de.dfki.lt.tr.dialmanagement.utils.EpistemicObjectUtils;
import de.dfki.lt.tr.dialmanagement.utils.FormulaUtils;
import de.dfki.lt.tr.dialmanagement.data.ActionSelectionResult;
import de.dfki.lt.tr.dialmanagement.utils.XMLPolicyReader;



/**
 * CAST wrapper for the dialogue manager.  Listens to new intentions and events inserted onto the Working Memory,
 * and compute the next appropriate action, if any is available.
 * 
 * 
 * @author Pierre Lison
 * @version 22/12/2010
 *
 */
public class DialogueManagement extends ManagedComponent {

	// the dialogue manager
	DialogueManager manager;

	// default parameters for the dialogue manager
	String policyFile = "subarchitectures/dialogue.sa/config/policies/yr2/basicforwardpolicy.xml";

	boolean useAcknowledgement = false;

	/**
	 * Construct a new dialogue manager, with default values
	 * 
	 */
	public DialogueManagement() {
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
			policyFile = _config.get("--policy");
		}
		if ((_config.containsKey("--useAck"))) {
			try {
				useAcknowledgement = Boolean.parseBoolean(_config.get("--useAck"));
				log("use acknowledgements when the user provides new information: "
						+ (useAcknowledgement ? "true" : "false"));
			}
			catch (Exception e) { }
		}
	}



	@Override
	public void start() {

		log("initialising with policy file: " + policyFile);
		try {
			manager = new DialogueManager(XMLPolicyReader.constructPolicy(policyFile), this.getLogger(".dm"));
		}
		catch (DialogueException e) {
			e.printStackTrace();
		} 

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
			if (includesInFormula(cintention.intent, "question-answered") || 
					includesInFormula(cintention.intent, "object-described")) {
				cintention = expandCommunicativeIntention(cintention);	
			}

			// running the dialogue manager to select the next action
			ActionSelectionResult result = manager.updateStateAndSelectAction(cintention);
			for (AbstractAction action : result.getActions()) {
				
				log("action chosen: " + action.toString());

				// if the action is not void or ill-formed, adds the new intention to the WM
				if (!action.toString().contains("%")) {

					// if it is a communicative intention
					if (action instanceof EventAction) {
						log("emitting an event...");
						Event ev = EpistemicObjectUtils.createSimpleEvent(action.asFormula(), 1.0f);
						addToWorkingMemory(newDataID(), ev);
						log("event successfully written to the working memory");
					}
					if (action instanceof RemoveVariableAction) {
						log("removing a variable...");
						manager.getDialogueState().removeInfoState(((RemoveVariableAction)action).getLabel());
					}
					else if (action instanceof IntentionAction && ((IntentionAction)action).getStatus() == IntentionAction.COMMUNICATIVE) {

						log("reacting with a communicative response...");
						IntentionalContent content = EpistemicObjectUtils.createIntentionalContent(action.asFormula(), 
								EpistemicObjectUtils.robotAgent , 1.0f);

						Intention response = new Intention (
								cintention.intent.frame, EpistemicObjectUtils.privateStatus, 
								cintention.intent.id, Arrays.asList(content));

						CommunicativeIntention cresponse = new CommunicativeIntention (response);
						addToWorkingMemory(newDataID(), cresponse);
						log("communicative intention successfully added to working memory");
					}

					// if it is an attributed intention to forward
					else if (action instanceof IntentionAction && ((IntentionAction)action).getStatus() == IntentionAction.ATTRIBUTED) {

						log("forwarding the communicative intention beyond dialogue.sa...");								
						IntentionalContent content = EpistemicObjectUtils.createIntentionalContent(action.asFormula(), 
								cintention.intent.content.get(0).agents, 1.0f);

						Intention response = new Intention (cintention.intent.frame, cintention.intent.estatus, 
								cintention.intent.id, Arrays.asList(content));

						addToWorkingMemory(cintention.intent.id, response);
						log("attributed intention successfully added to working memory");
						
						if (useAcknowledgement) {
							
							log("... and providing short acknowledgement");	
							
							IntentionalContent ackContent = EpistemicObjectUtils.createIntentionalContent(
									FormulaUtils.constructFormula("<state>(thanked ^ <agent>human ^ <patient>self)"), EpistemicObjectUtils.robotAgent , 1.0f);
							
							Intention ack = new Intention (
									cintention.intent.frame, EpistemicObjectUtils.privateStatus, 
									cintention.intent.id, Arrays.asList(ackContent));

							CommunicativeIntention cack = new CommunicativeIntention (ack);
							addToWorkingMemory(newDataID(), cack);
						}
					}
				}
				
				else {
					log("dialogue manager returned a void or ill-formed action, not doing anything");
				}
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
	 * @param the predicate
	 * @return true if the predicate is contained, false otherwise
	 */
	private boolean includesInFormula (Intention intent, String str)  {

		for (IntentionalContent content : intent.content) {
			if (FormulaUtils.getString(content.postconditions).contains(str)) {
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
			ActionSelectionResult result = manager.updateStateAndSelectAction(event);
			
			for (AbstractAction action : result.getActions()) {

				log("action chosen: " + action.toString());

				if (action instanceof EventAction) {
					log("emitting an event...");
					Event ev = EpistemicObjectUtils.createSimpleEvent(action.asFormula(), 1.0f);
					addToWorkingMemory(newDataID(), ev);
					log("event successfully written to the working memory");
				}
				if (action instanceof RemoveVariableAction) {
					log("removing a variable...");
					manager.getDialogueState().removeInfoState(((RemoveVariableAction)action).getLabel());
				}
				else if (action instanceof IntentionAction && ((IntentionAction)action).getStatus() == IntentionAction.COMMUNICATIVE) {

					log("reacting with a communicative response...");
					IntentionalContent content = EpistemicObjectUtils.createIntentionalContent(action.asFormula(), 
							EpistemicObjectUtils.robotAgent , 1.0f);

					Intention response = new Intention (
							event.frame, EpistemicObjectUtils.privateStatus, 
							event.id, Arrays.asList(content));

					CommunicativeIntention cresponse = new CommunicativeIntention (response);
					addToWorkingMemory(newDataID(), cresponse);
					log("communicative intention successfully added to working memory");
				}
			}
		}
		catch (DialogueException e) {
			e.printStackTrace();
		}
		catch (AlreadyExistsOnWMException e) {
			e.printStackTrace();
		}
	}



	/**
	 * Get the features to extract for the question
	 * 
	 * @param form the formula containing the question
	 * @return the list of features to extract
	 */
	private List<String> getFeaturesToExtractForQuestion(dFormula form) {
		
		String featureType = FormulaUtils.getString(FormulaUtils.
				getModalOperatorValue(form,"feature"));
		debug("feature type: " + featureType);

		List<String> featuresToExtract = new LinkedList<String>();
		if (FormulaUtils.getString(form).contains("object-described")) {
				featuresToExtract.add("color");
				featuresToExtract.add("shape");
		}
		else {
			featuresToExtract.add(featureType);
		}
		return featuresToExtract;
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

		List<IntentionalContent> newContents = new LinkedList<IntentionalContent>();
		
		for (IntentionalContent content : newCI.intent.content) {	

			List<String> featuresToExtract = getFeaturesToExtractForQuestion (content.postconditions);
			HashMap<ComplexFormula,Float> expandedFormulae = 
				expandFormula(content.postconditions, featuresToExtract);		

			debug("number of expanded formulae: " + expandedFormulae.size());
			
			for (ComplexFormula expandedFormula : expandedFormulae.keySet()) {
				
				float prob = content.probValue* expandedFormulae.get(expandedFormula);
				IntentionalContent newContent = 
					new IntentionalContent(content.agents, content.preconditions, 
						expandedFormula, prob);

				// in case we deal with a polar question
				if (FormulaUtils.getString(newContent.postconditions).contains(("hypo"))) {
					transformPolarQuestionHypothesis (newContent);
				}
				
				debug("expanded postcondition: " + FormulaUtils.getString(newContent.postconditions) + ", with prob. " + prob);				
				newContents.add(newContent);
			}	
		}
		
		newCI.intent.content = newContents;
		
		return newCI;
	}

	
	/**
	 * If the question encoded in the intentional content is a polar content, transform the hypothesis
	 * to 
	 * @param content
	 * @param featureType
	 */
	public void transformPolarQuestionHypothesis (IntentionalContent content) {
		
		String featureType = FormulaUtils.getString(FormulaUtils.
				getModalOperatorValue(content.postconditions,"feature"));
		
		dFormula featureValue = FormulaUtils.getModalOperatorValue(
				content.postconditions, featureType);	
		debug("feature value: " + FormulaUtils.getString(featureValue));

		dFormula hypothesis = FormulaUtils.getModalOperatorValue(content.postconditions, "hypo");
		debug("hypothesis: " + FormulaUtils.getString(hypothesis));		

		if (hypothesis != null && featureValue != null) {
			if (FormulaUtils.subsumes(hypothesis,featureValue)) {
				FormulaUtils.setModalOperatorValue(content.postconditions, "hypo", "valid");
			}
			else {
				FormulaUtils.setModalOperatorValue(content.postconditions, "hypo", "invalid");
			}
		}
		else if (hypothesis != null) {
			FormulaUtils.setModalOperatorValue(content.postconditions, "hypo", "noclue");
		}
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
	private HashMap<ComplexFormula,Float> expandFormula (dFormula form, List<String> featuresToExtract)
	throws UnknownSubarchitectureException, DoesNotExistOnWMException, DialogueException {

		if (form instanceof ComplexFormula) {

			HashMap<ComplexFormula,Float> results = new HashMap<ComplexFormula,Float>();
			results.put(new ComplexFormula(0,new LinkedList<dFormula>(),BinaryOp.conj), 1.0f);

			for (dFormula existingSubFormula : ((ComplexFormula)form).forms) {

				HashMap<ComplexFormula,Float> expandedFormulae = expandFormula(existingSubFormula, featuresToExtract);			
				HashMap<ComplexFormula,Float> newResults = new HashMap<ComplexFormula,Float>();

				for (dFormula curCFormula : results.keySet()) {

					for (dFormula expandedFormula : expandedFormulae.keySet()) {

						ComplexFormula newFormula = (ComplexFormula) FormulaUtils.copy(curCFormula);
						newFormula.forms.add(expandedFormula);
						newResults.put(newFormula, results.get(curCFormula) * expandedFormulae.get(expandedFormula));
					}			
				}		
				results = newResults;
			}
			return results;
		}

		else if (form instanceof ModalFormula) {


			HashMap<ComplexFormula, Float> newFormulae = new HashMap<ComplexFormula,Float>();
			HashMap<ComplexFormula,Float> expandedFormulae = expandFormula(((ModalFormula)form).form, featuresToExtract);

			for (ComplexFormula expandedFormula : expandedFormulae.keySet()) {

				ComplexFormula newFormula = new ComplexFormula(0, Arrays.asList((dFormula)new ModalFormula(
						0, ((ModalFormula)form).op, FormulaUtils.copy(expandedFormula))), BinaryOp.conj);
				newFormulae.put(newFormula, expandedFormulae.get(expandedFormula));
			}

			return newFormulae;
		}

		else if (form instanceof PointerFormula) {
			debug("found pointer: " + FormulaUtils.getString(form));
			WorkingMemoryAddress WMPointer= ((PointerFormula)form).pointer;
			if (WMPointer != null && existsOnWorkingMemory(WMPointer)) {			
				try {
					dBelief b = getMemoryEntry(WMPointer, dBelief.class);

					HashMap<ComplexFormula,Float> expandedFormulae =  EpistemicObjectUtils.getBeliefContent(b, featuresToExtract);

					for (ComplexFormula expandedFormula : expandedFormulae.keySet()) {
						expandedFormula.forms.add(new ModalFormula(0, "ref", form));
					}

					return expandedFormulae;
				}
				catch (ClassCastException e) {
					e.printStackTrace();
				}
			}
		}

		HashMap<ComplexFormula,Float> dummyHash = new HashMap<ComplexFormula,Float>();
		dummyHash.put(new ComplexFormula(0,Arrays.asList(FormulaUtils.copy(form)),BinaryOp.conj), 1.0f);
		return dummyHash;

	}
}
