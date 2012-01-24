// =================================================================                                                        
// Copyright (C) 2009-2011 Pierre Lison (plison@ifi.uio.no)                                                                
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


package de.dfki.lt.tr.dialmanagement.components;

import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;


import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues;
import de.dfki.lt.tr.beliefs.slice.events.Event;
import de.dfki.lt.tr.beliefs.slice.intentions.CommunicativeIntention;
import de.dfki.lt.tr.beliefs.slice.intentions.IntentionalContent;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.IntegerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.data.ActionSelectionResult;
import de.dfki.lt.tr.dialmanagement.data.DialogueState;
import de.dfki.lt.tr.dialmanagement.data.Observation;
import de.dfki.lt.tr.dialmanagement.data.actions.AbstractAction;
import de.dfki.lt.tr.dialmanagement.data.actions.DialStateAction;
import de.dfki.lt.tr.dialmanagement.data.conditions.AbstractCondition;
import de.dfki.lt.tr.dialmanagement.data.conditions.IntentionCondition;
import de.dfki.lt.tr.dialmanagement.data.conditions.PhonstringCondition;
import de.dfki.lt.tr.dialmanagement.data.policies.DialoguePolicy;
import de.dfki.lt.tr.dialmanagement.data.policies.PolicyEdge;
import de.dfki.lt.tr.dialmanagement.data.policies.PolicyNode;
import de.dfki.lt.tr.dialmanagement.utils.EpistemicObjectUtils;
import de.dfki.lt.tr.dialmanagement.utils.FormulaUtils;
import de.dfki.lt.tr.dialmanagement.utils.PolicyUtils;
import de.dfki.lt.tr.dialogue.slice.asr.PhonString;
import org.apache.log4j.Logger;


/**
 * <p>A simple, FSA-based dialogue manager.  The manager is based on a set of dialogue policies,
 * and a dialogue state comprising the dialogue history, the current node position in each policy,
 * and a shared information state (which can include a context and user model).
 * 
 * The dialogue manager is centered on two core mechanisms: <ul>
 * <li> the dialogue state update mechanism, which modifies the current dialogue state given a new
 *   observation (which can be a phonological string, a communicative intention or an event)
 * <li> the action selection mechanism, which selected a list of actions which can be executed
 *      for the current dialogue state.
 * </ul>
 * 
 * <p>The current dialogue manager features the following functionalities:  <ul>
 * <li> the ability to work with several FSA policies in parallel;
 * <li> the ability to import and export information onto and into a global information state;
 * <li> the ability to provide policies using various text or XML encodings;
 * <li> the ability to handle intentions or events encoding arbitrarily complex formulae;
 * <li> the ability to process intention or events with multiple alternatives;
 * <li> the ability of partially or fully underspecifying observations;
 * <li> the ability of passing arguments from observations to actions;
 * <li> the ability to handle time-related events (i.e. timeouts);
 * <li> the ability to provide actions with several alternative realizations to choose at runtime;
 * <li> the ability to provide lower and higher bounds on the observation probabilities. </ul><br>
 * 
 * @author Pierre Lison (plison@ifi.uio.no)
 * @version 21/12/2010
 *
 */
public class DialogueManager {

	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = false;

	private Logger logger = null;

	// the dialogue policies
	private List<DialoguePolicy> policies;

	// the dialogue state
	private DialogueState dialState;



	// ==============================================================
	// INITIALISATION
	// ==============================================================


	/**
	 * Create a new dialogue manager, initially with no policies and an 
	 * empty dialogue state
	 * 
	 */
	public DialogueManager(Logger _logger) {
		policies = new LinkedList<DialoguePolicy>();
		logger = _logger;
		dialState = new DialogueState(Logger.getLogger(logger.getName() + ".state"));
	}

	public DialogueManager() {
		this((Logger) null);
	}

	public DialogueManager(DialoguePolicy policy) throws DialogueException {
		this(policy, null);
	}

	/**
	 * Create a new dialogue manager with one single policy, and a dialogue state
	 * initialised with the initial node of the policy
	 * 
	 * @param policy the dialogue policy
	 * @throws DialogueException if the policy is ill-formed
	 */
	public DialogueManager (DialoguePolicy policy, Logger _logger) throws DialogueException {
		this(_logger);
		addDialoguePolicy(policy);
	}


	/**
	 * Add a new dialogue policy to the manager
	 * 
	 * @param policy the dialogue policy
	 * @throws DialogueException if the policy is ill-formed
	 */
	public void addDialoguePolicy(DialoguePolicy policy) throws DialogueException {
		policy.setLogger(Logger.getLogger(logger.getName() + ".policy"));
		if (!policy.isWellFormedPolicy()) {
			throw new DialogueException ("ERROR: not well-formed policy");
		}
		policies.add(policy);
		dialState.initialisePolicy(policy);

		log("Policy " + policy.getPolicyId() + "  successfully loaded");

	}



	// ==============================================================
	// UPDATE + ACTION SELECTION METHODS
	// ==============================================================




	/**
	 * Update the dialogue state given the set of alternative phonstrings, and subsequently
	 * select the next (machine) actions to perform, if any.
	 * 
	 * The update consists in: <ul>
	 * <li> adding the phonstrings list to the dialogue state history;
	 * <li> if one phonstring matches a policy edge in the policies currently handled
	 *      by the dialogue manager, move its position to the target node in the policy
	 *      (this is repeated for each policy);
	 * <li> finally, if the forementioned policy edge contains a function to update the 
	 *      global information state, inserts the provided information.
	 * </ul>
	 * 
	 * @param phonStrings the observation, encoded as a set of alternative phonStrings
	 * @return the result of the action selection
	 */
	public ActionSelectionResult updateStateAndSelectAction(List<PhonString> phonStrings) {

		Observation obs = new Observation(Observation.PHONSTRING);
		for (PhonString phon : phonStrings) {
			obs.addAlternative(phon, phon.confidenceValue);
		}

		if (updateState(obs)) {
			return extractActionSelectionResult();
		}
		else {
			return ActionSelectionResult.createVoidResult();
		}
	}


	/**
	 * Update the dialogue state given the communicative intention, and subsequently
	 * select the next (machine) actions to perform, if any.
	 * 
	 * The update consists in: <ul>
	 * <li> adding the communicative intention to the dialogue state history;
	 * <li> if the intention matches a policy edge in the policies currently handled
	 *      by the dialogue manager, move its position to the target node in the policy
	 *      (this is repeated for each policy);
	 * <li> finally, if the forementioned policy edge contains a function to update the 
	 *      global information state, inserts the provided information.
	 * </ul>
	 * 
	 * @param intention a communicative intention object
	 * @return the result of the action selection
	 */
	public ActionSelectionResult updateStateAndSelectAction (CommunicativeIntention intention) {

		Observation obs = new Observation(Observation.INTENTION);
		for (IntentionalContent icontent : intention.intent.content) {
			obs.addAlternative(EpistemicObjectUtils.translateIntoFormula(icontent), icontent.probValue);
		}

		if (updateState(obs)) {
			return extractActionSelectionResult();
		}
		else {
			return ActionSelectionResult.createVoidResult();
		}
	}




	/**
	 * Update the dialogue state given an external event, and subsequently
	 * select the next (machine) actions to perform, if any.
	 * 
	 * The update consists in: <ul>
	 * <li> adding the observation to the dialogue state history;
	 * <li> if the observation matches a policy edge in the policies currently handled
	 *      by the dialogue manager, move its position to the target node in the policy
	 *      (this is repeated for each policy);
	 * <li> finally, if the forementioned policy edge contains a function to update the 
	 *      global information state, inserts the provided information.
	 * </ul>
	 * 
	 * @param event an event object
	 * @return the result of the action selection
	 */
	public ActionSelectionResult updateStateAndSelectAction(Event event) {

		// we assume here that the event content is a BasicProbDistribution
		Observation obs = new Observation(Observation.EVENT);
		if (event.content instanceof BasicProbDistribution && 
				((BasicProbDistribution)event.content).values instanceof FormulaValues) {
			for (FormulaProbPair pair : ((FormulaValues)((BasicProbDistribution)event.content).values).values) {
				obs.addAlternative(pair.val, pair.prob);
			}
		}

		if (updateState(obs)) {
			return extractActionSelectionResult();
		}
		else {
			return ActionSelectionResult.createVoidResult();
		}
	} 



	/**
	 * Update the dialogue state given a timeout event (in milliseconds), and subsequently
	 * select the next (machine) actions to perform, if any.
	 * 
	 * The update consists in: <ul>
	 * <li> adding the observation to the dialogue state history;
	 * <li> if the observation matches a policy edge in the policies currently handled
	 *      by the dialogue manager, move its position to the target node in the policy
	 *      (this is repeated for each policy);
	 * <li> finally, if the forementioned policy edge contains a function to update the 
	 *      global information state, inserts the provided information.
	 * </ul>
	 * 
	 * @param timeout a timeout, in milliseconds
	 * @return the result of the action selection
	 */
	public ActionSelectionResult updateStateAndSelectAction(int timeout) {

		// we assume here that the event content is a BasicProbDistribution
		Observation obs = new Observation(Observation.TIMEOUT);
		obs.addAlternative(new IntegerFormula(0, timeout), 1.0f);

		if (updateState(obs) || timeout ==  0.0f) {
			return extractActionSelectionResult();
		}
		else {
			return ActionSelectionResult.createVoidResult();
		}
	} 



	// ==============================================================
	// STATE UPDATE METHODS
	// ==============================================================



	/**
	 * Update the dialogue state given the observation provided as argument
	 * 
	 * The update consists in: <ul>
	 * <li> adding the observation to the dialogue state history
	 * <li> if the observation matches a policy edge in the policies currently handled
	 *      by the dialogue manager, move its position to the target node in the policy
	 *      (this is repeated for each policy)
	 * <li> finally, if the forementioned policy edge contains a function to update the 
	 *      global information state, inserts the provided information
	 * </ul>
	 *
	 * @param observation the observation object
	 * @return true if the update resulted in at least one policy transition in the dialogue
	 * state, false otherwise
	 */
	private boolean updateState (Observation obs) {

		log("current state: " + dialState.toString());
		log("observation: " + obs.toString());

		// STEP 1: update the observation history
		dialState.addToHistory(obs);

		// STEP 2: update the node position in each dialogue policy, if possible
		boolean transitionPerformed = false;
		for (DialoguePolicy policy: policies) {
 
			try {
				// get the current node
				PolicyNode curNode = dialState.getPolicyPosition(policy.getPolicyId());

				// find the matching edges at the current node
				List<PolicyEdge> matchingEdges = policy.getMatchingEdges(curNode.getId(),obs, dialState);
				debug("Matching edges: " + matchingEdges);

				// sort the edges by preferential order
				PolicyUtils.sortEdges(matchingEdges);

				if (matchingEdges.size() > 0) {

					// select the best edge
					PolicyEdge selectedEdge = matchingEdges.get(0);
					log("best edge selected: " + selectedEdge);

					// performing the transition
					curNode = policy.getTargetNode(selectedEdge);
					log("now moving to node " + curNode.getId());
					dialState.setPolicyPosition(policy.getPolicyId(), curNode);
					transitionPerformed = true;

					// STEP 3: if the edge contains export variables, update 
					// the global information state as well
					updateInfoState (obs, selectedEdge);				

				}

				else {
					// non-valid observation at this position for the policy
					debug("Observation " + obs.toString() + " not applicable from node " + curNode.getId());
					debug("number of edges from node: " + policy.getAllOutgoingEdges(curNode.getId()).size());
					for (PolicyEdge matchingEdge: matchingEdges) {
						debug("Matching edge: " + matchingEdge.toString());
					}
					debug("Policy: " + policy.toString());
				}
			} 

			catch (DialogueException e) {
				debug("Warning: internal problem with policy " + policy.getPolicyId() +
				", aborting update process");
			}

		}

		return transitionPerformed;
	}



	/**
	 * Given an observation and a policy edge matching it (and containing an underspecified
	 * variable), udpate the information state with the new information
	 * 
	 * @param obs the observation
	 * @param selectedEdge the selected policy edge (which must match the observation
	 *        and contain an unserspecified variable)
	 *        
	 * @throws DialogueException if the extraction of the new information fails
	 */
	private void updateInfoState (Observation obs, PolicyEdge selectedEdge) 
	throws DialogueException {

		
		HashMap<String,dFormula> newInfos = new HashMap<String,dFormula>();
		
		// extract the arguments contained in the observation
		for (AbstractCondition cond: selectedEdge.getConditions()) {
			
			if (cond instanceof IntentionCondition) {
				newInfos.putAll(((IntentionCondition)cond).extractFilledArguments(obs));
			}
			else if (cond instanceof PhonstringCondition) {
				newInfos.putAll(((PhonstringCondition)cond).extractFilledArguments(obs));
			}
		}
		
		// for each argument, add a new belief to the information state
		for (String newInfoLabel : newInfos.keySet()) {
			
			// if the dialogue state already contains the same label, it is replaced
			if (dialState.hasInfoState(newInfoLabel)) {
				dialState.removeInfoState(newInfoLabel);
			}
			dFormula newInfoContent = newInfos.get(newInfoLabel);
			dBelief newBelief = 
				EpistemicObjectUtils.createSimpleBelief(newInfoContent, 1.0f, newInfoLabel);
			log("inserting the following information into the dialogue state: <" + newInfoLabel + ">" 
					+ FormulaUtils.getString(newInfoContent));
			dialState.addNewInfoState(newBelief);
		}
	}


	// ==============================================================
	// ACTION SELECTION METHODS
	// ==============================================================


	/**
	 * Returns the (possibly empty) list of policy actions which are executable 
	 * at the current dialogue state, wrapped in a ActionSelectionResult object.
	 * 
	 * If the selected actions are underspecified, the arguments are filled given
	 * the information provided in the information state.
	 * 
	 * @return the list of policy actions
	 */
	private ActionSelectionResult extractActionSelectionResult () {

		ActionSelectionResult result = new ActionSelectionResult();

		// loop on the policies
		for (DialoguePolicy policy: policies) {

			PolicyNode curNode;
			try {
				curNode = dialState.getPolicyPosition(policy.getPolicyId());

				for (AbstractAction action : curNode.getActions()) {

					if (action != null) {
						// if the outgoing action is underspecified, fill the arguments
						if (action.isUnderspecified()) {
							log("action " + action.getId() + " is underspecified");

							HashMap<String,dFormula> argumentValues = 
								dialState.extractArgumentValues(action.getUnderspecifiedArguments());

							action.fillArguments(argumentValues);
						} 
						
						log("action selected: " + action.toString());

						// if the action is a dialogue state action, add the new information
						// to the dialogue state
						if (action instanceof DialStateAction) {
							updateInfoState ((DialStateAction)action);
						}
						
						// else, simply add it to the result list
						else {
							result.addAction(action);
						}
					}
				}


			}
			catch (DialogueException e) {
				log("Warning: internal problem with the policy " + policy.getPolicyId() + 
				", aborting the action selection process");
			}
		}

		return result;
	}


	/**
	 * Update the information state with the information contained in a
	 * dialogue state action
	 * 
	 * @param action the dialogue state action containing the new information
	 * @throws DialogueException if the information cannot be inserted in the dialogue state
	 */
	private void updateInfoState (DialStateAction action) throws DialogueException {

		// introduce the postconditions, if any 
		log("adding new information to info state: " + action.toString());
			if (action.asFormula() instanceof ModalFormula) {
				String label = ((ModalFormula)action.asFormula()).op;
				dFormula content = ((ModalFormula)action.asFormula()).form;
				dialState.addNewInfoState(EpistemicObjectUtils.createSimpleBelief(content, 1.0f, label));
			}	
			else {
				log("WARNING: dialogue state action is not properly defined");
			}
	}


	// ==============================================================
	// GETTER METHODS
	// ==============================================================



	/**
	 * Get a list of all outdoing edges accessible from the current dialogue 
	 * state
	 * 
	 * @return the list of accessible edges
	 */
	public List<PolicyEdge> getAllOutgoingEdges() {
		List<PolicyEdge> edges = new LinkedList<PolicyEdge>();

		// loop on the policies
		for (DialoguePolicy policy : policies) {

			try {
				PolicyNode curNode = dialState.getPolicyPosition(policy.getPolicyId());
				List<PolicyEdge> accessibleEdges = policy.getAllOutgoingEdges(curNode.getId());
				edges.addAll(accessibleEdges);
			}
			catch (DialogueException e) {
				log("internal problem with policy " + policy.getPolicyId() + 
				", aborting extraction of outgoing edges");
			}
		}
		return edges;
	}


	/**
	 * Returns the dialogue state for the dialogue manager
	 * 
	 * @return the dialogue state
	 */
	public DialogueState getDialogueState() {
		return dialState;
	}


	/**
	 * Return the set of dialogue policies currently applied to the
	 * dialogue manager
	 * 
	 * @return the set of dialogue policies
	 */
	public List<DialoguePolicy> getDialoguePolicies() {
		return policies;
	}


	// ==============================================================
	// UTILITY METHODS
	// ==============================================================





	/**
	 * Logging
	 * @param s
	 */
	private void log (String s) {
		if (logger != null) {
			logger.info(s);
		}
		else if (LOGGING) {
			System.err.println("[dialmanager LOG] " + s);
		}
	}

	/**
	 * Debugging
	 * @param s
	 */
	private void debug (String s) {
		if (logger != null) {
			logger.debug(s);
		}
		else if (DEBUG) {
			System.err.println("[dialmanager DEBUG] " + s);
		}
	}

}
