// =================================================================                                                        
// Copyright (C) 2009-2011 Pierre Lison (pierre.lison@dfki.de)                                                                
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

import java.util.Arrays;
import java.util.Collection;
import java.util.List;

import de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;
import de.dfki.lt.tr.beliefs.slice.epobject.EpistemicObject;
import de.dfki.lt.tr.beliefs.slice.events.Event;
import de.dfki.lt.tr.beliefs.slice.intentions.Intention;
import de.dfki.lt.tr.beliefs.slice.intentions.IntentionalContent;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.data.actions.AbstractAction;
import de.dfki.lt.tr.dialmanagement.data.actions.VoidAction;
import de.dfki.lt.tr.dialmanagement.data.observations.Observation;
import de.dfki.lt.tr.dialmanagement.data.observations.ObservationContent;
import de.dfki.lt.tr.dialmanagement.data.ActionNode;
import de.dfki.lt.tr.dialmanagement.data.DialoguePolicy;
import de.dfki.lt.tr.dialmanagement.data.ObservationEdge;
import de.dfki.lt.tr.dialmanagement.utils.EpistemicObjectUtils;


/**
 * A simple, FSA-based dialogue manager.  The manager revolves around a dialogue policy and
 * a pointer to the current node in the policy.  The traversal of the policy is realised
 * with the nexAction method, which takes an observation as argument, and jumps to the
 * appropriate next action.
 * 
 * @author Pierre Lison (plison@dfki.de)
 * @version 3/10/2010
 *
 */
public class DialogueManager {


	public static boolean LOGGING = true;

	public static boolean DEBUG = true;

	// the dialogue policy
	DialoguePolicy policy;

	// the current action node
	ActionNode curNode;

	/**
	 * Create a new dialogue manager based on a dialogue policy
	 * 
	 * @param policy the dialogue policy (typically extracted from a configuration file)
	 */
	public DialogueManager(DialoguePolicy policy) {
		this.policy = policy;
		curNode = policy.getInitNode();
	}



	/**
	 * Jumps to the next appropriate action given the intention provided as argument.  
	 * The method extracts all alternative intentional contents and sorts them by decreasing 
	 * probability.  It then checks whether an observation matching the intentional content
	 * is available.  
	 * 
	 * If one exists, the method assigns curNode to be the node pointed by the observation edge,
	 * and returns it. Otherwise, a VoidAction is returned.
	 *   
	 * @param intention the intention
	 * @return the next action if one is available, or else a void action 
	 * @throws DialogueException if the content of the intention or the dialogue policy
	 *         is ill-formatted
	 */
	public AbstractAction nextAction(Intention intention) throws DialogueException {
	
	//	List<IntentionalContent> content = 
	//		EpistemicObjectUtils.sortIntentionalContent(intention.content);

		Observation obs = new Observation();
		for (IntentionalContent icontent : intention.content) {
			obs.addAlternative(icontent.postconditions, ObservationContent.INTENTION, icontent.probValue);
		}
	
		for (boolean underspecification : Arrays.asList(false, true)) {

				AbstractAction nextAction = nextAction (obs, underspecification);
				if (!(nextAction instanceof VoidAction)) {
					return nextAction;
				}
			}

		// else, return a void action
		return new VoidAction();
	}




	/**
	 * Jumps to the next appropriate action given the event provided as argument.  
	 * The method extracts all alternative contents for the event and sorts them by decreasing 
	 * probability.  It then checks whether an observation matching the content
	 * is available.  
	 * 
	 * If one exists, the method assigns curNode to be the node pointed by the observation edge,
	 * and returns it. Otherwise, a VoidAction is returned.
	 *   
	 * @param event the event
	 * @return the next action if one is available, or else a void action 
	 * @throws DialogueException if the content of the event or the dialogue policy
	 *         is ill-formatted
	 */
	public AbstractAction nextAction(Event event) throws DialogueException {

// 		ProbDistribution content = 
//			EpistemicObjectUtils.sortDiscreteDistribution(event.content);
/**
		for (boolean underspecification : Arrays.asList(false, true)) {
			
			for (FormulaProbPair pair : EpistemicObjectUtils.getFormulaProbPairs(content)) {
				
				IntentionObservation observ = new IntentionObservation (pair.val, pair.prob);
				debug("testing observation: " + observ.toString());
				AbstractAction nextAction = nextAction (observ,underspecification);
				if (!(nextAction instanceof VoidAction)) {
					return nextAction;
				}
			}
		}

 	*/
		// else, return a void action
		return new VoidAction(); 
	} 



	/**
	 * Jumps to the next appropriate action in the policy given a particular observation. 
	 * Two separate cases are distinguished:
	 * - if the dialogue policy has an outgoing observation edge starting from curNode and
	 *   whose content matches the content in obs, then assigns curNode to be the node pointed by 
	 *   the edge, and returns it
	 * - if no such observation edge is available from curNode, then returns a VoidAction,
	 *   and keep the curNode as it is
	 * 
	 * @param obs the observation (which might be a ShallowObservation, an EventObservation,
	 *            or an IntentionObservation)
	 *            
	 * @return if a node jump is available at the current node with the given observation, returns
	 *         the next action.   Else, returns a VoidAction
	 * @throws DialogueException 
	 *         
	 * @throws DialogueException exception thrown if the policy or the observation is ill-formed
	 */
	public AbstractAction nextAction (Observation obs) throws DialogueException {
		
		for (boolean underspecification : Arrays.asList(false, true)) {
			
			AbstractAction nextAction = nextAction (obs, underspecification);
			if (!(nextAction instanceof VoidAction)) {
				return nextAction;
			}
		}
		
		// else, return a void action
		return new VoidAction();
	}





	/**
	 * (Same as above, but including a flag to activate or deactivate the use of underspecified 
	 * observations)
	 */
	private AbstractAction nextAction (Observation obs, boolean underspecification) throws DialogueException {
		Collection<ObservationEdge> matchingEdges = curNode.getMatchingOutgoingObservations(obs);
		for (ObservationEdge matchingEdge : matchingEdges) {
			if (underspecification || !matchingEdge.getObservation().isUnderspecified()) {
				curNode = matchingEdge.getOutgoingAction();
				debug("FOUND! now going to " + curNode.getId());
				debug(policy.toString());
				return curNode.getAction();
			}
		}

		debug("Warning: observation " + obs.toString() + " not applicable from node " + curNode.getId());
		debug(policy.toString());
		for (ObservationEdge edge : curNode.getAllOutgoingObservations()) {
			debug ("obs: " + edge.getObservation().toString());
		}
		return new VoidAction();
	}



	/**
	 * Returns the action encapsulated in the current node
	 * 
	 * @return the current action
	 */
	public AbstractAction getCurrentAction () {
		return curNode.getAction();
	}


	/**
	 * Returns true if the manager reached a final state in the dialogue policy, 
	 * false otherwise
	 * 
	 * @return true if the policy is finished, false otherwise
	 */
	public boolean isFinished() {
		return policy.isFinalNode(curNode);
	}

	/**
	 * Returns true if the manager is set on the initial state of the dialogue
	 * policy, false otherwise
	 * 
	 * @return true if the policy is starting, false otherwise
	 */
	public boolean isStarting() {
		return policy.isInitNode(curNode);
	}



	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[dialmanager] " + s);
		}
	}

	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[dialmanager] " + s);
		}
	}
}
