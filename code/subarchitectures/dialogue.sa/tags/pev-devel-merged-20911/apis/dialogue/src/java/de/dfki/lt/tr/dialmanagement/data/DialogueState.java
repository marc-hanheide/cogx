
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


package de.dfki.lt.tr.dialmanagement.data;

import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;

import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues;
import de.dfki.lt.tr.beliefs.slice.epobject.EpistemicObject;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.data.Observation;
import de.dfki.lt.tr.dialmanagement.data.policies.DialoguePolicy;
import de.dfki.lt.tr.dialmanagement.data.policies.PolicyNode;
import de.dfki.lt.tr.dialmanagement.utils.FormulaUtils;
import org.apache.log4j.Logger;

/**
 * Representation of the global dialogue state manipulated by the dialogue manager.
 * The dialogue state includes: <ul>
 * <li> an observation history,
 * <li> a global information state (expressed as epistemic objects),
 * <li> the node position for each policy
 * </ul>
 * 
 * @author Pierre Lison (plison@ifi.uio.no)
 * @version 22/12/2010
 *
 */
public class DialogueState {

	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = false;

	private Logger logger = null;

	// the observation history
	List<Observation> history;

	// the global information state
	List<EpistemicObject> infoState;

	// the node position for each policy
	HashMap<String,PolicyNode> policyPositions;

	
	// ==============================================================
	// INITIALISATION METHODS
	// ==============================================================


	/**
	 * Initialise the dialogue state
	 */
	public DialogueState() {
		history = new LinkedList<Observation>();
		infoState = new LinkedList<EpistemicObject>();
		policyPositions = new HashMap<String,PolicyNode>();
	}

	public DialogueState(Logger _logger) {
		this();
		logger = _logger;
	}
	
	/**
	 * Initialise the policy in the shared dialogue state
	 * 
	 * @param policy the policy to initialise
	 * @throws DialogueException if the policy is already present
	 */
	public void initialisePolicy (DialoguePolicy policy) throws DialogueException {
		if (!policyPositions.containsKey(policy.getPolicyId())) {
			policyPositions.put(policy.getPolicyId(), policy.getInitNode());
		}
		else {
			throw new DialogueException
			("ERROR, policy " + policy.getPolicyId() + " already inserted in dialogue state");
		}
	}
	
	
	
	// ==============================================================
	// SETTER METHODS
	// ==============================================================


	/**
	 * Add a new observation to the history
	 * 
	 * @param obs the new observation
	 */
	public void addToHistory (Observation obs) {
		history.add(obs);
	}
	
	
	/**
	 * Change the node position for the referenced policy
	 * 
	 * @param id the policy identifier
	 * @param node the policy node (which should become the new position)
	 * @throws DialogueException if the referenced policy is not in the dialogue state
	 */
	public void setPolicyPosition (String policyId, PolicyNode node) throws DialogueException {
		if (policyPositions.containsKey(policyId)) {
			policyPositions.put(policyId, node);
		}
		else {
			throw new DialogueException("ERROR: policy " + policyId + " not contained in dialogue state");
		}
	}
	
	
	/**
	 * Add a new epistemic object to the global information state
	 * 
	 * @param obj the epistemic object to add to the state
	 */
	public void addNewInfoState (EpistemicObject obj) {
		infoState.add(obj);
	}
	
	
	/**
	 * Remove an epistemic object from the global information state
	 * (if object not there, do nothing)
	 * 
	 * @param obj the epistemic object to remove
	 */
	public void removeInfoState (EpistemicObject obj) {
		infoState.remove(obj);
	}
	
	
	
	
	/**
	 * Remove an epistemic object from the global information state, using
	 * the variable label
	 * 
	 * @param obj the epistemic object to remove
	 */
	public void removeInfoState (String label) {
		
		EpistemicObject toRemove = null;
		for (EpistemicObject o : infoState) {
			
			if (o instanceof dBelief && ((dBelief)o).content instanceof BasicProbDistribution) {
				BasicProbDistribution distrib = (BasicProbDistribution) ((dBelief)o).content;
				if (distrib.key.equals(label)) {
					toRemove = o;
				}
			}
		}
		if (toRemove != null) {
			infoState.remove(toRemove);
		}
	}
	
	
	// ==============================================================
	// GETTER METHODS
	// ==============================================================

	
	/**
	 * Get the current position for the referenced policy
	 * 
	 * @throws DialogueException if the policy not there in dialogue state
	 */
	public PolicyNode getPolicyPosition (String policyId) throws DialogueException {
		if (policyPositions.containsKey(policyId)) {
			return policyPositions.get(policyId);
		}
		else {
			throw new DialogueException ("ERROR, policy not available in dialogue state");
		}
	}
	
	
	/**
	 * Get the last observation in the state history
	 * 
	 * @return the last observation
	 * @throws DialoguException if the history is empty
	 */
	public Observation getLastObservation () throws DialogueException {
		if (history.size() > 0) {
		return history.get(history.size() -1);
		}
		else {
			throw new DialogueException("ERROR: not observation in history");
		}
	}

	
	/**
	 * Returns true if the information state contains a variable with 
	 * the provided label, otherwise returns false
	 * 
	 * @param label the variable label to verify
	 * @return true if an epistemic object exists with the following feature
	 *         label, else false
	 * @throws DialogueException if the epistemic object format not supported
	 */
	public boolean hasInfoState (String label) throws DialogueException {
		
		log("number of objects in information state: " + infoState.size());
		
		for (EpistemicObject o : infoState) {
			
			if (o instanceof dBelief && ((dBelief)o).content instanceof BasicProbDistribution) {
				BasicProbDistribution distrib = (BasicProbDistribution) ((dBelief)o).content;
				if (distrib.key.equals(label)) {
					return true;
				}
			}
			else {
				throw new DialogueException ("ERROR: format of epistemic object " + 
						((dBelief)o).id + " currently not supported");
			}
		}
		return false;
	}
	
	
	/**
	 * Returns the (most likely) formula associated with the given variable label, 
	 * if one exists.  Else throws a dialogue exception
	 * 
	 * @param label the variable label to search
	 * @return the formula (feature content) associated with the label
	 * @throws DialogueException if no formula can be found
	 */
	public dFormula getMostLikelyInfoStateContent (String label) throws DialogueException {
		
		for (EpistemicObject o : infoState) {
			
			if (o instanceof dBelief && ((dBelief)o).content instanceof BasicProbDistribution) {
				
				BasicProbDistribution distrib = (BasicProbDistribution) ((dBelief)o).content;
				if (distrib.key.equals(label)) {
					if (distrib.values instanceof FormulaValues) {
						List<FormulaProbPair> pairs = ((FormulaValues)distrib.values).values;
						return getMostLikelyFormula(pairs);
					}
				}
			}
			else {
				throw new DialogueException ("ERROR: format of epistemic object " + 
						((dBelief)o).id + " currently not supported");
			}
		}
		
		throw new DialogueException("ERROR: no formula could be found in " +
				"the information state for the label: " + label);
	}
	
	

	/**
	 * Returns the set of formulae associated with the given variable label (combined with
	 * their probabilities), if one exists.  Else throws a dialogue exception
	 * 
	 * @param label the variable label to search
	 * @return the formulae (feature content) associated with the label and their probabilities
	 * @throws DialogueException if no formula can be found
	 */
	public List<FormulaProbPair> getInfoStateContent (String label) throws DialogueException {
		
		for (EpistemicObject o : infoState) {
			
			if (o instanceof dBelief && ((dBelief)o).content instanceof BasicProbDistribution) {
				
				BasicProbDistribution distrib = (BasicProbDistribution) ((dBelief)o).content;
				if (distrib.key.equals(label)) {
					if (distrib.values instanceof FormulaValues) {
						List<FormulaProbPair> pairs = ((FormulaValues)distrib.values).values;
						return pairs;
					}
				}
			}
			else {
				throw new DialogueException ("ERROR: format of epistemic object " + 
						((dBelief)o).id + " currently not supported");
			}
		}
		
		throw new DialogueException("ERROR: no formula could be found in " +
				"the information state for the label: " + label);
	}
	
	

	/**
	 * Extract the argument values for the set of labels, and build a hashmap
	 * mapping each label to the value (if one can be found)
	 * 
	 * @param labels the set of labels
	 * @return the hashmap mapping the labels and the values
	 */
	public HashMap<String, dFormula> extractArgumentValues (Collection<String> labels) {
		
		HashMap<String,dFormula> arguments = new HashMap<String,dFormula>();
		
		for (String label : labels) {
			try {
			if (hasInfoState(label)) {
				dFormula filledContent = getMostLikelyInfoStateContent(label);
				arguments.put(label, filledContent);
			}
			else {
				log("sorry, no belief with label: " + label);
			}
			}
			catch (DialogueException e) {
				log("problem with the information state for label: " + label);
			}
		}
		return arguments;
	}

	
	
	
	// ==============================================================
	// UTILITY METHODS
	// ==============================================================
	

	/**
	 * Return the most likely formula in the provided set of formula pairs
	 * 
	 * @param formulaProbPairs the formula pairs
	 * @return the formula with the highest probability, if any
	 * @throws DialogueException if no formula has a prob. > 0
	 */
	private dFormula getMostLikelyFormula (List<FormulaProbPair> formulaProbPairs) 
		throws DialogueException {
		
		float highestProb = 0.0f;
		dFormula mostLikelyFormula = null;
		
		for (FormulaProbPair pair : formulaProbPairs) {
			
			if (pair.prob >= highestProb) {
				mostLikelyFormula = pair.val;
				highestProb = pair.prob;
			}
		}
		
		if (mostLikelyFormula != null) {
			return mostLikelyFormula;
		}
		else {
			throw new DialogueException("ERROR: not formula with probability > 0.0f");
		}
	}

	
	/**
	 * Returns a string representation of the dialogue state
	 */
	@Override
	public String toString() {
		
		String str = "";
		
		// outputting the history
		int i = 1;
		for (Observation obs: history) {
			str += "observation " + i +": " + obs.toString() + "\n";
			i++;
		}
		
		// outputting the policy positions
		for (String policy: policyPositions.keySet()) {
			str += policy + ": " + policyPositions.get(policy).getId() + "\n";
		}
		
		// outputting the information state
		for (EpistemicObject o : infoState) {
			if (o instanceof dBelief && ((dBelief)o).content instanceof BasicProbDistribution) {
				BasicProbDistribution distrib = (BasicProbDistribution) ((dBelief)o).content;
								
				if (distrib.values instanceof FormulaValues) {
					List<FormulaProbPair> pairs = ((FormulaValues)distrib.values).values;
					try {
						dFormula form = getMostLikelyFormula(pairs);
						str += distrib.key + "=" + FormulaUtils.getString(form) + "\n";
					} catch (DialogueException e) {
						e.printStackTrace();
					}
				}
			}
		}
		str = str.substring(0, str.length() - 1);
		return str;
	}
	
	/**
	 * Logging
	 * @param s
	 */
	private void log (String s) {
		if (logger != null) {
			logger.info(s);
		}
		else if (LOGGING) {
			System.err.println("[dialstate LOG] " + s);
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
			System.err.println("[dialstate DEBUG] " + s);
		}
	}
}
