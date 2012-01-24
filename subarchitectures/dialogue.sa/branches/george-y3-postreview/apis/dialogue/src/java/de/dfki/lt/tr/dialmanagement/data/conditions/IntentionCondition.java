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


package de.dfki.lt.tr.dialmanagement.data.conditions;

import java.util.HashMap;

import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.data.DialogueState;
import de.dfki.lt.tr.dialmanagement.data.Observation;
import de.dfki.lt.tr.dialmanagement.utils.FormulaUtils;
import de.dfki.lt.tr.dialmanagement.utils.PolicyUtils;


/**
 * Condition on an intentional object
 * 
 * @author Pierre Lison (plison@ifi.uio.no)
 * @version 22/12/2010
 *
 */
public class IntentionCondition extends AbstractCondition {


	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = false;

	// the content of the condition
	dFormula intentionCondition;


	/**
	 * Creates a policy condition with identifier and content (default probability
	 * thresholds are 0.0f and 1.0f)
	 * 
	 * @param id the identifier
	 * @param content the intention content (as formula)
	 */
	public IntentionCondition (String id, dFormula content) {
		super(id);
		this.intentionCondition = content;
	}



	/**
	 * Creates a policy condition with identifier and content (default probability
	 * thresholds are 0.0f and 1.0f)
	 * 
	 * @param id the identifier
	 * @param content the intention content (as string)
	 */
	public IntentionCondition (String id, String content) {
		super(id);
		try {
			this.intentionCondition = FormulaUtils.constructFormula(content);
		} catch (DialogueException e) {
			intentionCondition = new ElementaryFormula(0, content);
		}
	}

	/**
	 * Create a new policy condition given an identifier, a formula content, and 
	 * mimimum/maximum probabilities
	 * 
	 * @param id the identifier
	 * @param content the intention content (as formula)
	 * @param minProb the minimum probability
	 * @param maxProb the maximum probaiblity
	 */
	public IntentionCondition (String id, dFormula content, float minProb, float maxProb) {
		super(id);
		intentionCondition = content;
		this.minProb = minProb;
		this.maxProb = maxProb;
	} 



	/**
	 * Create a new policy condition given an identifier, a formula content, and 
	 * mimimum/maximum probabilities
	 * 
	 * @param id the identifier
	 * @param content the intention content (as string)
	 * @param minProb the minimum probability
	 * @param maxProb the maximum probaiblity
	 */
	public IntentionCondition (String id, String content, float minProb, float maxProb) {
		super(id);
		try {
			intentionCondition = FormulaUtils.constructFormula(content);
		} catch (DialogueException e) {
			intentionCondition = new ElementaryFormula(0, content);
		}
		this.minProb = minProb;
		this.maxProb = maxProb;
	} 


	/**
	 * Modifies the content of the condition
	 * 
	 * @param content
	 */
	public void setContent(dFormula content) {
		intentionCondition = content;
	}


	/**
	 * Checks if the condition matches the provided observation
	 */
	@Override
	public boolean matchCondition(Observation obs, DialogueState dialState) {
		return matchesObservation(obs);
	}

	/**
	 * Returns true if the policy condition matches a given observation
	 * 
	 * @param obs the observation
	 * @return true if a match is found, false otherwise
	 */
	public boolean matchesObservation (Observation obs) {

		debug("condition: " + this.toString() + "(" + this.intentionCondition.getClass().getSimpleName() + ")");

		for (dFormula alternative : obs.getAlternatives()) {

			debug("alternative: " + FormulaUtils.getString(alternative));
			
			if ((obs.getType() == Observation.INTENTION || 
					obs.getType() == Observation.PHONSTRING) && 
					obs.getProbability(alternative) >= minProb && 
					obs.getProbability(alternative) <= maxProb) {

				if (matchesAlternative (alternative)) {
					return true;
				}
			}
		}
		debug("match NOT found between: " + this.toString() + " and " + obs.toString());
		return false;
	}
	
	
	
	/**
	 * Returns true if the observation formula matches the intention condition,
	 * and false otherwise
	 * 
	 * @param alternative the observation formula
	 * @return true if match found, false otherwise
	 */
	private boolean matchesAlternative (dFormula alternative) {
		
		if (FormulaUtils.subsumes(intentionCondition, alternative)) {
			debug("match found between: " + this.toString() + " and " + FormulaUtils.getString(alternative));
			return true;
		} 

		// handling <post> operators when handling intentions
		else if (FormulaUtils.getString(alternative).contains("<post>") && 
				!FormulaUtils.getString(intentionCondition).contains("<post>")){

			debug("need to take the <post> modal operator into account");
			dFormula postCondFormula = new ModalFormula(0, "post", intentionCondition);
			if (FormulaUtils.subsumes(postCondFormula, alternative)) {
				debug("match found between: " + this.toString() + " and " + FormulaUtils.getString(alternative));
				return true;
			}
		} 
		
		return false;
	}


	/**
	 * If the condition contains underspecified variables which can be filled
	 * with the information in the observation, extract the information into
	 * a hashmap <label, value>
	 * 
	 * @param obs the observation containing the information
	 * @return a (possibly empty) hashmap with the extracted information
	 */
	public HashMap<String,dFormula> extractFilledArguments (Observation obs) {
		
		for (dFormula alternative : obs.getAlternatives()) {
			if (matchesAlternative(alternative)) {
				return PolicyUtils.extractFilledArguments(intentionCondition, alternative);
			}
		}
		
		return new HashMap<String,dFormula>();
	}
	

	/**
	 * Returns the node identifier
	 * 
	 * @return the identifier, as a string
	 */
	public String getId() {
		return id;
	}


	/**
	 * Returns a string representation of the policy condition
	 */
	@Override
	public String toString () {
		return "I[" + FormulaUtils.getString(intentionCondition) + " (" + minProb + ", " + maxProb + ")" + "]";
	}


	/**
	 * Returns the content of the condition
	 * @return
	 */
	public dFormula asFormula() {
		return intentionCondition;
	}


	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[policycondition] " + s);
		}
	}

	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[policycondition] " + s);
		}
	}


}
