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

import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.data.DialogueState;
import de.dfki.lt.tr.dialmanagement.data.Observation;
import de.dfki.lt.tr.dialmanagement.utils.FormulaUtils;


/**
 * Condition on an external event
 * 
 * @author Pierre Lison (plison@ifi.uio.no)
 * @version 22/12/2010
 *
 */
public class EventCondition extends AbstractCondition {


	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = false;

	// the content of the condition
	dFormula eventCondition;
	
	
	/**
	 * Creates a policy condition with identifier and content (default probability
	 * thresholds are 0.0f and 1.0f)
	 * 
	 * @param id the identifier
	 * @param content the event content (as formula)
	 */
	public EventCondition (String id, dFormula content) {
		super(id);
		this.eventCondition = content;
	}


	/**
	 * Create a new policy condition given an identifier, a formula content, and 
	 * mimimum/maximum probabilities
	 * 
	 * @param id the identifier
	 * @param content the event content (as formula)
	 * @param minProb the minimum probability
	 * @param maxProb the maximum probaiblity
	 */
	public EventCondition (String id, dFormula content, float minProb, float maxProb) {
		super(id);
		eventCondition = content;
		this.minProb = minProb;
		this.maxProb = maxProb;
	} 



	/**
	 * Create a new policy condition given an identifier, a formula content, and 
	 * mimimum/maximum probabilities
	 * 
	 * @param id the identifier
	 * @param content the event content (as string)
	 * @param minProb the minimum probability
	 * @param maxProb the maximum probaiblity
	 */
	public EventCondition (String id, String content, float minProb, float maxProb) {
		super(id);
		try {
			eventCondition = FormulaUtils.constructFormula(content);
		} catch (DialogueException e) {
			eventCondition = new ElementaryFormula(0, content);
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
		eventCondition = content;
	}

	
	
	/**
	 * Checks if the observation matches the event condition
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

		debug("condition: " + this.toString() + "(" + this.eventCondition.getClass().getSimpleName() + ")");
		
		for (dFormula alternative : obs.getAlternatives()) {
			
			debug("alternative: " + FormulaUtils.getString(alternative));
			if (FormulaUtils.subsumes(eventCondition, alternative) && 
					obs.getType() == Observation.EVENT && 
					obs.getProbability(alternative) >= minProb && 
					obs.getProbability(alternative) <= maxProb) {	
				debug("match found between: " + this.toString() + " and " + obs.toString());
				return true;
			} 


			// handling <post> operators when handling events
			else if (FormulaUtils.getString(alternative).contains("<post>") && 
					!FormulaUtils.getString(eventCondition).contains("<post>")){
				debug("need to take the <post> modal operator into account");
				dFormula postCondFormula = new ModalFormula(0, "post", eventCondition);
				if (FormulaUtils.subsumes(postCondFormula, alternative) && 
						obs.getProbability(alternative) >= minProb && 
						obs.getProbability(alternative) <= maxProb) {
					debug("match found between: " + this.toString() + " and " + obs.toString());
					return true;
				}
			} 
		}
		debug("match NOT found between: " + this.toString() + " and " + obs.toString());
		return false;
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
		return "I[" + FormulaUtils.getString(eventCondition) + " (" + minProb + ", " + maxProb + ")" + "]";
	}
	
 
	/**
	 * Returns the content of the condition
	 * @return
	 */
	public dFormula asFormula() {
		return eventCondition;
	}
	
	
	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[eventcondition] " + s);
		}
	}

	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[eventcondition] " + s);
		}
	}

}
