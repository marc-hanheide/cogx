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

import de.dfki.lt.tr.beliefs.slice.logicalcontent.IntegerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.data.DialogueState;
import de.dfki.lt.tr.dialmanagement.data.Observation;
import de.dfki.lt.tr.dialmanagement.utils.FormulaUtils;

/**
 * Timeout condition (transition automatically triggered after 
 * a specific number of milliseconds)
 * 
 * @author Pierre Lison (plison@ifi.uio.no)
 * @version 22/12/2010
 *
 */
public class TimeoutCondition extends AbstractCondition {

	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = false;

	// the timeout for the condition, in milliseconds
	int timeoutCondition;

	/**
	 * Creates a policy condition with identifier and content (default probability
	 * thresholds are 0.0f and 1.0f)
	 * 
	 * @param id the identifier
	 * @param timeout the timeout (in milliseconds)
	 */
	public TimeoutCondition (String id, int timeout) {
		super(id);
		this.timeoutCondition = timeout;
	}


	/**
	 * Create a new policy condition given an identifier, a formula content, and 
	 * mimimum/maximum probabilities
	 * 
	 * @param id the identifier
	 * @param timeout the timeout (in milliseconds)
	 * @param minProb the minimum probability
	 * @param maxProb the maximum probaiblity
	 */
	public TimeoutCondition (String id, int timeout, float minProb, float maxProb) {
		super(id);
		timeoutCondition = timeout;
		this.minProb = minProb;
		this.maxProb = maxProb;
	} 


	/**
	 * Modifies the content of the condition
	 * 
	 * @param content
	 */
	public void setContent(int timeout) {
		timeoutCondition = timeout;
	}

	
	public int getTimeout () {
		return timeoutCondition;
	}


	/**
	 * Checks if the observation matches the condition (in this case,
	 * if a sufficient amount of time has passed)
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

		debug("condition: " + this.toString() + "(timeout)");

		debug("type: " + obs.getType());
		debug("nb alternatives: " + obs.getAlternatives().size());
		if (obs.getType() == Observation.TIMEOUT) {
			for (dFormula alternative : obs.getAlternatives()) {

				debug("alternative: " + FormulaUtils.getString(alternative));

				if (alternative instanceof IntegerFormula) {
					int time = ((IntegerFormula)alternative).val;
					return (time >= timeoutCondition);
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
		return "[" + timeoutCondition + "ms " + " (" + minProb + ", " + maxProb + ")" + "]";
	}


	/**
	 * Returns the content of the condition
	 * @return
	 */
	public dFormula asFormula() {
		return new IntegerFormula(0, timeoutCondition);
	}


	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[timeoutcondition] " + s);
		}
	}

	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[timeoutcondition] " + s);
		}
	}


}
