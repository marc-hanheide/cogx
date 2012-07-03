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

import de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.data.DialogueState;
import de.dfki.lt.tr.dialmanagement.data.Observation;
import de.dfki.lt.tr.dialmanagement.utils.FormulaUtils;

/**
 * A condition on the dialogue state
 * 
 * Note: the dialogue state condition supported at the moment are limited
 * to simple modal formula of the type <VARIABLE-LABEL>variable-value
 * 
 * @author Pierre Lison (plison@ifi.uio.no)
 * @version 22/12/2010
 *
 */
public class DialStateCondition extends AbstractCondition {


	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = false;

	// the content of the condition
	ModalFormula dialstateCondition;
	
	
	/**
	 * Creates a policy condition with identifier and content (default probability
	 * thresholds are 0.0f and 1.0f)
	 * 
	 * @param id the identifier
	 * @param content the intention content (as formula)
	 */
	public DialStateCondition (String id, ModalFormula content) {
		super(id);
		this.dialstateCondition = content;
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
	public DialStateCondition (String id, ModalFormula content, float minProb, float maxProb) {
		super(id);
		dialstateCondition = content;
		this.minProb = minProb;
		this.maxProb = maxProb;
	} 


	
	/**
	 * Modifies the content of the condition
	 * 
	 * @param content
	 */
	public void setContent(ModalFormula content) {
		dialstateCondition = content;
	}

		

	/**
	 * Checks if the current dialogue state matches the condition
	 */
	@Override
	public boolean matchCondition(Observation obs, DialogueState dialState) {
		return matchesPreconditions(dialState);
	}
	

	/**
	 * Check if the provided dialogue state matches the precondition 
	 * required for the policy condition to apply
	 * 
	 * @param dialState the dialogue state
	 * @return true if the content of the dialogue state matches the requirements
	 *         false otherwise
	 */
	public boolean matchesPreconditions(DialogueState dialState) {
	
			String label = ((ModalFormula)dialstateCondition).op;
			dFormula value = ((ModalFormula)dialstateCondition).form;

			try {
				// checking if the dialogue state contains the provided label
				if (dialState.hasInfoState(label)) {	
					
					// if yes, loop on the alternative values
					for (FormulaProbPair pair: dialState.getInfoStateContent(label)) {

						debug("info state content: <"+label+">" + FormulaUtils.getString(pair.val));

						if (FormulaUtils.subsumes(value, pair.val) 
								&& pair.prob >= minProb && pair.prob <= maxProb) {
							return true;
						}

					}
				}	
				else {
					debug("unable to apply precondition: feature " + label + " is not specified " +
					"in dialogue state.  Assuming false");
					return false;
				}
			}
			catch (DialogueException e) {
				e.printStackTrace();
			}

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
		return "dialstatecond[" + FormulaUtils.getString(dialstateCondition) + " (" + minProb + ", " + maxProb + ")" + "]";
	}
	
 
	/**
	 * Returns the content of the condition
	 * @return
	 */
	public dFormula asFormula() {
		return dialstateCondition;
	}
	
	
	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[dialstatecondition] " + s);
		}
	}

	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[dialstatecondition] " + s);
		}
	}





}
