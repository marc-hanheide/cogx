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
import java.util.StringTokenizer;

import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.data.DialogueState;
import de.dfki.lt.tr.dialmanagement.data.Observation;
import de.dfki.lt.tr.dialmanagement.utils.FormulaUtils;


/**
 * Condition on a phonological string
 * 
 * @author Pierre Lison (plison@ifi.uio.no)
 * @version 22/12/2010
 *
 */
public class PhonstringCondition extends AbstractCondition{

	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = false;

	// the content of the condition
	String phonstringCondition;
	
	
	/**
	 * Creates a policy condition with identifier and content (default probability
	 * thresholds are 0.0f and 1.0f)
	 * 
	 * @param id the identifier
	 * @param content the condition content
	 */
	public PhonstringCondition (String id, String content) {
		super(id);
		this.phonstringCondition = content;
	}


	/**
	 * Create a new policy condition given an identifier, a formula content, and 
	 * mimimum/maximum probabilities
	 * 
	 * @param id the identifier
	 * @param content the phonstring content
	 * @param minProb the minimum probability
	 * @param maxProb the maximum probaiblity
	 */
	public PhonstringCondition (String id, String content, float minProb, float maxProb) {
		super(id);
		phonstringCondition = content;
		this.minProb = minProb;
		this.maxProb = maxProb;
	} 

	
	/**
	 * Modifies the content of the condition
	 * 
	 * @param content
	 */
	public void setContent(String content) {
		phonstringCondition = content;
	}

		
	/**
	 * Checks if the observation matches the condition
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

		debug("condition: " + this.toString() + "(phonstring)");
		
		for (dFormula alternative : obs.getAlternatives()) {
			
			debug("alternative: " + FormulaUtils.getString(alternative));
			if (FormulaUtils.subsumes(asFormula(), alternative) && 
					obs.getType() == Observation.PHONSTRING && 
					obs.getProbability(alternative) >= minProb && 
					obs.getProbability(alternative) <= maxProb) {	
				debug("match found between: " + this.toString() + " and " + obs.toString());
				return true;
			} 

		}
		debug("match NOT found between: " + this.toString() + " and " + obs.toString());
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
	public HashMap<String,dFormula> extractFilledArguments (Observation obs) throws DialogueException {

		HashMap<String,dFormula> filledArguments = new HashMap<String,dFormula>();

		if (!phonstringCondition.contains("%")) {
			return filledArguments;
		}
		
		for (dFormula alternative : obs.getAlternatives()) {

			if (alternative instanceof ElementaryFormula) {

				String[] initSplit = phonstringCondition.split("%");

				log("extracting argument in phonstring");

				if (initSplit.length > 2) {
					throw new DialogueException("More than two variables in an ElementaryFormula");
				}

				String varName = "";

				for (int i = 1 ; i < initSplit.length ; i++) {
					StringTokenizer t = new StringTokenizer(initSplit[i]);
					varName = t.nextToken().trim();
				}

				String[] fullSplit = phonstringCondition.split("%"+varName);

				String remainder = ((ElementaryFormula)alternative).prop;
				for (int j = 0 ; j < fullSplit.length ; j++) {
					remainder = remainder.replace(fullSplit[j], "");
				}
				filledArguments.put(varName, new ElementaryFormula(0,remainder));
				log("putting argument " + varName +": " + remainder);
			}
		}

		return filledArguments;
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
		return "[" + phonstringCondition + " (" + minProb + ", " + maxProb + ")" + "]";
	}
	
 
	/**
	 * Returns the content of the condition
	 * @return
	 */
	public dFormula asFormula() {
		return new ElementaryFormula(0, phonstringCondition);
	}
	
	
	
	
	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[phonstringcondition] " + s);
		}
	}

	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[phonstringcondition] " + s);
		}
	}


}
