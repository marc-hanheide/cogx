// =================================================================                                                        
// Copyright (C) 2009-2011 Pierre Lison (plison@dfki.de)                                                                
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

import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;

import java.util.Collection;
import java.util.HashMap;


/**
 * Representation of an observation for the dialogue manager, defined as a
 * set of (alternative) wrapped formulae associated with probabilities, plus a 
 * given type
 * 
 * @author Pierre Lison (plison@dfki.de)
 * @version 7/10/2010
 */

public class Observation {

	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = true;
	
	// the set of alternatives
	private HashMap<FormulaWrapper,Float> alternatives ;
	
	// the type of the observation
	private int observationType ;
	
	// defined type for an intention
	public static final int INTENTION = 0;
	
	// defined type for an event
	public static final int EVENT = 1;
	
	/**
	 * Creates an (empty) observation for a given type
	 * 
	 * @param observationType the type of the observation
	 * @throws DialogueException if the observation type is not allowed
	 */
	public Observation(int observationType) throws DialogueException {
		alternatives = new HashMap<FormulaWrapper, Float>();

		if (observationType == INTENTION || observationType == EVENT) {
			this.observationType = observationType;
		}
		else {
			throw new DialogueException("ERROR: observation type not allowed");
		}
	}
	
	/**
	 * Adds an alternative (as wrapped formula) to the observation
	 * 
	 * @param content the formula itself
	 * @param prob the probability of the formula
	 */
	public void addAlternative (FormulaWrapper content, float prob) {
		alternatives.put(content, prob);
	}
	
	/**
	 * Adds an alternative (as unwrapped formula) to the observation
	 * 
	 * @param formula the formula itself
	 * @param prob the probability of the formula
	 */
	public void addAlternative (dFormula formula, float prob) {
		alternatives.put(new FormulaWrapper (formula), prob);
	}
	
	
	/**
	 * Adds an alternative (as raw string) to the observation.  The formula
	 * associated to the string is first generated, and then inserted
	 * 
	 * @param str the string representing the formula
	 * @param prob the probability of the formula
	 */
	public void addAlternative (String str, float prob) {
		if (str.contains(" ") && !str.contains("^") && !str.contains("<")) {
			addAlternative(new FormulaWrapper ("\"" +str + "\""), prob);
		}
		else  {
			addAlternative(new FormulaWrapper (str), prob);
		}
	}
	
	/**
	 * Returns the set of alternative formulae contained in the observation
	 * 
	 * @return the set of alternative formulae
	 */
	public Collection<FormulaWrapper> getAlternatives () {
		return alternatives.keySet();
	}
	
	/**
	 * Returns the observation type
	 * 
	 * @return the type of the observation
	 */
	public int getType() {
		return observationType;
	}
	
	
	/**
	 * Returns the probability associated with the given formula.  If the formula
	 * is not included in the observation, returns 0.0f
	 * 
	 * @param content the formula
	 * @return the probability float
	 */
	public float getProbability (FormulaWrapper content) {
		if (alternatives.containsKey(content)) {
			return alternatives.get(content);
		}
		else {
			return 0.0f;
		}
	}
	
	/**
	 * Returns a string representation of the observation
	 */
	public String toString() {
		String result = "{";
		for (FormulaWrapper key : alternatives.keySet()) {
			result += "(" + key + ", "  + alternatives.get(key) + "), ";
		}
		return result.substring(0, result.length() -2) + "}";
	}
	
}
