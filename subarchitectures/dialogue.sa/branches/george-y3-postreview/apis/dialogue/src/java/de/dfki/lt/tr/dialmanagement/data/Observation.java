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

import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.utils.FormulaUtils;
import de.dfki.lt.tr.dialogue.slice.asr.PhonString;

import java.util.Collection;
import java.util.HashMap;

 
/**
 * Representation of an observation for the dialogue manager, defined as a
 * set of formulae (each of which is associated with a probability), plus
 * an observation type
 * 
 * @author Pierre Lison (plison@ifi.uio.no)
 * @version 16/12/2010
 */
 
public class Observation {

	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = false;
	
	// the set of alternatives
	private HashMap<dFormula,Float> alternatives ;
	
	// the type of the observation
	private int type ;
	
	// allowed observation types
	public static final int INTENTION = 0;
	public static final int EVENT = 1;
	public static final int PHONSTRING = 2;
	public static final int TIMEOUT = 3;
	
	
	// ==============================================================
	// OBSERVATION CONSTRUCTION METHODS
	// ==============================================================
	
	
	/**
	 * Creates an (empty) observation for a given type
	 * 
	 * @param observationType the type of the observation
	 * @throws DialogueException if the observation type is not allowed
	 */
	public Observation(int type) {
		alternatives = new HashMap<dFormula, Float>();

		if (type == INTENTION || type == EVENT || type == PHONSTRING || type == TIMEOUT) {
			this.type = type;
		}
		else {
			debug("non admissible observation type, setting observation as intention");
			this.type = INTENTION;
		}
	}
	
	
	/**
	 * Add a phonstring alternative to the observation
	 * 
	 * @param phon the phonological string
	 * @param prob the probability of the phonstring
	 */
	public void addAlternative(PhonString phon, float prob) {
		alternatives.put(new ElementaryFormula(0,phon.wordSequence), prob);
	}
	
	
	/**
	 * Adds an alternative (as formula) to the observation
	 * 
	 * @param formula the formula itself
	 * @param prob the probability of the formula
	 */
	public void addAlternative (dFormula formula, float prob) {
		alternatives.put(formula, prob);
	}
	
	
	/**
	 * Adds an alternative (as raw string) to the observation.  The formula
	 * associated to the string is first generated, and then inserted
	 * 
	 * @param str the string representing the formula
	 * @param prob the probability of the formula
	 */
	public void addAlternative (String str, float prob) {
		try {
		if (str.contains(" ") && !str.contains("^") && !str.contains("<")) {
				addAlternative(FormulaUtils.constructFormula("\"" +str + "\""), prob);
			} 
		else  {
			addAlternative(FormulaUtils.constructFormula(str), prob);
		}
		}
		catch (DialogueException e) {
			addAlternative(new ElementaryFormula(0, str), prob);
		}
	}
	
	
	
	// ==============================================================
	// GETTER METHODS
	// ==============================================================
	
	
	
	/**
	 * Returns the set of alternative formulae contained in the observation
	 * 
	 * @return the set of alternative formulae
	 */
	public Collection<dFormula> getAlternatives () {
		return alternatives.keySet();
	}
	
	/**
	 * Returns the observation type
	 * 
	 * @return the type of the observation
	 */
	public int getType() {
		return type;
	}
	
	
	/**
	 * Returns the probability associated with the given formula.  If the formula
	 * is not included in the observation, returns 0.0f
	 * 
	 * @param content the formula
	 * @return the probability float
	 */
	public float getProbability (dFormula content) {
		if (alternatives.containsKey(content)) {
			return alternatives.get(content);
		}
		else {
			return 0.0f;
		}
	}
	
	
	
	
	// ==============================================================
	// UTILITY METHODS
	// ==============================================================
	
	
	
	/**
	 * Returns a string representation of the observation
	 */
	public String toString() {
		String result = "{";
		if (alternatives.size() > 0) {
		for (dFormula key : alternatives.keySet()) {
			result += "(" + FormulaUtils.getString(key) + ", "  + alternatives.get(key) + "), ";
		}
		return result.substring(0, result.length() -2) + "}";
		}
		else {
			return "{emptyObs}";
		}
	}
	


	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[observation] " + s);
		}
	}

	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[observation] " + s);
		}
	}
}
