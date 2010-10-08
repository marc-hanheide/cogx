
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


package de.dfki.lt.tr.dialmanagement.data.policies;

import java.util.HashMap;

import de.dfki.lt.tr.beliefs.slice.logicalcontent.ComplexFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.UnderspecifiedFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.data.FormulaWrapper;
import de.dfki.lt.tr.dialmanagement.utils.FormulaUtils;


/**
 * Representation of an action in the dialogue policy
 *  
 * @author Pierre Lison (plison@dfki.de)
 * @version 8/10/2010
 */

public class PolicyAction extends FormulaWrapper {

	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = true;
	
	// the identifier of the action
	private String id;

	// is the action void or not
	private boolean isVoid = false;
	
	
	/**
	 * Constructs a (void) action with a given identifier
	 * 
	 * @param id the identifier
	 */
	public PolicyAction(String id) {
		super ("");
		this.id = id;
		isVoid = true;
	}
	
	/**
	 * Constructs an action with an identifier and a string
	 * representing the action formula
	 * 
	 * @param id the identifier
	 * @param str the string containing the formula
	 */
	public PolicyAction(String id, String str) {
		super(str);
		this.id = id;
	}
	
	/**
	 * Constructs an action with an identifier and a formula
	 * 
	 * @param id the identifier
	 * @param formula the formula for the action
	 */
	public PolicyAction(String id, dFormula formula) {
		super(formula);
		this.id = id;
	}
	

	/**
	 * Returns true if the action is void, else false
	 *
	 * @return true if void action, false otherwise
	 */
	public boolean isVoid() {
		return isVoid;
	}
	
	/**
	 * Returns the identifier of the action
	 * 
	 * @return the identifier
	 */
	public String getId() {
		return id;
	}
	
	
	/**
	 * If the action content contains underspecified subformulae (of the form %i),
	 * replace these with the formulae provided in "arguments"
	 * 
	 * @param arguments the filled arguments (mapping the argument number i to its formula)
	 */
	public void fillActionArguments (HashMap<Integer,dFormula> arguments) {
		content = fillActionArguments (content, arguments);
	}

	
	
	/**
	 * If the formula contains underspecified subformulae (of the form %i),
	 * replace these with the formulae provided in arguments, and returns the 
	 * filled formula
	 * 
	 * @param arguments the filled arguments (mapping the argument number i to its formula)
	 * @param formula the formula to fill
	 * @return the filled formula
	 */
	private dFormula fillActionArguments (dFormula formula, HashMap<Integer,dFormula> arguments) {
		
		if (formula instanceof ComplexFormula) {
			for (dFormula subform : ((ComplexFormula)formula).forms) {
				subform = fillActionArguments(subform, arguments);
			}
		}
		else if (formula instanceof ModalFormula) {
			((ModalFormula)formula).form = fillActionArguments(((ModalFormula)formula).form, arguments);
		}
		
		else if (formula instanceof UnderspecifiedFormula) {
			if (arguments.containsKey(((UnderspecifiedFormula)formula).id)) {
				formula = arguments.get(((UnderspecifiedFormula)formula).id);
			}
		}
		return formula;
	}
	
	
	/**
	 * Returns a string representation of the action
	 */
	@Override
	public String toString() {
		if (isVoid) {
			return "VoidAction";
		}
		else {
			return "CI[" + FormulaUtils.getString(content) + "]";
		}
	}
	
	
	/**
	 * Create a new void action
	 * @return the void action
	 */
	public static PolicyAction createVoidAction() {
		return new PolicyAction("void");
	}
	
	
	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[policyaction] " + s);
		}
	}
	
	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[policyaction] " + s);
		}
	}
}
