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


package de.dfki.lt.tr.dialmanagement.data.actions;

import java.util.Collection;
import java.util.HashMap;

import de.dfki.lt.tr.beliefs.slice.logicalcontent.ComplexFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.UnderspecifiedFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.utils.FormulaUtils;
import de.dfki.lt.tr.dialmanagement.utils.PolicyUtils;


/**
 * Policy action defined as a logical intention
 * 
 * @author Pierre Lison (plison@ifi.uio.no)
 * @version 21/12/2010
 *
 */
public class IntentionAction extends AbstractAction {


	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = false;
	
	// epistemic status of the intention
	int status;
	
	// list of allowed epistemic status
	public static final int COMMUNICATIVE = 0;
	public static final int PRIVATE = 1;
	public static final int ATTRIBUTED= 2;
	
	
	// the initial, possibly underspecified action content
	protected dFormula initContent;
	
	// instantiated action content (with filled arguments)
	dFormula instantiatedContent;
	
	
	/**
	 * Creates an intention action, with a unique identifier and
	 * a logical formula
	 * 
	 * @param id the identifier
	 * @param formula the (possibly underspecified) formula
	 */
	public IntentionAction(String id, dFormula formula) {
		super(id);
		try {
			initContent = formula;
			instantiatedContent = FormulaUtils.copy(initContent);
			} catch (DialogueException e) {
				e.printStackTrace();
			}
	}
	


	/**
	 * If the action is underspecified, and assuming the underspecification
	 * can be resolved using the provided arguments, bind the variables
	 * to the values in the arguments.
	 * 
	 * If the variable resolution fails, an exception is thrown.
	 * 
	 * (abstract class to be provided by the concrete class)

	 * @param arguments the arguments to fill
	 * @throws DialogueException if the variable resolution fails
	 */
	public void fillArguments (HashMap<String,dFormula> arguments) throws DialogueException {
		dFormula contentCopy;
		contentCopy = FormulaUtils.copy(initContent);
		instantiatedContent = fillArguments (contentCopy, arguments);
	}

	
	
	/**
	 * Returns a logical representation of the (instantiated) intention
	 */
	public dFormula asFormula() {
		return instantiatedContent;
	}
	
	
	/**
	 * Returns the status of the intention
	 * @return
	 */
	public int getStatus() {
		return status;
	}
	

	/**
	 * Set the status of the intention to be one of the allowed types
	 * @param status
	 */
	public void setStatus (int status) {
		if (status == COMMUNICATIVE || status == ATTRIBUTED || status == PRIVATE) {
			this.status = status;
		}
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
	private dFormula fillArguments (dFormula formula, HashMap<String,dFormula> arguments) 
	throws DialogueException {
		
		debug("filling action arguments");
		debug("number of arguments provided: " + arguments.size());

		if (formula instanceof ComplexFormula) {
			for (dFormula subform : ((ComplexFormula)formula).forms) {
				subform = fillArguments(subform, arguments);
			}
		}
		else if (formula instanceof ModalFormula) {
			((ModalFormula)formula).form = fillArguments(((ModalFormula)formula).form, arguments);
		}
		
		else if (formula instanceof UnderspecifiedFormula) {
			debug("reached underspec");
			if (arguments.containsKey(""+((UnderspecifiedFormula)formula).arglabel)) {
				formula = arguments.get(""+((UnderspecifiedFormula)formula).arglabel);
			}
		}

		if (FormulaUtils.getString(formula).contains("%")) {
			throw new DialogueException("ERROR, action could not be instantiated");
		}
		return formula;
	}
	
	


	
	
	/**
	 * Make a copy of the policy action
	 * 
	 * @return
	 * @throws DialogueException
	 */
	public IntentionAction copy () throws DialogueException {
		IntentionAction copy = new IntentionAction(id, FormulaUtils.copy(initContent));
		copy.setStatus(status);
		return copy;
	}
	
	
	

	/**
	 * Returns true if the formula contains underspecified subformulae,
	 * false otherwise
	 * 
	 * @return true if partly or fully underspecified, false otherwise
	 */
	public boolean isUnderspecified () {
		return (getUnderspecifiedArguments().size() > 0);
	}
	

	/**
	 * Returns the underspecified subformulae contained in the node
	 * 
	 * @return a collection of underspecified subformulae
	 */
	public Collection<String> getUnderspecifiedArguments () {
		return PolicyUtils.getUnderspecifiedArguments(initContent);
	}
	
	
	
	/**
	 * Returns a string representation of the action
	 */
	@Override
	public String toString() {
		debug("Action ID: " + id);

			String str = "";
			if (status == COMMUNICATIVE) {
				str += "CI";
			}
			else if (status == PRIVATE) {
				str += "PI";
			}
			else if (status == ATTRIBUTED) {
				str += "AI";
			}
			
			str += "[" + FormulaUtils.getString(instantiatedContent) + "]";
			return str;
	}
	

	
	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[intentionaction] " + s);
		}
	}
	
	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[intentionaction] " + s);
		}
	}


}
