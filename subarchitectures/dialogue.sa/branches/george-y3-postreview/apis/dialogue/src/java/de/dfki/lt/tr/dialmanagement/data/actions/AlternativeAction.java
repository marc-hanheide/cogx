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
import java.util.LinkedList;
import java.util.List;
import java.util.Random;

import de.dfki.lt.tr.beliefs.slice.logicalcontent.BinaryOp;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ComplexFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;


/**
 * Action defined as a (disjunctive) choise between a set of alternative
 * actions 
 * 
 * @author Pierre Lison (plison@ifi.uio.no)
 * @version 22/12/1010
 *
 */
public class AlternativeAction extends AbstractAction {

	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = false;
	
	// the set of alternative actions
	List<AbstractAction> alternativeActions;
	
	
	/**
	 * Builds a new AlternativeAction from an identifier and a list of actions
	 * 
	 * @param id the identifier
	 * @param alternativeActions the list of alternative actions
	 */
	public AlternativeAction(String id, List<AbstractAction> alternativeActions) {
		super(id);
		this.alternativeActions = alternativeActions;
	}

	
	/**
	 * Returns a representation of the action as a formula (in this case, 
	 * it is a disjunction of the formulae for each action)
	 * 
	 * @return the disjunctive formula
	 */
	@Override
	public dFormula asFormula() {
		List<dFormula> subFormulae = new LinkedList<dFormula>();
		for (AbstractAction altAction : alternativeActions) {
			subFormulae.add(altAction.asFormula());
		}
		return new ComplexFormula(0,subFormulae, BinaryOp.disj);
	}

	
	/**
	 * Fills the underspecified arguments in each action contained
	 * in the set
	 */
	@Override
	public void fillArguments(HashMap<String, dFormula> arguments)
			throws DialogueException {
		for (AbstractAction altAction : alternativeActions) {
			altAction.fillArguments(arguments);
		}
	}

	/**
	 * Returns the set of underspecified arguments contained in the
	 * action list
	 */
	@Override
	public Collection<String> getUnderspecifiedArguments() {
		List<String> arguments = new LinkedList<String>();
		for (AbstractAction altAction : alternativeActions) {
			arguments.addAll(altAction.getUnderspecifiedArguments());
		}
		return arguments;
	}

	/**
	 * Returns true if at least one of the contained action is 
	 * underspecified, and false otherwise
	 */
	@Override
	public boolean isUnderspecified() {
		return (getUnderspecifiedArguments().size() > 0);
	}
	
	
	/**
	 * Randomly select one of the action contained in the
	 * list of actions
	 * 
	 * @return one of the contained action (randomly selected)
	 */
	public AbstractAction selectRandomAction() {
		int selection = (new Random()).nextInt(alternativeActions.size());
		return alternativeActions.get(selection);
	}
	
	
	/**
	 * Returns a string representation of the action (as a list)
	 */
	@Override
	public String toString() {
		return alternativeActions.toString();
	}
	
	
	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[alternativeaction] " + s);
		}
	}
	
	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[alternativeaction] " + s);
		}
	}


}
