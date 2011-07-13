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

import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;


/**
 * Representation of a motor action, defined by a simple symbol
 * 
 * @author Pierre Lison (plison@ifi.uio.no)
 *
 */
public class MotorAction extends AbstractAction {


	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = false;
	
	// the symbol of the motor action
	String actionSymbol;
	
	
	/**
	 * Creates a new motor action, defined by an identifier and an 
	 * action symbol
	 * @param id the identifier
	 * @param actionSymbol the action symbol
	 */
	public MotorAction(String id, String actionSymbol) {
		super(id);
		this.actionSymbol = actionSymbol;
	}

	/**
	 * Returns the representation of the action as a logical formula
	 * (in this case, an elementary formula containing the action
	 * symbol)
	 * 
	 * @return the formula
	 */
	@Override
	public dFormula asFormula() {
		return new ElementaryFormula(0, actionSymbol);
	}

	/**
	 * Does nothing
	 */
	@Override
	public void fillArguments(HashMap<String, dFormula> arguments)  throws DialogueException  {	}

	
	/**
	 * Does nothing
	 */
	@Override
	public boolean isUnderspecified() {	return false; }

	
	/**
	 * Returns an empty set
	 */
	@Override
	public Collection<String> getUnderspecifiedArguments() {
		return new LinkedList<String>();
	}
	
	/**
	 * Returns the action symbol
	 */
	@Override
	public String toString() {
		return actionSymbol;
	}

	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[motoraction] " + s);
		}
	}
	
	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[motoraction] " + s);
		}
	}

}
