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

import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.utils.FormulaUtils;

/**
 * Abstract class providing generic functionalities for all kinds
 * of actions
 * 
 * @author Pierre Lison (plison@ifi.uio.no)
 * @version 21/12/2010
 *
 */
public abstract class AbstractAction {
	
	
	// the identifier of the action
	protected String id;
 
	
	/**
	 * Constructs a (void) action with a given identifier
	 * 
	 * @param id the identifier
	 */
	public AbstractAction(String id) {
		this.id = id;
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
	 * Returns true if the action is underspecified, false otherwise
	 * 
	 * (abstract class to be provided by the concrete class)
	 * 
	 * @return true is the action is underspecified, false otherwise
	 * 
	 */
	public abstract boolean isUnderspecified();

	/**
	 * Returns the collection of underspecified variables in the
	 * action (if any)
	 *
	 * (abstract class to be provided by the concrete class)
	 * 
	 * @return the list of underspecified variables
	 * 
	 */
	public abstract Collection<String> getUnderspecifiedArguments () ;

	
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
	public abstract void fillArguments (HashMap<String,dFormula> arguments)  throws DialogueException ;
	
	

	/**
	 * Returns a representation of the action as a logical formula
	 * 
	 * @return the formula
	 */
	public abstract dFormula asFormula(); 
		
	
	
	/**
	 * Check whether two policy actions are similar or not
	 * 
	 */
	@Override
	public boolean equals(Object o) {	
		if (!o.getClass().equals(this.getClass())) {
			return false;
		}
		else {
			return FormulaUtils.subsumes(((AbstractAction)o).asFormula(), this.asFormula());
		}
	}
	
	
}
