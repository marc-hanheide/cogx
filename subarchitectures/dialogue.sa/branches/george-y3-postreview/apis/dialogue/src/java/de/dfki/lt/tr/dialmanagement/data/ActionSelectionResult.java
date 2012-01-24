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

import java.util.LinkedList;
import java.util.List;

import de.dfki.lt.tr.dialmanagement.data.actions.AbstractAction;
 
/**
 * Result of the action selection process, encoded as a (possibly empty) set of 
 * policy actions
 * 
 * @author Pierre Lison (plison@ifi.uio.no)
 * @version 20/12/2012
 */
public class ActionSelectionResult {

	// the list of actions to execute
	List<AbstractAction> actions = new LinkedList<AbstractAction>();

	/**
	 * Create a result with an empty set of actions
	 * @return
	 */
	public static ActionSelectionResult createVoidResult() {
		return new ActionSelectionResult();
	}
	

	/**
	 * Add another action to the list of actions
	 * @param action
	 */
	public void addAction (AbstractAction action) {
		actions.add(action);
	}
	
	/**
	 * Returns the set of actions in the result
	 * @return
	 */
	public List<AbstractAction> getActions () {
		return actions;
	}

	/**
	 * Returns true if the list of actions is empty, false otherwise
	 * @return
	 */
	public boolean isVoid() {
		return (actions.size() == 0);
	}
	
	
	/**
	 * Returns a string representation of the action selection result
	 */
	@Override
	public String toString() {
		String str = "";
		for (AbstractAction action : actions) {
			str += action.toString() + "\n";
		}
		if (actions.size() == 0) {
			str = "voidResult";
		}
		return str;
	} 
	
}
