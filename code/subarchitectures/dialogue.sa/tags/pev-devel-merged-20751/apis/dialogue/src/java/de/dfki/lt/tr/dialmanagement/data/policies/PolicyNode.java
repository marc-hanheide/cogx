
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

package de.dfki.lt.tr.dialmanagement.data.policies;

import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;

import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.data.actions.AbstractAction;
import de.dfki.lt.tr.dialmanagement.data.actions.EmptyAction;
 
/**
 * An node included in a dialogue policy.  Its is made of an identifier and a 
 * (possibly empty) list of associated actions.
 * 
 * The node can also be marked as being an initial or final node.
 * 
 * @author Pierre Lison (plison@ifi.uio.no)
 * @version 20/12/2010
 */
public class PolicyNode {
 
	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = false;
	
	// the unique identifier for the node
	private String id;
	
	// the set of actions encapsulated by the node
	private HashMap<String,AbstractAction> actions;
 	
	
	
	// ==============================================================
	// NODE CONSTRUCTION METHODS
	// ==============================================================

	
	/**
	 * Constructs a new node, given a policy action
	 * 
	 * @param id the unique identifier for the node
	 * @param action the action
	 * @throws DialogueException if the action is a null value
	 */
	public PolicyNode(String id, List<AbstractAction> actions) {
		this.id = id;
		this.actions = new HashMap<String,AbstractAction>();
		for (AbstractAction action: actions) {
			this.actions.put(action.getId(), action);
		}
	}
	
	/**
	 * Constructs a new node given an identifier
	 * 
	 * @param id the unique identifier for the node
	 */
	public PolicyNode(String id) {
		this(id, new LinkedList<AbstractAction>());
	}
	
	
	/**
	 * Remove a policy action for the node
	 * 
	 * @param action the action
	 */
	public void removeAction (AbstractAction action) {
		actions.remove(action.getId());
	}
	
	/**
	 * Add a policy action for the node
	 * 
	 * @param action the action
	 */
	public void addAction (AbstractAction action) {
		actions.put(action.getId(), action);
	}
	
	
	/**
	 * Add an empty action to the node (which will be 
	 * replaced afterwards)
	 * 
	 * @param actionId the identifier of the empty action
	 */
	public void addEmptyAction (String actionId) {
		actions.put(actionId, new EmptyAction(actionId));
	}
		 
	
	// ==============================================================
	// GETTER METHODS
	// ==============================================================

	
	/**
	 * Return the actions encapsulated in the node
	 * 
	 * @return the list of actions
	 */
	public List<AbstractAction> getActions() {
			return new LinkedList<AbstractAction>(actions.values());
	}
	
	
	/**
	 * Returns the set of empty actions currently contained
	 * int the node
	 * 
	 * @return the list of actions which are currently empty
	 */
	public Collection<String> getEmptyActions() {
		
		Collection<String> emptyActions = new LinkedList<String>();
		
		for (String actionId : actions.keySet()) {
			if (actions.get(actionId) instanceof EmptyAction) {
				emptyActions.add(actionId);
			}
		}
		
		return emptyActions;
	}

	
	/**
	 * Returns the node identifier
	 * @return the identifier, as a string
	 */
	public String getId() {
		return id;
	}

	
	
	// ==============================================================
	// UTILITY METHODS
	// ==============================================================

	/**
	 * Make a copy of the policy node
	 * 
	 * NOTE: the actions themselves are not copied
	 * 
	 * @return
	 * @throws DialogueException
	 */
	public PolicyNode copy() throws DialogueException {
		List<AbstractAction> newActionsList = new LinkedList<AbstractAction>();
		for (AbstractAction oldAction : actions.values()) {
			newActionsList.add(oldAction);
		}
		PolicyNode copy = new PolicyNode(id, newActionsList);
		return copy;
	}


	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[policynode] " + s);
		}
	}

	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[policynode] " + s);
		}
	}
}
