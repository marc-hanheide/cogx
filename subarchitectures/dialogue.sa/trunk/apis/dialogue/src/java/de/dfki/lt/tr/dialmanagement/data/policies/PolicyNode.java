
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

import java.util.Collection;
import java.util.Vector;

import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.data.Observation;

/**
 * An node included in a dialogue policy.  Its is made of an identifier, 
 * a collection of outgoing edge, and an associated policy action. It 
 * can also be marked as being an initial or final node.
 * 
 * @author Pierre Lison (plison@dfki.de)
 * @version 08/10/2010
 */
public class PolicyNode {

	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = false;
	
	// the unique identifier for the node
	private String id;
	
	// the collection of outgoing edges
	private Vector<PolicyEdge> outgoingEdges;

	// the action encapsulated by the node
	private PolicyAction action;

	// is it an initial (starting) node or not
	private boolean isInitial = false;

	// is it a final node or not
	private boolean isFinal = false;
 
	/**
	 * Constructs a new node, given a policy action
	 * 
	 * @param id the unique identifier for the node
	 * @param action the action
	 * @throws DialogueException if the action is a null value
	 */
	public PolicyNode(String id, PolicyAction action) {
		this.id = id;
		outgoingEdges = new Vector<PolicyEdge>();
		this.action = action;
	}
	
	/**
	 * Constructs a new node given an identifier
	 * 
	 * @param id the unique identifier for the node
	 */
	public PolicyNode(String id) {
		this(id, null);
	}
	
	/**
	 * Sets the policy action for the node
	 * 
	 * @param action the action
	 */
	public void setPolicyAction (PolicyAction action) {
		this.action = action;
	}
	
	/**
	 * Returns the policy action for the node
	 * @return the action
	 */
	public PolicyAction getPolicyAction () {
		if (action != null) {
			return action;
		}
		else {
			return PolicyAction.createVoidAction();
		}
	}
	

	/**
	 * Adds a collection of outgoing edges to the node
	 * 
	 * @param edges the collection of edges to add
	 */
	public void addAllOutgoingEdges(Collection<PolicyEdge> edges) {
		for (PolicyEdge edge : edges) {
			addOutgoingEdge(edge);
		}
	}

	/**
	 * Adds an outgoing edge to the node
	 * 
	 * @param edge the observation edge to add
	 */
	public void addOutgoingEdge (PolicyEdge edge) {
		outgoingEdges.add(edge);
	}




	/**
	 * Remove one outgoing edge to the node, if it exists
	 * 
	 * @param edge the edge to remove
	 * @throws DialogueException if null value
	 */
	public void removeOutgoingEdge (PolicyEdge edge) {
			outgoingEdges.remove(edge);
	}


	/**
	 * Sets node as being initial
	 */
	public void setAsInitialNode () {
		isInitial = true;
	}

	
	/**
	 * Returns true if node is initial, false otherwise
	 * @return is initial node
	 */
	public boolean isInitialNode () {
		return isInitial;
	}


	/**
	 * Sets node as being final
	 */
	public void setAsFinalNode () {
		isFinal = true;
	}

	
	/**
	 * Returns true if node is final, false otherwise
	 * @return is final node
	 */
	public boolean isFinalNode () {
		return isFinal;
	}

	/**
	 * Returns the set of edges matching the given observation
	 * 
	 * @param obs the observation to match
	 * @return the collection of edges which match
	 */
	public Collection<PolicyEdge> getMatchingEdges (Observation obs) {
	
		Vector<PolicyEdge> edges = new Vector<PolicyEdge>();
		for (PolicyEdge edge: outgoingEdges) {
			debug("is edge " + edge.toString() + " matching with observation " + obs + "? " + edge.matchesObservation(obs));
			if (edge.matchesObservation(obs)) {
				edges.add(edge);
			}
		}
		return edges;
	}


	/**
	 * Returns all outgoing edges
	 * @return all outgoing edges
	 */
	public Collection<PolicyEdge> getAllOutgoingEdges () {
		return outgoingEdges;
	}

	

	
	/**
	 * Return the action encapsulated in the node
	 * @return the action
	 */
	public PolicyAction getAction() {
		if (action!=null) {
			return action;
		}
		else return PolicyAction.createVoidAction();
	}

	/**
	 * Returns the node identifier
	 * @return the identifier, as a string
	 */
	public String getId() {
		return id;
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
