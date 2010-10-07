
// =================================================================                                                        
// Copyright (C) 2009-2011 Pierre Lison (pierre.lison@dfki.de)                                                                
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

import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Vector;

import de.dfki.lt.tr.dialmanagement.arch.DialogueException;

/**
 * An action node in a dialogue policy
 * 
 * @author Pierre Lison (plison@dfki.de)
 * @version 14/06/2010
 */
public class PolicyNode {

	

	public static boolean LOGGING = true;

	public static boolean DEBUG = true;
	
	// the unique identifier for the node
	private String id;


	// the collection of fully specified outgoing observation edges
	private Vector<PolicyEdge> outgoingEdges;

	// the (dialogue) action encapsulated by the node
	private PolicyAction action;

	// is it an initial (starting) node
	private boolean isInitial = false;

	// is it a final node
	private boolean isFinal = false;



	/**
	 * Constructs a new action node, given an action
	 * 
	 * @param identifier the unique identifier for the node
	 * @param action the dialogue action
	 * @throws DialogueException if the action is a null value
	 */
	public PolicyNode(String identifier, PolicyAction action) throws DialogueException {
		id = identifier;
		outgoingEdges = new Vector<PolicyEdge>();
		if (action != null) {
			this.action = action;
		}
		else {
			throw new DialogueException("ERROR, action is a null value");
		}
	}


	/**
	 * Constructs a new action node, given an action and a set of incoming and 
	 * outgoing observation edges
	 * 
	 * @param identifier the unique identifier for the node
	 * @param action the dialogue action
	 * @param outgoingEdges the set of outgoing observation edges
	 * @throws DialogueException if one of the parameters is a null value
	 */
	public PolicyNode(String identifier, PolicyAction action, Collection<PolicyEdge> outgoingEdges) throws DialogueException {

		id = identifier;

		if (action != null && outgoingEdges != null) {

			this.outgoingEdges = new Vector<PolicyEdge>();

			addOutgoingEdges(outgoingEdges);

			this.action = action;
		}
		else {
			throw new DialogueException("ERROR, action is a null value");
		}
	}


	/**
	 * Adds a collection of outgoing observation edges to the node
	 * @param edges the collection of edges to add
	 * @throws DialogueException if null value, or if one of the observation is already
	 * used in the node
	 */
	public void addOutgoingEdges(Collection<PolicyEdge> edges) throws DialogueException {
		for (PolicyEdge edge : edges) {
			addOutgoingEdge(edge);
		}
	}

	/**
	 * Adds an outgoing edge to the node
	 * 
	 * @param edge the observation edge to add
	 * @throws DialogueException if null value, or if the observation is already
	 * used in the node
	 */
	public void addOutgoingEdge (PolicyEdge edge) throws DialogueException {
		
		if (edge==null || edge.getObservation() == null) {
			throw new DialogueException("ERROR, edge is a null value");
		}
		
	/**	for (PolicyEdge existingEdge : outgoingEdges)  {
			if (edge.getObservation().equals(existingEdge.getObservation()) && 
					!existingEdge.getObservation().isUnderspecified() && 
					!edge.getObservation().isUnderspecified()) {
				throw new DialogueException("ERROR, observation " + edge.getObservation().toString() +  "already in outgoing edges");
			}
		}  */
			
		outgoingEdges.add(edge);
	}




	/**
	 * Remove one outgoing edge to the node (if no outgoing edge corresponds to the observation,
	 * leave as such)
	 * 
	 * @param obs the observation
	 * @throws DialogueException if null value
	 */
	public void removeOutgoingEdge (PolicyEdge obs) throws DialogueException {

		if (obs != null) {
			outgoingEdges.remove(obs);
		}
		else {
			throw new DialogueException("ERROR, obs is a null value");
		}
	}


	/**
	 * Sets node as being initial
	 * 
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
	 * 
	 * @throws DialogueException if the node contains one or more outgoing edges
	 */
	public void setAsFinalNode () throws DialogueException {
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
	 * True if observation contained in fully realised outgoing edges, false otherwise
	 * 
	 * @param obs the observation
	 * @return true if observation found, false otherwise
	 */
	public boolean hasOutgoingObservation (Observation obs) {
		
		for (PolicyEdge edge: outgoingEdges) {
			
			if (obs.equals(edge.getObservation())) {
				return true;
			}
		}
		return false;
	}
	


	/**
	 * Returns the outgoing observation edge, identified by its observation.  If none found, throws
	 * an exception
	 * 
	 * @param obs the observation
	 * @return the outgoing observation edge
	 * @throws DialogueException if no edge corresponds to the observation
	 */
	public Collection<PolicyEdge> getMatchingEdges (Observation obs) throws DialogueException {
	
		Vector<PolicyEdge> edges = new Vector<PolicyEdge>();
		for (PolicyEdge edge: outgoingEdges) {
			if (edge.matchesWithObservation(obs)) {
				edges.add(edge);
			}
		}
		return edges;
	}


	/**
	 * Returns all outgoing observation edges
	 * @return all outgoing observation edges
	 */
	public Collection<PolicyEdge> getAllOutgoingObservations () {
		return outgoingEdges;
	}

	

	
	/**
	 * Return the action encapsulated in the node
	 * @return the action
	 */
	public PolicyAction getAction() {
		return action;
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
			System.out.println("[dialmanager] " + s);
		}
	}

	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[dialmanager] " + s);
		}
	}
}
