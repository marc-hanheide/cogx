
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
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Vector;

import de.dfki.lt.tr.dialmanagement.arch.DialogueException;

/**
 * Representation of a dialogue policy as a finite-state controller, constituted of 
 * action nodes and observation edges between them
 * 
 * @author Pierre Lison (plison@dfki.de)
 * @version 13/06/2010
 */ 
public class DialoguePolicy {

	// logging mode
	public static boolean LOGGING = true;

	// debugging mode
	public static boolean DEBUG = true;
	
	// set of action nodes
	private HashMap<String,PolicyNode> nodes;
	
	// identifier for the initial node
	private String initNode;
	
	// identifier for the final nodes
	private Collection<String> finalNodes;

	
	public static final int INTENTION = 0;
	public static final int EVENT = 1;
	
	
	/**
	 * Create a new dialogue policy with empty nodes and edges
	 */
	public DialoguePolicy() {
		this (new LinkedList<PolicyNode>());
	}
	
	/**
	 * Create a new dialogue policy with a set of nodes
	 * @param nodes the nodes
	 */
	public DialoguePolicy(Collection<PolicyNode> nodes) {
		
		this.nodes = new HashMap<String,PolicyNode>();
		
		for (PolicyNode n: nodes) {
			this.nodes.put(n.getId(), n);
		}
		
		finalNodes = new Vector<String>();

	}
	
	
	
	/**
	 * Checks that the policy is well-formed
	 * @throws DialogueException if the policy is not well-formed for some reason
	 * 
	 */
	public void ensureWellFormedPolicy () throws DialogueException {
				
		if (nodes == null  || initNode == null || finalNodes == null ) {
			throw new DialogueException("Warning: null values");
		}
		
		if (nodes.size() <= 0) {
			throw new DialogueException("Warning: no action");
		}
		
		if (!nodes.containsKey(initNode)) {
			throw new DialogueException("Warning: initNode not contained in nodes");
		}
		
		if (!nodes.get(initNode).isInitialNode()) {
			throw new DialogueException("Warning: initial node not explicitly set as such");
		}
		
		for (String f : finalNodes) {
			if (!nodes.containsKey(f)) {
				throw new DialogueException("Warning: final node not contained in nodes");
			}
			else if (!nodes.get(f).isFinalNode()) {
				throw new DialogueException("Warning: final node not explicitly set as such");
			}
		}
		
		
		if (finalNodes.size() <= 0) {
			throw new DialogueException("Warning: no final node");
		}
		
		for (PolicyNode a : nodes.values()) {
			
			if (a.getAction() == null) {
				throw new DialogueException("Warning: no action specified in node");
			}
			
		
		if (!isNodeLeadingToFinal(initNode, 0)) {
			throw new DialogueException("Warning: initial node not leading to any final node");
		}
		}
	}


	
	/**
	 * Returns true if the given node is leading to any final node, and false
	 * otherwise 
	 * 
	 * @param nodeId the identifier of the action node to test
	 * @return
	 */
	public boolean isNodeLeadingToFinal(String nodeId, int count) {
		
		if (count == 200) {
			return false;
		}
		else {
			count++;
		}
	
		if (finalNodes.contains(nodeId)) {
			return true;
		}
		
		LinkedList<PolicyEdge> observations = 
			new LinkedList<PolicyEdge>(nodes.get(nodeId).getAllOutgoingObservations());
		Collections.shuffle(observations);
		
		for (PolicyEdge e: observations) {
			PolicyNode nextNode = e.getOutgoingAction();
			if (finalNodes.contains(nextNode.getId())) {
				return true;
			}
			else if (isNodeLeadingToFinal(nextNode.getId(), count)) {
				return true;
			}
		}
		return false;
	}
	
	/**
	 * Add an edge between two existing action nodes 
	 * 
	 * @param edgeId the edge identifier
	 * @param obs the observation to create and set between the two nodes
	 * @param curPolicyNode the incoming action node
	 * @param nextPolicyNode the outgoing action node
	 * 
	 * @throws DialogueException if one of the two nodes does not exist
	 */
	public void addEdge (PolicyEdge edge, PolicyNode curPolicyNode, PolicyNode nextPolicyNode) 
		throws DialogueException {
		
		if (!nodes.containsKey(curPolicyNode.getId())) {
			throw new DialogueException("ERROR: curAction not present in nodes set");
		}
		
		if (!nodes.containsKey(nextPolicyNode.getId())) {
			throw new DialogueException("ERROR: nextAction not present in nodes set");
		}
		
		curPolicyNode.addOutgoingEdge(edge);
		debug("Adding outgoing edge : " + edge.toString() + " from " + curPolicyNode.getId() + " to " + nextPolicyNode.getId());
	}
	
	
	/**
	 * Add a new action node to the policy
	 * 
	 * @param nextNodeId the node identifier
	 * @param action the new action
	 * @return the node encapsulating the action
	 * @throws DialogueException 
	 */
	public PolicyNode addNode (String nodeId, PolicyAction action) throws DialogueException {
		PolicyNode node = new PolicyNode(nodeId, action);
		nodes.put(nodeId,node);
		return node;
	}
	
	
	/**
	 * Set node as being initial
	 * @param node
	 * @throws DialogueException
	 */
	public void setNodeAsInitial(PolicyNode node) throws DialogueException {
		initNode = node.getId();
		node.setAsInitialNode();
	}
	
	/**
	 * Set node as being final
	 * @param node
	 * @throws DialogueException
	 */
	public void setNodeAsFinal(PolicyNode node) throws DialogueException {
		finalNodes.add(node.getId());
		node.setAsFinalNode();
	}
	
	
	/**
	 * Returns true if one node in the policy has the identifier nodeId, false otherwise
	 * 
	 * @param nodeId the identifier to check
	 * @return
	 */
	public boolean hasNode(String nodeId) {
		return nodes.containsKey(nodeId);
	}
	
		
	
	/**
	 * Returns the node anchored by the identifier (if no node exists, returns null)
	 * 
	 * @param nodeId
	 * @return
	 */
	public PolicyNode getNode(String nodeId) {
		return nodes.get(nodeId);
	}
		
	
	public boolean isFinalNode(PolicyNode node) {
		return finalNodes.contains(node.getId());
	}
	
	public boolean isInitNode(PolicyNode node) {
		return initNode.equals(node.getId());
	}
	 
	/**
	 * Returns the initial node
	 * @return the initial node
	 */
	public PolicyNode getInitNode() {
		return nodes.get(initNode);
	}
	
	/**
	 * Returns a textual representation of the dialogue policy
	 */
	@Override
	public String toString() {
		String result = "Nodes = {";
		
		for (String nodeId : nodes.keySet()) {
			result += nodeId + ", ";
		}
		result = result.substring(0, result.length() -2) + "} \n\n";
		
		result += "Edges: \n";
		for (String nodeId : nodes.keySet()) {
			PolicyNode node = nodes.get(nodeId);		
			for (PolicyEdge edge: node.getAllOutgoingObservations()) {			
				result += edge.getIncomingAction().getId() + " -- " + edge.getObservation().toString() + 
				" --> " + edge.getOutgoingAction().getId() + "\n";			
			}
		}
		return result;
	}
	
	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[Dialogue policy] " + s);
		}
	}
	
	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[Dialogue policy] " + s);
		}
	}
}
