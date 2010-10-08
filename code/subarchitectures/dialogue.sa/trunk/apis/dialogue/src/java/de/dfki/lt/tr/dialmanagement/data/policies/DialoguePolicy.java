
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
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Vector;

import de.dfki.lt.tr.dialmanagement.arch.DialogueException;

/**
 * Representation of a dialogue policy as a finite-state controller, made of 
 * action nodes and observation edges between them
 * 
 * TODO: refactor to have a policy also well-formed during the incremental construction?
 * TODO: check whether the node, edge, action and observation identifiers are unique
 * 
 * @author Pierre Lison (plison@dfki.de)
 * @version 8/10/2010
 */ 
public class DialoguePolicy {

	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = true;
	
	// set of action nodes
	private HashMap<String,PolicyNode> nodes;
	
	private HashMap<String,PolicyEdge> edges;
	
	// identifier for the initial node
	private String initNode;
	
	// identifier for the final nodes
	private Collection<String> finalNodes;
	
	
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
		edges = new HashMap<String,PolicyEdge>();
		
		for (PolicyNode n: nodes) {
			this.nodes.put(n.getId(), n);
		}
		
		finalNodes = new Vector<String>();
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
		edges.put(edge.getId(), edge);
		debug("Adding outgoing edge : " + edge.toString() + " from " + curPolicyNode.getId() + " to " + nextPolicyNode.getId());
	}
	
	
	/**
	 * Adding a new node in the policy
	 * 
	 * @param node the node to add
	 */
	public void addNode (PolicyNode node) {
		if (node != null) {
			nodes.put(node.getId(), node);
		}
	}
	
	
	/**
	 * Set node as being initial
	 * @param node
	 */
	public void setNodeAsInitial(PolicyNode node) {
		initNode = node.getId();
		node.setAsInitialNode();
	}
	
	
	/**
	 * Set node as being final
	 * @param node
	 */
	public void setNodeAsFinal(PolicyNode node) {
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
	 * Returns the set of edges in the policy
	 * 
	 * @return the edges collection
	 */
	public Collection<PolicyEdge> getEdges () {
		return edges.values();
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
		
	/**
	 * Returns true if the node is a final node, else (or if the node does not
	 * exist in the policy) false
	 * 
	 * @param node the node to check
	 * @return true if node is final, false otherwise
	 */
	public boolean isFinalNode(PolicyNode node) {
		return finalNodes.contains(node.getId());
	}
	
	
	/**
	 * Returns true if the node is the initial node, else (or if the node does not
	 * exist in the policy) false
	 * 
	 * @param node the node to check
	 * @return true if node is final, false otherwise
	 */
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
	 * Returns the collection of nodes
	 * @return
	 */
	public Collection<PolicyNode> getNodes () {
		return nodes.values();
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
		result = result.substring(0, result.length() -2) + "} \n";
		
		result += "Edges = \n";
		for (String nodeId : nodes.keySet()) {
			PolicyNode node = nodes.get(nodeId);		
			for (PolicyEdge edge: node.getAllOutgoingObservations()) {			
				result += edge.getSourceNode().getId() + " -- " + edge.getObservation().toString() + 
				" --> " + edge.getTargetNode().getId() + "\n";			
			}
		}
		
		result += "Actions = \n";
		for (String nodeId : nodes.keySet()) {
			result += nodeId + " : " + nodes.get(nodeId).getAction().toString() + " \n";
		}
		
		return result;
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
		
		for (PolicyEdge e : edges.values()) {
			
			if (e.getObservation() == null) {
				throw new DialogueException("Warning: no observation specified in edge");
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
			PolicyNode nextNode = e.getTargetNode();
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
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[dialpolicy] " + s);
		}
	}
	
	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[dialpolicy] " + s);
		}
	}
}
