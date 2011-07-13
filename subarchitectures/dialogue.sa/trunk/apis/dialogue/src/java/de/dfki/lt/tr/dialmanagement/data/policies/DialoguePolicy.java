
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
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;
import java.util.Vector;

import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.data.DialogueState;
import de.dfki.lt.tr.dialmanagement.data.Observation;
import de.dfki.lt.tr.dialmanagement.data.conditions.AbstractCondition;
import org.apache.log4j.Logger;

/**
 * Representation of a dialogue policy as a finite-state controller, made of 
 * action nodes and observation edges between them
 * 
 * @author Pierre Lison (plison@ifi.uio.no)
 * @version 19/11/2010
 */ 
public class DialoguePolicy {

	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = false;

	private Logger logger = null;

	private static int policyCounter = 0;

	private String policyId;

	// hashmap of policy nodes
	private HashMap<String,PolicyNode> nodes;

	// hashmap of policy edges
	private HashMap<String,PolicyEdge> edges;

	// identifier for the initial node
	private String initNodeId = "";

	// set of identifiers for the final nodes
	private Collection<String> finalNodeIds;

	// maximum number of steps to perform between the initial and final
	// nodes when checking if the policy is well-formed
	public static final int MAX_STEPS = 400;




	// ==============================================================
	// POLICY CONSTRUCTION METHODS
	// ==============================================================



	/**
	 * Create a new dialogue policy, with empty sets of nodes and edges
	 */
	public DialoguePolicy() {
		this (null);
	}

	public DialoguePolicy(Logger _logger) {
		this (new LinkedList<PolicyNode>(), _logger);
	}

	/**
	 * Create a new dialogue policy with a set of nodes
	 * 
	 * @param nodes the nodes
	 */
	public DialoguePolicy(Collection<PolicyNode> nodes, Logger _logger) {
		logger = _logger;

		this.nodes = new HashMap<String,PolicyNode>();
		edges = new HashMap<String,PolicyEdge>();

		for (PolicyNode n: nodes) {
			this.nodes.put(n.getId(), n);
		}

		finalNodeIds = new Vector<String>();

		policyCounter++;
		policyId = "policy" + policyCounter;
	}

	public void setLogger(Logger _logger) {
		logger = _logger;
	}

	/**
	 * Returns the policy identifier
	 * @return
	 */
	public String getPolicyId () {
		return policyId;
	}

	/**
	 * Sets the policy identifier to a particular value
	 * 
	 * @param s the identifier of the policy
	 */
	public void setPolicyId(String s) {
		policyId = s;
	}


	/**
	 * Adding a new node in the policy
	 * 
	 * @param node the node to add
	 * @throws DialogueException if nodeID already exists
	 */
	public void addNode (PolicyNode node) throws DialogueException {
		if (node != null) {
			if (nodes.containsKey(node.getId())) {
				throw new DialogueException("ERROR: node with same ID already exists: " + node.getId());
			}
			nodes.put(node.getId(), node);
		}
		debug("Adding the node " + node.getId());
	}



	/**
	 * Add a collection of nodes to the policy
	 * @param nodes
	 * @throws DialogueException
	 */
	public void addNodes(Collection<PolicyNode> nodes) throws DialogueException {
		for (PolicyNode node: nodes) {
			addNode(node);
		}
	}


	/**
	 * Add a new edge to the policy.  
	 * 
	 * <p> If a node with the same ID already exists, modify the
	 * identifier by adding a random number at the end of it
	 * 
	 * @param edge the edge to add
	 */
	public void addEdge (PolicyEdge edge) {

		if (!edges.containsKey(edge.getId())) {
			edges.put(edge.getId(), edge);
		}
		else {
			String newEdgeId = edge.getId() + (new Random()).nextInt();
			edge.setId(newEdgeId);
			edges.put(newEdgeId, edge);
		}
		debug("Adding outgoing edge : " + edge.toString() + " from " +edge.getSourceNodeId() + " to " + edge.getTargetNodeId());
	}


	/**
	 * Add a collection of policy edges to the policy
	 * @param edges
	 */
	public void addEdges (Collection<PolicyEdge> edges) {
		for (PolicyEdge edge: edges) {
			addEdge(edge);
		}
	}


	/**
	 * Set a policy node as being the initial node of the policy or not.  
	 * 
	 * <p>If another node if already specified as being initial, it is 
	 * replaced by the new one (since only unique node can be initial)
	 * 
	 * @param node the node identifier
	 * @param val whether the node should be set as initial or not
	 */
	public void setNodeAsInitial(String nodeId, boolean val) {

		if (val && nodes.containsKey(nodeId)) {
			initNodeId = nodeId;
		}
		else if (isInitNode(nodeId)) {
			initNodeId = "";
		}
		else {
			log("WARNING: node " + nodeId + " not in policy, could not be set as initial");
		}
	}


	/**
	 * Set a policy node as being a final node of the policy or not.
	 * 
	 * @param nodeId the node identifier
	 * @param val whether the node should be set as final or not
	 */
	public void setNodeAsFinal(String nodeId, boolean val) {

		if (val && nodes.containsKey(nodeId)) {
			finalNodeIds.add(nodeId);
		}
		else if (isFinalNode(nodeId)){
			finalNodeIds.remove(nodeId);
		}
		else {
			log("WARNING: node " + nodeId + " not in policy, could not be set as final");
		}
	}



	// ==============================================================
	// POLICY INSPECTION METHODS
	// ==============================================================



	/**
	 * Returns true if one node in the policy has the 
	 * identifier nodeId, false otherwise
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
	public Collection<PolicyEdge> getAllEdges () {
		return edges.values();
	}


	/**
	 * Returns the edge anchored by the identifier (if no edge exists,
	 * return null)
	 * 
	 * @param edgeId
	 * @return
	 */
	public PolicyEdge getEdge(String edgeId) {
		return edges.get(edgeId);
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
	 * @param node the node identifier to check
	 * @return true if node is final, false otherwise
	 */
	public boolean isFinalNode(String nodeId) {
		return finalNodeIds.contains(nodeId);
	}


	/**
	 * Returns true if the node is the initial node, else (or if the node does not
	 * exist in the policy) false
	 * 
	 * @param nodeId the node identifier to check
	 * @return true if node is final, false otherwise
	 */
	public boolean isInitNode(String nodeId) {
		return initNodeId.equals(nodeId);
	}


	/**
	 * Returns the initial node
	 * @return the initial node
	 */
	public PolicyNode getInitNode() {
		return nodes.get(initNodeId);
	}



	/**
	 * Returns the set of final nodes
	 * 
	 * @return the collection of nodes which are final
	 */
	public Collection<PolicyNode> getFinalNodes() {

		Collection<PolicyNode> finalNodesC = new LinkedList<PolicyNode>();	
		for (String f : finalNodeIds) {
			if (nodes.containsKey(f)) {
				finalNodesC.add(nodes.get(f));
			}
		}

		return finalNodesC;
	}



	/**
	 * Returns the complete collection of nodes
	 * @return
	 */
	public Collection<PolicyNode> getAllNodes () {
		return nodes.values();
	}



	/**
	 * Returns the set of outgoing edges attached to the node identified by nodeId
	 * 
	 * @param nodeId the node identifier
	 * @return the collection of all edges starting from nodeId
	 */
	public List<PolicyEdge> getAllOutgoingEdges (String nodeId) {

		List<PolicyEdge> results = new LinkedList<PolicyEdge>();
		for (PolicyEdge edge : edges.values()) {
			if (edge.getSourceNodeId().equals(nodeId)) {
				results.add(edge);
			}
		}

		return results;
	}


	/**
	 * Returns the set of incoming edges attached to the node identified by nodeId
	 * 
	 * @param nodeId the node identifier
	 * @return the collection of all edges leading to nodeId
	 */
	public List<PolicyEdge> getAllIncomingEdges (String nodeId) {

		List<PolicyEdge> results = new LinkedList<PolicyEdge>();
		for (PolicyEdge edge : edges.values()) {
			if (edge.getTargetNodeId().equals(nodeId)) {
				results.add(edge);
			}
		}

		return results;
	}



	/**
	 * Returns true if the policy contains an edge with the identifier,
	 * false otherwise
	 * 
	 * @param edgeId the identifier
	 * @return
	 */
	public boolean hasEdge (String edgeId) {
		return edges.containsKey(edgeId);
	}


	/**
	 * Returns the target node of the given edge
	 * 
	 * @param edge the edge from which to extract the target
	 * @return the target node
	 * @throws DialogueException if the edge does not exist in the policy, or
	 *         if the node pointed by the edge does not exist
	 */
	public PolicyNode getTargetNode (PolicyEdge edge) throws DialogueException {
		if (edges.containsKey(edge.getId()) && nodes.containsKey(edge.getTargetNodeId())) {
			return nodes.get(edge.getTargetNodeId());
		}
		else {
			throw new DialogueException("ERROR, node " + edge.getTargetNodeId() + " not existing in policy");
		}
	}




	/**
	 * Returns the source node of the given edge
	 * 
	 * @param edge the edge from which to extract the target
	 * @return the target node
	 * @throws DialogueException if the edge does not exist in the policy, or
	 *         if the node pointed by the edge does not exist
	 */
	public PolicyNode getSourceNode (PolicyEdge edge) throws DialogueException {
		if (edges.containsKey(edge.getId()) && nodes.containsKey(edge.getSourceNodeId())) {
			return nodes.get(edge.getSourceNodeId());
		}
		else {
			throw new DialogueException("ERROR, node " + edge.getSourceNodeId() + " not existing in policy");
		}
	}




	/**
	 * Returns the set of matching outgoing edges attached to the node (identified 
	 * by nodeId) and which are matching the given observation. 
	 * 
	 * If preconditions on the information state are also attached to the edges,
	 * they are also verified prior to the computation of matches.
	 * 
	 * @param nodeId the node identifier
	 * @param obs the observation to match
	 * @param dialState the dialogue state
	 * @return the collection of all edges which satisfy the match
	 */
	public List<PolicyEdge> getMatchingEdges (String nodeId, Observation obs, 
			DialogueState dialState) {

		List<PolicyEdge> results = new LinkedList<PolicyEdge>();
		for (PolicyEdge edge : edges.values()) {
			if (edge.getSourceNodeId().equals(nodeId)) {
				if (edge.matchAllConditions(obs, dialState)) {
					results.add(edge);
				}
			}
		}

		return results;
	}


	// ==============================================================
	// METHODS TO COPY AND CONCATENATE POLICIES
	// ==============================================================



	/**
	 * Make a full copy of the dialogue policy
	 * (and of its nodes and edges)
	 * 
	 * @return the new policy
	 * 
	 * @throws DialogueException if the policy is not properly formatted
	 */
	public DialoguePolicy copy() throws DialogueException {

		DialoguePolicy newPolicy = new DialoguePolicy();

		for (PolicyNode node : nodes.values()) {
			PolicyNode nodecopy = node.copy();
			newPolicy.addNode(nodecopy);
		}

		for (PolicyEdge edge: edges.values()) {
			newPolicy.addEdge(edge.copy());
		}

		newPolicy.setNodeAsInitial(initNodeId,true);

		for (String finalNode : finalNodeIds) {
			newPolicy.setNodeAsFinal(finalNode,true);
		}

		return newPolicy;
	}





	/**
	 * Concatenate a new policy at the end of the existing one, and use the bridgeCondition
	 * to create new edges between the two.
	 * 
	 * <p>More precisely, new edges are created between each final node in the current policy
	 * and the initial node in the new policy, using the bridgeCondition.  
	 * 
	 * <p>Note that it means that the nodes specified as final in the current policy
	 * won't be final anymore after the concatenation.
	 * 
	 * @param subPolicy the new policy to concatenate at the end of the existing one
	 * @param bridgeCondition the bri
	 * @throws DialogueException
	 */
	public void concatenatePolicy(DialoguePolicy subPolicy, 
			AbstractCondition bridgeCondition) throws DialogueException {


		// extract the current final nodes
		List<String> intermediaryNodesIds = new LinkedList<String>();
		for (String finalNodeId : finalNodeIds) {
			intermediaryNodesIds.add(finalNodeId);
		}


		// loop on the nodes of the new policy
		for (PolicyNode newNode : subPolicy.getAllNodes()) {

			// add the node
			addNode(newNode);

			// if the node in the policy is an initial node, create
			// a new edge between it and each previously final node
			if (subPolicy.isInitNode(newNode.getId())) {

				for (String intermediateNodeId : intermediaryNodesIds) {
					PolicyEdge newEdge = new PolicyEdge("bridge"  + new Random().nextInt(), 
							intermediateNodeId, newNode.getId(), bridgeCondition);
					addEdge(newEdge);
				}	
			}
		}

		// reset the final nodes of the policy
		finalNodeIds = new LinkedList<String>();
		for (PolicyNode finalNode: subPolicy.getFinalNodes()) {
			finalNodeIds.add(finalNode.getId());
		}

		// add the edges from the new policy
		for (PolicyEdge e: subPolicy.getAllEdges()) {
			edges.put(e.getId(), e.copy());
		}	
	}


	// ==============================================================
	// METHODS TO CHECK THE WELL-FORMEDNESS OF THE POLICY
	// ==============================================================


	/**
	 * Returns true if the policy is well-formed, false otherwise.
	 * 
	 * <p>A policy is well-formed iff: <ul>
	 * <il> it contains a least one node;
	 * <il> one node is specified as initial, and at least one node is specified as final
	 * <li> each edge has valid source and target nodes, and specifies one condition
	 * <li> each node contains an action
	 * <li> it is possible to get from the initial to the final node (with a limit of 200 steps)
	 * <li> it is possible to get back from each final node to the initial node (with a limit of 200 steps)
	 * </ul>
	 * 
	 */
	public boolean isWellFormedPolicy () {

		if (nodes == null  || initNodeId == "" || finalNodeIds == null ) {
			debug("Warning: null values");
			return false;
		}

		if (nodes.size() == 0) {
			debug("Warning: no node in policy");
			return false;
		}

		if (!nodes.containsKey(initNodeId)) {
			debug("Warning: initNode not contained in nodes: " + initNodeId);
			return false;
		} 


		for (String f : finalNodeIds) {
			if (!nodes.containsKey(f)) {
				debug("Warning: final node not contained in nodes");
				return false;
			}
		}

		for (PolicyEdge edge: edges.values()) {
			if (edge.getSourceNodeId() == null) {
				debug("Warning: edge " + edge.getId() + 
						" does not have a source node (entered target node ID: " + edge.getSourceNodeId() + ")");
				return false;
			}
			if (edge.getTargetNodeId() == null) {
				debug("Warning: edge " + edge.getId() + 
						" does not have a target node (entered target node ID: " + edge.getTargetNodeId() + ")");
				return false;
			}
		}

		if (finalNodeIds.size() == 0) {
			debug("Warning: no final node");
			debug(toString());
			return false;
		}

		for (PolicyNode a : nodes.values()) {
			if (a.getActions() == null) {
				debug("Warning: no action specified in node");
				return false;
			}
		}

		debug("now trying to get from initial to final node...");
		if (!isNodeLeadingToFinal(initNodeId, 0)) {
			debug("Warning: initial node not leading to any final node");
			debug("policy: " + toString());
			return false;
		}

		for (String finalNodeId : finalNodeIds) {
			log("now trying to get from final to initial node...");
			if (!isNodeGoingBackToInitial(finalNodeId, 0)) {
				debug("Warning: final node " + finalNodeId +  " not tracking back to the initial node");
				debug("policy: " + toString());
				return false;
			}
		}
		for (PolicyEdge e : edges.values()) {
			if (e.getConditions().size() == 0) {
				debug("Warning: no observation specified in edge " + e.getId());
				return false;
			}
		}

		return true;
	}



	/**
	 * Returns true if the given node is leading to any final node, and false
	 * otherwise 
	 * 
	 * @param nodeId the identifier of the action node to test
	 * @return
	 */
	private boolean isNodeLeadingToFinal(String nodeId, int count) {

		if (count >= MAX_STEPS) {
			debug("last nodeId in trace: " + nodeId);
			return false;
		}
		else {
			count++;
		}

		if (finalNodeIds.contains(nodeId)) {
			return true;
		} 
		LinkedList<PolicyEdge> observations = 
			new LinkedList<PolicyEdge>(getAllOutgoingEdges(nodeId));
		Collections.shuffle(observations);

		for (PolicyEdge e: observations) {
			String nextNodeId = e.getTargetNodeId();
			if (isNodeLeadingToFinal(nextNodeId, count)) {
				return true;
			}
		}
		return false;
	}


	/**
	 * Returns true if is possible to go back to the initial node from
	 * the given node identifier
	 * 
	 * @param nodeId the node identifier from which to start the search
	 * @param count the count of steps till now
	 * @return true if the initial node is reached, false if the search failed
	 */
	private boolean isNodeGoingBackToInitial(String nodeId, int count) {

		if (count >= MAX_STEPS)  {
			debug("last nodeId in trace: " + nodeId);
			return false;
		}
		else {
			count++;
		}

		if (initNodeId.equals(nodeId)) {
			return true;
		}

		LinkedList<PolicyEdge> observations = 
			new LinkedList<PolicyEdge>(getAllIncomingEdges(nodeId));
		Collections.shuffle(observations);

		for (PolicyEdge e: observations) {
			String previousNodeId = e.getSourceNodeId();
			if (isNodeGoingBackToInitial(previousNodeId, count)) {
				return true;
			}
		}
		return false;
	}



	// ==============================================================
	// UTILITY METHODS
	// ==============================================================



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
		result += "From which initial node = " + initNodeId + " and final nodes: " + finalNodeIds + "\n";
		result += "Edges = \n";
		for (String nodeId : nodes.keySet()) {
			for (PolicyEdge edge: edges.values()) {
				if (edge.getSourceNodeId().equals(nodeId)) {

					String conditions = "";
					for (AbstractCondition condition : edge.getConditions()) {
						conditions += condition.toString() + " ^ ";
					}
					conditions = conditions.substring(0, conditions.length() - 3);

					result += edge.getSourceNodeId() + " -- " + conditions + 
					"\t(" + edge.getId() + ") --> " + edge.getTargetNodeId() + "\n";	
					if (!nodes.containsKey(edge.getTargetNodeId())) {
						result += "[WARNING: target node for edge " + edge.getId() + " not contained in policy!]\n";
					}
				}
			}
		}

		result += "Actions = \n";
		for (String nodeId : nodes.keySet()) {
			result += nodeId + " : " + nodes.get(nodeId).getActions().toString() + " \n";
		}

		return result;
	}

 
	/**
	 * Logging
	 * @param s
	 */
	private void log (String s) {
		if (logger != null) {
			logger.info(s);
		}
		else if (LOGGING) {
			System.err.println("[dialpolicy LOG] " + s);
		}
	}


	/**
	 * Debugging
	 * @param s
	 */
	private void debug (String s) {
		if (logger != null) {
			logger.debug(s);
		}
		else if (DEBUG) {
			System.err.println("[dialpolicy DEBUG] " + s);
		}
	}


}
