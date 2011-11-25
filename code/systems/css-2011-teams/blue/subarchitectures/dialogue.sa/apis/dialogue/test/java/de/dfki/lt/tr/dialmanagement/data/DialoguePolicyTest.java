
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


package de.dfki.lt.tr.dialmanagement.data;


//JUnit
import java.util.Random;

import org.junit.Test;

import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.data.policies.DialoguePolicy;
import de.dfki.lt.tr.dialmanagement.data.policies.PolicyAction;
import de.dfki.lt.tr.dialmanagement.data.policies.PolicyEdge;
import de.dfki.lt.tr.dialmanagement.data.policies.PolicyNode;
import de.dfki.lt.tr.dialmanagement.data.policies.PolicyCondition;


/**
 * Simple tests for constructing a dialogue policy
 * 
 * TODO: test for different types of ill-formed policies
 * @author Pierre Lison (plison@dfki.de)
 *
 */
public class DialoguePolicyTest {

	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = true;
	
	// parameters for the test
	public static int MAX_LEAVES = 6;
	public static int MAX_DEPTH = 4;
	public static int NB_POLICIES = 50;
	
	// counts for forging identifiers
	private static int count = 0;
	 
	
	
	/**
	 * Construct a minimal dialogue policy and ensure it is well-formed
	 * 
	 * @throws DialogueException
	 */
	@Test
	public void minimalPolicyConstruction () throws DialogueException {
		DialoguePolicy policy = new DialoguePolicy();
				
		PolicyNode an1 = new PolicyNode("start", new PolicyAction("start"));
		policy.addNode(an1);
		policy.setNodeAsInitial(an1);

		PolicyNode an2 = new PolicyNode("end", new PolicyAction("end"));
		policy.addNode(an2);
		PolicyEdge newEdge = new PolicyEdge("edge", an1, an2, new PolicyCondition("","edge", 1.0f, 1.0f));
		newEdge.setSourceNode(an1);
		newEdge.setTargetNode(an2);
		policy.addEdge(newEdge, an1, an2);
		policy.setNodeAsFinal(an2);
		
		policy.ensureWellFormedPolicy();
	}
	
	


	/**
	 * Construct NB_POLICIES random policies and verify that each of them is well-formed
	 * @throws DialogueException
	 */
	@Test
	public void randomPolicyConstructions () throws DialogueException {
		
		for (int i = 0; i < NB_POLICIES; i++) {
			randomPolicyConstruction (MAX_LEAVES, MAX_DEPTH);
		}
	}
	

	/**
	 * Construct a random policy with a maximum depth and a maximum number of leaves
	 * 
	 * @param maxLeaves max number of leaves
	 * @param maxDepth maximum depth
	 * @throws DialogueException
	 */
	private void randomPolicyConstruction (int maxLeaves, int maxDepth) throws DialogueException {
		DialoguePolicy policy = new DialoguePolicy();
				
		PolicyNode an1 = new PolicyNode("start", new PolicyAction(getNewId()));
		policy.addNode(an1);
		policy.setNodeAsInitial(an1);
		debug("initial node: " + an1);

		expandRandomSubTree (policy, an1, maxLeaves, maxDepth);
		
		policy.ensureWellFormedPolicy();

	}
	

	/**
	 * Expand a subtree of the policy starting at curNode
	 * 
	 * @param policy the policy to expand
	 * @param curNode the node at which to expand the policy with the subtree
	 * @param maxNbLeaves the maximum number of leaves
	 * @param depth the maximum depth
	 * @throws DialogueException
	 */
	private void expandRandomSubTree (DialoguePolicy policy, PolicyNode curNode, int maxNbLeaves, int depth) throws DialogueException {
		
	    Random generator = new Random();
	    int nbLeaves = generator.nextInt(maxNbLeaves);
	    
	    for (int i = 0; i < nbLeaves && depth > 0 ; i++) {
	    	
	    	PolicyEdge nextObs = new PolicyEdge(getNewId(), new PolicyCondition("",getNewId(), 1.0f, 1.0f));
	    	PolicyNode nextNode = new PolicyNode(getNewId(), new PolicyAction(getNewId()));
	    	policy.addNode(nextNode);
	    	nextObs.setTargetNode(nextNode);
	    	nextObs.setSourceNode(curNode);
			policy.addEdge(nextObs, curNode, nextNode);
			
			debug("creating new observation " + nextObs + " between node " + curNode + " and node " + nextNode);
	    	expandRandomSubTree(policy, nextNode, maxNbLeaves, depth -1);
	    }
	    
	    if (nbLeaves == 0 || depth == 0) {
	    	policy.setNodeAsFinal(curNode);
	    	debug("setting node " + curNode + " as final node");
	    }
	    
	}
	

	/**
	 * Returns a new id
	 * @return the ned id
	 */
	private static String getNewId() {
		count++;
		return "id" + count;
	}
	

	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[dialpolicytest] " + s);
		}
	}

	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[dialpolicytest] " + s);
		}
	}


}