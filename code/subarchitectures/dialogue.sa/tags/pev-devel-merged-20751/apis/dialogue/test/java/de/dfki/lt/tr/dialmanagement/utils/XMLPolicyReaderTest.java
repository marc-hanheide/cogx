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

package de.dfki.lt.tr.dialmanagement.utils;

import static org.junit.Assert.*;

import org.junit.Test;

import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.data.conditions.IntentionCondition;
import de.dfki.lt.tr.dialmanagement.data.policies.DialoguePolicy;

/**
 * Test for reading a XML policy file
 * 
 * @author Pierre Lison (plison@ifi.uio.no)
 * @version 09/10/2010
 */
public class XMLPolicyReaderTest {

	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = false;
	
	String policyFile = "config/policies/testing/policy6.xml";
	
	
	/**
	 * Readers a policy in the XML format
	 * 
	 * @throws DialogueException
	 */
	@Test
	public void testFileReading () throws DialogueException {
		DialoguePolicy policy = XMLPolicyReader.constructPolicy(policyFile);
		log(policy.toString());
		assertEquals(3, policy.getAllNodes().size());
		assertEquals(true, policy.isInitNode("startNode"));
		assertEquals(false, policy.isFinalNode("startNode"));
		assertEquals(false, policy.isInitNode("engagedNode"));
		assertEquals(false, policy.isFinalNode("engagedNode"));
		assertEquals(false, policy.isInitNode("closedNode"));
		assertEquals(true, policy.isFinalNode("closedNode"));
		assertEquals(1, policy.getAllOutgoingEdges(policy.getNode("startNode").getId()).size());
		assertEquals(2, policy.getAllEdges().size());
		
		assertEquals("startNode", policy.getEdge("greetingEdge").getSourceNodeId());
		assertEquals("engagedNode", policy.getEdge("greetingEdge").getTargetNodeId());
		assertEquals("greetingCond", policy.getEdge("greetingEdge").getConditions().get(0).getId());
		assertTrue(FormulaUtils.subsumes(policy.getEdge("greetingEdge").getConditions().get(0).asFormula(), 
				FormulaUtils.constructFormula("<state>(engagement-open ^ <agent>human ^ <agent>robot)")));
		assertEquals(0.0f, policy.getEdge("greetingEdge").getConditions().get(0).getMinimumProb(), 0.1f);
		assertEquals(1.0f, policy.getEdge("greetingEdge").getConditions().get(0).getMaximumProb(), 0.1f);
		assertTrue(policy.getEdge("greetingEdge").getConditions().get(0) instanceof IntentionCondition);
		
		assertEquals("engagedNode", policy.getEdge("closingEdge").getSourceNodeId());
		assertEquals("closedNode", policy.getEdge("closingEdge").getTargetNodeId());
		assertEquals("closingCond", policy.getEdge("closingEdge").getConditions().get(0).getId());
		assertTrue(FormulaUtils.subsumes(policy.getEdge("closingEdge").getConditions().get(0).asFormula(), 
				FormulaUtils.constructFormula("<state>(engagement-closed ^ <agent>human ^ <agent>robot)")));
		assertEquals(0.0f, policy.getEdge("closingEdge").getConditions().get(0).getMinimumProb(), 0.1f);
		assertEquals(1.0f, policy.getEdge("closingEdge").getConditions().get(0).getMaximumProb(), 0.1f);
		assertTrue(policy.getEdge("closingEdge").getConditions().get(0) instanceof IntentionCondition);
		
		debug("action for engagedNode: " + policy.getNode("engagedNode").getActions().get(0).asFormula());
		assertTrue(FormulaUtils.subsumes(policy.getNode("engagedNode").getActions().get(0).asFormula(), 
				FormulaUtils.constructFormula("<state>(engagement-open ^ <agent>robot ^ <agent>human)")));
		
		assertTrue(FormulaUtils.subsumes(policy.getNode("closedNode").getActions().get(0).asFormula(), 
				FormulaUtils.constructFormula("<state>(engagement-closed ^ <agent>robot ^ <agent>human)")));
	}
	 
	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[xmlpolicyreadertest] " + s);
		}
	}

	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[xmlpolicyreadertest] " + s);
		}
	}
}
