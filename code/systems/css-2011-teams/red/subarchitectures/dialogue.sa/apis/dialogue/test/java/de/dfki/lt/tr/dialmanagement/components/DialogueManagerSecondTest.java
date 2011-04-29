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

package de.dfki.lt.tr.dialmanagement.components;

import static org.junit.Assert.*;

import org.junit.Test;


import org.junit.Before;

import de.dfki.lt.tr.beliefs.slice.events.Event;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.UnknownFormula;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.data.Observation;
import de.dfki.lt.tr.dialmanagement.data.policies.DialoguePolicy;
import de.dfki.lt.tr.dialmanagement.data.policies.PolicyAction;
import de.dfki.lt.tr.dialmanagement.utils.EpistemicObjectUtils;
import de.dfki.lt.tr.dialmanagement.utils.PolicyUtils;
import de.dfki.lt.tr.dialmanagement.utils.TextPolicyReader;


/**
 * Test class for a simple interaction with openings and closings
 * 
 * @author Pierre Lison (plison@dfki.de)
 * @version 04/07/2010
 *
 */
public class DialogueManagerSecondTest {

	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = true;
	
	// the configuration files
	public static String POLICYFILE = "subarchitectures/dialogue.sa/config/policies/testing/policy2.txt";
	public static String OBSFILE = "subarchitectures/dialogue.sa/config/policies/testing/conditions2.txt";
	public static String ACTIONSFILE = "subarchitectures/dialogue.sa/config/policies/testing/actions2.txt";

	// the dialogue manager
	public DialogueManager manager;
	
	
	/**
	 * Construct the dialogue policy
	 * 
	 * @throws DialogueException if the configuration files are not well-formatted
	 */
	@Before
	public void constructPolicy() throws DialogueException {
		
		DialoguePolicy policy = TextPolicyReader.constructPolicy(POLICYFILE, OBSFILE, ACTIONSFILE);
		
		policy.ensureWellFormedPolicy();
		
		manager = new DialogueManager(policy);
	}
	

	/**
	 * Test a greeting
	 * 
	 * @throws DialogueException
	 */
	@Test
	public void testPolicyGreeting() throws DialogueException {
		
		Observation intent = PolicyUtils.createSimpleObservation("<state>(engagement-open ^ <agent>human ^ <agent>robot)");
		PolicyAction action1 = manager.nextAction(intent);
		assertEquals("CI[<state>(engagement-open ^ <agent>(robot) ^ <agent>(human))]", action1.toString());	
	}

	/**
	 * Test a closing
	 * 
	 * @throws DialogueException
	 */
	@Test
	public void testPolicyClosing1() throws DialogueException {
		testPolicyGreeting();
		Observation intent = PolicyUtils.createSimpleObservation("<state>(engagement-closed ^ <agent>human ^ <agent>robot)");
		PolicyAction action1 = manager.nextAction(intent);
		assertEquals( "CI[<state>(engagement-closed ^ <agent>(robot) ^ <agent>(human))]", action1.toString());
	}
	
	/**
	 * Test a failed closing (without first engaging with the user)
	 * 
	 * @throws DialogueException
	 */
	@Test
	public void testPolicyClosing2() throws DialogueException {
		Observation intent = PolicyUtils.createSimpleObservation("<state>(engagement-closed ^ <agent>human ^ <agent>robot)");
		PolicyAction action1 = manager.nextAction(intent);
		assertTrue(action1.isVoid());
	}
	
	/**
	 * Test a recognition error
	 * 
	 * @throws DialogueException
	 */
	@Test
	public void testPolicyRecognitionError() throws DialogueException {
		Event event = EpistemicObjectUtils.createSimpleEvent("recognition-error", 0.8f);
		PolicyAction action1 = manager.nextAction(event);
		assertEquals("CI[<state>(error-reported)]", action1.toString());
	}
	
	
	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[dialmanager_secondtest] " + s);
		}
	}
	
	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[dialmanager_secondtest] " + s);
		}
	}
}
