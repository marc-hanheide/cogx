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

import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.data.Observation;
import de.dfki.lt.tr.dialmanagement.data.policies.DialoguePolicy;
import de.dfki.lt.tr.dialmanagement.data.policies.PolicyAction;
import de.dfki.lt.tr.dialmanagement.utils.PolicyUtils;
import de.dfki.lt.tr.dialmanagement.utils.TextPolicyReader;


/**
 * Test class for a simple, Dora-style interaction
 * 
 * @author Pierre Lison (plison@dfki.de)
 * @version 04/07/2010
 *
 */
public class DialogueManagerGeneralTest {

	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = true;
	
	// the configuration files
	public static String POLICYFILE = "subarchitectures/dialogue.sa/config/policies/testing/policy3.txt";
	public static String OBSFILE = "subarchitectures/dialogue.sa/config/policies/testing/conditions3.txt";
	public static String ACTIONSFILE = "subarchitectures/dialogue.sa/config/policies/testing/actions3.txt";

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
	 * Test the policy with a single, high-confidence utterance
	 * @throws DialogueException
	 */
	@Test
	public void testPolicyDirect1() throws DialogueException {
		
		Observation intent = PolicyUtils.createSimpleObservation("Please find the cornflakes box");
		PolicyAction action1 = manager.nextAction(intent);
		assertEquals("CI[\"okay, searching for the cornflakes box!\"]", action1.toString());
		
	}
	
	/**
	 * Test the policy with a single, high-confidence utterance
	 * @throws DialogueException
	 */
	@Test
	public void testPolicyDirect2() throws DialogueException {
		
		Observation intent = PolicyUtils.createSimpleObservation("Please find the mug");
		PolicyAction action1 = manager.nextAction(intent);
		assertEquals("CI[\"okay, searching for the mug!\"]", action1.toString());
		
	}
	
	/**
	 * Test the policy with a lower-confidence utterance which must be confirmed
	 * @throws DialogueException
	 */
	@Test
	public void testPolicyConfirm() throws DialogueException {
		
		Observation intent = PolicyUtils.createSimpleObservation("Please find the cornflakes box", 0.4f);
		PolicyAction action1 = manager.nextAction(intent);
		assertEquals("CI[\"sorry, should I search for the cornflaxes box?\"]", action1.toString());
		Observation intent2 = PolicyUtils.createSimpleObservation("yes", 0.8f);
		PolicyAction action2 = manager.nextAction(intent2);
		assertEquals("CI[\"okay, searching for the cornflakes box!\"]", action2.toString());
	}
	
	/**
	 * Test the policy with a lower-confidence which is disproved, then repeated
	 * @throws DialogueException
	 */
	@Test
	public void testPolicyDisprove() throws DialogueException {
		
		Observation intent = PolicyUtils.createSimpleObservation("Please find the cornflakes box", 0.4f);
		PolicyAction action1 = manager.nextAction(intent);
		assertEquals("CI[\"sorry, should I search for the cornflaxes box?\"]", action1.toString());
		Observation intent2 = PolicyUtils.createSimpleObservation("no", 0.8f);
		PolicyAction action2 = manager.nextAction(intent2);
		assertEquals("CI[\"sorry, could you repeat please?\"]", action2.toString());
		testPolicyDirect2();
	}

	/**
	 * Test the policy with an unrecognised utternce which must be repeated
	 * @throws DialogueException
	 */
	@Test
	public void testPolicyRepeat() throws DialogueException {
		
		Observation i = PolicyUtils.createSimpleObservation("?", 0.8f);
		PolicyAction action1 = manager.nextAction(i);
		assertEquals("CI[\"sorry, could you repeat please?\"]", action1.toString());
		testPolicyConfirm();
	}
	

	/**
	 * Test the policy with an unrecognised utternce which must be repeated
	 * @throws DialogueException
	 */
	@Test
	public void testPolicyRepeat2() throws DialogueException {
		
		Observation i = PolicyUtils.createSimpleObservation("bla bla bla", 0.8f);
		PolicyAction action1 = manager.nextAction(i);
		assertEquals("CI[\"sorry, could you repeat please?\"]", action1.toString());
		testPolicyConfirm();
	}

	/**
	 * Test the policy with a failed attempt to recognise what the user said
	 * @throws DialogueException
	 */
	@Test
	public void testPolicyFailure() throws DialogueException {
		
		Observation i1 = PolicyUtils.createSimpleObservation("?", 0.8f);
		PolicyAction action1 = manager.nextAction(i1);
		assertEquals("CI[\"sorry, could you repeat please?\"]", action1.toString());
		Observation i2 = PolicyUtils.createSimpleObservation("?", 0.8f);
		PolicyAction action2 = manager.nextAction(i2);
		assertEquals("CI[\"sorry I couldn't understand you\"]", action2.toString());
	}

	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[dialmanager_generaltest] " + s);
		}
	}
	
	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[dialmanager_generaltest] " + s);
		}
	}
}
