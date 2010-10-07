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

package de.dfki.lt.tr.dialmanagement.components;

import static org.junit.Assert.*;

import org.junit.Test;

import org.junit.Before;

import de.dfki.lt.tr.beliefs.slice.logicalcontent.UnknownFormula;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.data.DialoguePolicy;
import de.dfki.lt.tr.dialmanagement.data.Observation;
import de.dfki.lt.tr.dialmanagement.data.PolicyAction;
import de.dfki.lt.tr.dialmanagement.utils.PolicyReader;


/**
 * Test class for a simple Dora-style interaction
 * 
 * @author Pierre Lison (plison@dfki.de)
 * @version 04/07/2010
 *
 */
public class DialogueManagerDoraTest {


	public static boolean LOGGING = true;
	
	public static boolean DEBUG = false;
	
	// the configuration files
	public static String POLICYFILE = "subarchitectures/dialogue.sa/config/policies/policyExampleDora.txt";
	public static String OBSFILE = "subarchitectures/dialogue.sa/config/policies/observationsDora.txt";
	public static String ACTIONSFILE = "subarchitectures/dialogue.sa/config/policies/actionsDora.txt";

	// the dialogue manager
	public DialogueManager manager;
	
	
	/**
	 * Construct the dialogue policy
	 * 
	 * @throws DialogueException if the configuration files are not well-formatted
	 */
	@Before
	public void constructPolicy() throws DialogueException {
		
		DialoguePolicy policy = PolicyReader.constructPolicy(POLICYFILE, OBSFILE, ACTIONSFILE);
		
		policy.ensureWellFormedPolicy();
		
		manager = new DialogueManager(policy);
	}
	
	
	private Observation createSimpleObservation (String s) throws DialogueException {
		return createSimpleObservation(s, 1.0f);
	}
	

	private Observation createSimpleObservation (String s, float f) throws DialogueException {
		Observation intent = new Observation (Observation.INTENTION);
		intent.addAlternative(s, f);
		intent.addAlternative(new UnknownFormula(0), 1-f);
		return intent;
	}
	
	/**
	 * Test the policy with a single, high-confidence utterance
	 * @throws DialogueException
	 */
	@Test
	public void testPolicyDirect1() throws DialogueException {
		
		Observation intent = createSimpleObservation("Please find the cornflakes box");
		PolicyAction action1 = manager.nextAction(intent);
		assertEquals(action1.toString(), "CI[\"okay, searching for the cornflakes box!\"]");
		
	}
	
	/**
	 * Test the policy with a single, high-confidence utterance
	 * @throws DialogueException
	 */
	@Test
	public void testPolicyDirect2() throws DialogueException {
		
		Observation intent = createSimpleObservation("Please find the mug");
		PolicyAction action1 = manager.nextAction(intent);
		assertEquals(action1.toString(), "CI[\"okay, searching for the mug!\"]");
		
	}
	
	/**
	 * Test the policy with a lower-confidence utterance which must be confirmed
	 * @throws DialogueException
	 */
	@Test
	public void testPolicyConfirm() throws DialogueException {
		
		Observation intent = createSimpleObservation("Please find the cornflakes box", 0.4f);
		PolicyAction action1 = manager.nextAction(intent);
		assertEquals(action1.toString(), "CI[\"sorry, should I search for the cornflaxes box?\"]");
		Observation intent2 = createSimpleObservation("yes", 0.8f);
		PolicyAction action2 = manager.nextAction(intent2);
		assertEquals(action2.toString(), "CI[\"okay, searching for the cornflakes box!\"]");
	}
	
	/**
	 * Test the policy with a lower-confidence which is disproved, then repeated
	 * @throws DialogueException
	 */
	@Test
	public void testPolicyDisprove() throws DialogueException {
		
		Observation intent = createSimpleObservation("Please find the cornflakes box", 0.4f);
		PolicyAction action1 = manager.nextAction(intent);
		assertEquals(action1.toString(), "CI[\"sorry, should I search for the cornflaxes box?\"]");
		Observation intent2 = createSimpleObservation("no", 0.8f);
		PolicyAction action2 = manager.nextAction(intent2);
		assertEquals(action2.toString(), "CI[\"sorry, could you repeat please?\"]");
		testPolicyDirect2();
	}

	/**
	 * Test the policy with an unrecognised utternce which must be repeated
	 * @throws DialogueException
	 */
	@Test
	public void testPolicyRepeat() throws DialogueException {
		
		Observation i = createSimpleObservation("?", 0.8f);
		PolicyAction action1 = manager.nextAction(i);
		assertEquals(action1.toString(), "CI[\"sorry, could you repeat please?\"]");
		testPolicyConfirm();
	}
	

	/**
	 * Test the policy with an unrecognised utternce which must be repeated
	 * @throws DialogueException
	 */
	@Test
	public void testPolicyRepeat2() throws DialogueException {
		
		Observation i = createSimpleObservation("bla bla bla", 0.8f);
		PolicyAction action1 = manager.nextAction(i);
		assertEquals(action1.toString(), "CI[\"sorry, could you repeat please?\"]");
		testPolicyConfirm();
	}

	/**
	 * Test the policy with a failed attempt to recognise what the user said
	 * @throws DialogueException
	 */
	@Test
	public void testPolicyFailure() throws DialogueException {
		
		Observation i1 = createSimpleObservation("?", 0.8f);
		PolicyAction action1 = manager.nextAction(i1);
		assertEquals(action1.toString(), "CI[\"sorry, could you repeat please?\"]");
		Observation i2 = createSimpleObservation("?", 0.8f);
		PolicyAction action2 = manager.nextAction(i2);
		assertEquals(action2.toString(), "CI[\"sorry I couldn't understand you\"]");
	}

	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("\"[dialmanager test\"] " + s);
		}
	}
	
	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[dialmanager test] " + s);
		}
	}
}
