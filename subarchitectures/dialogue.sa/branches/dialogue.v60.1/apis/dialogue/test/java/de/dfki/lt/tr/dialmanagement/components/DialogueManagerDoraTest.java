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

import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.data.DialoguePolicy;
import de.dfki.lt.tr.dialmanagement.data.actions.AbstractAction;
import de.dfki.lt.tr.dialmanagement.data.observations.EventObservation;
import de.dfki.lt.tr.dialmanagement.data.observations.IntentionObservation;
import de.dfki.lt.tr.dialmanagement.utils.FormulaUtils;
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
	public static String POLICYFILE = "config/policies/policyExampleDora.txt";
	public static String OBSFILE = "config/policies/observationsDora.txt";
	public static String ACTIONSFILE = "config/policies/actionsDora.txt";

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
	
	
	/**
	 * Test the policy with a single, high-confidence utterance
	 * @throws DialogueException
	 */
	@Test
	public void testPolicyDirect1() throws DialogueException {
		
		IntentionObservation intent = new IntentionObservation(FormulaUtils.constructFormula("\"Please find the cornflakes box\""), 1.0f);
		AbstractAction action1 = manager.nextAction(intent);
		assertEquals(action1.toString(), "I[\"okay, searching for the cornflakes box!\"]");
		
	}
	
	/**
	 * Test the policy with a single, high-confidence utterance
	 * @throws DialogueException
	 */
	@Test
	public void testPolicyDirect2() throws DialogueException {
		
		IntentionObservation intent = new IntentionObservation(FormulaUtils.constructFormula("\"Please find the mug\""), 1.0f);
		AbstractAction action1 = manager.nextAction(intent);
		assertEquals(action1.toString(), "I[\"okay, searching for the mug!\"]");
		
	}
	
	/**
	 * Test the policy with a lower-confidence utterance which must be confirmed
	 * @throws DialogueException
	 */
	@Test
	public void testPolicyConfirm() throws DialogueException {
		
		IntentionObservation intent = new IntentionObservation(FormulaUtils.constructFormula("\"Please find the cornflakes box\""), 0.4f, .4f);
		AbstractAction action1 = manager.nextAction(intent);
		assertEquals(action1.toString(), "I[\"sorry, should I search for the cornflaxes box?\"]");
		IntentionObservation intent2 = new IntentionObservation(FormulaUtils.constructFormula("yes"), 0.8f,0.8f);
		AbstractAction action2 = manager.nextAction(intent2);
		assertEquals(action2.toString(), "I[\"okay, searching for the cornflakes box!\"]");
	}
	
	/**
	 * Test the policy with a lower-confidence which is disproved, then repeated
	 * @throws DialogueException
	 */
	@Test
	public void testPolicyDisprove() throws DialogueException {
		
		IntentionObservation intent = new IntentionObservation(FormulaUtils.constructFormula("\"Please find the cornflakes box\""), 0.4f, .4f);
		AbstractAction action1 = manager.nextAction(intent);
		assertEquals(action1.toString(), "I[\"sorry, should I search for the cornflaxes box?\"]");
		IntentionObservation intent2 = new IntentionObservation(FormulaUtils.constructFormula("no"), 0.8f,0.8f);
		AbstractAction action2 = manager.nextAction(intent2);
		assertEquals(action2.toString(), "I[\"sorry, could you repeat please?\"]");
		testPolicyDirect2();
	}

	/**
	 * Test the policy with an unrecognised utternce which must be repeated
	 * @throws DialogueException
	 */
	@Test
	public void testPolicyRepeat() throws DialogueException {
		
		IntentionObservation i = new IntentionObservation(FormulaUtils.constructFormula("?"), 0.8f, 0.8f);
		AbstractAction action1 = manager.nextAction(i);
		assertEquals(action1.toString(), "I[\"sorry, could you repeat please?\"]");
		testPolicyConfirm();
	}
	

	/**
	 * Test the policy with an unrecognised utternce which must be repeated
	 * @throws DialogueException
	 */
	@Test
	public void testPolicyRepeat2() throws DialogueException {
		
		IntentionObservation i = new IntentionObservation(FormulaUtils.constructFormula("\"bla bla bla\""), 0.8f, 0.8f);
		AbstractAction action1 = manager.nextAction(i);
		assertEquals(action1.toString(), "I[\"sorry, could you repeat please?\"]");
		testPolicyConfirm();
	}

	/**
	 * Test the policy with a failed attempt to recognise what the user said
	 * @throws DialogueException
	 */
	@Test
	public void testPolicyFailure() throws DialogueException {
		
		IntentionObservation i1 = new IntentionObservation(FormulaUtils.constructFormula("?"), 0.8f, 0.8f);
		AbstractAction action1 = manager.nextAction(i1);
		assertEquals(action1.toString(), "I[\"sorry, could you repeat please?\"]");
		IntentionObservation i2 = new IntentionObservation(FormulaUtils.constructFormula("?"), 0.8f, 0.8f);
		AbstractAction action2 = manager.nextAction(i2);
		assertEquals(action2.toString(), "I[\"sorry I couldn't understand you\"]");
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