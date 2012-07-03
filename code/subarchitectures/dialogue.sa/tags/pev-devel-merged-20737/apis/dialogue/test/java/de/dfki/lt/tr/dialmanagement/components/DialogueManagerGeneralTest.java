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

package de.dfki.lt.tr.dialmanagement.components;

import static org.junit.Assert.*;

import org.junit.Before;
import org.junit.Test;


import de.dfki.lt.tr.beliefs.slice.intentions.CommunicativeIntention;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.data.ActionSelectionResult;
import de.dfki.lt.tr.dialmanagement.data.Observation;
import de.dfki.lt.tr.dialmanagement.data.actions.AbstractAction;
import de.dfki.lt.tr.dialmanagement.data.policies.DialoguePolicy;
import de.dfki.lt.tr.dialmanagement.utils.EpistemicObjectUtils;
import de.dfki.lt.tr.dialmanagement.utils.PolicyUtils;
import de.dfki.lt.tr.dialmanagement.utils.TextPolicyReader;


/**
 * Test class for a simple, Dora-style interaction
 * 
 * @author Pierre Lison (plison@ifi.uio.no)
 * @version 04/07/2010
 *
 */
public class DialogueManagerGeneralTest  {
 
	

	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = false;
	
	
	// the configuration files
	// the configuration files
	public static String POLICYFILE = "config/policies/testing/policy3.txt";
	public static String OBSFILE = "config/policies/testing/conditions3.txt";
	public static String ACTIONSFILE = "config/policies/testing/actions3.txt";

	protected DialogueManager manager ;
	
	

	/**
	 * Construct the policy based on the configuration files
	 * 
	 * @throws DialogueException if the files are ill-formatted
	 */
	@Before
	public void startDialogueManager() throws DialogueException {
		DialoguePolicy policy = TextPolicyReader.constructPolicy(POLICYFILE, OBSFILE, ACTIONSFILE);
		manager = new DialogueManager(policy);
	}
	
	
	 
	
	/**
	 * Test the policy with a single, high-confidence utterance
	 * @throws DialogueException
	 * @throws InterruptedException 
	 */
	@Test
	public void testPolicyDirect1() throws DialogueException, InterruptedException {
		
		CommunicativeIntention intent = EpistemicObjectUtils.createSimpleCommunicativeIntention("Please find the cornflakes box");
		ActionSelectionResult r = manager.updateStateAndSelectAction(intent);
		AbstractAction action1 = r.getActions().get(0);
		assertEquals("CI[okay, searching for the cornflakes box!]", action1.toString());
		
	}
	
	/**
	 * Test the policy with a single, high-confidence utterance
	 * @throws DialogueException
	 * @throws InterruptedException 
	 */
	@Test
	public void testPolicyDirect2() throws DialogueException, InterruptedException {
		
		CommunicativeIntention intent = EpistemicObjectUtils.createSimpleCommunicativeIntention("Please find the mug");
		ActionSelectionResult r = manager.updateStateAndSelectAction(intent);
		AbstractAction action1 = r.getActions().get(0);
		assertEquals("CI[okay, searching for the mug!]", action1.toString());
		
	}
	
	/**
	 * Test the policy with a lower-confidence utterance which must be confirmed
	 * @throws DialogueException
	 * @throws InterruptedException 
	 */
	@Test
	public void testPolicyConfirm() throws DialogueException, InterruptedException {
		
		CommunicativeIntention intent = EpistemicObjectUtils.createSimpleCommunicativeIntention("Please find the cornflakes box", 0.4f);
		ActionSelectionResult r = manager.updateStateAndSelectAction(intent);
		AbstractAction action1 = r.getActions().get(0);
	//	log("POLICY: " + manager.getPolicy().toString());
		assertEquals("CI[sorry, should I search for the cornflaxes box?]", action1.toString());
		CommunicativeIntention intent2 = EpistemicObjectUtils.createSimpleCommunicativeIntention("yes", 0.8f);
		ActionSelectionResult r2 = manager.updateStateAndSelectAction(intent2);
		AbstractAction action2 = r2.getActions().get(0);
		assertEquals("CI[okay, searching for the cornflakes box!]", action2.toString());
	}
	
	/**
	 * Test the policy with a lower-confidence which is disproved, then repeated
	 * @throws DialogueException
	 * @throws InterruptedException 
	 */
	@Test
	public void testPolicyDisprove() throws DialogueException, InterruptedException {
		
		CommunicativeIntention intent = EpistemicObjectUtils.createSimpleCommunicativeIntention("Please find the cornflakes box", 0.4f);
		ActionSelectionResult r = manager.updateStateAndSelectAction(intent);
		AbstractAction action1 = r.getActions().get(0);
		assertEquals("CI[sorry, should I search for the cornflaxes box?]", action1.toString());
		CommunicativeIntention intent2 = EpistemicObjectUtils.createSimpleCommunicativeIntention("no", 0.8f);
		ActionSelectionResult r2 = manager.updateStateAndSelectAction(intent2);
		AbstractAction action2 = r2.getActions().get(0);
		assertEquals("CI[sorry, could you repeat please?]", action2.toString());
		testPolicyDirect2();
	}

	/**
	 * Test the policy with an unrecognised utternce which must be repeated
	 * @throws DialogueException
	 * @throws InterruptedException 
	 */
	@Test
	public void testPolicyRepeat() throws DialogueException, InterruptedException {
		
		CommunicativeIntention i = EpistemicObjectUtils.createSimpleCommunicativeIntention("?", 0.8f);
		ActionSelectionResult r = manager.updateStateAndSelectAction(i);
		AbstractAction action1 = r.getActions().get(0);
		assertEquals("CI[sorry, could you repeat please?]", action1.toString());
		testPolicyConfirm();
	}
	

	/**
	 * Test the policy with an unrecognised utternce which must be repeated
	 * @throws DialogueException
	 * @throws InterruptedException 
	 */
	@Test
	public void testPolicyRepeat2() throws DialogueException, InterruptedException {
		
		CommunicativeIntention i = EpistemicObjectUtils.createSimpleCommunicativeIntention("bla bla bla", 0.8f);
		ActionSelectionResult r = manager.updateStateAndSelectAction(i);
		AbstractAction action1 = r.getActions().get(0);
		assertEquals("CI[sorry, could you repeat please?]", action1.toString());
		testPolicyConfirm();
	}

	/**
	 * Test the policy with a failed attempt to recognise what the user said
	 * @throws DialogueException
	 * @throws InterruptedException 
	 */
	@Test
	public void testPolicyFailure() throws DialogueException, InterruptedException {
		
		CommunicativeIntention i1 = EpistemicObjectUtils.createSimpleCommunicativeIntention("?", 0.8f);
		ActionSelectionResult r = manager.updateStateAndSelectAction(i1);
		AbstractAction action1 = r.getActions().get(0);
		assertEquals("CI[sorry, could you repeat please?]", action1.toString());
		CommunicativeIntention i2 = EpistemicObjectUtils.createSimpleCommunicativeIntention("?", 0.8f);
		ActionSelectionResult r2 = manager.updateStateAndSelectAction(i2);
		AbstractAction action2 = r2.getActions().get(0);
		assertEquals("CI[sorry I couldn't understand you]", action2.toString());
	}


	/**
	 * Logging
	 * @param s
	 */
	protected static void log (String s) {
		if (LOGGING) {
			System.out.println("[dialmanager_basictest] " + s);
		}
	}
	
	/**
	 * Debugging
	 * @param s
	 */
	protected static void debug (String s) {
		if (DEBUG) {
			System.out.println("[dialmanager_basictest] " + s);
		}
	}

}
