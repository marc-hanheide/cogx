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

import org.junit.Test;


import org.junit.Before;

import de.dfki.lt.tr.beliefs.slice.events.Event;
import de.dfki.lt.tr.beliefs.slice.intentions.CommunicativeIntention;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.UnknownFormula;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.data.ActionSelectionResult;
import de.dfki.lt.tr.dialmanagement.data.Observation;
import de.dfki.lt.tr.dialmanagement.data.actions.AbstractAction;
import de.dfki.lt.tr.dialmanagement.data.policies.DialoguePolicy;
import de.dfki.lt.tr.dialmanagement.utils.EpistemicObjectUtils;
import de.dfki.lt.tr.dialmanagement.utils.PolicyUtils;
import de.dfki.lt.tr.dialmanagement.utils.TextPolicyReader;


/**
 * Test class for a simple interaction with openings and closings
 * 
 * @author Pierre Lison (plison@ifi.uio.no)
 * @version 04/07/2010
 *
 */
public class DialogueManagerSecondTest {



	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = false;
	
	
	// the configuration files
	public static String POLICYFILE = "config/policies/testing/policy2.txt";
	public static String OBSFILE = "config/policies/testing/conditions2.txt";
	public static String ACTIONSFILE = "config/policies/testing/actions2.txt";

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
	 * Test a greeting
	 * 
	 * @throws DialogueException
	 * @throws InterruptedException 
	 */
	@Test
	public void testPolicyGreeting() throws DialogueException, InterruptedException {
		
		CommunicativeIntention intent = 
			EpistemicObjectUtils.createSimpleCommunicativeIntention("<state>(engagement-open ^ <agent>human ^ <agent>robot)");
		ActionSelectionResult r = manager.updateStateAndSelectAction(intent);
		AbstractAction action1 = r.getActions().get(0);
		assertEquals("CI[<state>(engagement-open ^ <agent>(robot) ^ <agent>(human))]", action1.toString());	
	}

	/**
	 * Test a closing
	 * 
	 * @throws DialogueException
	 * @throws InterruptedException 
	 */
	@Test
	public void testPolicyClosing1() throws DialogueException, InterruptedException {
		testPolicyGreeting();
		CommunicativeIntention intent = 
			EpistemicObjectUtils.createSimpleCommunicativeIntention("<state>(engagement-closed ^ <agent>human ^ <agent>robot)");
		ActionSelectionResult r = manager.updateStateAndSelectAction(intent);
		AbstractAction action1 = r.getActions().get(0);
		assertEquals( "CI[<state>(engagement-closed ^ <agent>(robot) ^ <agent>(human))]", action1.toString());
	}
	
	/**
	 * Test a failed closing (without first engaging with the user)
	 * 
	 * @throws DialogueException
	 * @throws InterruptedException 
	 */
	@Test
	public void testPolicyClosing2() throws DialogueException, InterruptedException {
		CommunicativeIntention intent = 
			EpistemicObjectUtils.createSimpleCommunicativeIntention("<state>(engagement-closed ^ <agent>human ^ <agent>robot)");
		ActionSelectionResult r = manager.updateStateAndSelectAction(intent);
		assertTrue(r.isVoid());
	}
	
	/**
	 * Test a recognition error
	 * 
	 * @throws DialogueException
	 * @throws InterruptedException 
	 */
	@Test
	public void testPolicyRecognitionError() throws DialogueException, InterruptedException {
		log("START TESTING RECOGNITION ERROR");
		Event event = EpistemicObjectUtils.createSimpleEvent("recognition-error", 0.8f);
		ActionSelectionResult r = manager.updateStateAndSelectAction(event);
		AbstractAction action1 = r.getActions().get(0);
		assertEquals("CI[<state>(error-reported)]", action1.toString());
		log("END TESTING RECOGNITION ERROR");
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
