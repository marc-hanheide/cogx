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

import java.util.Arrays;
import java.util.HashMap;

import org.junit.Test;
import org.junit.Before;


import de.dfki.lt.tr.beliefs.slice.intentions.CommunicativeIntention;
import de.dfki.lt.tr.beliefs.slice.intentions.Intention;
import de.dfki.lt.tr.beliefs.slice.intentions.IntentionalContent;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.UnknownFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.data.policies.DialoguePolicy;
import de.dfki.lt.tr.dialmanagement.data.policies.PolicyAction;
import de.dfki.lt.tr.dialmanagement.utils.FormulaUtils;
import de.dfki.lt.tr.dialmanagement.utils.EpistemicObjectUtils;
import de.dfki.lt.tr.dialmanagement.utils.TextPolicyReader;


/**
 * Test class for an interaction with full epistemic objects (events 
 * and intentions)
 * 
 * @author Pierre Lison (plison@dfki.de)
 * @version 04/07/2010
 *
 */
public class DialogueManagerWithEpObjectTest {

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
	 * Test the policy with a simple intention
	 * 
	 * @throws DialogueException
	 */
	@Test
	public void testPolicyWithSimpleIntention() throws DialogueException {
			
		dFormula postCondition = FormulaUtils.constructFormula("\"Please find the cornflakes box\"");
		IntentionalContent intent = 
			EpistemicObjectUtils.createIntentionalContent(postCondition, EpistemicObjectUtils.robotAgent, 1.0f);
			
		CommunicativeIntention intention = new CommunicativeIntention (new Intention(
				EpistemicObjectUtils.curFrame, EpistemicObjectUtils.attributedStatus, "", Arrays.asList(intent)));
		
		PolicyAction action1 = manager.nextAction(intention);
		assertEquals(new PolicyAction("", FormulaUtils.constructFormula("\"okay, searching for the cornflakes box!\"")), action1);
	}
	 
	 
	/**
	 * Test the policy with an uncertain intention
	 * 
	 * @throws DialogueException
	 */
	@Test
	public void testPolicyWithUncertainIntentions() throws DialogueException {
		
		dFormula postCondition1 = FormulaUtils.constructFormula("\"bla bla bla\"");
		IntentionalContent intent1 = 
			EpistemicObjectUtils.createIntentionalContent(postCondition1, EpistemicObjectUtils.robotAgent, 0.7f);
		
		dFormula postCondition2 = FormulaUtils.constructFormula("\"Please find the cornflakes box\"");
		IntentionalContent intent2 = 
			EpistemicObjectUtils.createIntentionalContent(postCondition2, EpistemicObjectUtils.robotAgent, 0.3f);
			
		CommunicativeIntention intention = new CommunicativeIntention (new Intention(
				EpistemicObjectUtils.curFrame, EpistemicObjectUtils.attributedStatus, "", Arrays.asList(intent1, intent2)));
		
		PolicyAction action1 = manager.nextAction(intention);
		assertEquals(new PolicyAction("", FormulaUtils.constructFormula("\"sorry, should I search for the cornflaxes box?\"")), action1);
	}
	
	
	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[dialmanager_withepobjectstest] " + s);
		}
	}
	
	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[dialmanager_withepobjectstest] " + s);
		}
	}
}
