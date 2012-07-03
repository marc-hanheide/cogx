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

import java.util.Arrays;

import org.junit.Test;
import org.junit.Before;


import de.dfki.lt.tr.beliefs.slice.intentions.CommunicativeIntention;
import de.dfki.lt.tr.beliefs.slice.intentions.Intention;
import de.dfki.lt.tr.beliefs.slice.intentions.IntentionalContent;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.data.ActionSelectionResult;
import de.dfki.lt.tr.dialmanagement.data.actions.AbstractAction;
import de.dfki.lt.tr.dialmanagement.data.actions.IntentionAction;
import de.dfki.lt.tr.dialmanagement.data.policies.DialoguePolicy;
import de.dfki.lt.tr.dialmanagement.utils.FormulaUtils;
import de.dfki.lt.tr.dialmanagement.utils.EpistemicObjectUtils;
import de.dfki.lt.tr.dialmanagement.utils.TextPolicyReader;


/**
 * Test class for an interaction with full epistemic objects (events 
 * and intentions)
 * 
 * @author Pierre Lison (plison@ifi.uio.no)
 * @version 04/07/2010
 *
 */
public class DialogueManagerWithEpObjectTest {


	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = false;
	
	
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
	 * Test the policy with a simple intention
	 * 
	 * @throws DialogueException
	 * @throws InterruptedException 
	 */
	@Test
	public void testPolicyWithSimpleIntention() throws DialogueException, InterruptedException {
			
		dFormula postCondition = FormulaUtils.constructFormula("Please find the cornflakes box");
		IntentionalContent intent = 
			EpistemicObjectUtils.createIntentionalContent(postCondition, EpistemicObjectUtils.robotAgent, 1.0f);
			
		CommunicativeIntention intention = new CommunicativeIntention (new Intention(
				EpistemicObjectUtils.curFrame, EpistemicObjectUtils.attributedStatus, "", Arrays.asList(intent)));
		
		ActionSelectionResult r = manager.updateStateAndSelectAction(intention);
		AbstractAction action1 = r.getActions().get(0);
		assertEquals("CI[okay, searching for the cornflakes box!]", action1.toString());
	}
	 


	/**
	 * Test the policy with an uncertain intention
	 * 
	 * @throws DialogueException
	 * @throws InterruptedException 
	 */
	@Test
	public void testPolicyWithUncertainIntentions() throws DialogueException, InterruptedException {
		
		dFormula postCondition1 = FormulaUtils.constructFormula("\"bla bla bla\"");
		IntentionalContent intent1 = 
			EpistemicObjectUtils.createIntentionalContent(postCondition1, EpistemicObjectUtils.robotAgent, 0.7f);
		
		dFormula postCondition2 = FormulaUtils.constructFormula("\"Please find the cornflakes box\"");
		IntentionalContent intent2 = 
			EpistemicObjectUtils.createIntentionalContent(postCondition2, EpistemicObjectUtils.robotAgent, 0.3f);
			
		CommunicativeIntention intention = new CommunicativeIntention (new Intention(
				EpistemicObjectUtils.curFrame, EpistemicObjectUtils.attributedStatus, "", Arrays.asList(intent1, intent2)));
		
		ActionSelectionResult r = manager.updateStateAndSelectAction(intention);
		AbstractAction action1 = r.getActions().get(0);
		assertEquals("CI[sorry, should I search for the cornflaxes box?]", action1.toString());
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
