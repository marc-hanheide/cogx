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

import org.junit.Before;
import org.junit.Test;

import de.dfki.lt.tr.beliefs.slice.intentions.CommunicativeIntention;
import de.dfki.lt.tr.beliefs.slice.intentions.Intention;
import de.dfki.lt.tr.beliefs.slice.intentions.IntentionalContent;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.data.ActionSelectionResult;
import de.dfki.lt.tr.dialmanagement.data.actions.AbstractAction;
import de.dfki.lt.tr.dialmanagement.data.actions.IntentionAction;
import de.dfki.lt.tr.dialmanagement.data.policies.DialoguePolicy;
import de.dfki.lt.tr.dialmanagement.utils.EpistemicObjectUtils;
import de.dfki.lt.tr.dialmanagement.utils.FormulaUtils;
import de.dfki.lt.tr.dialmanagement.utils.XMLPolicyReader;

/**
 * Tests for the yr2 CogX review meeting
 * 
 * @author Pierre Lison (plison@ifi.uio.no)
 * @version 11/10/2010
 *
 */
public class DialogueManagerYr2Test {


	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = false;
	
	// the configuration files
	public static String POLICYFILE = "config/policies/yr2/fullpolicy.xml";

	// the dialogue manager
	public DialogueManager manager;
	
	
	

	/**
	 * Construct the policy based on the configuration files
	 * 
	 * @throws DialogueException if the files are ill-formatted
	 */
	@Before
	public void startDialogueManager() throws DialogueException {
		DialoguePolicy policy = XMLPolicyReader.constructPolicy(POLICYFILE);
		manager = new DialogueManager(policy);
	}
	  
	
	/**
	 * where forwarding shouldn't fire
	 * 
	 * @throws DialogueException
	 * @throws InterruptedException 
	 */
	@Test
	public void blockforwarding() throws DialogueException, InterruptedException {
		
		dFormula formula = FormulaUtils.constructFormula("<belief>(anything ^ here)");
		
		IntentionalContent intent = 
			EpistemicObjectUtils.createIntentionalContent(formula, EpistemicObjectUtils.robotAgent, 1.0f);
			
		CommunicativeIntention intention = new CommunicativeIntention (new Intention(
				EpistemicObjectUtils.curFrame, EpistemicObjectUtils.attributedStatus, "", Arrays.asList(intent)));	

		ActionSelectionResult r = manager.updateStateAndSelectAction(intention);

		if (!r.isVoid()) {
		AbstractAction action = r.getActions().get(0);
		log ("selected action: " + action);
		assertNotSame(new IntentionAction("", new ModalFormula(0,"post",formula)), action);
	}
	}
	
	
	/**
	 * Intention forwarding with pre- and post-conditions
	 * 
	 * @throws DialogueException
	 * @throws InterruptedException 
	 */
//	@Test
	public void fullForwarding() throws DialogueException, InterruptedException {
		
		dFormula formula = FormulaUtils.constructFormula("<pre>(blabla) ^ <post>(<belief>(<lingref>(ball1_2) " + 
				" ^ <color>(red) ^ <objecttype>(ball) ^ <ref>([dialogue:4:7])))");
				
		IntentionalContent intent = 
			EpistemicObjectUtils.createIntentionalContent(formula, EpistemicObjectUtils.robotAgent, 1.0f);
		
		CommunicativeIntention intention = new CommunicativeIntention (new Intention(
				EpistemicObjectUtils.curFrame, EpistemicObjectUtils.attributedStatus, "", Arrays.asList(intent)));	

		ActionSelectionResult r = manager.updateStateAndSelectAction(intention);

		AbstractAction action = r.getActions().get(0);
		log ("selected action: " + action);
		
		dFormula expectedreply = FormulaUtils.constructFormula("<pre>(blabla) ^ <post>(<belief>[dialogue:4:7])");
		IntentionAction expectedAction = new IntentionAction("", expectedreply);
		expectedAction.setStatus(IntentionAction.ATTRIBUTED);
		assertEquals(expectedAction, action);
	}
	
	
//	@Test
	public void answeringQuestion () throws DialogueException, InterruptedException {
		
		dFormula formula = FormulaUtils.constructFormula("<pre>(<belief>[binder:8:7]) ^ <post>(<state>" + 
				"(question-answered ^ <agent>(self) ^ <about>(<lingref>ball1_2 ^ <color>red " + 
				"^ <objecttype>ball ^ <ref>[binder:3:7]) ^ <feature>color))");
		
		IntentionalContent intent = 
			EpistemicObjectUtils.createIntentionalContent(formula, EpistemicObjectUtils.robotAgent, 1.0f);
		
		CommunicativeIntention intention = new CommunicativeIntention (new Intention(
				EpistemicObjectUtils.curFrame, EpistemicObjectUtils.attributedStatus, "", Arrays.asList(intent)));	

		ActionSelectionResult r = manager.updateStateAndSelectAction(intention);

		AbstractAction action = r.getActions().get(0);
		log ("selected action: " + action);
		
		dFormula expectedreply = FormulaUtils.constructFormula("<pre>(<belief>[binder:8:7]) ^ <post>(<state>(grounded " +
				" ^ <about>[binder:3:7] ^ <content>(<color>red)))");
		assertEquals(new IntentionAction("", expectedreply), action);
		
	}
	
//	@Test
	public void answeringQuestion2() throws DialogueException, InterruptedException {
		
		dFormula formula = FormulaUtils.constructFormula("<pre>(<belief>[binder:5:7]) ^ <post>(" + 
			"<state>(question-answered ^ <agent>self ^ <about>(<lingref>ball1_1 ^ <color>blue "  + 
			" ^ <objecttype>ball ^ <ref>[binder:0:7]) ^ <feature>color ^ <hypo>blue))");
		

		IntentionalContent intent = 
			EpistemicObjectUtils.createIntentionalContent(formula, EpistemicObjectUtils.robotAgent, 1.0f);
		
		CommunicativeIntention intention = new CommunicativeIntention (new Intention(
				EpistemicObjectUtils.curFrame, EpistemicObjectUtils.attributedStatus, "", Arrays.asList(intent)));	

		ActionSelectionResult r = manager.updateStateAndSelectAction(intention);

		AbstractAction action = r.getActions().get(0);
		log ("selected action: " + action);
		
	//	assertEquals(new AbstractAction("", expectedreply), action);
	}
	
	
	@Test
	public void sayHello() throws DialogueException, InterruptedException {
		dFormula formula = FormulaUtils.constructFormula("<state>(engagement-open ^ <agent>(human) ^ <agent>(self))");
		
		IntentionalContent intent = 
			EpistemicObjectUtils.createIntentionalContent(formula, EpistemicObjectUtils.robotAgent, 1.0f);
		
		CommunicativeIntention intention = new CommunicativeIntention (new Intention(
				EpistemicObjectUtils.curFrame, EpistemicObjectUtils.attributedStatus, "", Arrays.asList(intent)));	

		ActionSelectionResult r = manager.updateStateAndSelectAction(intention);

		AbstractAction action = r.getActions().get(0);
		log ("selected action: " + action);
		
		dFormula expectedreply = FormulaUtils.constructFormula("<state>(engagement-open ^ <agent>(self) ^ <agent>(human))");
		assertEquals(new IntentionAction("", expectedreply), action);
	}
	
	
	@Test
	public void sayGoodbye() throws DialogueException, InterruptedException {
	
		sayHello();
		
		dFormula formula = FormulaUtils.constructFormula("<state>(engagement-closed ^ <agent>(human) ^ <agent>(self))");
		
		IntentionalContent intent = 
			EpistemicObjectUtils.createIntentionalContent(formula, EpistemicObjectUtils.robotAgent, 1.0f);
		
		CommunicativeIntention intention = new CommunicativeIntention (new Intention(
				EpistemicObjectUtils.curFrame, EpistemicObjectUtils.attributedStatus, "", Arrays.asList(intent)));	

		ActionSelectionResult r = manager.updateStateAndSelectAction(intention);

		AbstractAction action = r.getActions().get(0);
		log ("selected action: " + action);
		
		dFormula expectedreply = FormulaUtils.constructFormula("<state>(engagement-closed ^ <agent>(self) ^ <agent>(human))");
		assertEquals(new IntentionAction("", expectedreply), action);
	}
	
	
	@Test
	public void sayingThankyou() throws DialogueException, InterruptedException {
		
		sayHello();
		
		dFormula formula = FormulaUtils.constructFormula("<state>(appreciated ^ <agent>(human) ^ <patient>(self))");
		
		IntentionalContent intent = 
			EpistemicObjectUtils.createIntentionalContent(formula, EpistemicObjectUtils.robotAgent, 1.0f);
		
		CommunicativeIntention intention = new CommunicativeIntention (new Intention(
				EpistemicObjectUtils.curFrame, EpistemicObjectUtils.attributedStatus, "", Arrays.asList(intent)));	

		ActionSelectionResult r = manager.updateStateAndSelectAction(intention);

		AbstractAction action = r.getActions().get(0);
		log ("selected action: " + action);
		
		dFormula expectedreply = FormulaUtils.constructFormula("<state>(thanked ^ <agent>(human) ^ <patient>(self))");
		assertEquals(new IntentionAction("", expectedreply), action);		
	}
	
	
	// test for saying thank you


	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[dialoguemanageryr2test] " + s);
		}
	}
	
	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[formulautilstest] " + s);
		}
	}

	
}
