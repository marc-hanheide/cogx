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
import de.dfki.lt.tr.dialmanagement.data.policies.DialoguePolicy;
import de.dfki.lt.tr.dialmanagement.data.policies.PolicyAction;
import de.dfki.lt.tr.dialmanagement.utils.EpistemicObjectUtils;
import de.dfki.lt.tr.dialmanagement.utils.FormulaUtils;
import de.dfki.lt.tr.dialmanagement.utils.XMLPolicyReader;

/**
 * Tests for the yr2 CogX review meeting
 * 
 * TODO: deal with polar questions
 * 
 * @author Pierre Lison (plison@dfki.de)
 * @version 11/10/2010
 *
 */
public class DialogueManagerYr2Test {


	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = true;
	
	// the configuration files
	public static String POLICYFILE = "subarchitectures/dialogue.sa/config/policies/yr2/fullpolicy.xml";

	// the dialogue manager
	public DialogueManager manager;
	
	
	
	/**
	 * Construct the policy based on the configuration files
	 * 
	 * @throws DialogueException if the files are ill-formatted
	 */
	@Before
	public void constructPolicy() throws DialogueException {
		
		DialoguePolicy policy = XMLPolicyReader.constructPolicy(POLICYFILE);	
		policy.ensureWellFormedPolicy();	
		manager = new DialogueManager(policy);
	}
	
	/**
	 * where forwarding shouldn't fire
	 * 
	 * @throws DialogueException
	 */
	@Test
	public void blockforwarding() throws DialogueException {
		
		dFormula formula = FormulaUtils.constructFormula("<belief>(anything ^ here)");
		
		IntentionalContent intent = 
			EpistemicObjectUtils.createIntentionalContent(formula, EpistemicObjectUtils.robotAgent, 1.0f);
			
		CommunicativeIntention intention = new CommunicativeIntention (new Intention(
				EpistemicObjectUtils.curFrame, EpistemicObjectUtils.attributedStatus, "", Arrays.asList(intent)));	

		PolicyAction action = manager.nextAction(intention);
		log ("selected action: " + action);
		assertNotSame(new PolicyAction("", new ModalFormula(0,"post",formula)), action);
	}
	
	
	/**
	 * Intention forwarding with pre- and post-conditions
	 * 
	 * @throws DialogueException
	 */
	@Test
	public void fullForwarding() throws DialogueException {
		
		dFormula formula = FormulaUtils.constructFormula("<pre>(blabla) ^ <post>(<belief>(<lingref>(ball1_2) " + 
				" ^ <color>(red) ^ <objecttype>(ball) ^ <ref>([dialogue:4:7])))");
				
		IntentionalContent intent = 
			EpistemicObjectUtils.createIntentionalContent(formula, EpistemicObjectUtils.robotAgent, 1.0f);
		
		CommunicativeIntention intention = new CommunicativeIntention (new Intention(
				EpistemicObjectUtils.curFrame, EpistemicObjectUtils.attributedStatus, "", Arrays.asList(intent)));	

		PolicyAction action = manager.nextAction(intention);
		log ("selected action: " + action);
		
		dFormula expectedreply = FormulaUtils.constructFormula("<pre>(blabla) ^ <post>(<belief>[dialogue:4:7])");
		PolicyAction expectedAction = new PolicyAction("", expectedreply);
		expectedAction.setType(PolicyAction.ATTRIBUTED_INTENTION);
		assertEquals(expectedAction, action);
	}
	
	
	@Test
	public void answeringQuestion () throws DialogueException {
		
		dFormula formula = FormulaUtils.constructFormula("<pre>(<belief>[binder:8:7]) ^ <post>(<state>" + 
				"(question-answered ^ <agent>(robot) ^ <about>(<lingref>ball1_2 ^ <color>red " + 
				"^ <objecttype>ball ^ <ref>[binder:3:7]) ^ <feature>color))");
		
		IntentionalContent intent = 
			EpistemicObjectUtils.createIntentionalContent(formula, EpistemicObjectUtils.robotAgent, 1.0f);
		
		CommunicativeIntention intention = new CommunicativeIntention (new Intention(
				EpistemicObjectUtils.curFrame, EpistemicObjectUtils.attributedStatus, "", Arrays.asList(intent)));	

		PolicyAction action = manager.nextAction(intention);
		log ("selected action: " + action);
		
		dFormula expectedreply = FormulaUtils.constructFormula("<pre>(<belief>[binder:8:7]) ^ <post>(<state>(grounded " +
				" ^ <about>[binder:3:7] ^ <content>(<color>red)))");
		assertEquals(new PolicyAction("", expectedreply), action);
		
	}
	
	@Test
	public void answeringQuestion2() throws DialogueException {
		
		dFormula formula = FormulaUtils.constructFormula("<pre>(<belief>[binder:5:7]) ^ <post>(" + 
			"<state>(question-answered ^ <agent>robot ^ <about>(<lingref>ball1_1 ^ <color>blue "  + 
			" ^ <objecttype>ball ^ <ref>[binder:0:7]) ^ <feature>color ^ <hypo>blue))");
		

		IntentionalContent intent = 
			EpistemicObjectUtils.createIntentionalContent(formula, EpistemicObjectUtils.robotAgent, 1.0f);
		
		CommunicativeIntention intention = new CommunicativeIntention (new Intention(
				EpistemicObjectUtils.curFrame, EpistemicObjectUtils.attributedStatus, "", Arrays.asList(intent)));	

		PolicyAction action = manager.nextAction(intention);
		log ("selected action: " + action);
		
	//	assertEquals(new PolicyAction("", expectedreply), action);
	}
	
	
	@Test
	public void sayHello() throws DialogueException {
		dFormula formula = FormulaUtils.constructFormula("<state>(engagement-open ^ <agent>(human) ^ <agent>(robot))");
		
		IntentionalContent intent = 
			EpistemicObjectUtils.createIntentionalContent(formula, EpistemicObjectUtils.robotAgent, 1.0f);
		
		CommunicativeIntention intention = new CommunicativeIntention (new Intention(
				EpistemicObjectUtils.curFrame, EpistemicObjectUtils.attributedStatus, "", Arrays.asList(intent)));	

		PolicyAction action = manager.nextAction(intention);
		log ("selected action: " + action);
		
		dFormula expectedreply = FormulaUtils.constructFormula("<state>(engagement-open ^ <agent>(robot) ^ <agent>(human))");
		assertEquals(new PolicyAction("", expectedreply), action);
	}
	
	
	@Test
	public void sayGoodbye() throws DialogueException {
	
		sayHello();
		
		dFormula formula = FormulaUtils.constructFormula("<state>(engagement-closed ^ <agent>(human) ^ <agent>(robot))");
		
		IntentionalContent intent = 
			EpistemicObjectUtils.createIntentionalContent(formula, EpistemicObjectUtils.robotAgent, 1.0f);
		
		CommunicativeIntention intention = new CommunicativeIntention (new Intention(
				EpistemicObjectUtils.curFrame, EpistemicObjectUtils.attributedStatus, "", Arrays.asList(intent)));	

		PolicyAction action = manager.nextAction(intention);
		log ("selected action: " + action);
		
		dFormula expectedreply = FormulaUtils.constructFormula("<state>(engagement-closed ^ <agent>(robot) ^ <agent>(human))");
		assertEquals(new PolicyAction("", expectedreply), action);
	}
	
	
	@Test
	public void sayingThankyou() throws DialogueException {
		
		sayHello();
		
		dFormula formula = FormulaUtils.constructFormula("<state>(appreciated ^ <agent>(human) ^ <patient>(robot))");
		
		IntentionalContent intent = 
			EpistemicObjectUtils.createIntentionalContent(formula, EpistemicObjectUtils.robotAgent, 1.0f);
		
		CommunicativeIntention intention = new CommunicativeIntention (new Intention(
				EpistemicObjectUtils.curFrame, EpistemicObjectUtils.attributedStatus, "", Arrays.asList(intent)));	

		PolicyAction action = manager.nextAction(intention);
		log ("selected action: " + action);
		
		dFormula expectedreply = FormulaUtils.constructFormula("<state>(thanked ^ <agent>(human) ^ <patient>(robot))");
		assertEquals(new PolicyAction("", expectedreply), action);		
	}
	
	
	// test for saying thank you


	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[formulautilstest] " + s);
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
