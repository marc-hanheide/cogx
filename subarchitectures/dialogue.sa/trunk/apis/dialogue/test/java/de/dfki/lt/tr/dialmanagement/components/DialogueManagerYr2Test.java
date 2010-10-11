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

public class DialogueManagerYr2Test {


	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = true;
	
	// the configuration files
	public static String POLICYFILE = "subarchitectures/dialogue.sa/config/policies/yr2/basicforwardpolicy.xml";

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
	
	@Test
	public void ForwardingTest() throws DialogueException {
		
		dFormula formula = FormulaUtils.constructFormula("<belief>(anything ^ here)");
		
		IntentionalContent intent = 
			EpistemicObjectUtils.createIntentionalContent(formula, EpistemicObjectUtils.robotAgent, 1.0f);
			
		CommunicativeIntention intention = new CommunicativeIntention (new Intention(
				EpistemicObjectUtils.curFrame, EpistemicObjectUtils.attributedStatus, "", Arrays.asList(intent)));	

		PolicyAction action = manager.nextAction(intention);
		log ("selected action: " + action);
		assertEquals(new PolicyAction("", new ModalFormula(0,"post",formula)), action);
	}
	
	
	@Test
	public void HandlingPreconditions() throws DialogueException {
		
	}


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
