
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

import java.util.Arrays;

import org.junit.Before;
import org.junit.Test;
import static org.junit.Assert.*;


import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.data.ActionSelectionResult;
import de.dfki.lt.tr.dialmanagement.data.Observation;
import de.dfki.lt.tr.dialmanagement.data.actions.AbstractAction;
import de.dfki.lt.tr.dialmanagement.data.actions.DialStateAction;
import de.dfki.lt.tr.dialmanagement.data.policies.DialoguePolicy;

import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.UnknownFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.dialmanagement.components.DialogueManager;
import de.dfki.lt.tr.dialmanagement.utils.EpistemicObjectUtils;
import de.dfki.lt.tr.dialmanagement.utils.FormulaUtils;
import de.dfki.lt.tr.dialmanagement.utils.XMLPolicyReader;
import de.dfki.lt.tr.dialogue.slice.asr.PhonString;

public class DialogueManagerWithInformationStateTest {


	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = false;
	
	
	// the configuration file
	public final String POLICY_FILE = "config/policies/testing/policy8.xml";


	protected DialogueManager manager ;
	
	

	/**
	 * Construct the policy based on the configuration files
	 * 
	 * @throws DialogueException if the files are ill-formatted
	 */
	@Before
	public void startDialogueManager() throws DialogueException {
		DialoguePolicy policy = XMLPolicyReader.constructPolicy(POLICY_FILE);
		manager = new DialogueManager(policy);
		manager.updateStateAndSelectAction(0);
	}
	  
	
	@Test
	public void testWithoutInformation() throws DialogueException, InterruptedException {
		PhonString phon = new PhonString();
		phon.wordSequence = "What's your name?";
		phon.confidenceValue = 0.8f;
	
		manager.getDialogueState().removeInfoState("ROBOT_NAME");
		
		ActionSelectionResult r = manager.updateStateAndSelectAction(Arrays.asList(phon));
		assertTrue (r.isVoid());
	}
	

	@Test
	public void testWithInformation() throws DialogueException, InterruptedException {
		
		manager.getDialogueState().removeInfoState("ROBOT_NAME");
		dBelief nameInfo = EpistemicObjectUtils.createSimpleBelief(new ElementaryFormula(0,"Nao"), 0.99f, "ROBOT_NAME");
		manager.getDialogueState().addNewInfoState(nameInfo);
		
		PhonString phon = new PhonString();
		phon.wordSequence = "What's your name?";
		phon.confidenceValue = 0.8f;
		ActionSelectionResult r = manager.updateStateAndSelectAction(Arrays.asList(phon));
		AbstractAction action = r.getActions().get(0);
		assertTrue (action.asFormula() instanceof ElementaryFormula);
		assertEquals (action.toString(), "I'm Nao");
	}
	
	
	
	@Test
	public void testWithUserModel1() throws DialogueException, InterruptedException {
		testWithInformation();
		
		manager.getDialogueState().removeInfoState("USER_SPEED");
		dBelief nameInfo = EpistemicObjectUtils.createSimpleBelief(new ElementaryFormula(0,"slow"), 0.99f, "USER_SPEED");
		manager.getDialogueState().addNewInfoState(nameInfo);		
		ActionSelectionResult r = manager.updateStateAndSelectAction(1000);
		assertTrue(r.isVoid());
		
	}
	
	
	@Test
	public void testWithUserModel2() throws DialogueException, InterruptedException {
		testWithInformation();
		
		manager.getDialogueState().removeInfoState("USER_SPEED");
		dBelief nameInfo = EpistemicObjectUtils.createSimpleBelief(new ElementaryFormula(0,"fast"), 0.99f, "USER_SPEED");
		manager.getDialogueState().addNewInfoState(nameInfo);
		
		ActionSelectionResult r = manager.updateStateAndSelectAction(1000);	
		assertFalse(r.isVoid());	
	}
	

	
	@Test
	public void testWithUserModel3() throws DialogueException, InterruptedException {
		testWithInformation();
	
		manager.getDialogueState().removeInfoState("USER_SPEED");
		dBelief nameInfo = EpistemicObjectUtils.createSimpleBelief(new ElementaryFormula(0,"slow"), 0.99f, "USER_SPEED");
		manager.getDialogueState().addNewInfoState(nameInfo);
		
		ActionSelectionResult r = manager.updateStateAndSelectAction(2000);	
		assertEquals(r.getActions().get(0).toString(), "and what is yours?");
		
	}
	

	
	@Test
	public void testWithUserModel4() throws DialogueException, InterruptedException {
		testWithInformation();
		
		manager.getDialogueState().removeInfoState("USER_SPEED");
		dBelief nameInfo = EpistemicObjectUtils.createSimpleBelief(new ElementaryFormula(0,"fast"), 0.99f, "USER_SPEED");
		manager.getDialogueState().addNewInfoState(nameInfo);
		
		ActionSelectionResult r = manager.updateStateAndSelectAction(800);
		
		assertTrue(r.isVoid());
		
	}
	
	@Test
	public void testPostcondition1() throws DialogueException, InterruptedException {
		testWithInformation();
		
		manager.getDialogueState().removeInfoState("USER_SPEED");
		dBelief nameInfo = EpistemicObjectUtils.createSimpleBelief(new ElementaryFormula(0,"fast"), 0.99f, "USER_SPEED");
		manager.getDialogueState().addNewInfoState(nameInfo);
		
		ActionSelectionResult r = manager.updateStateAndSelectAction(1000);
		
		PhonString phon = new PhonString();
		phon.wordSequence="Pierre";
		phon.confidenceValue = 0.8f;
		manager.updateStateAndSelectAction(Arrays.asList(phon));

		ActionSelectionResult r2 = manager.updateStateAndSelectAction(0);		
		assertEquals(r2.getActions().get(0).getId(), "nextStep");
		
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
