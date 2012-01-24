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


package de.dfki.lt.tr.dialmanagement.utils;

import static org.junit.Assert.*;
import static org.junit.Assert.assertEquals;

import java.util.Arrays;

import org.junit.Before;
import org.junit.Test;

import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.components.DialogueManager;
import de.dfki.lt.tr.dialmanagement.data.ActionSelectionResult;
import de.dfki.lt.tr.dialmanagement.data.actions.AbstractAction;
import de.dfki.lt.tr.dialmanagement.data.actions.AlternativeAction;
import de.dfki.lt.tr.dialmanagement.data.policies.DialoguePolicy;
import de.dfki.lt.tr.dialmanagement.utils.FormulaUtils;
import de.dfki.lt.tr.dialmanagement.utils.TNOPolicyReader;
import de.dfki.lt.tr.dialogue.slice.asr.PhonString;

public class TNOPolicyReaderTest {

	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = false;
	
	String policyFile = "config/policies/tno/dialogiCat1.xml";
	

	// the dialogue manager
	public DialogueManager manager;
		
	
	
	/**
	 * Construct the policy based on the configuration files
	 * 
	 * @throws DialogueException if the files are ill-formatted
	 */
	@Before
	public void startDialogueManager() throws DialogueException {
		DialoguePolicy policy = TNOPolicyReader.constructPolicy(policyFile);
		manager = new DialogueManager(policy);
	}
	
	
	
	

	@Test(timeout = 1000)
	public void testPolicy1() throws DialogueException, InterruptedException {
		
		manager.updateStateAndSelectAction(2000);
		
		PhonString phon = new PhonString("", "Merlijn", 4, 0.8f, 0.9f, 1, false, null);
		ActionSelectionResult r = manager.updateStateAndSelectAction(Arrays.asList(phon));
		AbstractAction action =  r.getActions().get(0);
		assertEquals(action.toString(), "Wat een mooie naam*Happy*");
		
		manager.updateStateAndSelectAction(2000);
		
		PhonString phon2 = new PhonString("", "fijn", 4, 0.8f, 0.9f, 1, false, null);
		ActionSelectionResult r2 = manager.updateStateAndSelectAction(Arrays.asList(phon2));
		AbstractAction action2 =  r2.getActions().get(0);
		assertEquals(action2.toString(), "*Happy* Dat is mooi");
	}
	
	
	
	@Test(timeout = 1000)
	public void testPolicy2() throws DialogueException, InterruptedException {
		
		manager.updateStateAndSelectAction(2000);
		
		PhonString phon = new PhonString("", "Diane", 4, 0.8f, 0.9f, 1, false, null);
		ActionSelectionResult r = manager.updateStateAndSelectAction(Arrays.asList(phon));
		AbstractAction action =  r.getActions().get(0);
		assertEquals(action.toString(), "Wat een mooie naam*Happy*");
		
		manager.updateStateAndSelectAction(2000);
		
		PhonString phon2 = new PhonString("", "slecht", 4, 0.8f, 0.9f, 1, false, null);
		ActionSelectionResult r2 = manager.updateStateAndSelectAction(Arrays.asList(phon2));
		AbstractAction action2 =  r2.getActions().get(0);
		assertEquals(action2.toString(), "*Sad* Dat is vervelend");
	}

	
	
	@Test(timeout = 1000)
	public void testPolicy3() throws DialogueException, InterruptedException {
		
		manager.updateStateAndSelectAction(2000);
		
		PhonString phon = new PhonString("", "Diane", 4, 0.8f, 0.9f, 1, false, null);
		ActionSelectionResult r = manager.updateStateAndSelectAction(Arrays.asList(phon));
		AbstractAction action =  r.getActions().get(0);
		assertEquals(action.toString(), "Wat een mooie naam*Happy*");
		
		manager.updateStateAndSelectAction(2000);
		
		PhonString phon2 = new PhonString("", "bla bla bla", 4, 0.8f, 0.9f, 1, false, null);
		ActionSelectionResult r2 = manager.updateStateAndSelectAction(Arrays.asList(phon2));
		AbstractAction action2 =  r2.getActions().get(0);
		assertTrue(action2 instanceof AlternativeAction);
		String answer = ((AlternativeAction)action2).selectRandomAction().toString().trim();
		boolean correctAnswer = 
			(answer.equals("*NotUnderstood* Sorry ik heb je niet begrepen.") ||
					answer.equals("*NotUnderstood* Sorry, ik heb je niet goed verstaan."));
		assertTrue("provided answer: " + answer, correctAnswer);
	
		manager.updateStateAndSelectAction(2000);
		
		PhonString phon3 = new PhonString("", "fijn", 4, 0.8f, 0.9f, 1, false, null);
		ActionSelectionResult r3 = manager.updateStateAndSelectAction(Arrays.asList(phon3));
		AbstractAction action3 =  r3.getActions().get(0);
		assertEquals(action3.toString(), "*Happy* Dat is mooi");
		
	}
	
	
}
