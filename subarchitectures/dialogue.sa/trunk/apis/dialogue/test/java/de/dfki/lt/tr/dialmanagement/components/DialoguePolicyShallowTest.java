
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

import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.data.ActionSelectionResult;
import de.dfki.lt.tr.dialmanagement.data.actions.AbstractAction;
import de.dfki.lt.tr.dialmanagement.data.policies.DialoguePolicy;
import de.dfki.lt.tr.dialmanagement.utils.XMLPolicyReader;
import de.dfki.lt.tr.dialogue.slice.asr.PhonString;

/**
 * Test class for the dialogue manager, with simple shallow observations 
 * and actions
 *  
 * @author Pierre Lison (plison@ifi.uio.no)
 * @version 04/07/2010
 *
 */
public class DialoguePolicyShallowTest {

	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = false;
	
	// the configuration files
	public static String POLICYFILE = "config/policies/testing/policy7.xml";

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
	
	
	 
	@Test
	public void testPolicy1() throws DialogueException, InterruptedException {
		
		ActionSelectionResult r = manager.updateStateAndSelectAction(1500);

		AbstractAction action = r.getActions().get(0);
		
		PhonString phon = new PhonString("", "Merlijn", 4, 0.8f, 0.9f, 1, false, null);
		ActionSelectionResult r2 = manager.updateStateAndSelectAction(Arrays.asList(phon));

		action = r2.getActions().get(0);
		assertEquals(action.toString(), "Wat een mooie naam");
		
		manager.updateStateAndSelectAction(1500);
		
		PhonString phon2 = new PhonString("", "fijn", 4, 0.8f, 0.9f, 1, false, null);
		ActionSelectionResult r4 = manager.updateStateAndSelectAction(Arrays.asList(phon2));

		AbstractAction action2 = r4.getActions().get(0);
		assertEquals(action2.toString(), "Dat is mooi");
	}
	 
	  
	
	@Test
	public void testPolicy2() throws DialogueException, InterruptedException {
		
		manager.updateStateAndSelectAction(1500);
		
		PhonString phon = new PhonString("", "Diane", 4, 0.8f, 0.9f, 1, false, null);
		ActionSelectionResult r2 = manager.updateStateAndSelectAction(Arrays.asList(phon));

		AbstractAction action = r2.getActions().get(0);
		assertEquals(action.toString(), "Wat een mooie naam");
		
		manager.updateStateAndSelectAction(1500);
		
		PhonString phon2 = new PhonString("", "slecht", 4, 0.8f, 0.9f, 1, false, null);
		ActionSelectionResult r4 = manager.updateStateAndSelectAction(Arrays.asList(phon2));

		AbstractAction action2 = r4.getActions().get(0);
		assertEquals(action2.toString(), "Dat is vervelend");
	}

	
	
	@Test
	public void testPolicy3() throws DialogueException, InterruptedException {
		
		ActionSelectionResult r = manager.updateStateAndSelectAction(1500);
		
		PhonString phon = new PhonString("", "Diane", 4, 0.8f, 0.9f, 1, false, null);
		ActionSelectionResult r2 = manager.updateStateAndSelectAction(Arrays.asList(phon));

		AbstractAction action = r2.getActions().get(0);
		assertEquals(action.toString(), "Wat een mooie naam");
		
		manager.updateStateAndSelectAction(1500);
		
		PhonString phon2 = new PhonString("", "bla bla bla", 4, 0.8f, 0.9f, 1, false, null);
		ActionSelectionResult r4 = manager.updateStateAndSelectAction(Arrays.asList(phon2));

		AbstractAction action2 = r4.getActions().get(0);
		assertEquals(action2.toString(), "Sorry ik heb je niet begrepen");
		
		manager.updateStateAndSelectAction(1500);
		
		PhonString phon3 = new PhonString("", "fijn", 4, 0.8f, 0.9f, 1, false, null);
		ActionSelectionResult r6 = manager.updateStateAndSelectAction(Arrays.asList(phon3));

		AbstractAction action3 = r6.getActions().get(0);
		assertEquals(action3.toString(), "Dat is mooi");
		
	}
	 
	
	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[dialmanager_basictest] " + s);
		}
	}
	
	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[dialmanager_basictest] " + s);
		}
	}


}
