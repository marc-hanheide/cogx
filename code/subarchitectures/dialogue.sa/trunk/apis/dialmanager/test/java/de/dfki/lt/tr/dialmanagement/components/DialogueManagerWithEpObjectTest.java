// =================================================================                                                        
// Copyright (C) 2009-2011 Pierre Lison (pierre.lison@dfki.de)                                                                
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

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;

import org.junit.Test;


import org.junit.Before;


import de.dfki.lt.tr.beliefs.slice.intentions.Intention;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.data.DialoguePolicy;
import de.dfki.lt.tr.dialmanagement.data.actions.AbstractAction;
import de.dfki.lt.tr.dialmanagement.data.actions.IntentionAction;
import de.dfki.lt.tr.dialmanagement.utils.FormulaUtils;
import de.dfki.lt.tr.dialmanagement.utils.EpistemicObjectUtils;
import de.dfki.lt.tr.dialmanagement.utils.PolicyReader;


/**
 * Test class for a Dora-style interactions with full epistemic objects (events and 
 * intentions)
 * 
 * @author Pierre Lison (plison@dfki.de)
 * @version 04/07/2010
 *
 */
public class DialogueManagerWithEpObjectTest {


	public static boolean LOGGING = true;
	
	public static boolean DEBUG = false;
	
	// the configuration files
	public static String POLICYFILE = "apis/dialmanager/config/policyExampleDora.txt";
	public static String OBSFILE = "apis/dialmanager/config/observationsDora.txt";
	public static String ACTIONSFILE = "apis/dialmanager/config/actionsDora.txt";

	// the dialogue manager
	public DialogueManager manager;
	
	
	/**
	 * Construct the dialogue policy
	 * 
	 * @throws DialogueException if the configuration files are not well-formatted
	 */
	@Before
	public void constructPolicy() throws DialogueException {
		
		DialoguePolicy policy = PolicyReader.constructPolicy(POLICYFILE, OBSFILE, ACTIONSFILE);
		
		policy.ensureWellFormedPolicy();
		
		manager = new DialogueManager(policy);
	}
	
	
	@Test
	public void testPolicyWithSimpleIntention() throws DialogueException {
			
		Intention intention1 = EpistemicObjectUtils.createSimpleAttributedIntention(
				FormulaUtils.constructFormula("\"Please find the cornflakes box\""), 0.8f);
		
		AbstractAction action1 = manager.nextAction(intention1);
		assertEquals(action1, new IntentionAction(FormulaUtils.constructFormula("\"okay, searching for the cornflakes box!\"")));
	}
	
	
	@Test
	public void testPolicyWithUncertainIntentions() throws DialogueException {
		
		HashMap<dFormula, Float> postconditions = new HashMap<dFormula,Float>();
		postconditions.put(FormulaUtils.constructFormula("\"bla bla bla\""), 0.7f);
		postconditions.put(FormulaUtils.constructFormula("\"Please find the cornflakes box\""), 0.3f);
		
		Intention intention1 = EpistemicObjectUtils.createAttributedIntention(postconditions);
		AbstractAction action1 = manager.nextAction(intention1);
		assertEquals(action1, new IntentionAction(FormulaUtils.constructFormula("\"sorry, should I search for the cornflaxes box?\"")));
	}
	
	
	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[dialmanager test] " + s);
		}
	}
	
	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[dialmanager test] " + s);
		}
	}
}
