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

import org.junit.Test;


import org.junit.Before;

import de.dfki.lt.tr.beliefs.slice.logicalcontent.UnknownFormula;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.data.Observation;
import de.dfki.lt.tr.dialmanagement.data.policies.DialoguePolicy;
import de.dfki.lt.tr.dialmanagement.data.policies.PolicyAction;
import de.dfki.lt.tr.dialmanagement.utils.FormulaUtils;
import de.dfki.lt.tr.dialmanagement.utils.TextPolicyReader;


/**
 * TODO: tests with multiple arguments
 * 
 * @author plison
 *
 */
public class DialogueManagerWithArgumentsTest {

	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = true;

	// the configuration files
	public static String POLICYFILE = "subarchitectures/dialogue.sa/config/policies/testing/policy5.txt";
	public static String OBSFILE = "subarchitectures/dialogue.sa/config/policies/testing/conditions5.txt";
	public static String ACTIONSFILE = "subarchitectures/dialogue.sa/config/policies/testing/actions5.txt";

	// the dialogue manager
	public DialogueManager manager;



	private Observation createSimpleObservation (String s) throws DialogueException {
		return createSimpleObservation(s, 1.0f);
	}


	private Observation createSimpleObservation (String s, float f) throws DialogueException {
		Observation intent = new Observation (Observation.INTENTION);
		intent.addAlternative(s, f);
		intent.addAlternative(new UnknownFormula(0), 1-f);
		return intent;
	}

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
	 * Test the policy with a single, high-confidence utterance
	 * @throws DialogueException
	 */
	@Test
	public void testPolicyBasic() throws DialogueException {

		Observation intent = createSimpleObservation("Say ^ <Object>(Blabla)");
		PolicyAction action1 = manager.nextAction(intent);
		assertEquals("CI[Said ^ <Object>(Blabla)]", action1.toString());

	}


	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("\"[dialmanager test\"] " + s);
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
