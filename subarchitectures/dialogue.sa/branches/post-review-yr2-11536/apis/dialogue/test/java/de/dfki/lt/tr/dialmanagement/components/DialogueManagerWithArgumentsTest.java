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

import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.data.Observation;
import de.dfki.lt.tr.dialmanagement.data.policies.DialoguePolicy;
import de.dfki.lt.tr.dialmanagement.data.policies.PolicyAction;
import de.dfki.lt.tr.dialmanagement.utils.PolicyUtils;
import de.dfki.lt.tr.dialmanagement.utils.TextPolicyReader;


/**
 * Test with a policy including underspecified arguments
 * 
 * @author Pierre Lison (plison@dfki.de)
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
	 * Test the policy with an observation filling a single argument
	 * 
	 * @throws DialogueException
	 */
	@Test
	public void testPolicyBasic() throws DialogueException {

		Observation intent = PolicyUtils.createSimpleObservation("Say ^ <Object>(Blabla)");
		PolicyAction action1 = manager.nextAction(intent);
		assertEquals("CI[Said ^ <Object>(Blabla)]", action1.toString());

	}
	

	/**
	 * Test the policy with an observation filling two arguments
	 * 
	 * @throws DialogueException
	 */
	@Test
	public void testPolicySecond() throws DialogueException {

		Observation intent = PolicyUtils.createSimpleObservation("Say ^ <Subject>(ahah ^ <oho>4.3f) ^ <Object>(Blabla)");
		PolicyAction action1 = manager.nextAction(intent);
		assertEquals("CI[Said ^ <Subject>(ahah ^ <oho>(4.3f)) ^ <Object>(Blabla)]", action1.toString());

	}


	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[dialmanager_withargumentstest] " + s);
		}
	}

	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[dialmanager_withargumentstest] " + s);
		}
	}
}
