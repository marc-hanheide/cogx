
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
import de.dfki.lt.tr.dialmanagement.utils.PolicyUtils;
import de.dfki.lt.tr.dialmanagement.utils.TextPolicyReader;

/**
 * Test class for the dialogue manager, with simple shallow observations 
 * and actions
 *  
 * @author Pierre Lison (plison@dfki.de)
 * @version 04/07/2010
 *
 */
public class DialogueManagerBasicTest {

	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = true;
	
	// the configuration files
	public static String POLICYFILE = "subarchitectures/dialogue.sa/config/policies/testing/policy1.txt";
	public static String CONDFILE = "subarchitectures/dialogue.sa/config/policies/testing/conditions1.txt";
	public static String ACTIONSFILE = "subarchitectures/dialogue.sa/config/policies/testing/actions1.txt";

	// the dialogue manager
	public DialogueManager manager;
	
	
	
	/**
	 * Construct the policy based on the configuration files
	 * 
	 * @throws DialogueException if the files are ill-formatted
	 */
	@Before
	public void constructPolicy() throws DialogueException {
		
		DialoguePolicy policy = TextPolicyReader.constructPolicy(POLICYFILE, CONDFILE, ACTIONSFILE);	
		policy.ensureWellFormedPolicy();	
		manager = new DialogueManager(policy);
	}
	
	
	
	/**
	 * Test the policy with a simple traversal
	 * 
	 * @throws DialogueException if test fails
	 */
	@Test
	public void testPolicy() throws DialogueException {
		
		debug("init node" + manager.getCurrentAction().toString());
		Observation obs1 = PolicyUtils.createSimpleObservation("one");
		debug("observing " + obs1);
		manager.nextAction(obs1);
		debug("cur action: " + manager.getCurrentAction());
		Observation obs4 = PolicyUtils.createSimpleObservation("four");
		debug("observing " + obs4);
		manager.nextAction(obs4);
		debug("cur action: " + manager.getCurrentAction());
		assertTrue(manager.isFinished());
	}
	
	  
	/**
	 * Test the dialogue manager with an observation not on the list
	 * 
	 * @throws DialogueException if test fails
	 */
	@Test
	public void testWrongPolicy() throws DialogueException {
		
		debug("init node" + manager.getCurrentAction().toString());
		Observation obs1 = PolicyUtils.createSimpleObservation("six");	
		assertTrue (manager.nextAction(obs1).isVoid());
		
		
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
