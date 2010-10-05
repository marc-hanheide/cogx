
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

import org.junit.Test;


import org.junit.Before;

import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.data.DialoguePolicy;
import de.dfki.lt.tr.dialmanagement.data.actions.VoidAction;
import de.dfki.lt.tr.dialmanagement.data.observations.Observation;
import de.dfki.lt.tr.dialmanagement.utils.PolicyReader;

/**
 * Test class for the dialogue manager, with simple shallow observations and actions
 * 
 * @author Pierre Lison (plison@dfki.de)
 * @version 04/07/2010
 *
 */
public class DialogueManagerTest {


	public static boolean LOGGING = true;
	
	public static boolean DEBUG = false;
	
	// the configuration files
	public static String POLICYFILE = "subarchitectures/dialogue.sa/config/policies/simplepolicy.txt";
	public static String OBSFILE = "subarchitectures/dialogue.sa/config/policies/simpleobservations.txt";
	public static String ACTIONSFILE = "subarchitectures/dialogue.sa/config/policies/simpleactions.txt";

	// the dialogue manager
	public DialogueManager manager;
	
	/**
	 * Construct the policy based on the configuration files
	 * 
	 * @throws DialogueException if the files are ill-formatted
	 */
	@Before
	public void constructPolicy() throws DialogueException {
		
		DialoguePolicy policy = PolicyReader.constructPolicy(POLICYFILE, OBSFILE, ACTIONSFILE);
		
		policy.ensureWellFormedPolicy();
		
		manager = new DialogueManager(policy);
	}
	
	
	/**
	 * Test the policy with a simple traversal
	 * @throws DialogueException
	 */
	@Test
	public void testPolicy() throws DialogueException {
		
		log("init node" + manager.getCurrentAction().toString());
		Observation obs1 = new Observation("one", 1.0f);
		log("observing " + obs1);
		manager.nextAction(obs1);
		log("cur action: " + manager.getCurrentAction());
		Observation obs4 = new Observation("four", 1.0f);
		log("observing " + obs4);
		manager.nextAction(obs4);
		log("cur action: " + manager.getCurrentAction());
		assertTrue(manager.isFinished());
	}
	
	  
	/**
	 * Test the dialogue manager with an observation not on the list
	 * 
	 */
	@Test
	public void testWrongPolicy() throws DialogueException {
		
		log("init node" + manager.getCurrentAction().toString());
	
		Observation obs1 = new Observation("six",1.0f);
		
		assertTrue (manager.nextAction(obs1) instanceof VoidAction);
		
		
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
