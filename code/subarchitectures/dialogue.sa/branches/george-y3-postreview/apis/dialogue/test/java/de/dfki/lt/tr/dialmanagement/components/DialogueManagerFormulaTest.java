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

import org.junit.Test;


import org.junit.Before;

import de.dfki.lt.tr.beliefs.slice.intentions.CommunicativeIntention;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.UnknownFormula;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.data.ActionSelectionResult;
import de.dfki.lt.tr.dialmanagement.data.Observation;
import de.dfki.lt.tr.dialmanagement.data.actions.AbstractAction;
import de.dfki.lt.tr.dialmanagement.data.policies.DialoguePolicy;
import de.dfki.lt.tr.dialmanagement.utils.EpistemicObjectUtils;
import de.dfki.lt.tr.dialmanagement.utils.PolicyUtils;
import de.dfki.lt.tr.dialmanagement.utils.TextPolicyReader;


/**
 * Test class for an interaction using complex and modal formulae
 * 
 * @author Pierre Lison (plison@ifi.uio.no)
 * @version 04/07/2010
 *
 */
public class DialogueManagerFormulaTest {


	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = false;
	
	
	// the configuration files
	// the configuration files
	public static String POLICYFILE = "config/policies/testing/policy4.txt";
	public static String CONDFILE = "config/policies/testing/conditions4.txt";
	public static String ACTIONSFILE = "config/policies/testing/actions4.txt";

	protected DialogueManager manager ;
	
	
	
	/**
	 * Construct the policy based on the configuration files
	 * 
	 * @throws DialogueException if the files are ill-formatted
	 */
	@Before
	public void startDialogueManager() throws DialogueException {
		DialoguePolicy policy = TextPolicyReader.constructPolicy(POLICYFILE,CONDFILE, ACTIONSFILE);
		manager = new DialogueManager(policy);
	}
	
	


	/**
	 * Test the policy with an observation containing complex and modal formulae
	 * 
	 * @throws DialogueException if test fails
	 * @throws InterruptedException 
	 */
	@Test
	public void testPolicyDirect1() throws DialogueException, InterruptedException {
		
		CommunicativeIntention intent = 
			EpistemicObjectUtils.createSimpleCommunicativeIntention("<Belief>(<Ref>context1_1 ^ <ObjectType>ball)");
		ActionSelectionResult r = manager.updateStateAndSelectAction(intent);
		AbstractAction action1 = r.getActions().get(0);
		assertEquals("CI[okay]", action1.toString());
		
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
