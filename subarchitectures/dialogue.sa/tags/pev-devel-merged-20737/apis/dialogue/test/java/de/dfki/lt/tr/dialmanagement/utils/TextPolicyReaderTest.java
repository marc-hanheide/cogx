
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

import org.junit.Test;

import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.data.policies.DialoguePolicy;


/**
 * Test for reading a text policy file
 * 
 * @author Pierre Lison (plison@ifi.uio.no)
 * @version 09/10/2010
 */
public class TextPolicyReaderTest {

	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = false;
	
	public static String POLICYFILE = "config/policies/testing/policy1.txt";
	public static String OBSFILE = "config/policies/testing/conditions1.txt";
	public static String ACTIONSFILE = "config/policies/testing/actions1.txt";

	
	/**
	 * Reads a simple policy
	 * 
	 * @throws DialogueException
	 */
	@Test
	public void readSimplePolicy() throws DialogueException {
		
		DialoguePolicy policy = TextPolicyReader.constructPolicy(POLICYFILE, OBSFILE, ACTIONSFILE);	
		assertTrue(policy.isWellFormedPolicy())	;
	}
	
	
	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[textpolicyreadertest] " + s);
		}
	}
	
	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[textpolicyreadertest] " + s);
		}
	}
}
