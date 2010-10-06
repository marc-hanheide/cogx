
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
 

package de.dfki.lt.tr.dialmanagement.utils;


//JUnit
import java.util.Random;

import org.junit.Before;
import org.junit.Test;

import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.data.DialoguePolicy;


import static org.junit.Assert.*;



public class PolicyFileReaderTest {

	public static String POLICYFILE = "subarchitectures/dialogue.sa/config/policies/simplepolicy.txt";
	public static String OBSFILE = "subarchitectures/dialogue.sa/config/policies/simpleobservations.txt";
	public static String ACTIONSFILE = "subarchitectures/dialogue.sa/config/policies/simpleactions.txt";

	
	@Test
	public void readSimplePolicy() throws DialogueException {
		
		DialoguePolicy policy = PolicyReader.constructPolicy(POLICYFILE, OBSFILE, ACTIONSFILE);
		
		policy.ensureWellFormedPolicy();
		
	}
}
