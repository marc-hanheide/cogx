
// =================================================================                                                        
// Copyright (C) 2010 Geert-Jan M. Kruijff / DFKI GmbH (gj@dfki.de)                                                               
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

/**
 * UNIT TEST for the PerceptBuilder class
 * 
 * The unit tests illustrate assumptions on method parameters, how to access the individual methods, 
 * and how to get access to information in the objects that the methods return. 
 * 
 */

// Package
package test.beliefmodels.builders;

// Remark: CAST dependencies should be removed!
// CAST
import beliefmodels.arch.BeliefException;
import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;

// Junit
import org.junit.Before;
import org.junit.Test;
import static org.junit.Assert.*;

// Belief models
import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.history.PerceptHistory;
import beliefmodels.builders.PerceptBuilder;

public class PerceptBuilderTest {

	@Before
	public void setUp() throws Exception {
	}
	
	@Test
	public void NewHistoryForNullAddressFails () { 
		try { 
			PerceptHistory phist = PerceptBuilder.createNewPerceptHistory(null);
			fail("Creating a history for a null address should fail"); 
		} catch (BeliefException be) {
			assertEquals("Error when creating belief history: cannot create history for null", 
					be.getMessage());
		}
	} // end test
	
	
	
	
	

} // end class
