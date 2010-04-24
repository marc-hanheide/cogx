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

// Package
package test.beliefmodels.utils;

// JUnit
import org.junit.Test; 
import static org.junit.Assert.*;

// Belief models
import beliefmodels.builders.EpistemicStatusBuilder;
import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.epstatus.AttributedEpistemicStatus;
import beliefmodels.autogen.epstatus.PrivateEpistemicStatus;
import beliefmodels.autogen.epstatus.SharedEpistemicStatus;
import beliefmodels.utils.EpistemicStatusUtils;

/**
 * 
 * @author gj
 *
 */

public class EpistemicStatusUtilsTest {

	@Test
	public void IsPrivateReturnsTrueOnPrivate() { 
		try { 
			PrivateEpistemicStatus pstatus = EpistemicStatusBuilder.createNewPrivateEpistemicStatus("robot");
			if (EpistemicStatusUtils.isPrivate(pstatus)) 
			{
				; 
			} else {
				fail("Checking private status for true private status should have succeeded");
			}
		} catch (BeliefException be) { 
			fail("Checking private status for true private status should have succeeded: "+be.getMessage());
		} // end try.. catch
	} // end test
	
	
	
	
	
	
	
} // end class
