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
 * UNIT TEST for the SpatioTemporalFrameBuilder class
 * 
 * The unit tests illustrate assumptions on method parameters, how to access the individual methods, 
 * and how to get access to information in the objects that the methods return. 
 * 
 * @author 	Geert-Jan M. Kruijff, gj@dfki.de
 * @version 100415
 */

// Package
package test.beliefmodels.builders;


// Remark: CAST dependencies should be removed!
// CAST
import beliefmodels.arch.BeliefException;
import beliefmodels.builders.SpatioTemporalFrameBuilder;
import cast.cdl.CASTTime;

// JUnit
import org.junit.Before;
import org.junit.Test;
import static org.junit.Assert.*;

public class SpatioTemporalFrameBuilderTest {

	String place; 
	CASTTime startTime;
	CASTTime endTime;

	@Before
	public void setUp() throws Exception {
		startTime = new CASTTime();
		endTime = new CASTTime();
		place = "here";
	} // end setUp

	/**
	 * Creating an ST frame with instantiated parameters succeeds
	 */
	
	@Test
	public void CreateSTFrameWithFullParametersSucceeds() { 
		try { 
			SpatioTemporalFrameBuilder.createSimpleSpatioTemporalFrame(place, startTime, endTime);
		} catch (BeliefException be) {
			fail("Creating an ST frame with fully instantiated parameters should not fail");
		} // end try..catch
	} // end test
	
	/**
	 * Creating an ST frame with a null place parameter fails
	 */
	
	@Test
	public void CreateSTFrameWithNullPlaceFails () { 
		try { 
			SpatioTemporalFrameBuilder.createSimpleSpatioTemporalFrame(null, startTime, endTime);
		} catch (BeliefException be) {
			assertEquals("Error when creating ST frame: parameters cannot be null or empty",be.getMessage());
		} // end try.. catch
	} // end test
	
	/**
	 * Creating an ST frame with an empty place parameter fails
	 */
	
	@Test
	public void CreateSTFrameWithEmptyPlaceFails () { 
		try { 
			SpatioTemporalFrameBuilder.createSimpleSpatioTemporalFrame("", startTime, endTime);
		} catch (BeliefException be) {
			assertEquals("Error when creating ST frame: parameters cannot be null or empty",be.getMessage());
		} // end try.. catch
	} // end test
	
	/**
	 * Creating an ST frame with a null startTime parameter fails
	 */
	
	@Test
	public void CreateSTFrameWithNullStartTimeFails () { 
		try { 
			SpatioTemporalFrameBuilder.createSimpleSpatioTemporalFrame(place, null, endTime);
		} catch (BeliefException be) {
			assertEquals("Error when creating ST frame: parameters cannot be null or empty",be.getMessage());
		} // end try.. catch
	} // end test
	
	/**
	 * Creating an ST frame with a null endTime parameter fails
	 */
	
	@Test
	public void CreateSTFrameWithNullEndTimeFails () { 
		try { 
			SpatioTemporalFrameBuilder.createSimpleSpatioTemporalFrame(place, startTime, null);
		} catch (BeliefException be) {
			assertEquals("Error when creating ST frame: parameters cannot be null or empty",be.getMessage());
		} // end try.. catch
	} // end test
	
	
	
	
} // end class
