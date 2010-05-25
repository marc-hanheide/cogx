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
 * UNIT TEST for the FeatureValueBuilder class
 * 
 * The FeatureValue builder provides methods for constructing different types of values, that can be 
 * assigned to a feature. 
 * 
 * Values must be non-empty, non-null values. If a value is unknown, the method createNewUnknownValue 
 * should be used. Tests for empty/null values are specified for Object-derived values. Basic types such 
 * as boolean or int would fail on compile-time. 
 * 
 * The unit tests illustrate assumptions on method parameters, how to access the individual methods, 
 * and how to get access to information in the objects that the methods return. 
 * 
 */

package test.beliefmodels.builders;

//JUnit 
import org.junit.Before;
import org.junit.Test;
import static org.junit.Assert.*;

// Belief models
import beliefmodels.builders.FeatureValueBuilder; 
import beliefmodels.autogen.featurecontent.*;
import beliefmodels.arch.BeliefException;



public class FeatureValueBuilderTest {

	private String testString; 
	
	
	/**
	 * Initialize private global variables
	 */
	
	@Before
	public void init() { 
		testString = "testString";
		
	} // end init
	
	/**
	 * A StringValue object contains the value of a provided non-empty String
	 */
	
	@Test
	public void StringValueRetainsNonNullString () { 
		try { 
			StringValue svalue = FeatureValueBuilder.createNewStringValue(testString); 
			assertEquals(svalue.val, testString);
		} catch (BeliefException be) { 
			fail("StringValue should not fail when setting to a non-null value: "+be.getMessage());
		} // end try..catch
	} // end test
	
	/**
	 * Creating a StringValue object fails on a null parameter
	 */
	
	@Test
	public void StringValueFailsOnNullParamater () { 
		try {
			StringValue svalue = FeatureValueBuilder.createNewStringValue(null); 
			fail("StringValue cannot be set to a null value");
		} catch (BeliefException be) { 
			assertEquals("StringValue cannot be set to a null value", 
					be.getMessage());
		} // end try..catch
	} // end test 	
		
	
	/**
	 * Creating a StringValue object fails on an empty string parameter
	 */
	
	@Test
	public void StringValueFailsOnEmptyParamater () { 
		try {
			StringValue svalue = FeatureValueBuilder.createNewStringValue(""); 
			fail("StringValue cannot be set to an empty value");
		} catch (BeliefException be) { 
			assertEquals("StringValue cannot be set to an empty value", 
					be.getMessage());
		} // end try..catch
	} // end test 		
	
	
	/**
	 * Creating a BooleanValue from a boolean returns an object that retains that value. 
	 */
	
	@Test
	public void BooleanValueRetainsBoolean () { 
		BooleanValue bvalue = FeatureValueBuilder.createNewBooleanValue(true);
		assertEquals(bvalue.val, true);
	} // end test 
		
	/**
	 * Creating an IntegerValue from an int returns an object that retains that value
	 */
	
	@Test
	public void IntegerValueRetainsInt () {
		IntegerValue ivalue = FeatureValueBuilder.createNewIntegerValue(1);
		assertEquals (ivalue.val, 1);
	} // end test
	
	
	/**
	 * Creating an UnknownValue object with no parameters succeeds 
	 */
	
	@Test
	public void UnknownValueSucceeds () { 
		UnknownValue uvalue = FeatureValueBuilder.createNewUnknownValue();
	} // end test
	
	
	
} // end class
