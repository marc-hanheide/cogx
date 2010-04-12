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
 * UNIT TEST for the FormulaBuilder class
 * 
 * The unit tests illustrate assumptions on method parameters, how to access the individual methods, 
 * and how to get access to information in the objects that the methods return. 
 * 
 */

// Package
package test.beliefmodels.builders;

//JUnit 
import org.junit.Before;
import org.junit.Test;
import static org.junit.Assert.*;

// Belief models
import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.logicalcontent.BinaryOp;
import beliefmodels.autogen.logicalcontent.ComplexFormula;
import beliefmodels.autogen.logicalcontent.ElementaryFormula;
import beliefmodels.autogen.logicalcontent.Formula;
import beliefmodels.autogen.logicalcontent.ModalFormula;
import beliefmodels.autogen.logicalcontent.NegatedFormula;
import beliefmodels.autogen.logicalcontent.PointerFormula;
import beliefmodels.builders.FormulaBuilder;


public class FormulaBuilderTest {

	
	private String prop; // test proposition
	
	/**
	 *  Set up global private variables for the unit tests
	 */
	@Before
	public void setUp() { 
		prop = "proposition";
	} // end setUp 
	
	/**
	 * Creating an elementary formula from a non-null, non-empty value succeeds
	 */
	
	@Test
	public void ElementaryFormulaFromNonNullValue () { 
		try { 
			ElementaryFormula eform = FormulaBuilder.createNewElFormula(prop);
		} catch (BeliefException be) { 
			fail("Creating a formula from a non-null, non-empty value should not fail: "+be.getMessage());
		}
	} // end test
	
	/**
	 * Creating an elementary formula from a null-value fails. 
	 */
	
	@Test
	public void ElementaryFormulaFromNullFails () { 
		try { 
			ElementaryFormula eform = FormulaBuilder.createNewElFormula(null);
			fail("Creating an elementary formula from a null-value fails");
		} catch (BeliefException be) { 
			assertEquals("Cannot create an elementary formula from a null value", be.getMessage());
		} // end try.. catch
	} // end test
	
	/**
	 * Creating an elementary formula from an empty string fails
	 */
	
	@Test
	public void ElementaryFormulaFromEmptyStringFails () { 
		try { 
			ElementaryFormula eform = FormulaBuilder.createNewElFormula("");
			fail("Creating an elementary formula from an empty value fails");
		} catch (BeliefException be) { 
			assertEquals("Cannot create an elementary formula from an empty value", be.getMessage());
		} // end try.. catch
	} // end test	
	
	
} // end class
