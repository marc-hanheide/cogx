// =================================================================
// Copyright (C) 2010 DFKI GmbH Talking Robots 
// Geert-Jan M. Kruijff (gj@dfki.de)
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
// PACKAGE DEFINITION 
package de.dfki.lt.tr.beliefs.data;


public class FormulaTestDeactivate {

//	private Formula _formula;
//
//	
//	@Before
//	public void setUp() throws Exception {
//		_formula = new Formula ();
//	} // end setUp
//
//	/** A formula can be initialized */
//	@Test
//	public void canInitialize () { 
//		_formula = new Formula();
//		_formula.init();
//	} // end test
//	
//	/** An initialized formula has an identifier, by default set to -1 (int) */
//	@Test 
//	public void initializedFormulaHasIdentifierUnknown () {
//		try { 
//			_formula.init();
//			int id = _formula.getId();
//			assertEquals(id, -1);
//		} catch (BeliefNotInitializedException bnie) {
//			fail("We can retrieve the identifier for an initialized formula.");
//		} // end try..catch
//	} // end test
//	
//	/** An initialized formula can be given a new identifier, which is then returned when gotten */
//	@Test
//	public void initializedFormulaCanGetRetrievableNewIdentifier () 
//	{
//		try 
//		{
//			_formula.init();
//			int id = 1;
//			_formula.setId(id);
//			assertEquals(_formula.getId(),id);
//		}
//		catch (BeliefNotInitializedException bnie)
//		{
//			fail("An initialized formula can be given a new identifier, which is then returned when gotten");
//		}
//	} // end test
//	
//	/** Once initialized, we can turn the formula into an elementary proposition */
//	@Test
//	public void createPropositionFromInitializedFormula () { 
//		try 
//		{
//			_formula.init();
//			_formula.setId(1);
//			_formula.asProposition("prop");
//		} 
//		catch (BeliefNotInitializedException bnie)
//		{
//			fail("Once initialized, we can turn the formula into an elementary proposition");
//		}
//		catch (BeliefInvalidOperationException bioe)
//		{
//			fail("Once initialized, we can turn the formula into an elementary proposition");
//		}
//	} // end test 
//	
//	/** 
//	 * Once initialized, we can turn the formula into an elementary proposition and keep the identifier set before 
//	 * 
//	 * NOTE: this is a crucial test, as the formula gets recast to a new subclass (ElementaryFormula). The instance
//	 * of the new subclass needs to be explicitly assigned any identifier we have assigned earlier, else it gets lost. 
//	 * */
//	@Test
//	public void createPropositionFromInitializedFormulaRetainsEarlierId () { 
//		try 
//		{
//			_formula.init();
//			int id = 1;
//			_formula.setId(id);
//			_formula.asProposition("prop");
//			assertTrue(_formula.getId() == id);
//		} 
//		catch (BeliefNotInitializedException bnie)
//		{
//			fail("Once initialized, we can turn the formula into an elementary proposition");
//		}
//		catch (BeliefInvalidOperationException bioe)
//		{
//			fail("Once initialized, we can turn the formula into an elementary proposition");
//		}
//	} // end test 
//	
//	
//	
//	
//	
//	/** If something is a proposition, we can check for that */
//	@Test
//	public void formulaAsPropositionYieldsTrueWhenChecked () { 
//		try 
//		{
//			_formula.init();
//			_formula.setId(1);
//			_formula.asProposition("prop");
//			assertTrue(_formula.isProposition());
//		} 
//		catch (BeliefNotInitializedException bnie)
//		{
//			fail("Once initialized, we can turn the formula into an elementary proposition");
//		}
//		catch (BeliefInvalidOperationException bioe)
//		{
//			fail("Once initialized, we can turn the formula into an elementary proposition");
//		}	
//	} // end test
//
//	/** If something is a proposition, we can retrieve the proposition */
//	@Test
//	public void formulaAsPropositionReturnsProposition () 
//	{
//		try 
//		{
//			String prop = "prop";
//			_formula.init();
//			_formula.setId(1);
//			_formula.asProposition(prop);
//			assertEquals(_formula.getProposition(),prop);
//		} 
//		catch (BeliefNotInitializedException bnie)
//		{
//			fail("If something is a proposition, we can retrieve the proposition");
//		}
//		catch (BeliefInvalidOperationException bioe)
//		{
//			fail("If something is a proposition, we can retrieve the proposition");
//		}
//	} // end test
//	
//	/** We can set the probability for an instantiated formula */
//	@Test
//	public void formulaAsPropositionCanBeGivenProbability ()
//	{
//		try 
//		{
//			String prop = "prop";
//			float prob = 0.6f;
//			_formula.init();
//			_formula.setId(1);
//			_formula.asProposition(prop);
//			_formula.setProbability(prob);
//		} 
//		catch (BeliefNotInitializedException bnie)
//		{
//			fail("We can set the probability for an instantiated formula (not-init)");
//		}
//		catch (BeliefInvalidOperationException bioe)
//		{
//			fail("We can set the probability for an instantiated formula (invalid op)");
//		}
//	} // end test
//
//	/** We can get the probability we just set */
//	@Test
//	public void formulaAsPropositionReturnsGivenProbability ()
//	{
//		try 
//		{
//			String prop = "prop";
//			float prob = 0.6f;
//			_formula.init();
//			_formula.setId(1);
//			_formula.asProposition(prop);
//			_formula.setProbability(prob);
//			assertTrue(_formula.getProbability() == prob);
//		} 
//		catch (BeliefNotInitializedException bnie)
//		{
//			fail("We can set the probability for an instantiated formula (not-init)");
//		}
//		catch (BeliefInvalidOperationException bioe)
//		{
//			fail("We can set the probability for an instantiated formula (invalid op)");
//		}
//	} // end test
//	
//	
//	
//	
//	
//	
//	
	
} // end class
