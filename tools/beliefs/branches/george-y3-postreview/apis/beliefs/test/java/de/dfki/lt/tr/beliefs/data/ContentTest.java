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


//=================================================================
//IMPORTS

//Java
import static org.junit.Assert.assertNotNull;

import org.junit.Before;
import org.junit.Test;

import de.dfki.lt.tr.beliefs.data.genericproxies.Distribution;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;

/**
 * Unit tests for the <tt>de.dfki.lt.tr.beliefs.data.Content</tt> class. 
 * 
 * @author Geert-Jan M. Kruijff (gj@dfki.de)
 * @version 100521
 * @started 100521
 */


public class ContentTest {

	private Belief<dBelief> genericBelief;
	private Distribution<?> distribution;

	@Before
	public void setUp() throws Exception {
		genericBelief =Belief.create(dBelief.class);
		distribution =genericBelief.getContent();
	} // end setUp

	/** An initialized content object can be provided with a distribution */
	@Test
	public void contentNotNull () 
	{ 
			assertNotNull(distribution.get());
			assertNotNull(genericBelief.getContent().get());
	} // end test
	
	@Test
	public void setContentOfBelief() {
		distribution = FormulaDistribution.create();
		genericBelief.setContent(distribution);
		FormulaDistribution.create(genericBelief.getContent().get());
	}
//	
//	/** Cannot set a distribution for a non-initialized content structure */
//	@Test
//	public void cannotSetDistributionForNonInitializedContent () 
//	{
//		try { 
//			Content content = new Content();
//			ProbDistribution dist = new ProbDistribution();
//			content.setDistribution(dist);
//			fail("A non-initialized content object cannot be provided with a distribution");
//		} catch (BeliefNotInitializedException bnie) { 
//			assertEquals("Cannot set [distribution] for non-initialized content",bnie.getMessage());
//		}	
//	} // end test
//	
//	/** Cannot set content with a null distribution */
//	@Test
//	public void cannotUseNullToSetDistribution () 
//	{
//		try { 
//			Content content = new Content();
//			content.init();
//			content.setDistribution(null);
//			fail("A content object cannot be provided with a null distribution");
//		} catch (BeliefMissingValueException bmve) { 
//			assertEquals("Cannot set [distribution]: Provided parameter is null",bmve.getMessage());
//		}	
//	} // end test
//	
//	/** 
//	 * 	Creates a list of (formula,probability) pairs to base a distribution from.
//	 *  The list contains (1:prop1, 0.3f) and (2:prop2, 0.6f).  
//	 */
//	private LinkedList<FormulaProbPair> createDistributionBase () 
//	{
//		LinkedList fpList = new LinkedList<FormulaProbPair>();
//		try 
//		{ 
//			Formula fp1 = new Formula();
//			fp1.init();
//			fp1.setId(1);
//			fp1.asProposition("prop1");   // set the proposition 
//			fp1.setProbability(0.3f);	  // set the probability
//			fpList.add(fp1.getAsPair());  // get the (formula, probability) as pair, and add
//			Formula fp2 = new Formula();
//			fp2.init();
//			fp2.setId(2);
//			fp2.asProposition("prop2");
//			fp2.setProbability(0.6f);
//			fpList.add(fp2.getAsPair());
//		}
//		catch (BeliefInvalidOperationException bioe) 
//		{
//			System.out.println("Error while creating a distribution base:\n"+bioe.getMessage());
//		}
//		return fpList;
//	} // end createDistributionBase
//	
//	/** We can create a sample distribution with a non-empty list of (formula, probability) pairs */
//	@Test
//	public void buildDistributionFromFormulaProbPairsSucceeds () {
//		// set up the content structure
//		Content content = new Content();
//		content.init();
//		try 
//		{ 
//			// Create the FormulaValues structure from a list of (formula,probability) pairs
//			// The list contains (fp1:prop1, 0.3f) and (fp2:prop2, 0.6f)
//			FormulaValues formulaValues = new FormulaValues(this.createDistributionBase());
//			// Create the basic distribution with a given key
//			BasicProbDistribution baseDist = Content.createBasicDistribution("formula", formulaValues);
//			// Use the distribution for the content
//			content.setDistribution(baseDist);
//		}
//		catch (BeliefInvalidOperationException bioe)
//		{
//			System.out.println(bioe.getMessage());
//		}
//	} // end test
//	
//	/** Trying to create a distribution from a null list fails */
//	@Test 
//	public void cannotCreateDistributionFromNull () 
//	{
//		try 
//		{ 	
//			BasicProbDistribution baseDist = Content.createBasicDistribution("formula", null);
//		}
//		catch (BeliefMissingValueException bmve) 
//		{
//		    assertEquals(bmve.getMessage(), "Cannot create basic distribution: Provided distribution values is null");
//		}
//		catch (BeliefInvalidOperationException bmve)
//		{
//			fail(bmve.getMessage());
//		}
//	} // end test
//	
//	/** We can create a distribution from an empty list -- this just yields a big unknown formula*/
//	@Test
//	public void cannotCreateDistributionFromEmptyList () 
//	{
//		try 
//		{ 	
//			Content content = new Content();
//			content.init();
//			FormulaValues formulaValues = new FormulaValues(new LinkedList<FormulaProbPair>());
//			BasicProbDistribution baseDist = Content.createBasicDistribution("formula", formulaValues);
//		}
//		catch (BeliefMissingValueException bmve) 
//		{
//			fail("We can create a distribution from an empty list:\n"+bmve.getMessage());
//		}
//		catch (BeliefInvalidOperationException bioe) 
//		{
//			fail("We can create a distribution from an empty list:\n"+bioe.getMessage());
//		}
//	} // end test
//	
//	
//	/** Creating a sample distribution from a list with sum probs < 1.0 adds another formula */
//	@Test
//	public void creatingDistributionFromListWithProbLessThanOneCreatesAdditionalUnknownFormula ()
//	{
//		// set up the content structure
//		Content content = new Content();
//		content.init();
//		try 
//		{ 
//			// Create the FormulaValues structure from a list of (formula,probability) pairs
//			// The list contains (fp1:prop1, 0.3f) and (fp2:prop2, 0.6f)
//			FormulaValues formulaValues = new FormulaValues(this.createDistributionBase());
//			int fvSize = formulaValues.values.size();
//			// Create the basic distribution with a given key
//			BasicProbDistribution baseDist = Content.createBasicDistribution("formula", formulaValues);
//			// Use the distribution for the content
//			content.setDistribution(baseDist);
//			// Retrieve the (new) list of formulas in the distribution 
//			formulaValues = (FormulaValues) baseDist.values;
//			assertTrue(formulaValues.values.size() == fvSize+1);
//		}
//		catch (BeliefInvalidOperationException bioe)
//		{
//			System.out.println(bioe.getMessage());
//		} // end try..catch		
//	} // end test
//	
//	/** Creating a sample distribution from a list with sum probs > 1.0 fails */
//	@Test
//	public void cannotCreateDistributionFromListWithProbMoreThanOne ()
//	{
//		// set up the content structure
//		Content content = new Content();
//		content.init();
//		try 
//		{ 
//			// Create the FormulaValues structure from a list of (formula,probability) pairs
//			// The list contains (fp1:prop1, 0.3f) and (fp2:prop2, 0.6f)
//			LinkedList<FormulaProbPair> fpList = this.createDistributionBase();
//			Formula fp3 = new Formula();
//			fp3.init();
//			fp3.setId(3);
//			fp3.asProposition("prop3");
//			fp3.setProbability(0.4f);
//			fpList.add(fp3.getAsPair());
//			FormulaValues formulaValues = new FormulaValues(fpList);
//			// Create the basic distribution with a given key
//			BasicProbDistribution baseDist = Content.createBasicDistribution("formula", formulaValues);
//			fail("Creating a sample distribution from a list with sum probs > 1.0 fails");
//		}
//		catch (BeliefInvalidOperationException bioe)
//		{
//			assertTrue(bioe.getMessage().startsWith("Error in creating a feature distribution: "+
//					"probabilities of formula distribution sum up to more than 1"));
//		} // end try..catch		
//	} // end test
//	
//
//	/** Given a distribution we can retrieve a formula (and thus its probability) by its identifier */
//	@Test
//	public void canRetrieveFormulaFromDistributionByIdentifier () 
//	{
//		Content content = new Content();
//		content.init();
//		try { 
//			FormulaValues formulaValues = new FormulaValues(this.createDistributionBase());
//			BasicProbDistribution baseDist = Content.createBasicDistribution("formula", formulaValues);
//			content.setDistribution(baseDist);
//			// get the distribution we just stored
//			BasicProbDistribution storedDist = (BasicProbDistribution) content.getDistribution();
//			assertTrue (storedDist != null);
//			// print out the overall size of the distribution 
//			assertTrue( ((FormulaValues)storedDist.values).values.size() > 0);
//			// get the formula from the distribution by Id
//			Formula retrieved = content.getById(-100);
//		} 
//		catch (BeliefInvalidOperationException bioe)
//		{
//			fail(bioe.getMessage());
//		}
//		catch (BeliefMissingValueException bmve)
//		{
//			fail("Given a distribution we can retrieve a formula (and thus its probability) by its identifier:\n"
//					+bmve.getMessage());
//		}
//	} // end test
//	
//	/** We can create an initialized set of conditionally independent distributions */
//	@Test
//	public void canCreateConditionallyIndependentDistributions () 
//	{
//		try 
//		{
//			CondIndependentDistribs ciDists = Content.createConditionallyIndependentDistributions();
//		} 
//		catch (Exception e)
//		{
//			fail("We can create an initialized set of conditionally independent distributions");
//		}
//	} // end test
//	
//	/** We can add a conditionally independent distribution */
//	@Test
//	public void canAddConditionallyIndependentDistribution () 
//	{
//		try 
//		{
//			// create a dictionary of conditionally independent distributions
//			CondIndependentDistribs ciDists = Content.createConditionallyIndependentDistributions();
//			// create a new base distribution
//			FormulaValues formulaValues = new FormulaValues(this.createDistributionBase());
//			BasicProbDistribution baseDist = Content.createBasicDistribution("formula", formulaValues);
//			// add the new distribution directly to the dictionairy of conditionally independent dists
//			Content.addConditionallyIndependentDistribution(ciDists,baseDist);
//		} 
//		catch (Exception e)
//		{
//			fail("We can add a base distribution to a dictionary of conditionally independent distributions");
//		}	
//	} // end test
//	
//	/** We can retrieve a distribution we just added to the dictionary */
//	@Test
//	public void canRetrieveJustAddedConditionallyIndependentDistribution () 
//	{
//		try 
//		{
//			// create a dictionary of conditionally independent distributions
//			CondIndependentDistribs ciDists = Content.createConditionallyIndependentDistributions();
//			// create a new base distribution
//			FormulaValues formulaValues = new FormulaValues(this.createDistributionBase());
//			BasicProbDistribution baseDist = Content.createBasicDistribution("formula", formulaValues);
//			// add the new distribution directly to the dictionairy of conditionally independent dists
//			Content.addConditionallyIndependentDistribution(ciDists,baseDist);
//			// get the distribution
//			Content.getConditionallyIndependentDistribution(ciDists,"formula");
//		} 
//		catch (Exception e)
//		{
//			fail("We can retrieve a base distribution to a dictionary of conditionally independent distributions");
//		}	
//	} // end test
//	
//	/** We cannot retrieve a distribution with a key that is unknown in the dictionary */
//	@Test
//	public void cannotRetrieveUnknownDistribution () 
//	{
//		try 
//		{
//			// create a dictionary of conditionally independent distributions
//			CondIndependentDistribs ciDists = Content.createConditionallyIndependentDistributions();
//			// get some distribution
//			Content.getConditionallyIndependentDistribution(ciDists,"formula");
//			fail("We cannot retrieve a distribution with a key that is unknown in the dictionary");
//		} 
//		catch (BeliefMissingValueException e)
//		{
//			assertTrue(e.getMessage().startsWith("Error in retrieving conditionally independent distribution: Key"));
//		}	
//	} // end test
//	
//	/** We can remove a known distribution from the dictionary */
//	@Test
//	public void canRemoveAddedDistributionFromDictionary () 
//	{
//		try 
//		{
//			// create a dictionary of conditionally independent distributions
//			CondIndependentDistribs ciDists = Content.createConditionallyIndependentDistributions();
//			// create a new base distribution
//			FormulaValues formulaValues = new FormulaValues(this.createDistributionBase());
//			BasicProbDistribution baseDist = Content.createBasicDistribution("formula", formulaValues);
//			// add the new distribution directly to the dictionairy of conditionally independent dists
//			Content.addConditionallyIndependentDistribution(ciDists,baseDist);
//			// remove the distribution
//			Content.removeConditionallyIndependentDistribution(ciDists,"formula");
//		} 
//		catch (Exception e)
//		{
//			fail("We can remove a base distribution from a dictionary of conditionally independent distributions");
//		}	
//	} // end test
//	
//	/** We cannot retrieve a distribution from the dictionary anymore if we have just removed it */
//	@Test
//	public void cannotRetrieveRemovedDistributionFromDictionary () 
//	{
//		try 
//		{
//			// create a dictionary of conditionally independent distributions
//			CondIndependentDistribs ciDists = Content.createConditionallyIndependentDistributions();
//			// create a new base distribution
//			FormulaValues formulaValues = new FormulaValues(this.createDistributionBase());
//			BasicProbDistribution baseDist = Content.createBasicDistribution("formula", formulaValues);
//			// add the new distribution directly to the dictionairy of conditionally independent dists
//			Content.addConditionallyIndependentDistribution(ciDists,baseDist);
//			// remove the distribution
//			Content.removeConditionallyIndependentDistribution(ciDists,"formula");
//			// try to get the distribution we just removed
//			Content.getConditionallyIndependentDistribution(ciDists,"formula");
//			fail("We cannot retrieve a distribution from the dictionary anymore if we have just removed it");
//		} 
//		catch (BeliefInvalidOperationException bioe) 
//		{	
//			fail("We should be able to create a distribution for the dictionary");
//		}
//		catch (BeliefMissingValueException bmve)
//		{
//			assertTrue(bmve.getMessage().startsWith("Error in retrieving conditionally independent distribution: Key"));
//		}	
//	} // end test
//	
//	/** We "cannot" remove an unknown distribution from the dictionary */
//	@Test
//	public void cannotRemoveUnknownDistributionFromDictionary () 
//	{
//		try 
//		{
//			// create a dictionary of conditionally independent distributions
//			CondIndependentDistribs ciDists = Content.createConditionallyIndependentDistributions();	
//			// remove an unknown distribution
//			Content.removeConditionallyIndependentDistribution(ciDists,"formula");			
//		} 
//		catch (BeliefMissingValueException bmve)
//		{
//			assertTrue(bmve.getMessage().startsWith("Error in retrieving conditionally independent distribution: Key"));
//		}
//	} // end test
//	
//	/** We can set a dictionary of conditionally independent distributions as the content */
//	@Test
//	public void canSetConditionallyIndependentDistributionsAsDistribution () 
//	{
//		try { 
//			// Initialize the content structure
//			Content content = new Content();
//			content.init();
//			// create a dictionary of conditionally independent distributions
//			CondIndependentDistribs ciDists = Content.createConditionallyIndependentDistributions();
//			// create a new base distribution
//			FormulaValues formulaValues = new FormulaValues(this.createDistributionBase());
//			BasicProbDistribution baseDist = Content.createBasicDistribution("formula", formulaValues);
//			// add the new distribution directly to the dictionairy of conditionally independent dists
//			Content.addConditionallyIndependentDistribution(ciDists,baseDist);
//			// set the conditionally independent distribution dictionary as the dist in the content
//			content.setDistribution(ciDists);
//		} 
//		catch (BeliefException be) 
//		{
//			fail("We can set a dictionary of conditionally independent distributions as the content:\n"
//					+be.getMessage());
//		}	
//	} // end test
//	
//	/** Once we have provided the content structure with a dictionary, we can directly add base distributions */
//	@Test
//	public void canAddDistribitionToContentDictionary () 
//	{
//		try { 
//			// Initialize the content structure
//			Content content = new Content();
//			content.init();
//			// create a dictionary of conditionally independent distributions
//			CondIndependentDistribs ciDists = Content.createConditionallyIndependentDistributions();
//			// create a new base distribution
//			FormulaValues formulaValues = new FormulaValues(this.createDistributionBase());
//			BasicProbDistribution baseDist = Content.createBasicDistribution("formula", formulaValues);
//			// add the new distribution directly to the dictionairy of conditionally independent dists
//			Content.addConditionallyIndependentDistribution(ciDists,baseDist);
//			// set the conditionally independent distribution dictionary as the dist in the content
//			content.setDistribution(ciDists);
//			// now create another base distribution, 
//			BasicProbDistribution baseDist2 = Content.createBasicDistribution("formula2", formulaValues);
//			// add it to the content directly
//			content.addDistributionToDictionary(baseDist2);
//		} 
//		catch (BeliefException be) 
//		{
//			fail("We can add a distribution to the dictionary of conditionally independent distributions in the content:\n"
//					+be.getMessage());
//		}	
//	} // end test	
//	
//	/** We can get a distribution directly from the dictionary stored in the content */
//	@Test
//	public void canRetrieveAddedDistributionDirectlyFromContentDictionary () 
//	{
//		try { 
//			// Initialize the content structure
//			Content content = new Content();
//			content.init();
//			// create a dictionary of conditionally independent distributions
//			CondIndependentDistribs ciDists = Content.createConditionallyIndependentDistributions();
//			// create a new base distribution
//			FormulaValues formulaValues = new FormulaValues(this.createDistributionBase());
//			BasicProbDistribution baseDist = Content.createBasicDistribution("formula", formulaValues);
//			// add the new distribution directly to the dictionairy of conditionally independent dists
//			Content.addConditionallyIndependentDistribution(ciDists,baseDist);
//			// set the conditionally independent distribution dictionary as the dist in the content
//			content.setDistribution(ciDists);
//			// now create another base distribution, 
//			BasicProbDistribution baseDist2 = Content.createBasicDistribution("formula2", formulaValues);
//			// add it to the content directly
//			content.addDistributionToDictionary(baseDist2);
//			// get a distribution directly from the dictionary stored in the content
//			BasicProbDistribution retrieved = content.getDistributionFromDictionary("formula");
//		} 
//		catch (BeliefException be) 
//		{
//			fail("We can get a distribution from the dictionary of conditionally independent distributions in the content:\n"
//					+be.getMessage());
//		}				
//	} // end test
	
	
	
	
	
} // end class
