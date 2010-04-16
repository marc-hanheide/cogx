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
package test.beliefmodels.builders;

// Java
import java.util.ArrayList;
import java.util.Iterator; 
import java.util.LinkedList; 


// JUnit
import org.junit.Before;
import org.junit.Test;
import static org.junit.Assert.*;

// Beliefs
import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.autogen.featurecontent.UnknownValue;
import beliefmodels.autogen.distribs.BasicProbDistribution;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.DistributionWithExistDep;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.distribs.FeatureValues;
import beliefmodels.autogen.distribs.ProbDistribution;
import beliefmodels.builders.BeliefContentBuilder;
import beliefmodels.builders.FeatureValueBuilder;

/**
 * UNIT TEST for the BeliefContentBuilder class
 * 
 * The unit tests illustrate assumptions on method parameters, how to access the individual methods, 
 * and how to get access to information in the objects that the methods return. 
 * 
 * Several tests illustrate how to set up various types of distributions, on the basis of simple data items with 
 * associated probabilities. Typically, creating distributions fails from empty or null lists, or from 
 * <value,probability>-pairs that sum up to more than one. If pairs sum up to less than one, automatically an 
 * "unknown" value (UnknownValue object) is being added with the remainder (1-sum) probability. 
 */


public class BeliefContentBuilderTest {

	// A list of <feature-value, probability> pairs, summing up to 1.0f
	ArrayList<FeatureValueProbPair> fValPrPairs;
	// A basic distribution with <feature-value, probability> pairs, summing up to 1.0f
	BasicProbDistribution sampleBaseDistribution; 
	
	@Before
	public void setUp() throws Exception {
		FeatureValue fVal1 = FeatureValueBuilder.createNewStringValue("val1");
		FeatureValue fVal2 = FeatureValueBuilder.createNewStringValue("val2");
		FeatureValueProbPair fVal1Pr = new FeatureValueProbPair (fVal1, 0.4f);
		FeatureValueProbPair fVal2Pr = new FeatureValueProbPair (fVal2, 0.6f);
		fValPrPairs = new ArrayList<FeatureValueProbPair>();
		fValPrPairs.add(fVal1Pr);
		fValPrPairs.add(fVal2Pr);
		sampleBaseDistribution = BeliefContentBuilder.createNewFeatureDistribution("feat", fValPrPairs);
	}

	/**
	 * Creating a new base distribution from <feature-value, probability> pairs whose probabilities sum up to 
	 * 1.0 succeeds
	 */
	
	@Test
	public void CreateNewBaseDistributionSumUpToOneSucceeds () 
	{ 
		try { 
			// create a basic probability distribution over feature values
			BeliefContentBuilder.createNewFeatureDistribution("feats", fValPrPairs);
		} catch (BeliefException be) {
			fail("Creating a new base distribution from <feature-value, probability> pairs should have succeeded: "+
					be.getMessage());
		} // end try..catch
	} // end test
	
	/**
	 * Creating a new base distribution from null fails
	 */
	
	@Test
	public void CreatNewBaseDistributionFromNullFails () 
	{ 
		try { 
			BeliefContentBuilder.createNewFeatureDistribution("feats", null);
			fail("Creating a new base distribution from null should fail");
		} catch (BeliefException be) {
			assertEquals("Error in creating a feature distribution: values is null",be.getMessage());
		} // end try..catch
	} // end test
	
	/**
	 * Creating a new base distribution from an empty list fails
	 */
	
	@Test
	public void CreatNewBaseDistributionFromEmptyFails () 
	{ 
		try { 
			ArrayList<FeatureValueProbPair> fValPrPairs = new ArrayList<FeatureValueProbPair>();
			BeliefContentBuilder.createNewFeatureDistribution("feats", fValPrPairs);
			fail("Creating a new base distribution from null should fail");
		} catch (BeliefException be) {
			assertEquals("Error in creating a feature distribution: values is empty",be.getMessage());
		} // end try..catch
	} // end test
	
	/**
	 * Creating a new base distribution from <feature-value, probability> pairs whose probabilities sum up to 
	 * more than 1.0 fails
	 */
	
	@Test
	public void CreateNewBaseDistributionSumUpToMoreThanOneFails () 
	{ 
		try { 
			// create a basic probability distribution over feature values
			FeatureValue fVal1 = FeatureValueBuilder.createNewStringValue("val1");
			FeatureValue fVal2 = FeatureValueBuilder.createNewStringValue("val2");
			FeatureValueProbPair fVal1Pr = new FeatureValueProbPair (fVal1, 0.5f);
			FeatureValueProbPair fVal2Pr = new FeatureValueProbPair (fVal2, 0.6f);
			ArrayList<FeatureValueProbPair> fValPrPairs = new ArrayList<FeatureValueProbPair>();
			fValPrPairs.add(fVal1Pr);
			fValPrPairs.add(fVal2Pr);
			BeliefContentBuilder.createNewFeatureDistribution("feats", fValPrPairs);
			fail("Creating a new base distribution from <feature-value, probability> pairs should have failed "+
					"for probabilities summing up to more than one");
		} catch (BeliefException be) {
			assertEquals("Error in creating a feature distribution: probabilities of feature distribution "+
					"sum up to more than 1 [1.1]"
					, be.getMessage());
		} // end try..catch
	} // end test	
	
	
	/**
	 * Creating a new base distribution from <feature-value, probability> pairs whose probabilities sum up to 
	 * less than 1.0 succeeds
	 */
	
	@Test
	public void CreateNewBaseDistributionSumUpToLessThanOneSucceeds () 
	{ 
		try { 
			// create a basic probability distribution over feature values
			FeatureValue fVal1 = FeatureValueBuilder.createNewStringValue("val1");
			FeatureValue fVal2 = FeatureValueBuilder.createNewStringValue("val2");
			FeatureValueProbPair fVal1Pr = new FeatureValueProbPair (fVal1, 0.3f);
			FeatureValueProbPair fVal2Pr = new FeatureValueProbPair (fVal2, 0.6f);
			ArrayList<FeatureValueProbPair> fValPrPairs = new ArrayList<FeatureValueProbPair>();
			fValPrPairs.add(fVal1Pr);
			fValPrPairs.add(fVal2Pr);
			BeliefContentBuilder.createNewFeatureDistribution("feats", fValPrPairs);
		} catch (BeliefException be) {
			fail("Creating a new base distribution from <feature-value, probability> pairs should have succeeded "+
			"for probabilities summing up to less than one: "+be.getMessage());
		} // end try..catch
	} // end test	
	
	/**
	 * Creating a new base distribution from <feature-value, probability> pairs whose probabilities sum up to 
	 * less than 1.0 succeeds, and returns a distribution with an additional "UnknownValue" object with the 
	 * remainder (1-sum) probability. Note that if you use an ArrayList to create the original pairs list, you 
	 * need to cast back to that later on when retrieving the values from the distribution. 
	 */
	
	@Test
	public void CreateNewBaseDistributionSumUpToLessThanOneSucceedsWithUnknownValueAdded () 
	{ 
		try { 
			// create a basic probability distribution over feature values
			FeatureValue fVal1 = FeatureValueBuilder.createNewStringValue("val1");
			FeatureValue fVal2 = FeatureValueBuilder.createNewStringValue("val2");
			FeatureValueProbPair fVal1Pr = new FeatureValueProbPair (fVal1, 0.3f);
			FeatureValueProbPair fVal2Pr = new FeatureValueProbPair (fVal2, 0.6f);
			ArrayList<FeatureValueProbPair> fValPrPairs = new ArrayList<FeatureValueProbPair>();
			fValPrPairs.add(fVal1Pr);
			fValPrPairs.add(fVal2Pr);
			BasicProbDistribution baseDistribution = BeliefContentBuilder.createNewFeatureDistribution("feats", fValPrPairs);
			// The baseDistribution should now (a) contain an UnknownValue object, and (b) sum up to one
			// Initialize the sum
			float probSum = 0.0f;
			// Initialize the flag for checking for an UnknownValue object
			boolean unknownValuePresent = false; 
			// Get the feature values and iterate over them. 
			// (Note the embedding: dist.values gives the list of pairs, list.values gives the pairs themselves)
			FeatureValues fValues = (FeatureValues) baseDistribution.values;
			Iterator<FeatureValueProbPair> fvpbIter = ((ArrayList<FeatureValueProbPair>)fValues.values).iterator();
			while (fvpbIter.hasNext()) { 
				FeatureValueProbPair fvpPair = fvpbIter.next();
				FeatureValue fVal = fvpPair.val;
				if (fVal instanceof UnknownValue) { unknownValuePresent = true; }
				probSum += fvpPair.prob;
			} // end while
			// First test for the UnknownValue present
			if (!unknownValuePresent) {
				fail("Creating a base distribution from data summing up to less than one "+
						"should have succeeded, with the creation of an additional "+
						"UnknownValue object ["+unknownValuePresent+"]");
			} // end if
			// Second, test that the probabilities now sum up to 1.0f
			if (probSum != 1.0f) { 
				fail("Creating a base distribution from data summing up to less than one "+
						"should have succeeded, with the creation of an additional "+
						"UnknownValue object to sum up to 1.0f: ["+probSum+"]");				
			} // end if
		} catch (BeliefException be) {
			fail("Creating a new base distribution from <feature-value, probability> pairs should have succeeded "+
			"for probabilities summing up to less than one: "+be.getMessage());
		} // end try..catch
	} // end test	
	
	/**
	 * Create a new distribution with an existence probability succeeds
	 */
	
	@Test
	public void CreateNewDistributionWithExistDepSucceeds () { 
		try { 
			BeliefContentBuilder.createNewDistributionWithExistDep(0.5f,sampleBaseDistribution);
		} catch (BeliefException be) {
			fail("Creating a new distribution with an existence probability should have succeeded"
					+be.getMessage());
		} // end try..catch
	} // end test
	
	/**
	 * Creating a new distribution with an existence probability fails given a null source distribution
	 */
	
	@Test
	public void CreateNewDistributionWithExistDepFromNullFails () { 
		try { 
			BeliefContentBuilder.createNewDistributionWithExistDep(0.5f,null);
			fail("Creating a new distribution with an existence probability should fail given a null source distribution");
		} catch (BeliefException be) {
			assertEquals("Error in creating a distribution with an existence probability: "+
					"source distribution is null",be.getMessage());
		} // end try..catch
	} // end test
	
	/**
	 * Creating a new distribution with an existence probability outside the range (0.0f..1.0f] fails
	 */
	
	@Test
	public void CreateNewDistributionWithExistDepWithNegExistenceFails () { 
		try { 
			BeliefContentBuilder.createNewDistributionWithExistDep(-0.5f,sampleBaseDistribution);
			fail("Creating a new distribution with an existence probability should fail given negative probability");
		} catch (BeliefException be) {
			assertEquals("Error in creating a distribution with an existence probability: "+
					"existence probability [-0.5] is outside range (0.0f..1.0f]",be.getMessage());
		} // end try..catch
	} // end test
	
	
	/**
	 * Creating a new distribution with an existence probability outside the range (0.0f..1.0f] fails
	 */
	
	@Test
	public void CreateNewDistributionWithExistDepWithHugeExistenceFails () { 
		try { 
			BeliefContentBuilder.createNewDistributionWithExistDep(1000.0f,sampleBaseDistribution);
			fail("Creating a new distribution with an existence probability should fail given too large probability");
		} catch (BeliefException be) {
			assertEquals("Error in creating a distribution with an existence probability: "+
					"existence probability [1000.0] is outside range (0.0f..1.0f]",be.getMessage());
		} // end try..catch
	} // end test
	
	
	/**
	 * Creating a new conditionally independent distribution of a feature with <feature-value,probability> pairs succeeds
	 */
	
	@Test
	public void CreateNewConditionallyIndependentFeatureDistributionSucceeds () { 
		try { 
			// Create a new set of conditionally independent distributions
			CondIndependentDistribs cDists = BeliefContentBuilder.createNewCondIndependentDistribs();
			// Start from the sampleBaseDistribution of a feature with its feature-values and probabilities,
			BeliefContentBuilder.putNewCondIndependentDistrib(cDists, sampleBaseDistribution);
		} catch (BeliefException be) { 
			fail("Adding a new distribution to a map of conditionally independent distributions should have succeeded: "+
					be.getMessage());
		} // end try.. catch
	} // end test
	
	/**
	 * Creating a new conditionally independent distribution of a feature with <feature-value,probability> pairs 
	 * means we can successfully retrieve it again from the set of distributions
	 */
	
	@Test
	public void CreateNewConditionallyIndependentFeatureDistributionAddsDistribution () { 
		try { 
			// Create a new set of conditionally independent distributions
			CondIndependentDistribs cDists = BeliefContentBuilder.createNewCondIndependentDistribs();
			// Start from the sampleBaseDistribution of a feature with its feature-values and probabilities,
			BeliefContentBuilder.putNewCondIndependentDistrib(cDists, sampleBaseDistribution);
			// Now we should be able to get the distribution from cDists, by the key of the distribution
			BeliefContentBuilder.getConditionallyIndependentDistribution(cDists, sampleBaseDistribution.key); 
		} catch (BeliefException be) { 
			fail("Adding a new distribution to a map of conditionally independent distributions should have made it retrievable: "+
					be.getMessage());
		} // end try.. catch
	} // end test
	
	/**
	 * Creating a new conditionally independent distribution of a feature with <feature-value,probability> pairs means
	 * we can also again remove it from the set of distributions
	 */
	
	@Test
	public void RemoveNewlyAddedConditionallyIndependentFeatureDistributionSucceeds () { 
		try { 
			// Create a new set of conditionally independent distributions
			CondIndependentDistribs cDists = BeliefContentBuilder.createNewCondIndependentDistribs();
			// Start from the sampleBaseDistribution of a feature with its feature-values and probabilities,
			BeliefContentBuilder.putNewCondIndependentDistrib(cDists, sampleBaseDistribution);
			// Now we should be able to get the distribution from cDists, by the key of the distribution
			BeliefContentBuilder.removeConditionallyIndependentDistribution(cDists, sampleBaseDistribution.key); 
		} catch (BeliefException be) { 
			fail("Adding a new distribution to a map of conditionally independent distributions should have made it removable: "+
					be.getMessage());
		} // end try.. catch
	} // end test
	
	
	
	/**
	 * Adding a new feature to belief content that is not a set of conditionally independent distributions should not succeed
	 */
	
	@Test
	public void AddNewFeatureToBaseDistributionWithExistenceFails () { 
		try {
			DistributionWithExistDep baseExistDist = BeliefContentBuilder.createNewDistributionWithExistDep(0.5f,sampleBaseDistribution);
			BasicProbDistribution sampleBaseDistribution2 = sampleBaseDistribution;
			sampleBaseDistribution2.key = "feat2";
			BeliefContentBuilder.putNewFeatureInBeliefContent(baseExistDist, sampleBaseDistribution2);
			fail("Adding a new feature to belief content that is not a set of conditionally independent distributions should not succeed");
		} catch (BeliefException be) {
			assertEquals("Error in updating belief content with a new feature: "+ 
					"Content distribution is not set to be conditionally independent", 
					be.getMessage());
		} // 
	} // end test
	
	
	/**
	 * Putting a new feature into a map of conditionally independent feature distributions for a belief succeeds 	 
	 */
		
	@Test 
	public void AddNewFeatureToConditionallyIndependentFeatureDistributionSucceeds () { 
		try {
			// Create a conditionally independent distribution for a feature and its values
			CondIndependentDistribs cDists = BeliefContentBuilder.createNewCondIndependentDistribs();
			// Start from the sampleBaseDistribution of a feature with its feature-values and probabilities,
			BeliefContentBuilder.putNewCondIndependentDistrib(cDists, sampleBaseDistribution);
			// Create existential content on the basis of these, with an existence probability
			DistributionWithExistDep existContent = BeliefContentBuilder.createNewDistributionWithExistDep(0.5f,cDists);
			// Create a second conditionally independent distribution, for another feature 
			BasicProbDistribution sampleBaseDistribution2 = sampleBaseDistribution;
			sampleBaseDistribution2.key = "feat2";		
			// Add the new feature into the already formed content 
			BeliefContentBuilder.putNewFeatureInBeliefContent(existContent,sampleBaseDistribution2);
		} catch (BeliefException be) {
			fail("Adding a new distribution into a map of conditionally independent feature distributions for a belief should have succeeded: "+
					be.getMessage());
		} // end try..catch
	} // end test
	
	/**
	 * Adding two distributions, with identical keys, to a set of conditionally independent distributions fails. 
	 */
	
	@Test
	public void AddAlreadyPresentDistributionKeyToConditionallyIndependentDistsFails () { 
		try {
			// Create a conditionally independent distribution for a feature and its values
			CondIndependentDistribs cDists = BeliefContentBuilder.createNewCondIndependentDistribs();
			// Start from the sampleBaseDistribution of a feature with its feature-values and probabilities,
			BeliefContentBuilder.putNewCondIndependentDistrib(cDists, sampleBaseDistribution);
			// Create existential content on the basis of these, with an existence probability
			DistributionWithExistDep existContent = BeliefContentBuilder.createNewDistributionWithExistDep(0.5f,cDists);
			// Create a second conditionally independent distribution, for another feature 
			BasicProbDistribution sampleBaseDistribution2 = sampleBaseDistribution;
			// Add the new feature into the already formed content 
			BeliefContentBuilder.putNewFeatureInBeliefContent(existContent,sampleBaseDistribution2);
			// We should not get to this point: we should not be able to add two dists with the same key
			fail("Adding a distribution with an already present key in a map of conditionally independent feature distributions "
					+"for a belief should not have succeeded");
		} catch (BeliefException be) {
			assertEquals("Error in updating belief content with a new feature: "+ 
					"Feature key [feat] already present in set of conditionally independent distributions"  , 
					be.getMessage());
		} // end try..catch	
	} // end test
	
	
	
} // end class 
