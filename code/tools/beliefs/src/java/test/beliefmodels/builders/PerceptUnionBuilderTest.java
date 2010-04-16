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

// JUnit
import org.junit.Before;
import org.junit.Test;
import static org.junit.Assert.*;

// Beliefs
import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.featurecontent.*;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.beliefs.PerceptUnionBelief;
import beliefmodels.autogen.epstatus.PrivateEpistemicStatus;
import beliefmodels.autogen.distribs.BasicProbDistribution;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.DistributionWithExistDep;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.distribs.ProbDistribution;
import beliefmodels.autogen.history.CASTBeliefHistory;
import beliefmodels.builders.BeliefContentBuilder;
import beliefmodels.builders.FeatureValueBuilder; 
import beliefmodels.builders.PerceptBuilder; 
import beliefmodels.builders.PerceptUnionBuilder;

import beliefmodels.utils.EpistemicStatusUtils;

//Remark: CAST dependencies should be removed!
// CAST
import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;


/**
 * UNIT TEST for the PerceptUnionBuilder class
 * 
 * The unit tests illustrate assumptions on method parameters, how to access the individual methods, 
 * and how to get access to information in the objects that the methods return. 
 * 
 * @author 	Geert-Jan M. Kruijff, gj@dfki.de
 * @version 100414
 */

public class PerceptUnionBuilderTest {

	CASTTime curTime; 
	String id;
	String type;
	String curPlace;
	BasicProbDistribution featDist;
	CASTBeliefHistory hist;
	WorkingMemoryAddress wma; 
	PerceptBelief pBelief;
	DistributionWithExistDep content;
	
	/**
	 * Initializes the global private variables used in the test class
	 * @throws java.lang.Exception
	 */
	
	@Before
	public void setUp() throws Exception {
		curTime = new CASTTime();
		id = "id1";
		type="vision";
		curPlace="here";
		// create a basic probability distribution over feature values
		FeatureValue fVal1 = FeatureValueBuilder.createNewStringValue("val1");
		FeatureValue fVal2 = FeatureValueBuilder.createNewStringValue("val2");
		FeatureValueProbPair fVal1Pr = new FeatureValueProbPair (fVal1, 0.4f);
		FeatureValueProbPair fVal2Pr = new FeatureValueProbPair (fVal2, 0.6f);
		ArrayList<FeatureValueProbPair> fValPrPairs = new ArrayList<FeatureValueProbPair>();
		fValPrPairs.add(fVal1Pr);
		fValPrPairs.add(fVal2Pr);
		featDist = BeliefContentBuilder.createNewFeatureDistribution("feature", fValPrPairs);
		// Add the feature distribution as a conditionally independent distribution, add to existential dist
		CondIndependentDistribs cDists = BeliefContentBuilder.createNewCondIndependentDistribs();
		BeliefContentBuilder.putNewCondIndependentDistrib(cDists, featDist);
		content = BeliefContentBuilder.createNewDistributionWithExistDep(0.8f, cDists);
		// Add another feature distribution (just the same but with a different name
		BasicProbDistribution featDist2 = BeliefContentBuilder.createNewFeatureDistribution("feature2", fValPrPairs);
		BeliefContentBuilder.putNewFeatureInBeliefContent(content, featDist2);
		// Create the percept belief
		wma = new WorkingMemoryAddress(id,"vision");
		hist = PerceptBuilder.createNewPerceptHistory(wma);
		pBelief = PerceptBuilder.createNewPerceptBelief(id, type, curPlace, curTime, content, hist);
	} // end setUp
	
	/**
	 * Creating a union from instantiated parameters succeeds
	 */
	
	@Test
	public void CreateUnionFromInstantiatedParamatersSucceeds () {
		try {
			PerceptUnionBuilder.createNewPerceptUnionBelief(id, type, curPlace, curTime, content, hist);
		} catch (BeliefException be) {
			fail("Creating a union from instantiated parameters should have succeeded");
		} // end try..catch
	} // end test
	
	/**
	 * Creating a new PerceptUnionBelief with the Id parameter null fails
	 */
	
	@Test
	public void NewPerceptUnionBeliefForNullIdFails () { 
		try {
			PerceptUnionBuilder.createNewPerceptUnionBelief(null, type, curPlace, curTime, content, hist);
			fail("Creating a new PerceptUnionBelief with the Id parameter null fails");
		} catch (BeliefException be) {
			assertEquals("Error in constructing PerceptUnionBelief: parameters cannot be null or empty",be.getMessage());
		} // end try..catch
	} // end test
	
	/**
	 * Creating a new PerceptUnionBelief with the Id parameter empty fails
	 */
	
	@Test
	public void NewPerceptUnionBeliefForEmptyIdFails () { 
		try {
			PerceptUnionBuilder.createNewPerceptUnionBelief("", type, curPlace, curTime, content, hist);
			fail("Creating a new PerceptUnionBelief with the Id parameter empty fails");
		} catch (BeliefException be) {
			assertEquals("Error in constructing PerceptUnionBelief: parameters cannot be null or empty",be.getMessage());
		} // end try..catch
	} // end test
	
	/**
	 * Creating a new PerceptUnionBelief with the type parameter null fails
	 */
	
	@Test
	public void NewPerceptUnionBeliefForNullTypeFails () { 
		try {
			PerceptUnionBuilder.createNewPerceptUnionBelief(id, null, curPlace, curTime, content, hist);
			fail("Creating a new PerceptUnionBelief with the type parameter null fails");
		} catch (BeliefException be) {
			assertEquals("Error in constructing PerceptUnionBelief: parameters cannot be null or empty",be.getMessage());
		} // end try..catch
	} // end test
	
	/**
	 * Creating a new PerceptUnionBelief with the type parameter empty fails
	 */
	
	@Test
	public void NewPerceptUnionBeliefForEmptyTypeFails () { 
		try {
			PerceptUnionBuilder.createNewPerceptUnionBelief(id, "", curPlace, curTime, content, hist);
			fail("Creating a new PerceptUnionBelief with the type parameter empty fails");
		} catch (BeliefException be) {
			assertEquals("Error in constructing PerceptUnionBelief: parameters cannot be null or empty",be.getMessage());
		} // end try..catch
	} // end test
	
	/**
	 * Creating a new PerceptUnionBelief with the curPlace parameter null fails
	 */
	
	@Test
	public void NewPerceptUnionBeliefForNullPlaceFails () { 
		try {
			PerceptUnionBuilder.createNewPerceptUnionBelief(id, type, null, curTime, content, hist);
			fail("Creating a new PerceptUnionBelief with the curPlace parameter null fails");
		} catch (BeliefException be) {
			assertEquals("Error in constructing PerceptUnionBelief: parameters cannot be null or empty",be.getMessage());
		} // end try..catch
	} // end test	
	
	/**
	 * Creating a new PerceptUnionBelief with the curPlace parameter null fails
	 */
	
	@Test
	public void NewPerceptUnionBeliefForEmptyPlaceFails () { 
		try {
			PerceptUnionBuilder.createNewPerceptUnionBelief(id, type, "", curTime, content, hist);
			fail("Creating a new PerceptUnionBelief with the curPlace parameter empty fails");
		} catch (BeliefException be) {
			assertEquals("Error in constructing PerceptUnionBelief: parameters cannot be null or empty",be.getMessage());
		} // end try..catch
	} // end test	
	
	
	/**
	 * Creating a new PerceptUnionBelief with the curTime parameter null fails
	 */
	
	@Test
	public void NewPerceptUnionBeliefForNullTimeFails () { 
		try {
			PerceptUnionBuilder.createNewPerceptUnionBelief(id, type, curPlace, null, content, hist);
			fail("Creating a new PerceptUnionBelief with the curTime parameter null fails");
		} catch (BeliefException be) {
			assertEquals("Error in constructing PerceptUnionBelief: parameters cannot be null or empty",be.getMessage());
		} // end try..catch
	} // end test	
	
	/**
	 * Creating a new PerceptUnionBelief with the content parameter null fails
	 */
	
	@Test
	public void NewPerceptUnionBeliefForNullContentFails () { 
		try {
			PerceptUnionBuilder.createNewPerceptUnionBelief(id, type, curPlace, curTime, null, hist);
			fail("Creating a new PerceptUnionBelief with the content parameter null fails");
		} catch (BeliefException be) {
			assertEquals("Error in constructing PerceptUnionBelief: parameters cannot be null or empty",be.getMessage());
		} // end try..catch
	} // end test	
	
	/**
	 * Creating a new PerceptUnionBelief with the curTime parameter null fails
	 */
	
	@Test
	public void NewPerceptBeliefForNullHistoryFails () { 
		try {
			PerceptUnionBuilder.createNewPerceptUnionBelief(id, type, curPlace, curTime, content, null);
			fail("Creating a new PerceptUnionBelief with the hist parameter null fails");
		} catch (BeliefException be) {
			assertEquals("Error in constructing PerceptUnionBelief: parameters cannot be null or empty",be.getMessage());
		} // end try..catch
	} // end test	
	
	/**
	 * A new PerceptUnionBelief gets private epistemic status
	 */
				
	@Test
	public void NewPerceptUnionBeliefIsPrivate () {
		try { 
			PerceptUnionBelief puBelief = PerceptUnionBuilder.createNewPerceptUnionBelief(id, type, curPlace, curTime, content, hist);
			if (EpistemicStatusUtils.isPrivate(puBelief.estatus)) {
				; 
			} else { 
				fail("Creating a new PerceptUnionBelief with all parameters should yield by default a private belief");
			} // end if..else
		} catch (BeliefException be) {
			fail("Creating a new PerceptUnionBelief with all instantiated parameter should succeed: "+
					be.getMessage());
		} // end try..catch
	} // end test
	
	/**
	 * Creating a PerceptUnionBelief from a single PerceptBelief succeeds with all parameters instantiated
	 */
	
	@Test
	public void NewPerceptUnionBeliefFromPerceptBeliefSucceeds () { 
		try { 
			PerceptUnionBuilder.createNewSingleUnionBelief(pBelief,wma,id);
		} catch (BeliefException be) {
			fail("Creating a PerceptUnionBelief from a single PerceptBelief and all parameters instantiated should have succeeded");
		} // end try..catch
	} // end test
	
	/**
	 * Creating a PerceptUnionBelief from a null belief fails
	 */
	
	@Test
	public void NewPerceptUnionBeliefFromNullPerceptBeliefFails () { 
		try { 
			PerceptUnionBuilder.createNewSingleUnionBelief(null,wma,id);
		} catch (BeliefException be) {
			assertEquals("Error in constructing PerceptUnionBelief: source belief is null", be.getMessage());
		} // end try..catch
	} // end test
	
	/**
	 * Creating a PerceptUnionBelief from a null address fails
	 */
	
	@Test
	public void NewPerceptUnionBeliefFromNullAddressFails () { 
		try { 
			PerceptUnionBuilder.createNewSingleUnionBelief(pBelief,null,id);
		} catch (BeliefException be) {
			assertEquals("Error in constructing PerceptUnionBelief: address for source belief is null or has empty information", 
					be.getMessage());
		} // end try..catch
	} // end test
	
	/**
	 * Creating a PerceptUnionBelief from a null values address fails
	 */
	
	@Test
	public void NewPerceptUnionBeliefFromNullValuesAddressFails () { 
		try { 
			PerceptUnionBuilder.createNewSingleUnionBelief(pBelief,new WorkingMemoryAddress(null,null),id);
		} catch (BeliefException be) {
			assertEquals("Error in constructing PerceptUnionBelief: address for source belief is null or has empty information",
					be.getMessage());
		} // end try..catch
	} // end test
	
	/**
	 * Creating a PerceptUnionBelief from an empty values address fails
	 */
	
	@Test
	public void NewPerceptUnionBeliefFromEmptyValuesAddressFails () { 
		try { 
			PerceptUnionBuilder.createNewSingleUnionBelief(pBelief,new WorkingMemoryAddress("",""),id);
		} catch (BeliefException be) {
			assertEquals("Error in constructing PerceptUnionBelief: address for source belief is null or has empty information",
					be.getMessage());
		} // end try..catch
	} // end test
	
	/**
	 * Creating a PerceptUnionBelief from a null Id fails
	 */
	
	@Test
	public void NewPerceptUnionBeliefFromNullIdFails () { 
		try { 
			PerceptUnionBuilder.createNewSingleUnionBelief(pBelief,wma,null);
		} catch (BeliefException be) {
			assertEquals("Error in constructing PerceptUnionBelief: id for union belief cannot be null or empty", 
					be.getMessage());
		} // end try..catch
	} // end test
	
	/**
	 * Creating a PerceptUnionBelief from an empty Id fails
	 */
	
	@Test
	public void NewPerceptUnionBeliefFromEmptyIdFails () { 
		try { 
			PerceptUnionBuilder.createNewSingleUnionBelief(pBelief,wma,"");
		} catch (BeliefException be) {
			assertEquals("Error in constructing PerceptUnionBelief: id for union belief cannot be null or empty",
					be.getMessage());
		} // end try..catch
	} // end test	
	
	/**
	 * Creating a PerceptUnionBelief from a single PerceptBelief succeeds with all parameters instantiated, including
	 * the existence probability for constructing a probability distribution
	 */
	
	@Test
	public void NewPerceptUnionBeliefFromPerceptBeliefWithExistenceProbSucceeds () { 
		try { 
			PerceptUnionBuilder.createNewSingleUnionBelief(pBelief,wma,0.7f,id);
		} catch (BeliefException be) {
			fail("Creating a PerceptUnionBelief from a single PerceptBelief and all parameters instantiated should have succeeded: "
					+be.getMessage());
		} // end try..catch
	} // end test
	 
	/**
	 * Creating a union as the merger of a percept belief and a union fails if the union and the percept model 
	 * content as conditionally independent distributions over identical features. 
	 */
	 
	@Test
	public void NewPerceptUnionBeliefFromPerceptBeliefWithSameFeaturesAsUnionFails () { 
		try {
			// Set up the union (based on pBelief), and another percept belief (pBelief2) identical to pBelief
			PerceptUnionBelief puBelief = PerceptUnionBuilder.createNewSingleUnionBelief(pBelief,wma,0.7f,"pu1");
			PerceptBelief pBelief2 = pBelief;
			pBelief2.id = "id2";
			// Construct a list of working memory address, one for the percept and one for the union
			ArrayList<WorkingMemoryAddress> wmaList = new ArrayList<WorkingMemoryAddress>();
			wmaList.add(wma); // assume this is the wma for the percept
			wmaList.add(wma); // assume this is the wma for the union
			// Create a new union on the basis of the merger of the union and the "new" percept belief
			PerceptUnionBuilder.createNewDoubleUnionBelief(pBelief2, wmaList, puBelief, 0.5f, "pu2");
			// We shouldn't get to this point
			fail("Creating a PerceptUnionBelief from a percept belief and a union belief with identical features "+
					"should have failed");
		} catch (BeliefException be) {
			assertEquals("Error in adding a new conditionally independent distribution: "+
					"Feature key [feature2] already present in set of conditionally independent distributions" ,
					be.getMessage());
		} // end try..catch
	} // end test
	
	/**
	 * Creating a union as the merger of a percept belief and an already existing union succeeds if the union and the 
	 * percept model content "from different modalities" i.e. with conditionally independent distributions over strictly
	 * different features. 
	 */
	
	@Test
	public void NewPerceptUnionBeliefFromPerceptBeliefWithDifferentFeaturesSucceeds () { 
		try {
			// Set up the union (based on pBelief)
			PerceptUnionBelief puBelief = PerceptUnionBuilder.createNewSingleUnionBelief(pBelief,wma,0.7f,"pu1");	
			// Set up another percept belief (pBelief2) different to pBelief
			// Create a basic probability distribution over feature values
			FeatureValue fVal1 = FeatureValueBuilder.createNewStringValue("val11");
			FeatureValue fVal2 = FeatureValueBuilder.createNewStringValue("val12");
			FeatureValueProbPair fVal1Pr = new FeatureValueProbPair (fVal1, 0.3f);
			FeatureValueProbPair fVal2Pr = new FeatureValueProbPair (fVal2, 0.7f);
			ArrayList<FeatureValueProbPair> fValPrPairs = new ArrayList<FeatureValueProbPair>();
			fValPrPairs.add(fVal1Pr);
			fValPrPairs.add(fVal2Pr);
			featDist = BeliefContentBuilder.createNewFeatureDistribution("hfeature", fValPrPairs);
			// Add the feature distribution as a conditionally independent distribution, add to existential dist
			CondIndependentDistribs cDists = BeliefContentBuilder.createNewCondIndependentDistribs();
			BeliefContentBuilder.putNewCondIndependentDistrib(cDists, featDist);
			DistributionWithExistDep hapticContent = BeliefContentBuilder.createNewDistributionWithExistDep(0.8f, cDists);
			// Add another feature distribution (just the same but with a different name
			BasicProbDistribution featDist2 = BeliefContentBuilder.createNewFeatureDistribution("hfeature2", fValPrPairs);
			BeliefContentBuilder.putNewFeatureInBeliefContent(hapticContent, featDist2);
			// Create the percept belief
			curTime = new CASTTime();
			String hapticId = "id3";
			String hapticType="haptic";
			curPlace="here";
			WorkingMemoryAddress wmaH = new WorkingMemoryAddress(hapticId,"haptic");
			CASTBeliefHistory hapticHist = PerceptBuilder.createNewPerceptHistory(wmaH);
			PerceptBelief hapticPBelief = PerceptBuilder.createNewPerceptBelief(hapticId, hapticType, curPlace, 
					curTime, hapticContent, hapticHist);
			// Construct a list of working memory address, one for the percept and one for the union
			ArrayList<WorkingMemoryAddress> wmaList = new ArrayList<WorkingMemoryAddress>();
			wmaList.add(wma); // assume this is the wma for the percept
			wmaList.add(wmaH); // assume this is the wma for the union
			PerceptUnionBuilder.createNewDoubleUnionBelief(hapticPBelief, wmaList, puBelief, 0.5f, "pu2");
		} catch (BeliefException be) {
			fail("Creating a new union from a percept and a union with strictly different features should have succeeded: "
					+be.getMessage());
		} // end try..catch		
	} // end test
	
	
	
	
	

} // end class
