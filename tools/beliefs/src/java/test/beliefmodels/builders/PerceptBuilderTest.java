
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
import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;

// Junit
import org.junit.Before;
import org.junit.Test; 
import static org.junit.Assert.*;

// Belief models
import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.distribs.ProbDistribution;
import beliefmodels.autogen.history.CASTBeliefHistory;
import beliefmodels.builders.PerceptBuilder;

public class PerceptBuilderTest {

	CASTTime curTime; 
	String id;
	String type;
	String curPlace;
	ProbDistribution content;
	CASTBeliefHistory hist;
	
	@Before
	public void setUp() throws Exception {
		curTime = new CASTTime();
		id = "id";
		type="vision";
		curPlace="here";
		content = new ProbDistribution();
		hist = PerceptBuilder.createNewPerceptHistory(new WorkingMemoryAddress(id,"vision"));
	} // end setUp
	
	/**
	 * Creating a new history for a null object (or pointer) should fail. 
	 */
	
	@Test
	public void NewHistoryForNullFails () { 
		try { 
			CASTBeliefHistory phist = PerceptBuilder.createNewPerceptHistory(null);
			fail("Creating a history for a null address should fail"); 
		} catch (BeliefException be) {
			assertEquals("Error when creating belief history: cannot create history for null", 
					be.getMessage());
		} // end try..catch
	} // end test
	
	
	/** 
	 * Creating a new history for an address with null pointers should fail
	 */
	
	@Test
	public void NewHistoryForNullAddressFails () { 
		WorkingMemoryAddress nullWMA = new WorkingMemoryAddress(null,null);
		try {
			CASTBeliefHistory phist = PerceptBuilder.createNewPerceptHistory(nullWMA);
			fail("Creating a history for an address with null values should fail"); 
		} catch (BeliefException be) {
			assertEquals("Error when creating belief history: cannot create history for null", 
					be.getMessage());
		} // end try..catch
	} // end test

	/**
	 * Creating a new history for an address with empty values should fail
	 */
	
	@Test
	public void NewHistoryForEmptyAddressValuesFails () {  
		WorkingMemoryAddress emptyWMA = new WorkingMemoryAddress("",""); 
		try {
			CASTBeliefHistory phist = PerceptBuilder.createNewPerceptHistory(emptyWMA);
			fail("Creating a history for an address with empty values should fail"); 
		} catch (BeliefException be) {
			assertEquals("Error when creating belief history: cannot create history for empty id/subarchitecture", 
					be.getMessage());
		} // end try..catch
	} // end test
	
	
	/**
	 * Creating a new PerceptBelief with no parameter empty or null succeeds
	 */
	
	@Test
	public void NewPerceptBeliefForFilledInParametersSucceeds () { 
		try {
			PerceptBuilder.createNewPerceptBelief(id, type, curPlace, curTime, content, hist);
		} catch (BeliefException be) {
			fail("Creating a new PerceptBelief with all parameters instantiated should not fail");
		} // end try..catch
	} // end test
	
	/**
	 * Creating a new PerceptBelief with the Id parameter null fails
	 */
	
	@Test
	public void NewPerceptBeliefForNullIdFails () { 
		try {
			PerceptBuilder.createNewPerceptBelief(null, type, curPlace, curTime, content, hist);
			fail("Creating a new PerceptBelief with the Id parameter null fails");
		} catch (BeliefException be) {
			assertEquals("Error in constructing PerceptBelief: parameters cannot be null or empty",be.getMessage());
		} // end try..catch
	} // end test
	
	/**
	 * Creating a new PerceptBelief with the Id parameter empty fails
	 */
	
	@Test
	public void NewPerceptBeliefForEmptyIdFails () { 
		try {
			PerceptBuilder.createNewPerceptBelief("", type, curPlace, curTime, content, hist);
			fail("Creating a new PerceptBelief with the Id parameter empty fails");
		} catch (BeliefException be) {
			assertEquals("Error in constructing PerceptBelief: parameters cannot be null or empty",be.getMessage());
		} // end try..catch
	} // end test
	
	/**
	 * Creating a new PerceptBelief with the type parameter null fails
	 */
	
	@Test
	public void NewPerceptBeliefForNullTypeFails () { 
		try {
			PerceptBuilder.createNewPerceptBelief(id, null, curPlace, curTime, content, hist);
			fail("Creating a new PerceptBelief with the type parameter null fails");
		} catch (BeliefException be) {
			assertEquals("Error in constructing PerceptBelief: parameters cannot be null or empty",be.getMessage());
		} // end try..catch
	} // end test
	
	/**
	 * Creating a new PerceptBelief with the type parameter empty fails
	 */
	
	@Test
	public void NewPerceptBeliefForEmptyTypeFails () { 
		try {
			PerceptBuilder.createNewPerceptBelief(id, "", curPlace, curTime, content, hist);
			fail("Creating a new PerceptBelief with the type parameter empty fails");
		} catch (BeliefException be) {
			assertEquals("Error in constructing PerceptBelief: parameters cannot be null or empty",be.getMessage());
		} // end try..catch
	} // end test
	
	/**
	 * Creating a new PerceptBelief with the curPlace parameter null fails
	 */
	
	@Test
	public void NewPerceptBeliefForNullPlaceFails () { 
		try {
			PerceptBuilder.createNewPerceptBelief(id, type, null, curTime, content, hist);
			fail("Creating a new PerceptBelief with the curPlace parameter null fails");
		} catch (BeliefException be) {
			assertEquals("Error in constructing PerceptBelief: parameters cannot be null or empty",be.getMessage());
		} // end try..catch
	} // end test	
	
	/**
	 * Creating a new PerceptBelief with the curPlace parameter null fails
	 */
	
	@Test
	public void NewPerceptBeliefForEmptyPlaceFails () { 
		try {
			PerceptBuilder.createNewPerceptBelief(id, type, "", curTime, content, hist);
			fail("Creating a new PerceptBelief with the curPlace parameter empty fails");
		} catch (BeliefException be) {
			assertEquals("Error in constructing PerceptBelief: parameters cannot be null or empty",be.getMessage());
		} // end try..catch
	} // end test	
	
	
	/**
	 * Creating a new PerceptBelief with the curTime parameter null fails
	 */
	
	@Test
	public void NewPerceptBeliefForNullTimeFails () { 
		try {
			PerceptBuilder.createNewPerceptBelief(id, type, curPlace, null, content, hist);
			fail("Creating a new PerceptBelief with the curTime parameter null fails");
		} catch (BeliefException be) {
			assertEquals("Error in constructing PerceptBelief: parameters cannot be null or empty",be.getMessage());
		} // end try..catch
	} // end test	
	
	/**
	 * Creating a new PerceptBelief with the content parameter null fails
	 */
	
	@Test
	public void NewPerceptBeliefForNullContentFails () { 
		try {
			PerceptBuilder.createNewPerceptBelief(id, type, curPlace, curTime, null, hist);
			fail("Creating a new PerceptBelief with the content parameter null fails");
		} catch (BeliefException be) {
			assertEquals("Error in constructing PerceptBelief: parameters cannot be null or empty",be.getMessage());
		} // end try..catch
	} // end test	
	
	/**
	 * Creating a new PerceptBelief with the curTime parameter null fails
	 */
	
	@Test
	public void NewPerceptBeliefForNullHistoryFails () { 
		try {
			PerceptBuilder.createNewPerceptBelief(id, type, curPlace, curTime, content, null);
			fail("Creating a new PerceptBelief with the hist parameter null fails");
		} catch (BeliefException be) {
			assertEquals("Error in constructing PerceptBelief: parameters cannot be null or empty",be.getMessage());
		} // end try..catch
	} // end test	
	
				
		
	
	
	
	
	
	
	
} // end class
