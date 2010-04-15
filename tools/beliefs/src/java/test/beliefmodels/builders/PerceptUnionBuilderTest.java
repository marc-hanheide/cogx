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

// JUnit
import org.junit.Before;
import org.junit.Test;
import static org.junit.Assert.*;

// Beliefs
import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.distribs.ProbDistribution;
import beliefmodels.autogen.history.CASTBeliefHistory;
import beliefmodels.builders.PerceptBuilder; 
import beliefmodels.builders.PerceptUnionBuilder;

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
	ProbDistribution content;
	CASTBeliefHistory hist;
	
	
	/**
	 * Initializes the global private variables used in the test class
	 * @throws java.lang.Exception
	 */
	
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
	 * Creating a new PerceptBelief with the Id parameter null fails
	 */
	
	@Test
	public void NewPerceptBeliefForNullIdFails () { 
		try {
			PerceptUnionBuilder.createNewPerceptUnionBelief(null, type, curPlace, curTime, content, hist);
			fail("Creating a new PerceptBelief with the Id parameter null fails");
		} catch (BeliefException be) {
			assertEquals("Error in constructing PerceptUnionBelief: parameters cannot be null or empty",be.getMessage());
		} // end try..catch
	} // end test
	
	/**
	 * Creating a new PerceptBelief with the Id parameter empty fails
	 */
	
	@Test
	public void NewPerceptBeliefForEmptyIdFails () { 
		try {
			PerceptUnionBuilder.createNewPerceptUnionBelief("", type, curPlace, curTime, content, hist);
			fail("Creating a new PerceptBelief with the Id parameter empty fails");
		} catch (BeliefException be) {
			assertEquals("Error in constructing PerceptUnionBelief: parameters cannot be null or empty",be.getMessage());
		} // end try..catch
	} // end test
	
	/**
	 * Creating a new PerceptBelief with the type parameter null fails
	 */
	
	@Test
	public void NewPerceptBeliefForNullTypeFails () { 
		try {
			PerceptUnionBuilder.createNewPerceptUnionBelief(id, null, curPlace, curTime, content, hist);
			fail("Creating a new PerceptBelief with the type parameter null fails");
		} catch (BeliefException be) {
			assertEquals("Error in constructing PerceptUnionBelief: parameters cannot be null or empty",be.getMessage());
		} // end try..catch
	} // end test
	
	/**
	 * Creating a new PerceptBelief with the type parameter empty fails
	 */
	
	@Test
	public void NewPerceptBeliefForEmptyTypeFails () { 
		try {
			PerceptUnionBuilder.createNewPerceptUnionBelief(id, "", curPlace, curTime, content, hist);
			fail("Creating a new PerceptBelief with the type parameter empty fails");
		} catch (BeliefException be) {
			assertEquals("Error in constructing PerceptUnionBelief: parameters cannot be null or empty",be.getMessage());
		} // end try..catch
	} // end test
	
	/**
	 * Creating a new PerceptBelief with the curPlace parameter null fails
	 */
	
	@Test
	public void NewPerceptBeliefForNullPlaceFails () { 
		try {
			PerceptUnionBuilder.createNewPerceptUnionBelief(id, type, null, curTime, content, hist);
			fail("Creating a new PerceptBelief with the curPlace parameter null fails");
		} catch (BeliefException be) {
			assertEquals("Error in constructing PerceptUnionBelief: parameters cannot be null or empty",be.getMessage());
		} // end try..catch
	} // end test	
	
	/**
	 * Creating a new PerceptBelief with the curPlace parameter null fails
	 */
	
	@Test
	public void NewPerceptBeliefForEmptyPlaceFails () { 
		try {
			PerceptUnionBuilder.createNewPerceptUnionBelief(id, type, "", curTime, content, hist);
			fail("Creating a new PerceptBelief with the curPlace parameter empty fails");
		} catch (BeliefException be) {
			assertEquals("Error in constructing PerceptUnionBelief: parameters cannot be null or empty",be.getMessage());
		} // end try..catch
	} // end test	
	
	
	/**
	 * Creating a new PerceptBelief with the curTime parameter null fails
	 */
	
	@Test
	public void NewPerceptBeliefForNullTimeFails () { 
		try {
			PerceptUnionBuilder.createNewPerceptUnionBelief(id, type, curPlace, null, content, hist);
			fail("Creating a new PerceptBelief with the curTime parameter null fails");
		} catch (BeliefException be) {
			assertEquals("Error in constructing PerceptUnionBelief: parameters cannot be null or empty",be.getMessage());
		} // end try..catch
	} // end test	
	
	/**
	 * Creating a new PerceptBelief with the content parameter null fails
	 */
	
	@Test
	public void NewPerceptBeliefForNullContentFails () { 
		try {
			PerceptUnionBuilder.createNewPerceptUnionBelief(id, type, curPlace, curTime, null, hist);
			fail("Creating a new PerceptBelief with the content parameter null fails");
		} catch (BeliefException be) {
			assertEquals("Error in constructing PerceptUnionBelief: parameters cannot be null or empty",be.getMessage());
		} // end try..catch
	} // end test	
	
	/**
	 * Creating a new PerceptBelief with the curTime parameter null fails
	 */
	
	@Test
	public void NewPerceptBeliefForNullHistoryFails () { 
		try {
			PerceptUnionBuilder.createNewPerceptUnionBelief(id, type, curPlace, curTime, content, null);
			fail("Creating a new PerceptBelief with the hist parameter null fails");
		} catch (BeliefException be) {
			assertEquals("Error in constructing PerceptUnionBelief: parameters cannot be null or empty",be.getMessage());
		} // end try..catch
	} // end test	
	
				
	

} // end class
