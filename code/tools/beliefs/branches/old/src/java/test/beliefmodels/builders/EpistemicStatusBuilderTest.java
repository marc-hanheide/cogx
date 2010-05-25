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
 * UNIT TEST for the EpistemicStatusBuilder class
 * 
 * The unit tests illustrate assumptions on method parameters, how to access the individual methods, 
 * and how to get access to information in the objects that the methods return. 
 * 
 */


// PACKAGE 
package test.beliefmodels.builders;

// JUnit 
import org.junit.Before;
import org.junit.Test;
import static org.junit.Assert.*;

// Java
import java.util.*;

// Belief model 
import beliefmodels.builders.EpistemicStatusBuilder;
import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.epstatus.AttributedEpistemicStatus;
import beliefmodels.autogen.epstatus.PrivateEpistemicStatus;
import beliefmodels.autogen.epstatus.SharedEpistemicStatus;

public class EpistemicStatusBuilderTest {

	private String agent;
	private List<String> agents;
	
	@Before 
	public void init () { 
		agent = "a1";	
		agents = new ArrayList<String>();
		agents.add("a2");
	} // end init
	
	/**
	 *  A non-null private status is constructed for a non-null agent identifier
	 */
	
	@Test
	public void PrivateEpistemicStatusNonNullForNonNullAgent () { 
		try {
			PrivateEpistemicStatus pstatus = EpistemicStatusBuilder.createNewPrivateEpistemicStatus(agent);
			assertTrue("Non-null pstatus for non-null agent object", pstatus != null);
		} catch (BeliefException be) { 
			fail("Pstatus should not have thrown a BeliefException when provided an agent identifier");
		} // end try.. catch		
	} // end test
	
	/**
	 * Setting private epistemic status fails if agent identifier is an empty string or null
	 */
	
	@Test
	public void PrivateEpistemicStatusFailsForNullAgent () { 
		try {
			PrivateEpistemicStatus pstatus = EpistemicStatusBuilder.createNewPrivateEpistemicStatus("");
			fail("Pstatus cannot be set when provided an empty agent identifier");
		} catch (BeliefException be) { 
			assertEquals("Error when setting private epistemic status: agent is null or empty", 
					be.getMessage());
		} // end try.. catch
		try {
			PrivateEpistemicStatus pstatus = EpistemicStatusBuilder.createNewPrivateEpistemicStatus(null);
			fail("Pstatus cannot be set when provided a null agent identifier");
		} catch (BeliefException be) { 
			assertEquals("Error when setting private epistemic status: agent is null or empty", 
					be.getMessage());
		} // end try.. catch	
	} // end test	
	
	/**
	 * Setting private epistemic status PSTATUS for a specific agent identifier returns a status object with 
	 * that agent as identifier. The agent can be accessed as PSTATUS.agent. 
	 */
	
	@Test
	public void PrivateEpistemicStatusForNonNullAgentRetainsAgent () { 
		try {
			PrivateEpistemicStatus pstatus = EpistemicStatusBuilder.createNewPrivateEpistemicStatus(agent);
			assertTrue("Non-null pstatus for non-null agent object", pstatus != null);			
			assertEquals(agent, pstatus.agent);
		} catch (BeliefException be) { 
			fail("Pstatus should not have thrown a BeliefException when provided an agent identifier");
		} // end try.. catch		
	} // end test
	
	/**
	 * Setting attributed epistemic status ASTATUS for a non-null agent identifier, and a non-null/non-empty 
	 * list of agent identifiers, returns a non-null status object. 
	 */
	
	@Test
	public void AttributedEpistemicStatusForNonNullArgsReturnsNonNull () { 
		try { 
			AttributedEpistemicStatus astatus = EpistemicStatusBuilder.createNewAttributedEpistemicStatus(agent, agents);
			assertTrue("Non-null astatus for non-null agent and non-null agents", astatus != null);	
		} catch (BeliefException be) { 
			fail("Astatus should not have thrown a BeliefException when provided agent, agents identifiers");
		} // end try..catch
	} // end test
	
	/**
	 * Setting attributed epistemic status fails if the agent identifier is an empty string or null, or if the agents 
	 * list is either null or empty. 
	 */
	
	@Test 
	public void AttributedEpistemicStatusNullForNullParamaters () { 
		// Test for the empty agent
		try {
			AttributedEpistemicStatus astatus = EpistemicStatusBuilder.createNewAttributedEpistemicStatus("", agents);
			fail("Astatus cannot be initialized with an empty attributing agent");
		} catch (BeliefException be) {
			assertEquals("Error when setting attributed epistemic status: Attributing agent is null or empty", 
					be.getMessage());
		} // end try..catch
		// Test for the null agent
		try {
			AttributedEpistemicStatus astatus = EpistemicStatusBuilder.createNewAttributedEpistemicStatus(null, agents);
			fail("Astatus cannot be initialized with a null attributing agent");
		} catch (BeliefException be) {
			assertEquals("Error when setting attributed epistemic status: Attributing agent is null or empty", 
					be.getMessage());
		} // end try..catch		
		// Test for the null agents list
		try {
			AttributedEpistemicStatus astatus = EpistemicStatusBuilder.createNewAttributedEpistemicStatus(agent, null);
			fail("Astatus cannot be initialized with a null attributed agents list");
		} catch (BeliefException be) {
			assertEquals("Error when setting attributed epistemic status: Attributed-to agents is null", 
					be.getMessage());
		} // end try..catch
		// Test for the empty agents list
		try {
			List<String> emptyAgents = new ArrayList<String>();
			AttributedEpistemicStatus astatus = EpistemicStatusBuilder.createNewAttributedEpistemicStatus(agent, emptyAgents);
			fail("Astatus cannot be initialized with an empty attributed agents list");
		} catch (BeliefException be) {
			assertEquals("Error when setting attributed epistemic status: Attributed-to agents length is 0", 
					be.getMessage());
		} // end try..catch
	} // end test
	
	/**
	 * Setting the attributing agent and attributed-to agents paramaters returns an AStatus object that retains these values
	 * as astatus.agent and astatus.attribagents. Note that the astatus.attribagents is an Agents object (cast in Java 
	 * to a java.util.LinkedList object). 
	 */
	
	@Test
	public void AttributedEpistemicStatusRetainsValuesForNonNullParamaters () { 
		try {
			AttributedEpistemicStatus astatus = EpistemicStatusBuilder.createNewAttributedEpistemicStatus(agent, agents);
			assertEquals(agent, astatus.agent); 
			assertEquals(agents, astatus.attribagents);
		} catch (BeliefException be) {
			fail("Setting Astatus with non-empty parameters should not have thrown an exception: "+be.getMessage());
		} // end try..catch	
	} // end test 
	

	/** 
	 * Setting shared epistemic status with a non-null / non-empty list of agents returns a non-null object. 
	 */
	
	@Test
	public void SharedEpistemicStatusNonNullForNonNullParamater () { 
		try {
			SharedEpistemicStatus sstatus = EpistemicStatusBuilder.createNewSharedEpistemicStatus(agents);
			assertTrue("Shared status is a non-null object when set with a non-null paramater", sstatus != null);
		} catch (BeliefException be) {
			fail("Setting shared status with non-empty parameters should not have thrown an exception: "+be.getMessage());
		} // end try..catch
	} // end test
	
	/**
	 * Setting shared epistemic status with a null agents list fails. 
	 */
	
	@Test 
	public void SharedEpistemicStatusFailsForNullParameter () { 
		try {
			SharedEpistemicStatus sstatus = EpistemicStatusBuilder.createNewSharedEpistemicStatus(null);
			fail("Shared status fails when setting with a null paramater");
		} catch (BeliefException be) {
			assertEquals("Error when setting shared epistemic status: agents list is null",
					be.getMessage());
		} // end try..catch
	} // end test
	
	/**
	 * Setting shared epistemic status with an empty agents list fails. 
	 */
	
	@Test 
	public void SharedEpistemicStatusFailsForEmptyParameter () { 
		try {
			List<String> emptyAgents = new ArrayList<String>();
			SharedEpistemicStatus sstatus = EpistemicStatusBuilder.createNewSharedEpistemicStatus(emptyAgents);
			fail("Shared status fails when setting with an empty list paramater");
		} catch (BeliefException be) {
			assertEquals("Error when setting shared epistemic status: agents list length is 0",
					be.getMessage());
		} // end try..catch
	} // end test
	
} // end class
