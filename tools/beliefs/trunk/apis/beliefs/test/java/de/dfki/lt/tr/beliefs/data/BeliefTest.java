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

// Java
import java.util.LinkedList;

// JUnit
import org.junit.Before;
import org.junit.Test;
import static org.junit.Assert.*;

// Belief API
import de.dfki.lt.tr.beliefs.util.BeliefInvalidOperationException;
import de.dfki.lt.tr.beliefs.util.BeliefInvalidQueryException;
import de.dfki.lt.tr.beliefs.util.BeliefMissingValueException;
import de.dfki.lt.tr.beliefs.util.BeliefNotInitializedException;

//=================================================================
// TEST CLASS

/**
 * Unit tests for the <tt>de.dfki.lt.tr.beliefs.data.Belief</tt> class. 
 * 
 * @author Geert-Jan M. Kruijff (gj@dfki.de)
 * @version 100521
 * @started 100510
 */

public class BeliefTest {

	private Belief belief; 
	
	@Before
	public void setUp () { 
		belief = new Belief();
	} 
	
	/** Tests that the class initializes */
	@Test
	public void canInitialize () { 
		belief.init();
	} // end test
	
	/** A bare initialized belief does not have an identifier */
	@Test
	public void initBareBeliefThrowsExceptionOnNullId () { 
		belief.init();
		try { 
			belief.getId();
		} catch (BeliefMissingValueException bmve) {
			assertEquals("In querying a belief: Missing value for [Id]", bmve.getMessage());
		} // end try..catch
	} // end test
	
	/** A bare initialized belief can be given an identifier */
	@Test
	public void bareBeliefCanBeSetAnId () { 
		belief.init();
		belief.setId("newId");
	} // end test
	
	/** The identifier of a non-initialized belief cannot be set */
	@Test
	public void cannotSetIdForNonInitializedBelief () { 
		belief = new Belief();
		try {
			belief.setId("newId");
		} catch (BeliefNotInitializedException bnie) {
			assertEquals("Cannot set [Id] for non-initialized belief", bnie.getMessage());
		} // end try..catch
	} // end test
	
	/** Setting the identifier of a belief yields the same identifier when queried */
	@Test
	public void setIdForBeliefGetsSameId () {
		belief.init();
		String newId = "newId";
		belief.setId(newId);
		assertEquals(belief.getId(),newId);
	} // end test
	
	/** A bare initialized belief can be given a type */
	@Test
	public void bareBeliefCanBeSetAType () {
		belief.init();
		belief.setType("percept");
	} // end test
	
	/** The type for a non-initialized belief cannot be set */
	@Test
	public void cannotSetTypeForNonInitializedBelief () { 
		belief = new Belief();
		try {
			belief.setType("newType");
		} catch (BeliefNotInitializedException bnie) {
			assertEquals("Cannot set [type] for non-initialized belief", bnie.getMessage());
		} // end try..catch
	} // end test
	
	/** Setting the type of a belief yields the same type when queried */
	@Test
	public void setTypeForBeliefGetsSameType () {
		belief.init();
		String newType = "newType";
		belief.setType(newType);
		assertEquals(belief.getType(),newType);
	} // end test
	
	/** An initialized belief has a private epistemic status */
	@Test
	public void bareInitializedBeliefIsPrivate () { 
		belief.init();
		assertEquals(belief.isPrivate(), true);
	} // end test
	
	/** An initialized belief has a private epistemic status with agent id "self" */
	@Test
	public void bareInitializedBeliefIsPrivateStatusSelf () { 
		belief.init();
		String agent = belief.getPrivate(); 
		assertEquals(agent, "self");
	} // end test
	
	/** Getting the agent for a non-initialized belief fails */
	@Test
	public void cannotGetAgentForNonInitializedBelief () { 
		belief = new Belief();
		try { 
			belief.getPrivate();
		} catch (BeliefNotInitializedException bnie) { 
			assertEquals("Cannot get [agent] for private status for non-initialized belief", bnie.getMessage());
		} 
	} // end test
	
	/** An initialized belief can be set an agent id for its default private status */
	@Test
	public void bareInitializedBeliefCanBeSetPrivateAgentId () { 
		belief.init();
		belief.setPrivate("newself");
	} // end test
	
	/** The agent Id of an initialized private belief cannot be updated to empty */
	@Test 
	public void bareInitializedBeliefCannotBeSetToEmptyPrivateAgentId () { 
		try { 
			belief.init();
			belief.setPrivate("");
			fail("A private belief should not be updated with an empty identifier");
		} catch (BeliefMissingValueException bmve) { 
			assertEquals("Cannot set [agent] in private status: Provided identifier is null/empty", bmve.getMessage());
		} // 
	} // end test 
	
	
	/** The agent Id of an initialized private belief cannot be updated to null */
	@Test 
	public void bareInitializedBeliefCannotBeSetToNull() { 
		try { 
			belief.init();
			belief.setPrivate(null);
			fail("A private belief should not be updated with a null identifier");
		} catch (BeliefMissingValueException bmve) { 
			assertEquals("Cannot set [agent] in private status: Provided identifier is null/empty", bmve.getMessage());
		} // 
	} // end test 
	
	
	
	
	/** An initialized belief can be set an agent id for its default private status, which it then also returns */
	@Test
	public void bareInitializedBeliefReturnsSetPrivateAgentId () { 
		belief.init();
		String agent = "newself";
		belief.setPrivate(agent);
		assertEquals(belief.getPrivate(), agent);
	} // end test
	
	
	/** An initialized belief can be set to attributed status */
	@Test
	public void setBareInitializedBeliefToAttributedStatus () { 
		belief.init();
		String attributingAgent = "self";
		LinkedList attributedAgents = new LinkedList<String>();
		attributedAgents.add("human");
		belief.setAttributed(attributingAgent, attributedAgents);
	} // end test
	
	// private method; refactoring out setting up the belief as attributed 
	private void setBeliefToAttributed () { 
		belief.init();
		String attributingAgent = "self";
		LinkedList attributedAgents = new LinkedList<String>();
		attributedAgents.add("human");
		belief.setAttributed(attributingAgent, attributedAgents);
	} // end setBeliefToAttributed
	
	
	/** An initialized belief set to attributed status does not have private epistemic status */
	@Test
	public void attributedBeliefDoesNotHavePrivateStatus () { 
		this.setBeliefToAttributed();
		assertEquals(belief.isPrivate(), false);
	} // end test
	
	/** An initialized belief set to attributed status does have attributed epistemic status */
	@Test
	public void attributedBeliefDoesHaveAttributedStatus () { 
		this.setBeliefToAttributed();
		assertEquals(belief.isAttributed(), true);
	} // end test
	
	/** A belief with attributed status can return the attributing agent*/
	@Test
	public void attributedBeliefCanReturnAttributingAgent () { 
		this.setBeliefToAttributed();
		assertEquals(belief.getAttributingAgent(),"self");
	} // end test
	
	/** A belief with attributed status can return the attributed-to agents */
	@Test
	public void attributedBeliefCanReturnAttributedToAgents () { 
		this.setBeliefToAttributed(); 
		LinkedList attribAgents = belief.getAttributedToAgents(); 
		assertEquals((String)attribAgents.element(),"human");
	} // end test
	
	/** A belief cannot be set to attributed status with an empty attributing agent */
	@Test
	public void cannotSetBeliefAttributedStatusWithEmptyAttributingAgent () { 
		belief.init();
		try { 
			String attributingAgent = "";
			LinkedList attributedAgents = new LinkedList<String>();
			attributedAgents.add("human");
			belief.setAttributed(attributingAgent, attributedAgents);
		} catch (BeliefMissingValueException bmve) { 
			assertEquals("Cannot set [attributing agent] in attributed status: Provided identifier is null/empty", bmve.getMessage());
		}
	} // end test
	
	/** A belief cannot be set to attributed status with a null attributing agent */
	@Test
	public void cannotSetBeliefAttributedStatusWithNullAttributingAgent () { 
		belief.init();
		try { 
			String attributingAgent = null;
			LinkedList attributedAgents = new LinkedList<String>();
			attributedAgents.add("human");
			belief.setAttributed(attributingAgent, attributedAgents);
		} catch (BeliefMissingValueException bmve) { 
			assertEquals("Cannot set [attributing agent] in attributed status: Provided identifier is null/empty", bmve.getMessage());
		}
	} // end test
	
	/** A belief cannot be set to attributed status with an empty attributed agents list */
	@Test
	public void cannotSetBeliefAttributedStatusWithEmptyAttributedAgents () { 
		belief.init();
		try { 
			String attributingAgent = "self";
			belief.setAttributed(attributingAgent, new LinkedList<String>());
		} catch (BeliefMissingValueException bmve) { 
			assertEquals("Cannot set [attributed agent] in attributed status: Provided list is null/empty", bmve.getMessage());
		}
	} // end test
	
	/** A belief cannot be set to attributed status with a null attributing agent */
	@Test
	public void cannotSetBeliefAttributedStatusWithNullAttributedAgents () { 
		belief.init();
		try { 
			String attributingAgent = "self";
			belief.setAttributed(attributingAgent, null);
		} catch (BeliefMissingValueException bmve) { 
			assertEquals("Cannot set [attributed agent] in attributed status: Provided list is null/empty", bmve.getMessage());
		}
	} // end test
	
	/** An initialized belief can be set to shared status */
	@Test
	public void setInitializedBeliefToSharedStatus () { 
		belief.init();
		LinkedList agents = new LinkedList<String>();
		agents.add("self");
		agents.add("human");
		belief.setShared(agents);
	}
	
	/** A non-initialized belief cannot be set to shared status */
	@Test
	public void cannotSetSharedStatusForNonInitializedBelief () { 
		belief = new Belief();
		try { 
			LinkedList agents = new LinkedList<String>();
			agents.add("self");
			agents.add("human");
			belief.setShared(agents);
		} catch (BeliefNotInitializedException bnie) {
			assertEquals("Cannot set [shared status] for non-initialized belief",bnie.getMessage());
		} // end try..catch	
	} // end test
	
	/** An initialized belief cannot be set to shared status with an empty list*/
	@Test
	public void cannotSetSharedStatusWithEmptyAgentsList() { 
		belief.init();
		try { 
			LinkedList agents = new LinkedList<String>();
			belief.setShared(agents);
		} catch (BeliefMissingValueException bnie) {
			assertEquals("Cannot set [agents] in shared status: Provided list is null/empty",bnie.getMessage());
		} // end try..catch	
	} // end test
	
	/** An initialized belief cannot be set to shared status with an empty list*/
	@Test
	public void cannotSetSharedStatusWithNull () { 
		belief.init();
		try { 
			belief.setShared(null);
		} catch (BeliefMissingValueException bnie) {
			assertEquals("Cannot set [agents] in shared status: Provided list is null/empty",bnie.getMessage());
		} // end try..catch	
	} // end test
	
	
	/** A private belief can be updated to shared status */
	@Test
	public void updatePrivateBeliefToSharedStatus () {
		try { 
			belief.init();
			belief.setPrivate("self");
			LinkedList otherAgents = new LinkedList<String>();
			otherAgents.add("human");
			belief.updatePrivateToShared(otherAgents);
		} catch (BeliefInvalidOperationException bioe) {
			fail("A private belief can be updated to shared status");
		}
	}
		
	/** A private belief cannot be updated to shared status with an empty list */
	@Test
	public void cannotUpdatePrivateBeliefToSharedWithEmptyAgentsList () { 
		belief.init();
		belief.setPrivate("self");
		try { 
			LinkedList otherAgents = new LinkedList<String>();
			belief.updatePrivateToShared(otherAgents);
			fail("A private belief cannot be updated to shared status with an empty list");
		} catch (BeliefMissingValueException bmve) {
			assertEquals("Cannot update private to shared status with [agents]: Provided list is null/empty",bmve.getMessage());
		} catch (BeliefInvalidOperationException bioe) {
			fail("A private belief cannot be updated to shared status with an empty list");
		}
	} // end test
	
	/** A private belief cannot be updated to shared status with a null list */
	@Test
	public void cannotUpdatePrivateBeliefToSharedWithNullAgentsList () { 
		belief.init();
		belief.setPrivate("self");
		try { 
			belief.updatePrivateToShared(null);
			fail("A private belief cannot be updated to shared status with a null list");
		} catch (BeliefMissingValueException bmve) {
			assertEquals("Cannot update private to shared status with [agents]: Provided list is null/empty",bmve.getMessage());
		} catch (BeliefInvalidOperationException bioe) {
			fail("A private belief cannot be updated to shared status with a null list");
		}
	} // end test
	
	/** A non-private belief cannot be updated as private belief to shared status */
	@Test
	public void cannotUpdateNonPrivateBeliefToSharedStatus () { 
		setBeliefToAttributed();
		try { 
			LinkedList otherAgents = new LinkedList<String>();
			otherAgents.add("human");
			belief.updatePrivateToShared(otherAgents); 
		} catch (BeliefInvalidOperationException biqe) { 
			assertEquals("Cannot update a non-private belief to shared status",biqe.getMessage());
		} 
	} // end test 
	
	
	/** An attributed belief can be updated to shared status */
	@Test 
	public void updateAttributedBeliefToSharedStatus () { 
		try { 
			setBeliefToAttributed();
			belief.updateAttributedToShared();
		} catch (BeliefInvalidOperationException bioe) {
			fail("An attributed belief can be updated to shared status");
		}
	} // end test 
	
	/** A non-attributed belief cannot be updated as attributed belief to shared status */
	@Test
	public void cannotUpdateNonAttributedBeliefToSharedStatus () { 
		belief.init();
		try { 
			belief.updateAttributedToShared(); 
		} catch (BeliefInvalidOperationException biqe) { 
			assertEquals("Cannot update a non-attributed belief to shared status",biqe.getMessage());
		}
	} // end test 
	
	
	/** A non-initialized belief cannot be updated as attributed belief to shared status */
	@Test
	public void cannotUpdateNonInitializedBeliefToSharedStatus () { 
		belief = new Belief();
		try { 
			belief.updateAttributedToShared(); 
			fail("A non-initialized belief cannot be updated as attributed belief to shared status");
		} catch (BeliefNotInitializedException bnie) { 
			assertEquals("Cannot update a non-initialized belief to shared status",bnie.getMessage());
		} catch (BeliefInvalidOperationException bioe) {
			fail("A non-initialized belief cannot be updated as attributed belief to shared status");
		}
	} // end test 
	
	
	
} // end class