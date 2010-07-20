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

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

import java.util.LinkedList;
import java.util.List;

import org.junit.Before;
import org.junit.Test;

import de.dfki.lt.tr.beliefs.data.genericproxies.GenericBelief;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.beliefs.util.BeliefInvalidOperationException;
import de.dfki.lt.tr.beliefs.util.BeliefMissingValueException;

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

	private GenericBelief<?, ?> genericBelief;

	@Before
	public void setUp() {
		genericBelief = Belief.create(dBelief.class);
	}

	/** Tests that the class initializes */
	@Test
	public void getId() {
		assertEquals(genericBelief.getId(), "");
	} // end test

	//	
	// /** A bare initialized belief does not have an identifier */
	// @Test
	// public void initBareBeliefThrowsExceptionOnNullId () {
	// belief.init();
	// try {
	// belief.getId();
	// } catch (BeliefMissingValueException bmve) {
	// assertEquals("In querying a belief: Missing value for [Id]",
	// bmve.getMessage());
	// } // end try..catch
	// } // end test
	//	

	/** A bare initialized belief can be given an identifier */
	@Test
	public void bareBeliefCanBeSetAnId() {
		genericBelief.setId("newId");
		assertEquals(genericBelief.getId(), "newId");
	} // end test

	/** A bare initialized belief can be given a type */
	@Test
	public void bareBeliefCanBeSetAType() {
		genericBelief.setType("percept");
		assertEquals(genericBelief.getType(), "percept");
	} // end test

	/** An initialized belief has a private epistemic status */
	@Test
	public void bareInitializedBeliefIsPrivate() {
		assertEquals(genericBelief.isPrivate(), true);
	} // end test

	/**
	 * An initialized belief has a private epistemic status with agent id "self"
	 */
	@Test
	public void bareInitializedBeliefIsPrivateStatusSelf() {
		String agent = genericBelief.getPrivate();
		assertEquals(agent, "self");
	} // end test

	/**
	 * An initialized belief can be set an agent id for its default private
	 * status
	 */
	@Test
	public void bareInitializedBeliefCanBeSetPrivateAgentId() {
		genericBelief.setPrivate("newself");
		String agent = genericBelief.getPrivate();
		assertEquals(agent, "newself");
	} // end test

	/** An initialized belief can be set to attributed status */
	@Test
	public void setBareInitializedBeliefToAttributedStatus() {
		String attributingAgent = "self";
		LinkedList<String> attributedAgents = new LinkedList<String>();
		attributedAgents.add("human");
		genericBelief.setAttributed(attributingAgent, attributedAgents);
	} // end test

	// private method; refactoring out setting up the belief as attributed
	private void setBeliefToAttributed() {
		String attributingAgent = "self";
		LinkedList<String> attributedAgents = new LinkedList<String>();
		attributedAgents.add("human");
		genericBelief.setAttributed(attributingAgent, attributedAgents);
	} // end setBeliefToAttributed

	/**
	 * An initialized belief set to attributed status does not have private
	 * epistemic status
	 */
	@Test
	public void attributedBeliefDoesNotHavePrivateStatus() {
		this.setBeliefToAttributed();
		assertEquals(genericBelief.isPrivate(), false);
	} // end test

	/**
	 * An initialized belief set to attributed status does have attributed
	 * epistemic status
	 */
	@Test
	public void attributedBeliefDoesHaveAttributedStatus() {
		this.setBeliefToAttributed();
		assertEquals(genericBelief.isAttributed(), true);
	} // end test

	/** A belief with attributed status can return the attributing agent */
	@Test
	public void attributedBeliefCanReturnAttributingAgent() {
		this.setBeliefToAttributed();
		assertEquals(genericBelief.getAttributingAgent(), "self");
	} // end test

	/**
	 * A belief with attributed status can return the attributed-to agents
	 */
	@Test
	public void attributedBeliefCanReturnAttributedToAgents() {
		this.setBeliefToAttributed();
		List<String> attribAgents = genericBelief.getAttributedToAgents();
		assertEquals(attribAgents.get(0), "human");
	} // end test

	/**
	 * A belief cannot be set to attributed status with an empty attributing
	 * agent
	 */
	@Test
	public void cannotSetBeliefAttributedStatusWithEmptyAttributingAgent() {
		try {
			String attributingAgent = "";
			List<String> attributedAgents = new LinkedList<String>();
			attributedAgents.add("human");
			genericBelief.setAttributed(attributingAgent, attributedAgents);
		} catch (BeliefMissingValueException bmve) {
			assertEquals(
					"Cannot set [attributing agent] in attributed status: Provided identifier is null/empty",
					bmve.getMessage());
		}
	} // end test

	/**
	 * A belief cannot be set to attributed status with an empty attributed
	 * agents list
	 */
	@Test
	public void cannotSetBeliefAttributedStatusWithEmptyAttributedAgents() {
		try {
			String attributingAgent = "self";
			genericBelief.setAttributed(attributingAgent,
					new LinkedList<String>());
		} catch (BeliefMissingValueException bmve) {
			assertEquals(
					"Cannot set [attributed agent] in attributed status: Provided list is null/empty",
					bmve.getMessage());
		}
	} // end test

	/** An initialized belief can be set to shared status */
	@Test
	public void setInitializedBeliefToSharedStatus() {

		List<String> agents = new LinkedList<String>();
		agents.add("self");
		agents.add("human");
		genericBelief.setShared(agents);
	}

	/**
	 * An initialized belief cannot be set to shared status with an empty list
	 */
	@Test
	public void cannotSetSharedStatusWithEmptyAgentsList() {
		try {
			List<String> agents = new LinkedList<String>();
			genericBelief.setShared(agents);
		} catch (BeliefMissingValueException bnie) {
			assertEquals(
					"Cannot set [agents] in shared status: Provided list is null/empty",
					bnie.getMessage());
		} // end try..catch
	} // end test

	/** A private belief can be updated to shared status */
	@Test
	public void updatePrivateBeliefToSharedStatus() {
		try {

			genericBelief.setPrivate("self");
			List<String> otherAgents = new LinkedList<String>();
			otherAgents.add("human");
			genericBelief.updatePrivateToShared(otherAgents);
		} catch (BeliefInvalidOperationException bioe) {
			fail("A private belief can be updated to shared status");
		}
	}

	/**
	 * A private belief cannot be updated to shared status with an empty list
	 */
	@Test
	public void cannotUpdatePrivateBeliefToSharedWithEmptyAgentsList() {
		genericBelief.setPrivate("self");
		try {
			List<String> otherAgents = new LinkedList<String>();
			genericBelief.updatePrivateToShared(otherAgents);
			fail("A private belief cannot be updated to shared status with an empty list");
		} catch (BeliefMissingValueException bmve) {
			assertEquals(
					"Cannot update private to shared status with [agents]: Provided list is null/empty",
					bmve.getMessage());
		} catch (BeliefInvalidOperationException bioe) {
			fail("A private belief cannot be updated to shared status with an empty list");
		}
	} // end test

	/**
	 * A non-private belief cannot be updated as private belief to shared status
	 */
	@Test
	public void cannotUpdateNonPrivateBeliefToSharedStatus() {
		setBeliefToAttributed();
		try {
			List<String> otherAgents = new LinkedList<String>();
			otherAgents.add("human");
			genericBelief.updatePrivateToShared(otherAgents);
		} catch (BeliefInvalidOperationException biqe) {
			assertEquals("Cannot update a non-private belief to shared status",
					biqe.getMessage());
		}
	} // end test

	/** An attributed belief can be updated to shared status */
	@Test
	public void updateAttributedBeliefToSharedStatus() {
		try {
			setBeliefToAttributed();
			genericBelief.updateAttributedToShared();
		} catch (BeliefInvalidOperationException bioe) {
			fail("An attributed belief can be updated to shared status");
		}
	} // end test

	/**
	 * A non-attributed belief cannot be updated as attributed belief to shared
	 * status
	 */
	@Test
	public void cannotUpdateNonAttributedBeliefToSharedStatus() {
		try {
			genericBelief.updateAttributedToShared();
		} catch (BeliefInvalidOperationException biqe) {
			assertEquals(
					"Cannot update a non-attributed belief to shared status",
					biqe.getMessage());
		}
	} // end test

} // end class