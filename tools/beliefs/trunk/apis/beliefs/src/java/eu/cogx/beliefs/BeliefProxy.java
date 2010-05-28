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
package eu.cogx.beliefs;

//=================================================================
// IMPORTS

// Java
import java.util.LinkedList;

import de.dfki.lt.tr.beliefs.slice.epstatus.AttributedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.PrivateEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.SharedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.beliefs.util.BeliefInvalidOperationException;
import de.dfki.lt.tr.beliefs.util.BeliefInvalidQueryException;
import de.dfki.lt.tr.beliefs.util.BeliefMissingValueException;
import de.dfki.lt.tr.beliefs.util.BeliefNotInitializedException;

//=================================================================
// CLASS

/**
 * The <tt>Belief</tt> class provides access to the underlying, slice-based data
 * structures for situated beliefs. The class focuses on accessing the various
 * parameters of a belief (identifier, type), and accessing and updating
 * epistemic status. The class also provides the principle access to content and
 * the frame of a belief. Computations on these are performed in separate
 * classes (<tt>Content</tt> and <tt>Frame</tt>, respectively).
 * <p>
 * To create a new belief, it suffices to call the default constructor. Most
 * methods for accessing the belief however assume that the belief at least has
 * been properly initialized. (If not, a <tt>BeliefNotInitializedException</tt>
 * is thrown.) This initialization is therefore best done at creation time:
 * 
 * <pre>
 * Belief belief = new Belief();
 * belief.init();
 * </pre>
 * <p>
 * 
 * A "bare" initialized belief is by default a private belief, with as agent
 * identifier "self."
 * <p>
 * The class provides several methods for accessing and updating the epistemic
 * status of a belief. For each status (<i>private, attributed</i>, and
 * <i>shared</i>) there are <tt>setX, getX</tt> and <tt>isX</tt> methods.
 * Furthermore, the class has individual methods for updating private or
 * attributed beliefs to shared beliefs. An update method constructs the list of
 * agents for which the shared belief holds, from the agents of the original
 * belief and -possibly- a list of additional agents. The following code
 * illustrates the creation of a private perceptual belief, setting its agent
 * identifier to "robot", and then updating it to a shared belief between the
 * robot and the human.
 * <p>
 * 
 * <pre>
 * Belief belief = new Belief();
 * belief.init();
 * belief.setType(&quot;percept&quot;);
 * belief.setPrivate(&quot;robot&quot;);
 * //... now something happens 
 * //... which implies the belief is shared with &quot;human&quot;
 * LinkedList otherAgents = new LinkedList&lt;String&gt;();
 * otherAgents.add(&quot;human&quot;);
 * try {
 * 	belief.updatePrivateToShared(otherAgents);
 * } catch (BeliefNotInitializedException bnie) {
 * 	System.out.println(bnie.getMessage());
 * } catch (BeliefInvalidQueryException biqe) {
 * 	System.out.println(biqe.getMessage());
 * }
 * // now the belief is shared, between &quot;robot&quot; and &quot;human&quot;
 * LinkedList agents = belief.getShared();
 * </pre>
 * <p>
 * 
 * Modeling the content of a belief is done through the (separate)
 * <tt>Content</tt> class. The <tt>Belief</tt> class provides the access to this
 * information, via <tt>get/set</tt> methods:
 * 
 * <pre>
 * Content content = new Content();
 * content.init();
 * // ... flesh out the content
 * belief.setContent(content);
 * </pre>
 * 
 * 
 * @see de.dfki.lt.tr.beliefs.data.Content
 * @see de.dfki.lt.tr.beliefs.data.Frame
 * @author Geert-Jan M. Kruijff (gj@dfki.de)
 * @version 100520
 * @started 100510
 */

public class BeliefProxy implements Proxy {

	protected final dBelief _belief;

	/**
	 * Initializes the internal datastructures
	 */

	public BeliefProxy(String id) {
		_belief = new dBelief();
		_belief.id = id;
		_belief.estatus = new PrivateEpistemicStatus("self");
		_belief.frame = new FrameProxy().getFrame();
		_belief.content = new ConditionallyIndependentDistributionProxy().getContent();
	}

	private void checkConsistency() {
		assert (!_belief.id.isEmpty());
	}

	public BeliefProxy(dBelief belief) {
		this._belief = belief;
	}

	public final Ice.Object get() {
		return _belief;
	}
	
	/**
	 * Returns the identifier of the belief
	 * 
	 * @return The id of the belief
	 * @throws BeliefMissingValueException
	 *             If the belief has a null or empty identifier
	 */

	public String getId() throws BeliefMissingValueException {
		checkConsistency();
		return _belief.id;
	} // end getId

	/**
	 * Returns the type of the belief
	 * 
	 * @return The type of the belief
	 * @throws BeliefMissingValueException
	 *             If the belief has a null or empty type
	 */

	public String getType() throws BeliefMissingValueException {
		checkConsistency();
		return _belief.type;
	} // end getType

	/**
	 * Sets the type of the belief (as a String)
	 * 
	 * @param newType
	 *            The type for the belief
	 * @throws BeliefNotInitializedException
	 *             If the belief has not yet been initialized
	 */

	public void setType(String newType) throws BeliefNotInitializedException {
		_belief.type = newType;
	} // end setType

	/**
	 * Returns true if the belief is a private belief
	 * 
	 * @return boolean True if the belief is a private belief; false otherwise
	 * @throws BeliefNotInitializedException
	 *             If the belief has not been initialized
	 */

	public boolean isPrivate() throws BeliefNotInitializedException {
		if (_belief.estatus instanceof PrivateEpistemicStatus) {
			return true;
		} else {
			return false;
		}
	} // end isPrivate

	/**
	 * If the belief is a private belief, return the agent identifier for that
	 * status
	 * 
	 * @return String The agent identifier of the private belief
	 * @throws BeliefInvalidQueryException
	 *             If the belief is not a private belief
	 */

	public String getPrivate() throws BeliefInvalidQueryException,
			BeliefNotInitializedException {
		if (this.isPrivate()) {
			return ((PrivateEpistemicStatus) _belief.estatus).agent;
		} else {
			throw new BeliefInvalidQueryException("Invalid query on belief ["
					+ _belief.id + "]: Belief does not have private status");
		} // end if..else check for appropriate status
	} // end getPrivate

	/**
	 * If the belief is a private belief, set the agent identifier given the
	 * provided value
	 * 
	 * @param String
	 *            agent The agent identifier of the private belief
	 * @throws BeliefInvalidQueryException
	 *             If the belief is not a private belief
	 * @throws BeliefNotInitializedException
	 *             If the belief has not been initialized
	 * @throws BeliefMissingValueException
	 *             If one of the arguments is null/empty
	 */

	public void setPrivate(String agent) throws BeliefInvalidQueryException,
			BeliefNotInitializedException, BeliefMissingValueException {
		if (agent == null || agent.equals("")) {
			throw new BeliefMissingValueException(
					"Cannot set [agent] in private status: Provided identifier is null/empty");
		} else {
			checkConsistency();
			if (this.isPrivate()) {
				((PrivateEpistemicStatus) _belief.estatus).agent = agent;
			} else {
				throw new BeliefInvalidQueryException(
						"Invalid query on belief [" + _belief.id
								+ "]: Belief does not have private status");
			} // end if..else check for appropriate status
		} // end if..else check for init belief
	} // end setPrivate

	/**
	 * Set the epistemic status of a belief to attributed.
	 * 
	 * @param String
	 *            attributingAgent The agent that holds the belief about others
	 * @param LinkedList
	 *            attributedAgents The "others" (agents) to which the belief is
	 *            attributed
	 * @throws BeliefNotInitializedException
	 *             If the belief has not been initialized
	 * @throws BeliefMissingValueException
	 *             If one of the parameters is null/empty
	 * @version unsafe operation, no check made whether another status is set
	 *          (nor is previous version stored to history) (100520)
	 */

	public void setAttributed(String attributingAgent,
			LinkedList<String> attributedAgents)
			throws BeliefNotInitializedException, BeliefMissingValueException {
		if (attributingAgent == null || attributingAgent.equals("")) {
			throw new BeliefMissingValueException(
					"Cannot set [attributing agent] in attributed status: Provided identifier is null/empty");
		}
		if (attributedAgents == null || attributedAgents.size() == 0) {
			throw new BeliefMissingValueException(
					"Cannot set [attributed agent] in attributed status: Provided list is null/empty");
		}
		checkConsistency();
		AttributedEpistemicStatus astatus = new AttributedEpistemicStatus();
		astatus.agent = attributingAgent;
		astatus.attribagents = attributedAgents;
		_belief.estatus = astatus;
	} // end setAttributed

	/**
	 * Returns true if the belief has attributed status
	 * 
	 * @throws BeliefNotInitializedException
	 *             If the belief has not been properly initialized
	 */

	public boolean isAttributed() throws BeliefNotInitializedException {
		if (_belief.estatus instanceof AttributedEpistemicStatus) {
			return true;
		} else {
			return false;
		}
	} // isAttributed

	/**
	 * Returns the attributing agent for a belief with attributed status
	 * 
	 * @return String The agent attributing the belief to others
	 * @throws BeliefNotInitializedException
	 *             If the belief has not been properly initialized
	 * @throws BeliefInvalidQueryException
	 *             If the belief is not of attributed status
	 */

	public String getAttributingAgent() throws BeliefNotInitializedException,
			BeliefInvalidQueryException {
		if (this.isAttributed()) {
			return ((AttributedEpistemicStatus) _belief.estatus).agent;
		} else {
			throw new BeliefInvalidQueryException("Invalid query on belief ["
					+ _belief.id + "]: Belief does not have attributed status");
		}
	} // end getAttributingAgent

	/**
	 * Returns the attributed-to agents for a belief with attributed status
	 * 
	 * @return LinkedList A list of the agents to whom the belief is attribued
	 * @throws BeliefNotInitializedException
	 *             If the belief has not been properly initialized
	 * @throws BeliefInvalidQueryException
	 *             If the belief is not of attributed status
	 */
	public LinkedList<String> getAttributedToAgents()
			throws BeliefNotInitializedException, BeliefInvalidQueryException {
		if (this.isAttributed()) {
			return (LinkedList<String>) ((AttributedEpistemicStatus) _belief.estatus).attribagents;
		} else {
			throw new BeliefInvalidQueryException("Invalid query on belief ["
					+ _belief.id + "]: Belief does not have attributed status");
		}
	} // end getAttributedToAgents

	/**
	 * Sets the belief to shared status, with the agents sharing the belief
	 * identified by the Ids on the given list
	 * 
	 * @param LinkedList
	 *            agents The agents sharing the belief
	 * @throws BeliefMissingValueException
	 *             If the list is null or empty
	 * @throws BeliefNotInitializedException
	 *             If the belief has not been properly initialized
	 */

	public void setShared(LinkedList<String> agents)
			throws BeliefMissingValueException, BeliefNotInitializedException {
		if (agents == null || agents.size() == 0) {
			throw new BeliefMissingValueException(
					"Cannot set [agents] in shared status: Provided list is null/empty");
		} else {
			SharedEpistemicStatus sstatus = new SharedEpistemicStatus();
			sstatus.cgagents = agents;
			_belief.estatus = sstatus;
		} // end if..else check for proper argument
	} // end setShared

	/**
	 * Returns whether the belief has shared epistemic status.
	 * 
	 * @return boolean True if the belief has shared epistemic status
	 * @throws BeliefNotInitializedException
	 *             If the belief has not been properly initialized
	 */

	public boolean isShared() throws BeliefNotInitializedException {
		return (_belief.estatus instanceof SharedEpistemicStatus);
	} // end isShared

	/**
	 * Returns the list of agents for a belief with shared status
	 * 
	 * @return LinkedList A list of the agents who are sharing the belief
	 * @throws BeliefNotInitializedException
	 *             If the belief has not been properly initialized
	 * @throws BeliefInvalidQueryException
	 *             If the belief is not of shared status
	 */

	public LinkedList<String> getShared() throws BeliefNotInitializedException,
			BeliefInvalidQueryException {
		if (!this.isShared()) {
			throw new BeliefInvalidQueryException("Invalid query on belief ["
					+ _belief.id + "]: Belief does not have shared status");
		}
		return (LinkedList<String>) ((SharedEpistemicStatus) _belief.estatus).cgagents;
	} // end getShared

	/**
	 * Updates the belief (assumed and checked to be private) to shared status.
	 * The list of agents that share the belief is composed from the agent for
	 * which the original private belief held, and the provided list of
	 * additional "other" agents.
	 * 
	 * @param LinkedList
	 *            otherAgents The other agents sharing the belief with the agent
	 *            of the original private belief
	 * @throws BeliefMissingValueException
	 *             If the list is null or empty
	 * @throws BeliefNotInitializedException
	 *             If the belief has not been properly initialized
	 * @throws BeliefInvalidOperationException
	 *             If the belief is not a private belief
	 */

	public void updatePrivateToShared(LinkedList<String> otherAgents)
			throws BeliefMissingValueException, BeliefNotInitializedException,
			BeliefInvalidOperationException {
		if (otherAgents == null || otherAgents.size() == 0) {
			throw new BeliefMissingValueException(
					"Cannot update private to shared status with [agents]: Provided list is null/empty");
		}
		if (!this.isPrivate()) {
			throw new BeliefInvalidOperationException(
					"Cannot update a non-private belief to shared status");
		}
		LinkedList<String> agents = new LinkedList<String>();
		agents.add(((PrivateEpistemicStatus) _belief.estatus).agent);
		agents.addAll(otherAgents);
		SharedEpistemicStatus sstatus = new SharedEpistemicStatus();
		sstatus.cgagents = agents;
		_belief.estatus = sstatus;
	} // end updatePrivateToShared

	/**
	 * Updates the attributed belief to shared status. The list of agents
	 * sharing the belief is composed of the attributing agent, and the
	 * attributed-to agents.
	 * 
	 * @throws BeliefInvalidOperationException
	 *             If the belief is not an attributed belief
	 * @throws BeliefNotInitializedException
	 *             If the belief has not been properly initialized
	 */

	public void updateAttributedToShared()
			throws BeliefNotInitializedException,
			BeliefInvalidOperationException {
		if (!this.isAttributed()) {
			throw new BeliefInvalidOperationException(
					"Cannot update a non-attributed belief to shared status");
		}
		LinkedList<String> agents = new LinkedList<String>();
		agents.add(((AttributedEpistemicStatus) _belief.estatus).agent);
		agents
				.addAll(((AttributedEpistemicStatus) _belief.estatus).attribagents);
		SharedEpistemicStatus sstatus = new SharedEpistemicStatus();
		sstatus.cgagents = agents;
		_belief.estatus = sstatus;
	} // end updateAttributedToShared

	/**
	 * Sets the content of the belief to the provided object.
	 * 
	 * @param contentDelegate
	 *            The content to store in the belief
	 * @see ConditionallyIndependentDistributionProxy
	 * @throws BeliefNotInitializedException
	 *             If the belief is not initialized
	 * @throws BeliefMissingValueException
	 *             If the content is empty or null
	 */

	public void setContent(ContentProxy<?> contentDelegate)
			throws BeliefNotInitializedException, BeliefMissingValueException {
		_belief.content = contentDelegate.getContent();
	} // end setContent

	/**
	 * Returns the content of the belief, as a Content object
	 * 
	 * @return Content The content of the belief
	 * @see ConditionallyIndependentDistributionProxy
	 * @throws BeliefNotInitializedException
	 *             If the belief is not initialized
	 */

	public ContentProxy<?> getContent() {
		return ContentProxy.createDelegate(_belief.content);
	} // end getContent

	/**
	 * Sets the frame of the belief
	 * 
	 * @param frameProxy
	 *            The frame to be used
	 * @throws BeliefMissingValueException
	 *             If the frame is null
	 * @throws BeliefInvalidOperationException
	 *             If the belief is null/empty
	 * @see FrameProxy
	 */
	public void setFrame(FrameProxy frameProxy)
			throws BeliefMissingValueException, BeliefInvalidOperationException {
		if (frameProxy == null) {
			throw new BeliefMissingValueException(
					"Cannot set frame: Provided frame is null");
		}
		_belief.frame = frameProxy.getFrame();
	} // end setFrame

} // end class
