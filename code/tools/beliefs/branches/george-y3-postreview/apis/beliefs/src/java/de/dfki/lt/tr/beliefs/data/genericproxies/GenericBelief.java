package de.dfki.lt.tr.beliefs.data.genericproxies;

import java.util.LinkedList;
import java.util.List;

import de.dfki.lt.tr.beliefs.data.abstractproxies.Proxy;
import de.dfki.lt.tr.beliefs.data.abstractproxies.ProxyFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;
import de.dfki.lt.tr.beliefs.slice.epstatus.AttributedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.EpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.PrivateEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.SharedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.framing.AbstractFrame;
import de.dfki.lt.tr.beliefs.slice.framing.SpatioTemporalFrame;
import de.dfki.lt.tr.beliefs.slice.framing.TemporalInterval;
import de.dfki.lt.tr.beliefs.slice.history.AbstractBeliefHistory;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.beliefs.util.BeliefException;
import de.dfki.lt.tr.beliefs.util.BeliefInvalidOperationException;
import de.dfki.lt.tr.beliefs.util.BeliefInvalidQueryException;
import de.dfki.lt.tr.beliefs.util.BeliefMissingValueException;

public class GenericBelief<T extends dBelief, C extends Distribution<?>> extends Proxy<T> {

	public static <T2 extends dBelief, C2 extends Distribution<?>> GenericBelief<T2, C2> create(
			Class<? extends T2> type, ProxyFactory<ProbDistribution, ? extends C2> factory,
			dBelief o) {
		return new GenericBelief<T2, C2>(type, factory, o);
	}

	protected static <T2 extends dBelief> T2 createDefault(Class<? extends T2> type) {
		T2 internal;
		try {
			internal = type.newInstance();
		} catch (InstantiationException e) {
			throw new BeliefException(e.getMessage());
		} catch (IllegalAccessException e) {
			throw new BeliefException(e.getMessage());
		}
		internal.content = new ProbDistribution();
		internal.id = "";
		internal.estatus = new PrivateEpistemicStatus("self");
		internal.frame = new SpatioTemporalFrame("here", new TemporalInterval(),1.0f);
		internal.hist = new AbstractBeliefHistory();
		internal.type = "";
		return internal;

	}
	
	protected final ProxyFactory<ProbDistribution, ? extends C> contentFactory;

	protected GenericBelief(Class<? extends T> class1,
			ProxyFactory<ProbDistribution, ? extends C> factory, dBelief content) {
		super(class1, content);
		contentFactory = factory;
	}
	
	public EpistemicStatus getStatus() {
		return _content.estatus;
	}



	/**
	 * Returns the attributed-to agents for a belief with attributed status
	 * 
	 * @return LinkedList A list of the agents to whom the belief is attribued
	 * @throws BeliefNotInitializedException
	 *             If the belief has not been properly initialized
	 * @throws BeliefInvalidQueryException
	 *             If the belief is not of attributed status
	 */
	public List<String> getAttributedToAgents() {
		if (this.isAttributed()) {
			return ((AttributedEpistemicStatus) _content.estatus).attribagents;
		} else {
			throw new BeliefInvalidQueryException("Invalid query on belief ["
					+ _content.id + "]: Belief does not have attributed status");
		}
	} // end getAttributedToAgents

	/**
	 * Returns the attributing agent for a belief with attributed status
	 * 
	 * @return String The agent attributing the belief to others
	 * @throws BeliefNotInitializedException
	 *             If the belief has not been properly initialized
	 * @throws BeliefInvalidQueryException
	 *             If the belief is not of attributed status
	 */

	public String getAttributingAgent() {
		if (this.isAttributed()) {
			return ((AttributedEpistemicStatus) _content.estatus).agent;
		} else {
			throw new BeliefInvalidQueryException("Invalid query on belief ["
					+ _content.id + "]: Belief does not have attributed status");
		}
	} // end getAttributingAgent

	/**
	 * Returns the content of the belief, as a Content object
	 * 
	 * @return Content The content of the belief
	 * @see ConditionallyIndependentDistributionProxy
	 * @throws BeliefNotInitializedException
	 *             If the belief is not initialized
	 */

	public C getContent() {
		return contentFactory.create(_content.content);
	} // end getContent
	
	public <T2 extends Proxy<? extends ProbDistribution>> T2 getContent(ProxyFactory<ProbDistribution, T2> type)  {
		return type.create(this.get().content);
	}


	public GenericFrame<? extends AbstractFrame> getFrame() {
		return GenericFrame.create(AbstractFrame.class, _content.frame);
	}

	public <T2 extends Proxy<? extends AbstractFrame>> T2 getFrame(ProxyFactory<AbstractFrame, T2> type) {
		return type.create(_content.frame);
	}


	/**
	 * Returns the identifier of the belief
	 * 
	 * @return The id of the belief
	 * @throws BeliefMissingValueException
	 *             If the belief has a null or empty identifier
	 */

	public String getId() {
		return _content.id;
	} // end getId

	/**
	 * If the belief is a private belief, return the agent identifier for that
	 * status
	 * 
	 * @return String The agent identifier of the private belief
	 * @throws BeliefInvalidQueryException
	 *             If the belief is not a private belief
	 */

	public String getPrivate() {
		if (this.isPrivate()) {
			return ((PrivateEpistemicStatus) _content.estatus).agent;
		} else {
			throw new BeliefInvalidQueryException("Invalid query on belief ["
					+ _content.id + "]: Belief does not have private status");
		} // end if..else check for appropriate status
	} // end getPrivate

	/**
	 * Returns the list of agents for a belief with shared status
	 * 
	 * @return LinkedList A list of the agents who are sharing the belief
	 * @throws BeliefNotInitializedException
	 *             If the belief has not been properly initialized
	 * @throws BeliefInvalidQueryException
	 *             If the belief is not of shared status
	 */

	public List<String> getShared() {
		if (!this.isShared()) {
			throw new BeliefInvalidQueryException("Invalid query on belief ["
					+ _content.id + "]: Belief does not have shared status");
		}
		return ((SharedEpistemicStatus) _content.estatus).cgagents;
	} // end getShared

	/**
	 * Returns the type of the belief
	 * 
	 * @return The type of the belief
	 * @throws BeliefMissingValueException
	 *             If the belief has a null or empty type
	 */

	public String getType() {
		return _content.type;
	} // end getType

	/**
	 * Returns true if the belief has attributed status
	 * 
	 * @throws BeliefNotInitializedException
	 *             If the belief has not been properly initialized
	 */

	public boolean isAttributed() {
		if (_content.estatus instanceof AttributedEpistemicStatus) {
			return true;
		} else {
			return false;
		}
	} // isAttributed

	/**
	 * Returns true if the belief is a private belief
	 * 
	 * @return boolean True if the belief is a private belief; false otherwise
	 * @throws BeliefNotInitializedException
	 *             If the belief has not been initialized
	 */

	public boolean isPrivate() {
		if (_content.estatus instanceof PrivateEpistemicStatus) {
			return true;
		} else {
			return false;
		}
	} // end isPrivate

	/**
	 * Returns whether the belief has shared epistemic status.
	 * 
	 * @return boolean True if the belief has shared epistemic status
	 * @throws BeliefNotInitializedException
	 *             If the belief has not been properly initialized
	 */

	public boolean isShared() {
		return (_content.estatus instanceof SharedEpistemicStatus);
	} // end isShared

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
			List<String> attributedAgents) {
		AttributedEpistemicStatus astatus = new AttributedEpistemicStatus();
		astatus.agent = attributingAgent;
		astatus.attribagents = attributedAgents;
		_content.estatus = astatus;
	} // end setAttributed

	/**
	 * Sets the content of the belief to the provided object.
	 * 
	 * @param contentProxy
	 *            The content to store in the belief
	 * @see ConditionallyIndependentDistributionProxy
	 * @throws BeliefNotInitializedException
	 *             If the belief is not initialized
	 * @throws BeliefMissingValueException
	 *             If the content is empty or null
	 */

	public void setContent(C contentProxy) {
		_content.content = (ProbDistribution) contentProxy.get().clone();
	} // end setContent

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
	public void setFrame(GenericFrame<?> frameProxy) {
		if (frameProxy == null) {
			throw new BeliefMissingValueException(
					"Cannot set frame: Provided frame is null");
		}
		_content.frame = frameProxy.get();
	} // end setFrame

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

	public void setPrivate(String agent) throws BeliefInvalidQueryException {
		if (this.isPrivate()) {
			((PrivateEpistemicStatus) _content.estatus).agent = agent;
		} else {
			throw new BeliefInvalidQueryException("Invalid query on belief ["
					+ _content.id + "]: Belief does not have private status");
		} // end if..else check for appropriate status
	} // end setPrivate

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

	public void setShared(List<String> agents) {
		if (agents == null || agents.size() == 0) {
			throw new BeliefMissingValueException(
					"Cannot set [agents] in shared status: Provided list is null/empty");
		} else {
			SharedEpistemicStatus sstatus = new SharedEpistemicStatus();
			sstatus.cgagents = agents;
			_content.estatus = sstatus;
		} // end if..else check for proper argument
	} // end setShared

	/**
	 * Sets the type of the belief (as a String)
	 * 
	 * @param newType
	 *            The type for the belief
	 * @throws BeliefNotInitializedException
	 *             If the belief has not yet been initialized
	 */

	public void setType(String newType) {
		_content.type = newType;
	} // end setType

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Object#toString()
	 */
	@Override
	public String toString() {

		return "CondIndepedentFormulaBeliefProxy [_proxyFor=" + _proxyFor
				+ ", getId()=" + getId() + ", getType()=" + getType()
				+ ", distribution: "
				+ contentFactory.create(_content.content).toString() + "]";
	}

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

	public void updateAttributedToShared() {
		if (!this.isAttributed()) {
			throw new BeliefInvalidOperationException(
					"Cannot update a non-attributed belief to shared status");
		}
		LinkedList<String> agents = new LinkedList<String>();
		agents.add(((AttributedEpistemicStatus) _content.estatus).agent);
		agents
				.addAll(((AttributedEpistemicStatus) _content.estatus).attribagents);
		SharedEpistemicStatus sstatus = new SharedEpistemicStatus();
		sstatus.cgagents = agents;
		_content.estatus = sstatus;
	} // end updateAttributedToShared

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

	public void updatePrivateToShared(List<String> otherAgents) {
		if (!this.isPrivate()) {
			throw new BeliefInvalidOperationException(
					"Cannot update a non-private belief to shared status");
		}
		if (otherAgents.isEmpty()) {
			throw new BeliefMissingValueException(
					"Cannot update private to shared status with [agents]: Provided list is null/empty");
		}
		List<String> agents = new LinkedList<String>();
		agents.add(((PrivateEpistemicStatus) _content.estatus).agent);
		agents.addAll(otherAgents);
		SharedEpistemicStatus sstatus = new SharedEpistemicStatus();
		sstatus.cgagents = agents;
		_content.estatus = sstatus;
	} // end updatePrivateToShared

	public void setId(String string) {
		_content.id = string;

	}

}
