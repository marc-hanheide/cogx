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
// IMPORTS

// Java
import java.util.LinkedList;

// Slice
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;
import de.dfki.lt.tr.beliefs.slice.epstatus.AttributedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.PrivateEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.SharedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;

// Belief API Util 
import de.dfki.lt.tr.beliefs.util.BeliefInvalidOperationException;
import de.dfki.lt.tr.beliefs.util.BeliefInvalidQueryException;
import de.dfki.lt.tr.beliefs.util.BeliefMissingValueException;
import de.dfki.lt.tr.beliefs.util.BeliefNotInitializedException;

//=================================================================
// CLASS

/**
 * The <tt>Belief</tt> class provides access to the underlying, slice-based data structures for
 * situated beliefs. The class focuses on accessing the various parameters of a belief (identifier, 
 * type), and accessing and updating epistemic status. The class also provides the principle access
 * to content and the frame of a belief. Computations on these are performed in separate classes (<tt>Content</tt> 
 * and <tt>Frame</tt>, respectively). 
 * <p>
 * To create a new belief, it suffices to call the default constructor. Most methods for accessing the belief 
 * however assume that the belief at least has been properly initialized. (If not, a <tt>BeliefNotInitializedException</tt> is thrown.)
 * This initialization is therefore best done at creation time: 
 * 
 * <pre>
 * 		Belief belief = new Belief();
 * 		belief.init();
 * </pre><p>
 * 
 * A "bare" initialized belief is by default a private belief, with as agent identifier "self." 
 * <p>
 * The class provides several methods for accessing and updating the epistemic status of a belief. For each status 
 * (<i>private, attributed</i>, and <i>shared</i>) there are <tt>setX, getX</tt> and <tt>isX</tt> methods. Furthermore, 
 * the class has individual methods for updating private or attributed beliefs to shared beliefs. An update method constructs the  
 * list of agents for which the shared belief holds, from the agents of the original belief and -possibly- a list of additional agents. 
 * The following code illustrates the creation of a private perceptual belief, setting its agent identifier to "robot", and then updating it to a 
 * shared belief between the robot and the human. 
 * <p>
 * <pre>
 * 		Belief belief = new Belief();
 * 		belief.init();
 * 		belief.setType("percept");
 * 		belief.setPrivate("robot");
 * 		//... now something happens 
 * 		//... which implies the belief is shared with "human"
 * 		LinkedList otherAgents = new LinkedList<String>();
 * 		otherAgents.add("human");
 * 		try { 
 * 			belief.updatePrivateToShared(otherAgents);
 * 		} catch (BeliefNotInitializedException bnie) { 
 * 			System.out.println(bnie.getMessage());
 * 		} catch (BeliefInvalidQueryException biqe) { 
 * 			System.out.println(biqe.getMessage());
 * 		} 
 * 		// now the belief is shared, between "robot" and "human"
 * 		LinkedList agents = belief.getShared();
 * </pre><p>
 * 
 * Modeling the content of a belief is done through the (separate) <tt>Content</tt> class. The <tt>Belief</tt> class provides the access 
 * to this information, via <tt>get/set</tt> methods: 
 * 
 * <pre>
 * 		Content content = new Content();
 * 		content.init();
 * 		// ... flesh out the content
 * 		belief.setContent(content);
 * </pre>
 * 
 * 
 * @see		de.dfki.lt.tr.beliefs.data.Content
 * @see		de.dfki.lt.tr.beliefs.data.Frame
 * @author 	Geert-Jan M. Kruijff (gj@dfki.de)
 * @version	100520
 * @started	100510
 */

public class Belief {
 
	private dBelief _belief = null;
	
	/** 
	 *	Initializes the internal datastructures
	 */
	
	public void init () {
		_belief = new dBelief();
		_belief.estatus = new PrivateEpistemicStatus("self");
	} // end init
	
	/**
	 * Returns whether the belief is empty/null (i.e. has not been initialized yet)
	 * @return boolean True if the belief is empty/null (i.e. not initialized)
	 */
	
	public boolean isEmpty() 	
	{
		return (_belief == null);
	}
		
	/**
	 * Returns the identifier of the belief
	 *
	 * @return The id of the belief
	 * @throws BeliefMissingValueException If the belief has a null or empty identifier
	 */
	
	public String getId () 
	throws BeliefMissingValueException 
	{ 
		if (_belief.id == null || _belief.id.equals("")) {
			throw new BeliefMissingValueException("In querying a belief: Missing value for [Id]");
		} else {
			return _belief.id; 
		} // end if..else check for set Id
	} // end getId
	
	
	/**
	 * Sets the identifier of the belief
	 * 
	 * @param newId The identifier for the belief
	 * @throws BeliefNotInitializedException If the belief has not yet been initialized
	 */
	
	public void setId (String newId) 
	throws BeliefNotInitializedException
	{ 
		if (_belief != null) {
			_belief.id = newId;
		} else {
			throw new BeliefNotInitializedException("Cannot set [Id] for non-initialized belief");
		}
	} // end setId
	
	/**
	 * Returns the type of the belief
	 *
	 * @return The type of the belief
	 * @throws BeliefMissingValueException If the belief has a null or empty type
	 */
	
	public String getType () 
	throws BeliefMissingValueException 
	{ 
		if (_belief.type == null || _belief.type.equals("")) {
			throw new BeliefMissingValueException("In querying a belief: Missing value for [type]");
		} else {
			return _belief.type; 
		} // end if..else check for set type
	} // end getType
	
	
	/**
	 * Sets the type of the belief (as a String)
	 * @param newType The type for the belief
	 * @throws BeliefNotInitializedException If the belief has not yet been initialized
	 */
	
	public void setType (String newType) 
	throws BeliefNotInitializedException 
	{
		if (_belief != null) {
			_belief.type = newType;
		} else {
			throw new BeliefNotInitializedException("Cannot set [type] for non-initialized belief");
		} // end if..else
	} // end setType
	
	
	/**
	 * Returns true if the belief is a private belief
	 * @return boolean True if the belief is a private belief; false otherwise
	 * @throws BeliefNotInitializedException If the belief has not been initialized
	 */
	
	public boolean isPrivate () 
	throws BeliefNotInitializedException 
	{ 
		if (_belief != null) {
			if (_belief.estatus instanceof PrivateEpistemicStatus) { 
				return true;
			} else {
				return false; 
			}
		} else {
			throw new BeliefNotInitializedException("Cannot query [epistemic status] for non-initialized belief");
		}
	} // end isPrivate
	
	/**
	 * If the belief is a private belief, return the agent identifier for that status
	 * @return String The agent identifier of the private belief 
	 * @throws BeliefInvalidQueryException If the belief is not a private belief
	 */
	
	public String getPrivate () 
	throws BeliefInvalidQueryException, BeliefNotInitializedException
	{ 
		if (_belief != null) {
			if (this.isPrivate()) { 
				return ((PrivateEpistemicStatus)_belief.estatus).agent;
			} else { 
				throw new BeliefInvalidQueryException ("Invalid query on belief ["+_belief.id+"]: Belief does not have private status");
			} // end if..else check for appropriate status
		} else { 
			throw new BeliefNotInitializedException("Cannot get [agent] for private status for non-initialized belief");
		} // end if..else check for init belief
	} // end getPrivate
	
	
	/**
	 * If the belief is a private belief, set the agent identifier given the provided value
	 * @param  String agent The agent identifier of the private belief 
	 * @throws BeliefInvalidQueryException If the belief is not a private belief
	 * @throws BeliefNotInitializedException If the belief has not been initialized
	 * @throws BeliefMissingValueException If one of the arguments is null/empty
	 */
	
	public void setPrivate (String agent) 
	throws BeliefInvalidQueryException, BeliefNotInitializedException, BeliefMissingValueException 
	{ 	
		if (agent == null || agent.equals("")) 
		{ 
			throw new BeliefMissingValueException("Cannot set [agent] in private status: Provided identifier is null/empty");
		} 
		else if (_belief != null) 
		{
			if (this.isPrivate()) 
			{ 
				((PrivateEpistemicStatus)_belief.estatus).agent = agent;
			} 
			else 
			{ 
				throw new BeliefInvalidQueryException ("Invalid query on belief ["+_belief.id+"]: Belief does not have private status");
			} // end if..else check for appropriate status
		} 
		else 
		{ 
			throw new BeliefNotInitializedException("Cannot set [agent] for private status for non-initialized belief");
		} // end if..else check for init belief
	} // end setPrivate
	
	
	
	
	/** 
	 *  Set the epistemic status of a belief to attributed. 
	 *  @param String attributingAgent The agent that holds the belief about others
	 *  @param LinkedList attributedAgents The "others" (agents) to which the belief is attributed
	 *  @throws BeliefNotInitializedException If the belief has not been initialized
	 *  @throws BeliefMissingValueException If one of the parameters is null/empty
	 *  @version unsafe operation, no check made whether another status is set (nor is previous version stored to history) (100520) 
	 */
	
	public void setAttributed (String attributingAgent, LinkedList attributedAgents) 
	throws BeliefNotInitializedException, BeliefMissingValueException  
	{ 
		if (attributingAgent == null || attributingAgent.equals("")) 
		{ 
			throw new BeliefMissingValueException("Cannot set [attributing agent] in attributed status: Provided identifier is null/empty");
		} 
		if (attributedAgents == null || attributedAgents.size() == 0) 
		{ 
			throw new BeliefMissingValueException("Cannot set [attributed agent] in attributed status: Provided list is null/empty");
		} 		
		if (_belief != null) 
		{ 
			AttributedEpistemicStatus astatus = new AttributedEpistemicStatus();
			astatus.agent = attributingAgent;
			astatus.attribagents = attributedAgents;
			_belief.estatus = astatus;
		} 
		else 
		{
			throw new BeliefNotInitializedException("Cannot set [attributed status] for non-initialized belief");
		}
	} // end setAttributed
	
	/**
	 * Returns true if the belief has attributed status
	 * @throws BeliefNotInitializedException If the belief has not been properly initialized
	 */
	
	public boolean isAttributed () 
	throws BeliefNotInitializedException
	{ 
		if (_belief != null) {
			if (_belief.estatus instanceof AttributedEpistemicStatus) { 
				return true;
			} else {
				return false; 
			}
		} else {
			throw new BeliefNotInitializedException("Cannot query [epistemic status] for non-initialized belief");
		}
	} // isAttributed
	
	
	/** 
	 * Returns the attributing agent for a belief with attributed status
	 * @return String The agent attributing the belief to others
	 * @throws BeliefNotInitializedException If the belief has not been properly initialized
	 * @throws BeliefInvalidQueryException If the belief is not of attributed status
	 */
	
	public String getAttributingAgent () 
	throws BeliefNotInitializedException, BeliefInvalidQueryException 
	{ 
		if (_belief != null) {
			if (this.isAttributed()) { 
				return ((AttributedEpistemicStatus)_belief.estatus).agent; 
			} else {
				throw new BeliefInvalidQueryException("Invalid query on belief ["+_belief.id+"]: Belief does not have attributed status");
			}
		} else {
			throw new BeliefNotInitializedException("Cannot query attributing [agent] in attributed status for non-initialized belief");
		}	
	} // end getAttributingAgent
	
	/**
	 * Returns the attributed-to agents for a belief with attributed status
	 * @return LinkedList A list of the agents to whom the belief is attribued
	 * @throws BeliefNotInitializedException If the belief has not been properly initialized
	 * @throws BeliefInvalidQueryException If the belief is not of attributed status
	 */
	public LinkedList<String> getAttributedToAgents () 
	throws BeliefNotInitializedException, BeliefInvalidQueryException 
	{
		if (_belief != null) {
			if (this.isAttributed()) { 
				return (LinkedList<String>)((AttributedEpistemicStatus)_belief.estatus).attribagents; 
			} else {
				throw new BeliefInvalidQueryException("Invalid query on belief ["+_belief.id+"]: Belief does not have attributed status");
			}
		} else {
			throw new BeliefNotInitializedException("Cannot query attributed-to [agents] in attributed status for non-initialized belief");
		}
	} // end getAttributedToAgents
	
	/**
	 * Sets the belief to shared status, with the agents sharing the belief identified by the Ids on the given list
	 * @param LinkedList agents The agents sharing the belief
	 * @throws BeliefMissingValueException If the list is null or empty
	 * @throws BeliefNotInitializedException If the belief has not been properly initialized
	 */
	
	public void setShared (LinkedList agents) 
	throws BeliefMissingValueException, BeliefNotInitializedException 
	{
		if (agents == null || agents.size() == 0) {
			throw new BeliefMissingValueException("Cannot set [agents] in shared status: Provided list is null/empty");
		} else {
			if (_belief != null) { 
				SharedEpistemicStatus sstatus = new SharedEpistemicStatus();
				sstatus.cgagents = agents;
				_belief.estatus = sstatus;
			} else { 
				throw new BeliefNotInitializedException("Cannot set [shared status] for non-initialized belief");
			}
		} // end if..else check for proper argument
	} // end setShared
	
	/**
	 * Returns whether the belief has shared epistemic status. 
	 * @return boolean True if the belief has shared epistemic status
	 * @throws BeliefNotInitializedException If the belief has not been properly initialized
	 */
	
	public boolean isShared () 
	throws BeliefNotInitializedException 
	{
		if (_belief == null) 
		{
			throw new BeliefNotInitializedException("Cannot query [shared status] for non-initialized belief");
		}
		return (_belief.estatus instanceof SharedEpistemicStatus);
	} // end isShared
	
	
	/**
	 * Returns the list of agents for a belief with shared status
	 * @return LinkedList A list of the agents who are sharing the belief
	 * @throws BeliefNotInitializedException If the belief has not been properly initialized
	 * @throws BeliefInvalidQueryException If the belief is not of shared status
	 */
	
	public LinkedList<String> getShared () 
	throws BeliefNotInitializedException, BeliefInvalidQueryException
	{ 
		if (_belief == null) 
		{ 
			throw new BeliefNotInitializedException("Cannot query [agents] in shared status for non-initialized belief"); 
		}
		if (!this.isShared())
		{
			throw new BeliefInvalidQueryException("Invalid query on belief ["+_belief.id+"]: Belief does not have shared status");
		}
		return (LinkedList<String>)((SharedEpistemicStatus)_belief.estatus).cgagents;
	} // end getShared
		
	/**
	 * Updates the belief (assumed and checked to be private) to shared status. The list of agents that share the belief is composed from 
	 * the agent for which the original private belief held, and the provided list of additional "other" agents.
	 * @param LinkedList otherAgents The other agents sharing the belief with the agent of the original private belief
	 * @throws BeliefMissingValueException If the list is null or empty
	 * @throws BeliefNotInitializedException If the belief has not been properly initialized
	 * @throws BeliefInvalidOperationException If the belief is not a private belief
	 */
	
	public void updatePrivateToShared (LinkedList otherAgents)
	throws BeliefMissingValueException, BeliefNotInitializedException, BeliefInvalidOperationException 
	{
		if (_belief == null) 
		{ 
			throw new BeliefNotInitializedException ("Cannot update a non-initialized belief to shared status");
		}
		if (otherAgents == null || otherAgents.size() == 0) 
		{
			throw new BeliefMissingValueException("Cannot update private to shared status with [agents]: Provided list is null/empty");
		}
		if (!this.isPrivate())
		{ 	
			throw new BeliefInvalidOperationException("Cannot update a non-private belief to shared status");
		} 
		LinkedList agents = new LinkedList<String>();
		agents.add(((PrivateEpistemicStatus)_belief.estatus).agent);
		agents.addAll(otherAgents);
		SharedEpistemicStatus sstatus = new SharedEpistemicStatus();
		sstatus.cgagents = agents;
		_belief.estatus = sstatus;
	} // end updatePrivateToShared
	
	/**
	 * Updates the attributed belief to shared status. The list of agents sharing the belief is composed of the attributing agent, 
	 * and the attributed-to agents. 
	 * @throws BeliefInvalidOperationException If the belief is not an attributed belief
	 * @throws BeliefNotInitializedException If the belief has not been properly initialized
	 */
	
	public void updateAttributedToShared () 
	throws BeliefNotInitializedException, BeliefInvalidOperationException
	{ 
		if (_belief == null) 
		{ 
			throw new BeliefNotInitializedException ("Cannot update a non-initialized belief to shared status");
		}
		if (!this.isAttributed()) 
		{
			throw new BeliefInvalidOperationException("Cannot update a non-attributed belief to shared status");
		}
		LinkedList agents = new LinkedList<String>();
		agents.add(((AttributedEpistemicStatus)_belief.estatus).agent);
		agents.addAll(((AttributedEpistemicStatus)_belief.estatus).attribagents);
		SharedEpistemicStatus sstatus = new SharedEpistemicStatus();
		sstatus.cgagents = agents;
		_belief.estatus = sstatus;
	} // end updateAttributedToShared
	
	/**
	 * Sets the content of the belief to the provided object.
	 * @param  content 	The content to store in the belief
	 * @see Content
	 * @throws BeliefNotInitializedException If the belief is not initialized
	 * @throws BeliefMissingValueException If the content is empty or null
	 */
	
	public void setContent (Content content) 
	throws BeliefNotInitializedException, BeliefMissingValueException 
	{
		// ensure that the content and the belief are non-null, non-empty
		if (_belief == null) 
		{
			throw new BeliefNotInitializedException("Cannot set [content] in a non-initialized belief"); 
		}
		if (content == null || content.isEmpty()) 
		{
			throw new BeliefMissingValueException("Cannot set [content] in belief: Provided object is null/empty ");
		} 
		// set the content parameter
		_belief.content = content.getDistribution();	
	} // end setContent
	
	/** 
	 * Returns the content of the belief, as a Content object
	 * @return Content The content of the belief
	 * @see Content
	 * @throws BeliefNotInitializedException If the belief is not initialized
	 */
	
	public Content getContent() 
	throws BeliefNotInitializedException 
	{
		// ensure that the belief is non-null, non-empty
		if (_belief == null) 
		{
			throw new BeliefNotInitializedException("Cannot get [content] from a non-initialized belief"); 
		}	
		// return the content
		Content content = new Content();
		content.init();
		content.setDistribution(_belief.content);
		return content;
	} // end getContent
	
	
	/**
	 * Sets the frame of the belief
	 * @param 	frame 	The frame to be used
	 * @throws	BeliefMissingValueException If the frame is null
	 * @throws	BeliefInvalidOperationException If the belief is null/empty
	 * @see		Frame
	 */
	public void setFrame (Frame frame)
	throws BeliefMissingValueException, BeliefInvalidOperationException
	{
		if (frame == null)
		{
			throw new BeliefMissingValueException("Cannot set frame: Provided frame is null");
		}
		if (this.isEmpty()) 
		{
			throw new BeliefInvalidOperationException("Cannot set frame for non-initialized belief");
		}
		_belief.frame = frame.getFrame();
	} // end setFrame
	
} // end class
