package binder.builders;

import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryPointer;
import binder.arch.BinderException;
import binder.autogen.beliefs.PerceptBelief;
import binder.autogen.distribs.ProbDistribution;
import binder.autogen.epstatus.AttributedEpistemicStatus;
import binder.autogen.epstatus.EpistemicStatus;
import binder.autogen.epstatus.PrivateEpistemicStatus;
import binder.autogen.epstatus.SharedEpistemicStatus;
import binder.autogen.framing.SimpleSpatioTemporalFrame;
import binder.autogen.framing.SpatioTemporalFrame;
import binder.autogen.framing.TemporalInterval;
import binder.autogen.history.PerceptHistory;

public class PerceptBuilder {

	private static int increment = 0;
	
	public static String ROBOT_AGENT = "robot";
	
	
	/**
	 * Create a new spatio-temporal frame with a place, a start time and a end time
	 * 
	 * @param place a string describing the place
	 * @param startTime a starting time
	 * @param endTime an ending time
	 * @return the constructed spatio-temporal frame
	 */
	public static SimpleSpatioTemporalFrame createNewSpatioTemporalFrame 
		(String place, CASTTime startTime, CASTTime endTime) {
		
		TemporalInterval interval = new TemporalInterval(startTime, endTime);
		return new SimpleSpatioTemporalFrame(place, interval);
	}
	
	
	/**
	 * Create a private epistemic status for an agent
	 * 
	 * @param agent id of the agent
	 * @return the new private epistemic status
	 * @throws BinderException 
	 * 			null arguments
	 */
	public static PrivateEpistemicStatus createNewPrivateEpistemicStatus (String agent) throws BinderException {
		if (agent == null || agent.equals("")) {
			throw new BinderException("error, agent is null or empty");
		}
		return new PrivateEpistemicStatus(agent);
	}
	
	
	/**
	 * Create a new attributed epistemic status for an agent to a set of external agents
	 * 
	 * @param agent
	 * @param agents
	 * @return the new attributed epistemic status
	 * @throws BinderException 
	 * 			null arguments
	 */
	public static AttributedEpistemicStatus createNewAttributedEpistemicStatus (String agent, String[] agents) throws BinderException {
		if (agent == null || agent.equals("")) {
			throw new BinderException("error, agent is null or empty");
		}
		if (agents == null) {
			throw new BinderException("error, agents is null");
		}
		else if (agents.length == 0) {
			throw new BinderException("error, agents length is 0");
		}
		return new AttributedEpistemicStatus(agent, agents);
	}
	
	
	/**
	 * Return a shared epistemic status (common ground) for a group of agents
	 * 
	 * @param agents for the group
	 * @return the new shared epistemic status
	 * @throws BinderException 
	 * 			null argument
	 */
	public static SharedEpistemicStatus createNewSharedEpistemicStatus (String[] agents) throws BinderException {
		if (agents == null) {
			throw new BinderException("error, agents is null");
		}
		else if (agents.length == 0) {
			throw new BinderException("error, agents length is 0");
		}
		return new SharedEpistemicStatus (agents);
	}
	
	
	/**
	 * Construct a new perceptual belief
	 * 
	 * @param curPlace the current place
	 * @param curTime the curernt time
	 * @param content the belief content
	 * @param hist the percept history
	 * @return the resulting belief
	 * @throws BinderException 
	 */
	public static PerceptBelief createNewPerceptBelief (String curPlace, CASTTime curTime, ProbDistribution content, PerceptHistory hist) 
		throws BinderException {
		
		if (curPlace == null || curTime == null || content == null || hist == null) {
			throw new BinderException("error, one of the belief component is null");
		}
		
		// constructing the spatio-temporal frame
		SpatioTemporalFrame frame = 
			PerceptBuilder.createNewSpatioTemporalFrame(curPlace, curTime, curTime);
		
		// constructing the epistemic status
		EpistemicStatus status = 
			PerceptBuilder.createNewPrivateEpistemicStatus(ROBOT_AGENT);
	
		// forging a new identifier
		String id = getNewBeliefId();
		
		// and creating the belief
		return new PerceptBelief(frame,status,id, content,hist);
	}
	
	
	public static PerceptHistory createNewPerceptHistory (WorkingMemoryPointer origin) {
		return new PerceptHistory(origin);
	}
	
	/**
	 * Forge a new, unique belief identifier
	 * 
	 * @return the new id
	 */
	private static String getNewBeliefId() {
		increment++;
		return "b" + increment;
	}
}
