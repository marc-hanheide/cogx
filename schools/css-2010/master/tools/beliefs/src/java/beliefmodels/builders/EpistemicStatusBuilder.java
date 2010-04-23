
// =================================================================                                                        
// Copyright (C) 2009-2011 Pierre Lison (pierre.lison@dfki.de)                                                                
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


package beliefmodels.builders;

import java.util.List;

import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.epstatus.AttributedEpistemicStatus;
import beliefmodels.autogen.epstatus.PrivateEpistemicStatus;
import beliefmodels.autogen.epstatus.SharedEpistemicStatus;

public class EpistemicStatusBuilder {

	
	
	public static final String ROBOT_AGENT = "robot";
	
	

	/**
	 * Create a private epistemic status for an agent
	 * 
	 * @param agent id of the agent
	 * @return the new private epistemic status
	 * @throws BinderException 
	 * 			null arguments
	 */
	public static PrivateEpistemicStatus createNewPrivateEpistemicStatus (String agent) throws BeliefException {
		if (agent == null || agent.equals("")) {
			throw new BeliefException("Error when setting private epistemic status: agent is null or empty");
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
	public static AttributedEpistemicStatus createNewAttributedEpistemicStatus (String agent, List<String> agents) throws BeliefException {
		if (agent == null || agent.equals("")) {
			throw new BeliefException("Error when setting attributed epistemic status: Attributing agent is null or empty");
		}
		if (agents == null) {
			throw new BeliefException("Error when setting attributed epistemic status: Attributed-to agents is null");
		}
		else if (agents.size() == 0) {
			throw new BeliefException("Error when setting attributed epistemic status: Attributed-to agents length is 0");
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
	public static SharedEpistemicStatus createNewSharedEpistemicStatus (List<String> agents) throws BeliefException {
		if (agents == null) {
			throw new BeliefException("Error when setting shared epistemic status: agents list is null");
		}
		else if (agents.size() == 0) {
			throw new BeliefException("Error when setting shared epistemic status: agents list length is 0");
		}
		return new SharedEpistemicStatus (agents);
	}
	
}
