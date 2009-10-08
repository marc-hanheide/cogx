//
//  AbstractBeliefFactory.java
//  
//
//  Created by Geert-Jan Kruijff on 9/24/09.
//  Copyright 2009 __MyCompanyName__. All rights reserved.
//

// ---------------------------------------------------------
// PACKAGE
// ---------------------------------------------------------

package comsys.processing.reference.belieffactories;

// ---------------------------------------------------------
// BELIEFMODEL imports
// ---------------------------------------------------------

import beliefmodels.adl.Agent; 
import beliefmodels.adl.AgentStatus;
import beliefmodels.adl.PrivateAgentStatus;
import beliefmodels.adl.AttributedAgentStatus;
import beliefmodels.adl.MutualAgentStatus;
import beliefmodels.adl.Belief; 
import beliefmodels.adl.SpatialInterval; 
import beliefmodels.adl.TemporalInterval; 
import beliefmodels.adl.Perspective;
import beliefmodels.adl.SpatioTemporalFrame; 
import beliefmodels.adl.SpatioTemporalFrame; 

// ---------------------------------------------------------
// COMSYS / LF imports
// ---------------------------------------------------------

import comsys.processing.reference.BeliefFactory;

import comsys.datastructs.lf.LogicalForm;


public abstract class AbstractBeliefFactory 
	implements BeliefFactory
	
{
	
	
	public abstract String getSort();
	

	/** constructs a belief with default settings for agents (human) and spatiotemporal frame (here-and-now) */ 
	
	public Belief constructBelief (LogicalForm lf) { 
		AgentStatus as = createAgentStatus("human");
		return constructBelief(lf,as);
	} // end method
	
	/** constructs a belief with default settings for spatiotemporal frame (here-and-now) */ 
	
	public Belief constructBelief (LogicalForm lf, AgentStatus as) { 
		return constructBelief(lf,as,createHereNowFrame(toAgentSet(as))); 
	} // end method		
	
	
	public abstract Belief constructBelief (LogicalForm lf, AgentStatus as, SpatioTemporalFrame frame); 
	
	
	public Agent[] toAgentSet(AgentStatus as) {
		if (as instanceof PrivateAgentStatus) {
			return new Agent[] {((PrivateAgentStatus) as).ag};
		}
		else if (as instanceof AttributedAgentStatus) {
			return new Agent[] {((AttributedAgentStatus) as).ag, ((AttributedAgentStatus) as).ag2};
		}
		else if (as instanceof MutualAgentStatus) {
			MutualAgentStatus mutual = (MutualAgentStatus) as;
			Agent[] helper = new Agent[mutual.ags.length];
			for (int i = 0; i < helper.length; i++) {
				helper[i] = mutual.ags[i];
			}
			return helper;
		}
		else {
			return new Agent[] { };
		}
	}
	
	
	/** creates an agents-list consisting of a single agent. */
	
	public static AgentStatus createAgentStatus(String id) { 
		//System.out.println("AbstractBeliefFactory.createAgentStatus: private");
		PrivateAgentStatus priv = new PrivateAgentStatus();
		priv.ag = new Agent();
		priv.ag.id = id;
		return priv;
	} // end method
	
	public static AgentStatus createAgentStatus(String id1, String id2) {
		//System.out.println("AbstractBeliefFactory.createAgentStatus: attributed");
		AttributedAgentStatus attrib = new AttributedAgentStatus();
		attrib.ag = new Agent();
		attrib.ag.id = id1;
		attrib.ag2 = new Agent();
		attrib.ag2.id = id2;
		return attrib;
	}
	
	public static AgentStatus createAgentStatus(String[] ags) {
		//System.out.println("AbstractBeliefFactory.createAgentStatus: mutual");
		MutualAgentStatus mutual = new MutualAgentStatus();
		mutual.ags = new Agent[ags.length];
		for (int i = 0; i < mutual.ags.length; i++) {
			mutual.ags[i] = new Agent();
			mutual.ags[i].id = ags[i];
		}
		return mutual;
	}
	
	/** adds the given agent to the list of agents for the given belief. 
	 
	 @returns Belief the updated belief
	 */
	
/*
	protected Belief addAgent (Belief belief, String id) { 
		Agent[] agents = belief.ags; 
		if (agents == null) { 
			agents = createAgent(id);
		} else { 
			Agent[] helper = new Agent[1+agents.length];
			for (int i=0; i < agents.length; i++) { helper[i] = agents[i]; }
			Agent agent = new Agent();
			agent.id = id;
			helper[agents.length] = agent;
			agents = helper;
		} // end if..else
		belief.ags = agents;
		return belief;
	} // end method
*/
	
	/** 
	 creates a spatiotemporal frame with a spatial interval identified with "here", and 
	 a temporal interval identified as "now", starting and ending "now." the perspective 
	 is initialized by the given agent. 
	 */ 

	protected SpatioTemporalFrame createHereNowFrame(Agent[] agents) { 
		SpatialInterval spInt = new SpatialInterval();
		spInt.id = "here";
		TemporalInterval tmpInt = new TemporalInterval();
		tmpInt.id = "now";
		tmpInt.start = "now";
		tmpInt.end   = "now";
		Perspective persp = new Perspective();
		persp.id = "here";
		persp.ags = agents;
		SpatioTemporalFrame frame = new SpatioTemporalFrame();
		frame.spatialint = spInt;
		frame.tempint = tmpInt;
		frame.persp = persp;
		return frame;
	} // end method
	
	
	
} // end class
