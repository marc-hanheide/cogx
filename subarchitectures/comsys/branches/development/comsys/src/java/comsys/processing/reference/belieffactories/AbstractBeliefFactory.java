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

package comsys.components.reference.belieffactories;

// ---------------------------------------------------------
// BELIEFMODEL imports
// ---------------------------------------------------------

import beliefmodels.adl.Agents; 
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


public class AbstractBeliefFactory 
	implements BeliefFactory
	
{

	/** constructs a belief with default settings for agents (human) and spatiotemporal frame (here-and-now) */ 
	
	public Belief constructBelief (LogicalForm lf) { 
		Agent[] agents = createAgents("human");
		return constructBelief(lf,agents);
	} // end method
	
	/** constructs a belief with default settings for spatiotemporal frame (here-and-now) */ 
	
	public Belief constructBelief (LogicalForm lf, Agents agents) { 
		return constructBelief(lf,agents,createHereNowFrame(agents)); 
	} // end method		
	
	/** creates an agents-list consisting of a single agent. */
	
	protected Agents createAgents(String id) { 
		Agent[] agents = new Agent[1];
		agents[0] = id;		
		return agents; 
	} // end method
	
	/** adds the given agent to the list of agents for the given belief. 
	 
	 @returns Belief the updated belief
	 */
	
	protected Belief addAgent (Belief belief, String id) { 
		Agent[] agents = belief.agents; 
		if (agents == null) { 
			agents = createAgents(id);
		} else { 
			Agent[] helper = new Agent[1+agents.length];
			for (int i=0; i < agents.length; i++) { helper[i] = agents[i]; }
			helper[agents.length] = id;
			agents = helper;
		} // end if..else
		belief.agents = agents;
		return belief;
	} // end method
	
	/** 
	 creates a spatiotemporal frame with a spatial interval identified with "here", and 
	 a temporal interval identified as "now", starting and ending "now." the perspective 
	 is initialized by the given agent. 
	 */ 

	protected SpatioTemporalFrame createHereNowFrame(Agents agents) { 
		SpatialInterval spInt = new SpatialInterval();
		spInt.id = "here";
		TemporalInterval tmpInt = new TemporalInterval();
		tmpInt.id = "now";
		tmpInt.start = "now";
		tmpInt.end   = "now";
		Perspective persp = new Perspective();
		persp.id = "here";
		persp.agents = agents;
		SpatioTemporalFrame frame = new SpatioTemporalFrame();
		frame.spatialint = spInt;
		frame.tempint = tmpInt;
		frame.persp = persp;
		return frame;
	} // end method
	
	
	
} // end class
