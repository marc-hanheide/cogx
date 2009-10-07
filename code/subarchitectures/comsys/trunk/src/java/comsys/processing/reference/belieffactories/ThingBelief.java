//
//  ThingBelief.java
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
import beliefmodels.adl.Belief; 
import beliefmodels.adl.SpatioTemporalFrame; 

// ---------------------------------------------------------
// COMSYS / LF imports
// ---------------------------------------------------------

import comsys.datastructs.lf.LFNominal;
import comsys.datastructs.lf.LogicalForm;
import comsys.processing.reference.BeliefFactory;

/** 
 the class <b>ThingBelief</b> turns a logical form, describing a Thing object, 
 into a Belief about a MaterialObject with several associated properties. 
 
 @author	Geert-Jan Kruijff (gj@dfki.de)
 @started	090924
 @version	090924
 */ 

public class ThingBelief 
	extends AbstractBeliefFactory
{

	private final String sort = "thing";
	
	public String getSort() { return sort; }
	
	

	
	public Belief constructBelief (LogicalForm lf, AgentStatus as, SpatioTemporalFrame frame) { 
		// initialize the belief
		LFNominal root = lf.root;
		assert root.sort.equals(sort);
		Belief belief = new Belief();
		belief.id = "thing";
		belief.sigma = frame;
		belief.ags = as; 
		// create the formula
		
		
		return belief;
	
	} // end method
		
	
} // end class
