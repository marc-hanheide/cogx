//
//  BeliefFactory.java
//  
//
//  Created by Geert-Jan Kruijff on 9/24/09.
//  Copyright 2009 __MyCompanyName__. All rights reserved.
//

// ---------------------------------------------------------
// PACKAGE
// ---------------------------------------------------------

package comsys.processing.reference;

// ---------------------------------------------------------
// IMPORTS
// ---------------------------------------------------------

import beliefmodels.adl.Agent;
import beliefmodels.adl.Belief;
import beliefmodels.adl.SpatioTemporalFrame;


import comsys.datastructs.lf.LogicalForm;


public interface BeliefFactory {

	public String getSort(); 
	
	public Belief  constructBelief (LogicalForm lf, Agent[] agents, SpatioTemporalFrame frame); 
	
} // end interface
