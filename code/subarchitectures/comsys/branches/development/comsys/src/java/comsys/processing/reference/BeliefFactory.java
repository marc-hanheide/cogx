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

package comsys.components.reference;



public interface BeliefFactory {

	public String getSort(); 
	
	public Belief constructBelief (LogicalForm lf); 
	
} // end interface
