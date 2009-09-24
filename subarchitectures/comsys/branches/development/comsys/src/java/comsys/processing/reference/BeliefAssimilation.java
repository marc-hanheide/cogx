//
//  BeliefAssimilation.java
//  
//
//  Created by Geert-Jan Kruijff on 9/24/09.
//  Copyright 2009 __MyCompanyName__. All rights reserved.
//

// ---------------------------------------------------------
// PACKAGE
// ---------------------------------------------------------

package comsys.components.reference;

// ---------------------------------------------------------
// BINDER imports
// ---------------------------------------------------------

// ---------------------------------------------------------
// BELIEFMODEL imports
// ---------------------------------------------------------

import beliefmodels.adl.Belief;

// ---------------------------------------------------------
// COMSYS / LF imports
// ---------------------------------------------------------

import comsys.datastructs.lf.LFNominal;
import comsys.datastructs.lf.LogicalForm;

import comsys.processing.reference.BeliefFactory; 

// ---------------------------------------------------------
// JAVA imports
// ---------------------------------------------------------

import java.util.HashMap; 


/**
 the class <b>BeliefAssimilation</b> assimilates beliefs from reading / binding information, 
 and an abductive proof that establishes the contextually most likely interpretation. 
 

 
 
 @author Geert-Jan Kruijff (gj@dfki.de)
 @started 090924
 @version 090924
*/ 


public class BeliefAssimilation {

	/** HashMap with factories, indexed by sort */
	private HashMap factories = new HashMap(); 
	
	
	/** 
	 constructBelief turns a logical form into a belief class, using the belief factory for the 
	 sort of the root of the logical form. 
	 
	 SILENT ASSUMPTION: this method will primarily be called for logical forms with restrictive references.  
	 attributive content will then be added using the assimilate method. 
	 */ 
	
	public Belief constructBelief (LogicalForm lf) {
		// get the factory for the root
		LFNominal root = lf.root; 
		assert root != null;
		BeliefFactory factory = (BeliefFactory) factories.get(root.sort);
		assert factory != null; 
		return factory.constructBelief(lf);
	} // end constructBelief
	
	
	
	
	
	public void registerBeliefFactory (BeliefFactory bf) { 
		factories.put(bf.getSort(),bf);
	} // end method
	
	
	
} // end class
