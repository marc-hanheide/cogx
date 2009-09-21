//
//  ThingProxy.java
//  
//
//  Created by Geert-Jan Kruijff on 9/21/09.
//  Copyright 2009 __MyCompanyName__. All rights reserved.
//

package comsys.processing.reference.proxyfactories;

// -------------------------------------------------------
// BINDER imports
// -------------------------------------------------------

import binder.autogen.core.Proxy; 

// -------------------------------------------------------
// COMSYS imports
// -------------------------------------------------------

import comsys.datastructs.lf.LogicalForm; 
import comsys.processing.reference.ProxyFactory;
import comsys.processing.reference.ProxyResults;

/**
	Thing-proxy construction does 
*/ 


public class ThingProxy 
		extends AbstractProxyFactory 
{

	private String sort = "thing";
	
	public String getSort () { return sort; }
	
	public ProxyResults constructProxy (LogicalForm lf) { 
		assert lf.root.sort.equals("thing");
		ProxyResults prxs = new ProxyResults();
		
		
		
		
		return prxs; 
	} // end method
	
} // end class
