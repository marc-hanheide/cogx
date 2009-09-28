//
//  RestrictorResolution.java
//  
//
//  Created by Geert-Jan Kruijff on 9/21/09.
//  Copyright 2009 __MyCompanyName__. All rights reserved.
//

package comsys.processing.reference;

// -------------------------------------------------------
// BINDER imports
// -------------------------------------------------------

import binder.autogen.core.Proxy;

// -------------------------------------------------------
// COMSYS imports
// -------------------------------------------------------

import cast.cdl.CASTTime;
import comsys.datastructs.comsysEssentials.RefReading;
import comsys.datastructs.comsysEssentials.RefReadings;

import comsys.processing.reference.proxyfactories.ThingProxy;

// -------------------------------------------------------
// JAVA imports
// -------------------------------------------------------

import java.util.HashMap;

// -------------------------------------------------------
// LF imports
// -------------------------------------------------------

import comsys.datastructs.lf.LogicalForm;


/**
 For each logical form subtree in the set of restrictors for a referential reading of a logical form, 
 we construct a proxy. 
 
 @author	Geert-Jan Kruijff
 @email		gj@dfki.de
 @started	090921
 @version	090922 
 */ 


public class RestrictorProxyConstruction {

	
	private HashMap proxyFactories = null; 
	
	public RestrictorProxyConstruction () { 
		init();
	} // end 
	
	private void init () {
		proxyFactories = new HashMap();
		ThingProxy thingProxy = new ThingProxy();
		
		proxyFactories.put(thingProxy.getSort(), thingProxy);
	} // end init
	
	public ProxyResults constructProxy (LogicalForm lf, CASTTime timestamp) { 
	//	assert proxyFactories.containsKey(lf.root.sort);
		if (proxyFactories.containsKey(lf.root.sort)) {
			return ((ProxyFactory)proxyFactories.get(lf.root.sort)).constructProxy(lf, timestamp);
		}
		else {
			return new ProxyResults();
		}
	} // end constructProxy

	
	
} // end class


